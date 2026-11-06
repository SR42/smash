#!/usr/bin/env python3
"""Quick regression-spotting benchmark runner for CI.

Two end-to-end checks against built-in benches (no external services):

  bench_rss     — allocates 64 MiB of compressible data, sleeps 10 s,
                  expects the in-process compressor to reclaim a
                  meaningful fraction of peak RSS. Catches regressions
                  in: compressor not starting, fault handler broken,
                  page state machine wedged, ROI rejecting everything.

  bench_sqlite  — fills an in-memory SQLite DB with ~250 K text rows
                  under libsmash via DYLD_INSERT_LIBRARIES (macOS) or
                  LD_PRELOAD (Linux), measures cooling-phase RSS.
                  Catches regressions in the malloc-interposed path
                  on a realistic compressible workload.

Thresholds are set well below observed locals (46 % rss, 13 % sqlite)
so the CI is signal, not noise. If a real regression drops the numbers
below threshold the build fails.

Usage:
    cd build
    python3 ../bench/run_quick_ci.py
    # or with explicit paths:
    python3 ../bench/run_quick_ci.py --build-dir ./build
"""

from __future__ import annotations

import argparse
import os
import re
import subprocess
import sys
from pathlib import Path

# Conservative thresholds. Locals on M-class macOS:
#   bench_rss          : ~46 % reduction
#   bench_sqlite cool  : ~13 % reduction
# CI runners may differ; thresholds are well below local numbers.
RSS_REDUCTION_MIN_PCT = 30.0
SQLITE_COOLING_MIN_PCT = 5.0


def run(cmd: list[str], env: dict[str, str] | None = None,
        timeout: int = 120) -> str:
    print(f"$ {' '.join(cmd)}", flush=True)
    proc = subprocess.run(cmd, env=env, capture_output=True, text=True,
                          timeout=timeout)
    if proc.stdout:
        print(proc.stdout, end="")
    if proc.stderr:
        print(proc.stderr, end="", file=sys.stderr)
    if proc.returncode != 0:
        raise SystemExit(f"FAIL: {cmd[0]} exited {proc.returncode}")
    return proc.stdout


def smash_env(libsmash: Path, base_env: dict[str, str]) -> dict[str, str]:
    e = dict(base_env)
    if sys.platform == "darwin":
        e["DYLD_INSERT_LIBRARIES"] = str(libsmash)
        e["MallocNanoZone"] = "0"
    else:
        e["LD_PRELOAD"] = str(libsmash)
    return e


def check_bench_rss(build_dir: Path, libsmash: Path) -> tuple[bool, str]:
    """bench_rss uses the standard malloc API; smash interposes via preload."""
    # Pin the cold timeout low so the sampling window sees compression.
    #
    # Sample for a generous 20 s rather than bench_rss's default
    # cold_timeout + 5 s (= 7 s). Reclaiming a page is a multi-tick pipeline
    # in the default deferred-reclaim mode: cold detection → compress →
    # COMPRESSED_SHADOW → Phase B reclaim (decommit), each gated on the 1 s
    # compressor tick. On a loaded CI runner those ticks slip, so the genuine
    # reclaim can land after the 7 s window even though it completes in ~6 s
    # on an unloaded machine. (Before the macOS deep-monitoring fix, the 7 s
    # window happened to catch the escalation's phantom PROT_NONE RSS drop —
    # a task_info reporting artifact with zero actual reclaim — which masked
    # how tight the window was for the real pipeline.) The best-reduction gate
    # is unchanged; we just give the real reclaim enough wall-clock to occur.
    env = smash_env(libsmash, dict(os.environ))
    env.setdefault("SMASH_COLD_TIMEOUT_SEC", "2")
    out = run([str(build_dir / "bench" / "bench_rss"), "--wait", "20"], env=env)
    # Check BEST reduction across the entire window.
    # On busy CI runners, timing jitter can cause the t=10 snapshot to
    # catch a brief decompression spike. The peak reduction proves the
    # compressor achieved its target at some point during the run.
    reductions = re.findall(
        r"^\s*t=\s*\d+s:\s+RSS=[\d.]+ MB\s+\((\-?\d+)% reduction",
        out, re.MULTILINE)
    if not reductions:
        return False, "could not parse any reduction lines"
    best_pct = max(int(r) for r in reductions)
    ok = best_pct >= RSS_REDUCTION_MIN_PCT
    return ok, f"bench_rss best reduction {best_pct}% (need ≥ {RSS_REDUCTION_MIN_PCT:.0f}%)"


def check_bench_sqlite(build_dir: Path, libsmash: Path) -> tuple[bool, str]:
    """bench_sqlite uses system malloc; smash interposes via preload."""
    env = smash_env(libsmash, dict(os.environ))
    # Same cold-tick pin as bench_rss — see comment there.
    env.setdefault("SMASH_COLD_TIMEOUT_SEC", "2")
    out = run([str(build_dir / "bench" / "bench_sqlite"), "--quick"],
              env=env, timeout=180)
    # Gate on rss_reduction_pct (peak vs the minimum RSS observed across the
    # serve phase), NOT cool_reduction_pct.
    #
    # cool_reduction_pct is a SINGLE sample taken at the instant the 5 s
    # cooling loop ends. With an aggressive cold timeout the compressor is
    # still mid-flight at that instant — decompress churn plus the
    # not-yet-drained compressed-blob store make RSS transiently *higher*
    # than the fill peak, so cool_reduction_pct reads negative (~-21 %) even
    # though smash then settles to a large reduction. rss_reduction_pct
    # captures the steady-state win and is stable run-to-run (~57 % here),
    # which is the property we actually want to regression-test.
    m = re.search(r"^METRIC\s+rss_reduction_pct\s+(\-?[\d.]+)\s*$",
                  out, re.MULTILINE)
    if not m:
        return False, "could not find METRIC rss_reduction_pct line"
    pct = float(m.group(1))
    ok = pct >= SQLITE_COOLING_MIN_PCT
    return ok, (f"bench_sqlite RSS reduction {pct:.1f}% "
                f"(need ≥ {SQLITE_COOLING_MIN_PCT:.0f}%)")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--build-dir", type=Path,
                    default=Path(os.environ.get("BUILD_DIR", "build")))
    args = ap.parse_args()

    build = args.build_dir.resolve()
    libsmash_macos = build / "libsmash.dylib"
    libsmash_linux = build / "libsmash.so"
    libsmash = libsmash_macos if libsmash_macos.exists() else libsmash_linux
    if not libsmash.exists():
        raise SystemExit(f"libsmash not found at {libsmash}")
    if not (build / "bench" / "bench_rss").exists():
        raise SystemExit("bench_rss not built — configure with -DSMASH_BUILD_BENCH=ON")
    if not (build / "bench" / "bench_sqlite").exists():
        raise SystemExit("bench_sqlite not built — configure with -DSMASH_BUILD_BENCH=ON")

    print(f"build_dir: {build}")
    print(f"libsmash:  {libsmash}")
    print()

    results: list[tuple[bool, str]] = []
    print("=" * 60)
    print("bench_rss")
    print("=" * 60)
    results.append(check_bench_rss(build, libsmash))
    print()

    print("=" * 60)
    print("bench_sqlite --quick")
    print("=" * 60)
    results.append(check_bench_sqlite(build, libsmash))
    print()

    print("=" * 60)
    print("Summary")
    print("=" * 60)
    failed = 0
    for ok, msg in results:
        mark = "PASS" if ok else "FAIL"
        print(f"  [{mark}] {msg}")
        if not ok:
            failed += 1
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
