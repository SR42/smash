#!/usr/bin/env python3
"""Summarize a SMASH_PRESSURE_LOG file (see src/util/pressure_stats.h).

usage: pressure_report.py LOG [--ticks]

Per process:
  * Little's Law: lambda (entries/s), mean compressed population L, and
    W = L / lambda, versus the residence times measured directly at exit.
  * How compressed pages leave: refault (read/write), shadow discard, freed,
    or still compressed at exit (censored), with median residence time.
  * Payoff vs cost: net GB*s of RSS saved (compressed pages minus blob bytes)
    per CPU-second spent (compressor CPU + fault handling on app threads).
  * Scarcity: compressor stretch (phase wall / phase CPU; 1.0 = never waited
    for a core), PSI cpu some/full share of wall time, loadavg / nproc.
"""
import sys
from collections import defaultdict


def parse(line):
    kind, _, rest = line.partition(" ")
    d = {}
    for tok in rest.split():
        k, _, v = tok.partition("=")
        d[k] = v
    return kind, d


def hist_quantile(buckets, q):
    """Bucket b>=1 covers [2^(b-1), 2^b) ms; report the geometric midpoint."""
    n = sum(buckets)
    if n == 0:
        return None
    acc = 0
    for b, c in enumerate(buckets):
        acc += c
        if acc >= q * n:
            return 0.5 if b == 0 else 2 ** (b - 1) * 1.414
    return None


def fmt_ms(ms):
    if ms is None:
        return "-"
    return f"{ms:.0f}ms" if ms < 1000 else f"{ms / 1000:.1f}s"


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    show_ticks = "--ticks" in sys.argv
    procs = defaultdict(lambda: {"meta": {}, "ticks": [], "hist": {}})
    with open(sys.argv[1]) as f:
        for line in f:
            kind, d = parse(line.strip())
            if "pid" not in d:
                continue
            p = procs[d["pid"]]
            if kind == "meta":
                p["meta"] = d
            elif kind == "tick":
                p["ticks"].append({k: int(v) for k, v in d.items()})
            elif kind == "hist":
                p["hist"][d["kind"]] = (int(d["n"]), int(d["age_ms_sum"]),
                                        [int(x) for x in d["buckets"].split(",")])

    for pid, p in procs.items():
        ticks = p["ticks"]
        if not ticks:
            continue
        page = int(p["meta"].get("page", 4096))
        nproc = int(p["meta"].get("nproc", 1))
        T = sum(t["dt_ms"] for t in ticks) / 1000.0
        S = lambda k: sum(t[k] for t in ticks)
        # Time-weighted means.
        tw = lambda f: sum(f(t) * t["dt_ms"] for t in ticks) / max(1, S("dt_ms"))
        L = tw(lambda t: t["L"])
        Lall = tw(lambda t: t["L"] + t["shadow"])
        entries = S("enter")
        lam = entries / T if T else 0
        saved_gbs = sum((t["L"] * page - t["blob_kb"] * 1024) * t["dt_ms"] / 1000
                        for t in ticks) / 2**30
        rss_mean = tw(lambda t: t["rss_kb"]) / 1024
        comp_cpu = S("comp_cpu_us") / 1e6
        fault = S("fault_us") / 1e6
        decomp = S("decomp_us") / 1e6
        proc_cpu = S("proc_cpu_us") / 1e6
        app_cpu = proc_cpu - comp_cpu
        pc, pw = S("phase_cpu_us"), S("phase_wall_us")
        psi_some = S("psi_some_us") / 1e6 / T if T else 0
        psi_full = S("psi_full_us") / 1e6 / T if T else 0
        load = tw(lambda t: t["load_x100"]) / 100

        print(f"== pid {pid}: {T:.1f}s, nproc={nproc}, mean RSS {rss_mean:.0f} MB")
        print(f"  Little's Law: lambda={lam:.0f} pages/s  L={L:.0f} (+shadow {Lall - L:.0f})"
              f"  W=L/lambda={fmt_ms(1000 * Lall / lam) if lam else '-'}")
        h = p["hist"]
        if h:
            tot = sum(v[0] for v in h.values()) or 1
            parts = []
            for k in ("refault_read", "refault_write", "shadow_discard", "prefetch", "restored", "freed", "censored"):
                if k in h and h[k][0]:
                    n, age, b = h[k]
                    parts.append(f"{k} {100 * n / tot:.0f}% (n={n}, median {fmt_ms(hist_quantile(b, .5))},"
                                 f" mean {fmt_ms(age / n)})")
            print("  exits: " + "; ".join(parts))
            if p["meta"].get("write_bit") == "0":
                print("  (no fault write bit on this platform: refault_write includes reads)")
            done = [h[k] for k in ("refault_read", "refault_write", "shadow_discard", "prefetch", "restored", "freed") if k in h]
            n_done = sum(v[0] for v in done)
            if n_done:
                print(f"  measured mean W (completed residences) = {fmt_ms(sum(v[1] for v in done) / n_done)};"
                      f" censored residences are lower bounds")
        else:
            print("  (no hist lines: process did not exit through stop())")
        cost = comp_cpu + fault
        print(f"  payoff: {saved_gbs:.2f} GB*s saved net of blobs; cost {comp_cpu:.2f}s compressor CPU"
              f" + {fault:.2f}s fault handling ({decomp:.2f}s decompress)"
              f"  => {saved_gbs / cost if cost else float('inf'):.1f} GB*s per CPU*s")
        print(f"  CPU: process {proc_cpu:.1f}s, app {app_cpu:.1f}s ({app_cpu / T:.2f} cores),"
              f" compressor {comp_cpu / T:.3f} cores")
        print(f"  scarcity: stretch {pw / pc if pc else float('nan'):.2f}x"
              f"  PSI some {100 * psi_some:.0f}% full {100 * psi_full:.0f}%"
              f"  load {load:.1f}/{nproc}")
        if show_ticks:
            print("   t_s     L  enter   refault  comp_ms  fault_ms  stretch  psi%   rssMB")
            for t in ticks:
                st = t["phase_wall_us"] / t["phase_cpu_us"] if t["phase_cpu_us"] else 0
                print(f"  {t['t_ms'] / 1000:5.0f} {t['L']:6d} {t['enter']:6d} {t['rr'] + t['rw']:8d}"
                      f" {t['comp_cpu_us'] / 1000:8.1f} {t['fault_us'] / 1000:9.1f} {st:7.2f}"
                      f" {100 * t['psi_some_us'] / max(1, t['dt_ms'] * 1000):5.0f} {t['rss_kb'] / 1024:7.0f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
