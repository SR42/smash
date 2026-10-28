# Cutting a libsmash Release

Releases publish drop-in `libsmash` binaries for every supported platform and attach them to a GitHub Release. The workflow is `.github/workflows/release.yml`.

## How to cut one

```bash
gh release create vX.Y.Z-alpha --target master --prerelease \
  --title "libsmash vX.Y.Z-alpha" --notes "…release notes…"
```

- **Publishing a Release is what triggers the build.** The workflow runs on `release: published`. A bare `git tag` push does **not** trigger it, and neither does `workflow_dispatch` — the latter builds and uploads the artifacts as *workflow artifacts* for testing but skips the `publish` job (it is gated on `github.event_name == 'release'`). Use `workflow_dispatch` (`gh workflow run release.yml --ref <branch>`) to validate workflow changes on a branch without cutting a release.
- **Version convention:** all releases so far are `vX.Y.Z-alpha` pre-releases (`--prerelease`). Full mode is still experimental; large-only is the production-supported config.
- **Notes use `append_body: true`,** so the workflow *appends* a per-platform usage block (Linux `LD_PRELOAD`, macOS `xattr -d` + preload, arm64e zip instructions) to whatever notes you write. Don't duplicate usage instructions in `--notes`.
- The build takes a few minutes; the assets appear on the Release when the `publish` job finishes. **Always confirm the assets actually attached** — a release with missing assets is worse than none. Check the `verify-macos-arm64e` job went green, not just that the run "succeeded."

## The five assets

| asset | arch | notes |
|---|---|---|
| `libsmash-linux-x86_64.so` | x86-64 | glibc 2.28 floor (see below) |
| `libsmash-linux-aarch64.so` | ARM64 | glibc 2.28 floor |
| `libsmash-macos-arm64.dylib` | arm64 only | drop-in for ordinary arm64 apps |
| `libsmash-macos-fat.zip` | arm64 + arm64e | for **arm64e** apps; zipped to preserve the signature |
| `SHA256SUMS.txt` | — | checksums over all four binaries |

## Linux artifacts (`build-linux`)

Built inside **manylinux_2_28** (glibc 2.28, AlmaLinux 8 base) containers — one per arch — so they import only glibc ≤ 2.28 symbols. This is the drop-in floor: covers AL2023 (2.34), Ubuntu 20.04+ (2.31), Debian 11+ (2.31), RHEL/Rocky/Alma 8+ (2.28), Fedora. We deliberately do **not** use manylinux2014 (glibc 2.17 / CentOS 7): its headers/stdlib are too old to compile smash (missing `sys/random.h`, `MADV_FREE`/`MADV_PAGEOUT`, C++20 `<concepts>`).

Only *imported* (`*UND*`) symbols set the floor. smash deliberately **exports** re-export aliases at `GLIBC_2.33`/`2.34` via its version script (`src/smash_version_script.map.in`) so it out-versions modern libc's `malloc`/`stat`/`new`; those defined symbols must not count against the import floor. The stage step fails the build loudly if the highest *imported* glibc version sorts above 2.28.

Checkout/upload actions run on the **host** runner (modern glibc); only the *build* runs inside the container via `docker run` on the bind-mounted workspace. Using the manylinux image as a job `container:` makes every JS action die with "GLIBC_x not found" — its glibc is older than actions/checkout's bundled Node needs.

lz4 and zstd are statically linked, so there are no third-party shared-lib runtime deps beyond libc/libstdc++/libdl/libpthread/libm.

alloc8 comes from FetchContent like everything else: the commit `SMASH_ALLOC8_COMMIT` in CMakeLists.txt pins, with `patches/alloc8/*.patch` applied. Until 2026-10-28 both release jobs checked out alloc8's default branch beside smash and passed it as `ALLOC8_DIR`, so a released library carried whatever alloc8 was that day and none of smash's fixes to it.

## macOS artifacts — the arm64 / arm64e split

This is the subtle part; it went through several wrong turns before landing. See [macos-arm64e.md](macos-arm64e.md) for the full mechanism. In short:

- **libsmash must be fat (arm64 + arm64e) to interpose into arm64e processes.** An arm64-only dylib fails to load into an arm64e app with `incompatible architecture (have 'arm64', need 'arm64e')`.
- **The release ships both:** the fat build is thinned to `libsmash-macos-arm64.dylib` (byte-identical arm64 slice, drop-in for ordinary apps) and also shipped whole as `libsmash-macos-fat.zip`. The fat one is zipped because the arm64e slice's **code signature** must arrive intact; a raw downloaded binary is not the delivery vehicle for it.
- Both are **re-signed ad-hoc** (`codesign --force --sign -`) after `lipo`/copy — `lipo` strips the signature, and arm64e *mandates* a valid signature to load.

### Why the CI verification is split across two runners

`release.yml` has two macOS jobs:

- **`build-macos` (runs-on: `macos-14`)** — compiles the fat dylib, thins it, signs both, zips the fat one. Its checks are **static only**: arm64e slice present (`lipo -archs`) + `codesign -v`. It must **not** try to *execute* an arm64e binary.
- **`verify-macos-arm64e` (runs-on: `macos-26`)** — downloads the shipped zip, unzips, and preloads the dylib into a **real arm64e probe process**. This is the end-to-end proof the arm64e slice loads through the same round-trip a user gets. `publish` depends on this job.

The reason for the split — and the trap to remember: **macOS 14 (and 13/15) cannot exec a third-party arm64e binary at all.** Hosted runners on those images lack the `-arm64e_preview_abi` boot-arg, so any third-party arm64e process is **SIGKILLed** (exit 137) regardless of how it is signed. **macOS 26 (Tahoe) runs third-party arm64e** with no boot-arg and no SIP change. So the dynamic load check has to run on a `macos-26` runner; running it on `macos-14` always fails and has nothing to do with the artifact being correct.

The verify job is **capability-gated**: it first runs the bare arm64e probe (no dylib) inside an `if`. If the runner can't exec arm64e, that fails cleanly and the job *skips* the load check with a `::warning::` rather than blocking the release (the static slice + signature checks in `build-macos` still ran). On macOS 26 the bare probe succeeds, so the `DYLD_INSERT_LIBRARIES` load check runs and asserts `arm64e probe output: [ok]`.

> Debugging note: `exit 137` from an arm64e probe on macOS ≤ 15 means "this OS can't exec third-party arm64e," **not** "the dylib/signature is broken." Don't chase a signing bug for it — move the check to a macos-26 runner. Conversely, a fat dylib appearing to work on a dev machine may just mean that machine is on macOS 26 (or has SIP disabled); it is not evidence the *shipped* artifact loads for stock users. The CI probe on macos-26 is the source of truth, which is why it is a hard gate on `publish`.
