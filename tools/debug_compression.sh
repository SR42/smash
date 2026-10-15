#!/usr/bin/env bash
# Test different smash compression configurations to isolate bug
set -euo pipefail

NCC=/local/home/emerydb/new-kaena/KaenaCompilerContainerBuild/src/KaenaCompiler
SMASH=$NCC/smash/build/libsmash.so
PYTHON=/opt/venvs/3.13/bin/python3
HLO=$NCC/test7_fsdp_c474_b8_63l_135416_def_modular.hlo
OUT=$NCC/smash/build/debug_compress_$(date +%Y%m%d_%H%M%S)
RUNS=${1:-3}

mkdir -p "$OUT"

export PYTHONPATH="$NCC:$NCC/build/python/3.13/install"

run_compile() {
  local label="$1"
  local run_num="$2"
  local extra_env="$3"
  
  local outneff="$OUT/${label}_run${run_num}.neff"
  local logfile="$OUT/${label}_run${run_num}.log"
  
  echo -n "  Run $run_num: "
  
  rm -f "$outneff"
  
  local start_time=$(date +%s)
  
  export PYTHONMALLOC=malloc
  export LD_PRELOAD="$SMASH"
  
  # Set config vars
  for var in $extra_env; do
    export "$var"
  done
  
  "$PYTHON" -m neuronxcc.driver.CommandDriver compile \
    --framework XLA "$HLO" --target trn2 --output "$outneff" \
    > "$logfile" 2>&1
  local rc=$?
  
  local end_time=$(date +%s)
  local wall_time=$((end_time - start_time))
  
  local status="FAIL"
  local error_type="unknown"
  if [ -f "$outneff" ]; then
    status="PASS"
    error_type="none"
  else
    if grep -q "bad_alloc" "$logfile" 2>/dev/null; then
      error_type="bad_alloc"
    elif grep -q "Assertion" "$logfile" 2>/dev/null; then
      error_type=$(grep "Assertion" "$logfile" | head -1 | cut -c1-80)
    elif grep -q "SIGSEGV\|signal 11" "$logfile" 2>/dev/null; then
      error_type="SIGSEGV"
    fi
  fi
  
  echo "$status (${wall_time}s) $error_type"
  echo "$label,$run_num,$status,$wall_time,$error_type" >> "$OUT/results.csv"
  
  # Clean up
  unset LD_PRELOAD 2>/dev/null || true
  for var in $extra_env; do
    local name="${var%%=*}"
    unset "$name" 2>/dev/null || true
  done
}

echo "label,run,status,wall_s,error_type" > "$OUT/results.csv"

echo "=== Smash Compression Debug Test ==="
echo "Running $RUNS iterations per config"
echo ""

# Config 1: No compression at all (COLD_TICKS=9999)
echo "no_compress (COLD_TICKS=9999):"
for i in $(seq 1 $RUNS); do
  run_compile "no_compress" "$i" "SMASH_VM_GIB=48 SMASH_COLD_TICKS=9999 SMASH_VERY_COLD_TICKS=99999"
done

# Config 2: LZ4 instead of zstd (SMASH_CODEC=lz4; SMASH_USE_LZ4=1 was never read)
echo ""
echo "lz4_only (SMASH_CODEC=lz4):"
for i in $(seq 1 $RUNS); do
  run_compile "lz4_only" "$i" "SMASH_VM_GIB=48 SMASH_COLD_TIMEOUT_SEC=10 SMASH_CODEC=lz4 SMASH_DEFER_MADVISE=1"
done

# Config 3: Fast tier only (no deep tier)
echo ""
echo "fast_tier_only (VERY_COLD_TICKS=9999):"
for i in $(seq 1 $RUNS); do
  run_compile "fast_tier_only" "$i" "SMASH_VM_GIB=48 SMASH_COLD_TIMEOUT_SEC=10 SMASH_VERY_COLD_TICKS=9999 SMASH_DEFER_MADVISE=1"
done

# Config 4: Single compressor worker
echo ""
echo "single_worker (SMASH_COMPRESSOR_WORKERS=1):"
for i in $(seq 1 $RUNS); do
  run_compile "single_worker" "$i" "SMASH_VM_GIB=48 SMASH_COLD_TIMEOUT_SEC=10 SMASH_COMPRESSOR_WORKERS=1 SMASH_DEFER_MADVISE=1"
done

# Config 5: Full mode default (for comparison)
echo ""
echo "full_default:"
for i in $(seq 1 $RUNS); do
  run_compile "full_default" "$i" "SMASH_VM_GIB=48 SMASH_COLD_TIMEOUT_SEC=10 SMASH_DEFER_MADVISE=1"
done

echo ""
echo "=== Summary ==="
echo ""
echo "Pass rates:"
for cfg in no_compress lz4_only fast_tier_only single_worker full_default; do
  total=$(grep "^$cfg," "$OUT/results.csv" | wc -l)
  if [ "$total" -gt 0 ]; then
    pass=$(grep "^$cfg," "$OUT/results.csv" | grep ",PASS," | wc -l)
    echo "  $cfg: $pass/$total"
  fi
done

echo ""
echo "Results in: $OUT/results.csv"
cat "$OUT/results.csv"
