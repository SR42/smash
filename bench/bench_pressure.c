// bench_pressure.c - time/space trade-off under CPU scarcity
//
// A 2x2 for evaluating compression policy (see SMASH_PRESSURE_LOG and
// scripts/pressure_report.py):
//
//   data:  cold   90% of the heap is written once and never touched again;
//                 compression should win (long residence, no refaults).
//          churn  the whole heap is re-read every --period seconds; with the
//                 cold timeout below the period, pages compress and then
//                 refault on the next sweep (short residence: pure overhead).
//   cores: idle   --workers app threads well below the core count.
//          busy   add --spin CPU-bound threads so no core is spare.
//
// Work threads loop over a hot 10% of the heap doing read-modify-write until
// --secs elapse and report throughput (ops/s). Churn sweeps are part of the
// work threads' job, so refault cost lands on the app as it would in real
// code. A sampler thread records mean RSS.
//
// usage: bench_pressure cold|churn [--mb N] [--workers N] [--spin N]
//                                   [--secs N] [--period S]
// prints one line: mode=... ops_per_s=... rss_mean_mb=... rss_max_mb=... rss_fail=N
#include <pthread.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#if defined(__APPLE__)
#include <mach/mach.h>
#endif

enum { kObj = 256 };
static size_t nobj, nhot;
static unsigned char** objs;
static int churn, nworkers = 2, nspin = 0, secs = 30;
static double period = 5.0;
static atomic_int stop_flag;
static atomic_ullong total_ops;

static double now(void) {
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  return t.tv_sec + t.tv_nsec * 1e-9;
}

static uint64_t rng(uint64_t* s) {
  uint64_t x = *s;
  x ^= x >> 12; x ^= x << 25; x ^= x >> 27;
  *s = x;
  return x * 0x2545F4914F6CDD1DULL;
}

static size_t rss_kb(void) {
#if defined(__APPLE__)
  mach_task_basic_info_data_t info;
  mach_msg_type_number_t c = MACH_TASK_BASIC_INFO_COUNT;
  if (task_info(mach_task_self(), MACH_TASK_BASIC_INFO, (task_info_t)&info, &c) != KERN_SUCCESS) return 0;
  return info.resident_size / 1024;
#else
  FILE* f = fopen("/proc/self/statm", "r");
  unsigned long sz = 0, res = 0;
  if (f) { if (fscanf(f, "%lu %lu", &sz, &res) != 2) res = 0; fclose(f); }
  return res * (unsigned long)sysconf(_SC_PAGESIZE) / 1024;
#endif
}

// Moderately compressible content (~3-4x with LZ4): small alphabet + runs.
static void fill(unsigned char* p, uint64_t* s) {
  for (int i = 0; i < kObj; i += 8) {
    uint64_t r = rng(s);
    memset(p + i, 'a' + (int)(r % 6), 8);
    if ((r >> 8) % 4 == 0) p[i + (r >> 16) % 8] = (unsigned char)(r >> 24);
  }
}

static void* worker(void* arg) {
  uintptr_t id = (uintptr_t)arg;
  uint64_t s = 0x9E3779B97F4A7C15ULL * (id + 1);
  uint64_t ops = 0, sink = 0;
  double next_sweep = now() + period;
  size_t lo = nobj * id / nworkers, hi = nobj * (id + 1) / nworkers;
  while (!atomic_load_explicit(&stop_flag, memory_order_relaxed)) {
    for (int k = 0; k < 4096; k++) {
      unsigned char* p = objs[rng(&s) % nhot];
      p[rng(&s) % kObj] += 1;  // read-modify-write keeps the hot set hot
      ops++;
    }
    if (churn && now() >= next_sweep) {
      // Re-read this worker's slice of the whole heap (refaults whatever
      // was compressed since the last sweep).
      for (size_t i = lo; i < hi; i++) sink += objs[i][(i * 31) % kObj];
      next_sweep = now() + period;
    }
  }
  atomic_fetch_add(&total_ops, ops);
  return (void*)(uintptr_t)sink;
}

static void* spinner(void* arg) {
  (void)arg;
  volatile uint64_t x = 1;
  while (!atomic_load_explicit(&stop_flag, memory_order_relaxed))
    for (int i = 0; i < 100000; i++) x = x * 6364136223846793005ULL + 1;
  return NULL;
}

int main(int argc, char** argv) {
  if (argc < 2) { fprintf(stderr, "usage: bench_pressure cold|churn [opts]\n"); return 2; }
  churn = !strcmp(argv[1], "churn");
  size_t mb = 512;
  for (int i = 2; i + 1 < argc; i += 2) {
    if (!strcmp(argv[i], "--mb")) mb = strtoul(argv[i + 1], 0, 10);
    else if (!strcmp(argv[i], "--workers")) nworkers = atoi(argv[i + 1]);
    else if (!strcmp(argv[i], "--spin")) nspin = atoi(argv[i + 1]);
    else if (!strcmp(argv[i], "--secs")) secs = atoi(argv[i + 1]);
    else if (!strcmp(argv[i], "--period")) period = atof(argv[i + 1]);
  }
  nobj = mb * 1024 * 1024 / kObj;
  nhot = nobj / 10;
  objs = malloc(nobj * sizeof *objs);
  uint64_t s = 7;
  for (size_t i = 0; i < nobj; i++) { objs[i] = malloc(kObj); fill(objs[i], &s); }

  pthread_t w[256], sp[256];
  double t0 = now();
  for (int i = 0; i < nworkers; i++) pthread_create(&w[i], 0, worker, (void*)(uintptr_t)i);
  for (int i = 0; i < nspin; i++) pthread_create(&sp[i], 0, spinner, 0);
  // A failed read returns 0 (seen near vm.max_map_count, where fopen's
  // allocation can fail). Skip those samples rather than average them in.
  double rss_sum = 0; size_t rss_max = 0; int nsamp = 0, nfail = 0;
  while (now() - t0 < secs) {
    usleep(100000);
    size_t r = rss_kb();
    if (r == 0) { nfail++; continue; }
    rss_sum += r; nsamp++;
    if (r > rss_max) rss_max = r;
  }
  atomic_store(&stop_flag, 1);
  for (int i = 0; i < nworkers; i++) pthread_join(w[i], 0);
  for (int i = 0; i < nspin; i++) pthread_join(sp[i], 0);
  double el = now() - t0;
  printf("mode=%s workers=%d spin=%d ops_per_s=%.0f rss_mean_mb=%.1f rss_max_mb=%.1f"
         " rss_fail=%d\n",
         argv[1], nworkers, nspin, atomic_load(&total_ops) / el,
         nsamp ? rss_sum / nsamp / 1024 : 0.0, rss_max / 1024.0, nfail);
  return 0;
}
