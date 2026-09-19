#pragma once
// uffd_uapi.h - the userfaultfd write-protect and PAGEMAP_SCAN UAPI smash uses
// for write tracking where soft-dirty is unavailable (arm64 has no
// CONFIG_MEM_SOFT_DIRTY). Both are stable kernel ABI; the definitions are
// repeated here so a binary built against older headers (release builds use an
// old glibc floor) still uses the feature on a new enough kernel. The values
// match include/uapi/linux/{userfaultfd,fs}.h.
#if defined(__linux__)
#include <cstdint>
#include <linux/ioctl.h>
#include <linux/types.h>
#include <linux/userfaultfd.h>
#include <sys/syscall.h>

#ifndef __NR_userfaultfd
#  if defined(__x86_64__)
#    define __NR_userfaultfd 323
#  elif defined(__aarch64__)
#    define __NR_userfaultfd 282
#  endif
#endif

#ifndef UFFD_USER_MODE_ONLY
#define UFFD_USER_MODE_ONLY 1
#endif
#ifndef UFFD_FEATURE_WP_UNPOPULATED
#define UFFD_FEATURE_WP_UNPOPULATED (1 << 13)   // Linux 6.4
#endif
#ifndef UFFD_FEATURE_WP_ASYNC
#define UFFD_FEATURE_WP_ASYNC (1 << 15)         // Linux 6.7
#endif
#ifndef UFFDIO_REGISTER_MODE_WP
#define UFFDIO_REGISTER_MODE_WP ((__u64)1 << 1)
#endif

namespace smash::uffd {

// PAGEMAP_SCAN (Linux 6.7), an ioctl on /proc/self/pagemap.
constexpr uint64_t kPageIsWritten = 1u << 1;   // PAGE_IS_WRITTEN
constexpr uint64_t kScanWpMatching = 1u << 0;  // PM_SCAN_WP_MATCHING

struct PageRegion {                            // struct page_region
    uint64_t start, end, categories;
};

struct PmScanArg {                             // struct pm_scan_arg
    uint64_t size, flags, start, end, walk_end, vec, vec_len, max_pages;
    uint64_t category_inverted, category_mask, category_anyof_mask, return_mask;
};

constexpr unsigned long kPagemapScan = _IOWR('f', 16, PmScanArg);  // PAGEMAP_SCAN

}  // namespace smash::uffd
#endif  // __linux__
