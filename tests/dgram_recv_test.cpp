// dgram_recv_test.cpp - receiving a datagram into a buffer smash has made
// inaccessible must not lose it.
//
// On a message socket (UDP, AF_UNIX SOCK_DGRAM / SOCK_SEQPACKET, raw) the
// kernel takes the datagram off the queue before copying it out; if the copy
// faults it frees the datagram and returns EFAULT. smash's EFAULT retry then
// received the NEXT datagram: the first was lost, silently. Under
// SMASH_SOFTDIRTY=0 every watched page is read-only, so any receive into a
// heap buffer on a watched page did this.
//
// The precondition is proved on the raw kernel entry first (a raw recvfrom
// into the buffer fails with EFAULT and the datagram is gone: the next raw
// receive gets the second one). Then each libc entry must return datagram 1
// and then datagram 2: recv, recvfrom (source address too), recvmsg, read,
// readv and __recv_chk on UDP; recv on AF_UNIX SOCK_DGRAM and SOCK_SEQPACKET;
// UDP with only the buffer's second page watched; recvmmsg; recvmsg with an
// SCM_RIGHTS fd (exactly one new descriptor, no leak from the peek); a
// non-blocking receive with nothing queued (EAGAIN, then the datagram); and
// an AF_UNIX stream socket (unchanged path). And recvmmsg / sendmmsg at the
// versions callers bind (GLIBC_2.12 / 2.14 on x86-64, 2.17 on aarch64) must
// resolve to libsmash: exported at the base version only, they never ran.
#include <arpa/inet.h>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dirent.h>
#include <dlfcn.h>
#include <fcntl.h>
#include <initializer_list>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/syscall.h>
#include <sys/uio.h>
#include <unistd.h>

extern "C" ssize_t __recv_chk(int fd, void* buf, size_t len, size_t buflen, int flags);

namespace {

constexpr size_t kBlock = 2u << 20;
constexpr size_t kDgram = 6000;     // spans two pages of the buffer
size_t g_ps = 4096;
int g_failures = 0;
int g_probe = -1;                   // a file for raw access probes

#define FAIL(...) do { std::printf("FAIL: " __VA_ARGS__); std::printf("\n"); ++g_failures; } while (0)

// A heap block whose page `watched_page` (0 or 1) is not writable by the
// kernel, the pages before it writable. Proved with a raw pread.
unsigned char* armedBuffer(int watched_page) {
    auto* b = static_cast<unsigned char*>(malloc(kBlock));
    std::memset(b, 0x11, kBlock);
    for (int i = 0; i < 120; ++i) {
        if (watched_page == 1) b[0] = 0x22;               // page 0 writable
        const size_t probe_len = (watched_page + 1) * g_ps;
        const ssize_t r = ::syscall(SYS_pread64, g_probe, b, probe_len, 0);
        const bool ok = watched_page == 0 ? (r == -1 && errno == EFAULT)
                                          : (r == static_cast<ssize_t>(g_ps));
        if (ok) return b;
        usleep(250 * 1000);
    }
    free(b);
    return nullptr;
}

void fillDgram(unsigned char* d, int n) { for (size_t i = 0; i < kDgram; ++i) d[i] = static_cast<unsigned char>(n * 37 + i % 251); }
bool isDgram(const unsigned char* d, ssize_t len, int n) {
    if (len != static_cast<ssize_t>(kDgram)) return false;
    for (size_t i = 0; i < kDgram; i += 97) if (d[i] != static_cast<unsigned char>(n * 37 + i % 251)) return false;
    return true;
}
int whichDgram(const unsigned char* d, ssize_t len) {
    for (int n = 1; n <= 2; ++n) if (isDgram(d, len, n)) return n;
    return 0;
}

// A lost datagram must fail the test, not hang it: every receiving socket
// gives up after 3 s.
bool withTimeout(int fd) {
    timeval tv{3, 0};
    return setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) == 0;
}

// A connected pair of the given family/type: fds[0] receives, fds[1] sends.
bool makePair(int family, int type, int fds[2]) {
    if (family == AF_UNIX) return socketpair(AF_UNIX, type, 0, fds) == 0 && withTimeout(fds[0]);
    const int r = socket(AF_INET, type, 0), s = socket(AF_INET, type, 0);
    sockaddr_in a{};
    a.sin_family = AF_INET;
    a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    socklen_t al = sizeof(a);
    if (r < 0 || s < 0 || bind(r, reinterpret_cast<sockaddr*>(&a), sizeof(a)) != 0 ||
        getsockname(r, reinterpret_cast<sockaddr*>(&a), &al) != 0 ||
        connect(s, reinterpret_cast<sockaddr*>(&a), sizeof(a)) != 0)
        return false;
    fds[0] = r;
    fds[1] = s;
    return withTimeout(r);
}

bool sendTwo(int s) {
    unsigned char d[kDgram];
    fillDgram(d, 1);
    if (send(s, d, kDgram, 0) != static_cast<ssize_t>(kDgram)) return false;
    fillDgram(d, 2);
    return send(s, d, kDgram, 0) == static_cast<ssize_t>(kDgram);
}

// Receives with `recv1` into a watched buffer, then the second datagram into
// a stack buffer: must be datagram 1, then 2.
template <typename Recv>
void dgramCase(const char* name, int family, int type, int watched_page, Recv recv1) {
    int fds[2];
    if (!makePair(family, type, fds)) { FAIL("%s: socket setup", name); return; }
    unsigned char* b = armedBuffer(watched_page);
    if (!b) { FAIL("%s: the buffer never became unwritable to the kernel (SMASH_SOFTDIRTY=0?)", name); return; }
    if (!sendTwo(fds[1])) { FAIL("%s: send", name); return; }
    errno = 0;
    const ssize_t r = recv1(fds[0], b);
    const int got = whichDgram(b, r);
    unsigned char second[kDgram + 16];
    const ssize_t r2 = ::recv(fds[0], second, sizeof(second), MSG_DONTWAIT);
    if (got != 1) FAIL("%s: returned %zd (errno %d), datagram %d instead of 1%s", name, r, errno, got,
                       got == 2 ? ": datagram 1 was lost" : "");
    else if (!isDgram(second, r2, 2)) FAIL("%s: the next receive did not get datagram 2 (%zd)", name, r2);
    else std::printf("%s: datagram 1 then 2\n", name);
    close(fds[0]);
    close(fds[1]);
    free(b);
}

int countFds() {
    int n = 0;
    if (DIR* d = opendir("/proc/self/fd")) {
        while (dirent* e = readdir(d)) if (e->d_name[0] != '.') ++n;
        closedir(d);
    }
    return n;
}

}  // namespace

int main() {
    setvbuf(stdout, nullptr, _IONBF, 0);   // keep partial output if a case hangs
    if (!dlsym(RTLD_DEFAULT, "smash_in_arena")) { std::fprintf(stderr, "FAIL: not running under libsmash\n"); return 1; }
    g_ps = static_cast<size_t>(sysconf(_SC_PAGESIZE));
    char path[] = "/tmp/smash_dgram_XXXXXX";
    g_probe = mkstemp(path);
    if (g_probe < 0) { std::printf("SKIP: mkstemp\n"); return 77; }
    unlink(path);
    {
        static unsigned char z[1 << 16];
        if (::syscall(SYS_pwrite64, g_probe, z, sizeof(z), 0) != sizeof(z)) { std::printf("SKIP: probe file\n"); return 77; }
    }

    // recvmmsg / sendmmsg at the versions callers bind.
    for (const char* name : {"recvmmsg", "sendmmsg"}) {
        const char* vers[] = {name[0] == 'r' ? "GLIBC_2.12" : "GLIBC_2.14", "GLIBC_2.17"};
        void* f = nullptr;
        const char* used = nullptr;
        for (const char* v : vers) if ((f = dlvsym(RTLD_DEFAULT, name, v))) { used = v; break; }
        Dl_info di{};
        if (!f || !dladdr(f, &di) || !di.dli_fname || !std::strstr(di.dli_fname, "smash"))
            FAIL("%s@%s resolves to %s, not libsmash", name, used ? used : "?",
                 di.dli_fname ? di.dli_fname : "nothing");
        else
            std::printf("%s@%s -> %s\n", name, used, di.dli_fname);
    }

    // Precondition: on this kernel a faulting UDP receive loses the datagram.
    {
        int fds[2];
        unsigned char* b = armedBuffer(0);
        if (!makePair(AF_INET, SOCK_DGRAM, fds) || !b || !sendTwo(fds[1])) {
            std::printf("FAIL: precondition setup\n");
            return 1;
        }
        const long r = ::syscall(SYS_recvfrom, fds[0], b, kDgram + 16, 0, nullptr, nullptr);
        const int e = errno;
        unsigned char next[kDgram + 16];
        const ssize_t r2 = ::recv(fds[0], next, sizeof(next), MSG_DONTWAIT);
        std::printf("raw recvfrom into the watched buffer: %ld (errno %d); next receive: datagram %d\n",
                    r, e, whichDgram(next, r2));
        if (r != -1 || e != EFAULT || !isDgram(next, r2, 2)) {
            std::printf("FAIL: precondition: expected EFAULT and datagram 1 gone\n");
            return 1;
        }
        close(fds[0]);
        close(fds[1]);
        free(b);
    }

    dgramCase("udp recv", AF_INET, SOCK_DGRAM, 0,
              [](int fd, unsigned char* b) { return recv(fd, b, kDgram + 16, 0); });
    dgramCase("udp recvfrom", AF_INET, SOCK_DGRAM, 0, [](int fd, unsigned char* b) {
        // The source address goes into the watched block too.
        auto* from = reinterpret_cast<sockaddr_in*>(b + kBlock - 2 * g_ps);
        auto* fromlen = reinterpret_cast<socklen_t*>(b + kBlock - 2 * g_ps + 64);
        *fromlen = sizeof(sockaddr_in);
        const ssize_t r = recvfrom(fd, b, kDgram + 16, 0, reinterpret_cast<sockaddr*>(from), fromlen);
        if (r >= 0 && (from->sin_family != AF_INET || *fromlen != sizeof(sockaddr_in))) return ssize_t{-2};
        return r;
    });
    dgramCase("udp recvmsg", AF_INET, SOCK_DGRAM, 0, [](int fd, unsigned char* b) {
        iovec v[2] = {{b, 3000}, {b + 3000, kDgram + 16 - 3000}};
        msghdr m{};
        m.msg_iov = v;
        m.msg_iovlen = 2;
        return recvmsg(fd, &m, 0);
    });
    dgramCase("udp read", AF_INET, SOCK_DGRAM, 0,
              [](int fd, unsigned char* b) { return read(fd, b, kDgram + 16); });
    dgramCase("udp readv", AF_INET, SOCK_DGRAM, 0, [](int fd, unsigned char* b) {
        iovec v[2] = {{b, 3000}, {b + 3000, kDgram + 16 - 3000}};
        return readv(fd, v, 2);
    });
    dgramCase("udp __recv_chk", AF_INET, SOCK_DGRAM, 0,
              [](int fd, unsigned char* b) { return __recv_chk(fd, b, kDgram + 16, kDgram + 16, 0); });
    dgramCase("udp recv, second page watched", AF_INET, SOCK_DGRAM, 1,
              [](int fd, unsigned char* b) { return recv(fd, b, kDgram + 16, 0); });
    dgramCase("unix dgram recv", AF_UNIX, SOCK_DGRAM, 0,
              [](int fd, unsigned char* b) { return recv(fd, b, kDgram + 16, 0); });
    dgramCase("unix seqpacket recv", AF_UNIX, SOCK_SEQPACKET, 0,
              [](int fd, unsigned char* b) { return recv(fd, b, kDgram + 16, 0); });
    dgramCase("udp recvmmsg", AF_INET, SOCK_DGRAM, 0, [](int fd, unsigned char* b) {
        iovec v{b, kDgram + 16};
        mmsghdr mm{};
        mm.msg_hdr.msg_iov = &v;
        mm.msg_hdr.msg_iovlen = 1;
        const int n = recvmmsg(fd, &mm, 1, 0, nullptr);
        return n == 1 ? static_cast<ssize_t>(mm.msg_len) : ssize_t{-1};
    });

    // Non-blocking, nothing queued: EAGAIN, and the datagram sent after it
    // arrives intact.
    {
        int fds[2];
        unsigned char* b = armedBuffer(0);
        if (makePair(AF_INET, SOCK_DGRAM, fds) && b) {
            const ssize_t r0 = recv(fds[0], b, kDgram + 16, MSG_DONTWAIT);
            const int e0 = errno;
            unsigned char d[kDgram];
            fillDgram(d, 1);
            send(fds[1], d, kDgram, 0);
            const ssize_t r = recv(fds[0], b, kDgram + 16, 0);
            if (r0 != -1 || e0 != EAGAIN) FAIL("nonblocking: empty queue returned %zd errno %d", r0, e0);
            else if (!isDgram(b, r, 1)) FAIL("nonblocking: then got %zd bytes, not datagram 1", r);
            else std::printf("nonblocking: EAGAIN, then datagram 1\n");
            close(fds[0]);
            close(fds[1]);
        } else {
            FAIL("nonblocking: setup");
        }
        free(b);
    }

    // SCM_RIGHTS over AF_UNIX dgram: exactly one new descriptor.
    {
        int fds[2];
        unsigned char* b = armedBuffer(0);
        if (b && socketpair(AF_UNIX, SOCK_DGRAM, 0, fds) == 0 && withTimeout(fds[0])) {
            unsigned char d[kDgram];
            fillDgram(d, 1);
            iovec sv{d, kDgram};
            alignas(cmsghdr) char ctl[CMSG_SPACE(sizeof(int))] = {};
            msghdr sm{};
            sm.msg_iov = &sv;
            sm.msg_iovlen = 1;
            sm.msg_control = ctl;
            sm.msg_controllen = sizeof(ctl);
            cmsghdr* c = CMSG_FIRSTHDR(&sm);
            c->cmsg_level = SOL_SOCKET;
            c->cmsg_type = SCM_RIGHTS;
            c->cmsg_len = CMSG_LEN(sizeof(int));
            std::memcpy(CMSG_DATA(c), &g_probe, sizeof(int));
            sendmsg(fds[1], &sm, 0);
            const int before = countFds();
            iovec rv{b, kDgram + 16};
            char* rctl = reinterpret_cast<char*>(b + kBlock - 4 * g_ps);   // in the watched block
            msghdr rm{};
            rm.msg_iov = &rv;
            rm.msg_iovlen = 1;
            rm.msg_control = rctl;
            rm.msg_controllen = CMSG_SPACE(sizeof(int));
            const ssize_t r = recvmsg(fds[0], &rm, 0);
            const int after = countFds();
            cmsghdr* rc = CMSG_FIRSTHDR(&rm);
            int passed = -1;
            if (rc && rc->cmsg_type == SCM_RIGHTS) std::memcpy(&passed, CMSG_DATA(rc), sizeof(int));
            if (!isDgram(b, r, 1)) FAIL("scm_rights: got %zd bytes, not datagram 1", r);
            else if (after != before + 1 || passed < 0 || fcntl(passed, F_GETFD) < 0)
                FAIL("scm_rights: %d descriptors before, %d after (want +1), passed fd %d", before, after, passed);
            else std::printf("scm_rights: datagram 1, exactly one new descriptor\n");
            if (passed >= 0) close(passed);
            close(fds[0]);
            close(fds[1]);
        } else {
            FAIL("scm_rights: setup");
        }
        free(b);
    }

    // A stream socket: the usual retry path, bytes in order.
    {
        int fds[2];
        unsigned char* b = armedBuffer(0);
        if (b && socketpair(AF_UNIX, SOCK_STREAM, 0, fds) == 0 && withTimeout(fds[0])) {
            unsigned char d[kDgram];
            fillDgram(d, 1);
            send(fds[1], d, kDgram, 0);
            size_t got = 0;
            while (got < kDgram) {
                const ssize_t r = recv(fds[0], b + got, kDgram - got, 0);
                if (r <= 0) break;
                got += static_cast<size_t>(r);
            }
            if (!isDgram(b, static_cast<ssize_t>(got), 1)) FAIL("stream: bytes lost or wrong (%zu)", got);
            else std::printf("stream: all bytes in order\n");
            close(fds[0]);
            close(fds[1]);
        } else {
            FAIL("stream: setup");
        }
        free(b);
    }

    close(g_probe);
    if (g_failures) { std::printf("dgram_recv_test: %d FAILURES\n", g_failures); return 1; }
    std::printf("dgram_recv_test: all checks passed\n");
    return 0;
}
