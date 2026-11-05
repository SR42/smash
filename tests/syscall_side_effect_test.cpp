// syscall_side_effect_test.cpp - an out-parameter the kernel writes AFTER it
// has done the work must not make smash's EFAULT retry do the work twice.
//
// wait4/waitid reap the child, accept dequeues the connection, a receive
// consumes the data, sendfile transfers it, sendmmsg sends the datagram,
// sigprocmask and prlimit64 apply the new mask or limit, an interrupted
// nanosleep has slept -- and only then do they write their result. When that
// write faults (a watched or compressed smash page), the kernel returns
// EFAULT with the work done. smash's wrappers retried, doing it again: the
// child's exit lost (ECHILD), the connection closed and the next one taken,
// stream data and datagrams consumed, the file sent twice, the datagram sent
// nine times, the new mask/limit reported as the old one, the whole sleep
// again. A control buffer the kernel cannot write is not even an EFAULT: the
// message arrives with MSG_CTRUNC and the passed descriptor is gone.
//
// Each case puts the out-parameter on the first page of a heap block that the
// kernel cannot write (SMASH_SOFTDIRTY=0: a watched page is PROT_READ; proved
// with a raw pread into it -> EFAULT), everything else on the stack, and
// checks the call did its work exactly once and reported it. The precondition
// is proved on the raw kernel entry first: a raw wait4 into the block reaps
// the child and returns EFAULT. accept4 and prlimit64 are called through
// dlsym (checked to resolve into libsmash): glibc versions them above the
// base version smash exports them at, so a direct call binds glibc's.
#include <cerrno>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <dlfcn.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <pthread.h>
#include <sys/mman.h>
#include <sys/resource.h>
#include <sys/sendfile.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/syscall.h>
#include <sys/uio.h>
#include <sys/wait.h>
#include <unistd.h>

extern "C" ssize_t __recvfrom_chk(int fd, void* buf, size_t len, size_t buflen, int flags,
                                  struct sockaddr* addr, socklen_t* addrlen);

namespace {

constexpr size_t kBlock = 2u << 20;
size_t g_ps = 4096;
int g_failures = 0;
int g_zero = -1;       // /dev/zero: can the kernel write the page?
int g_pipe[2];         // can the kernel read it?

#define FAIL(...) do { std::printf("FAIL: " __VA_ARGS__); std::printf("\n"); ++g_failures; } while (0)
#define PASS(...) do { std::printf(__VA_ARGS__); std::printf("\n"); } while (0)

// A heap block whose first page the kernel cannot write, prep(b) applied
// first: everything a case needs is written before the wait, since a write is
// what keeps a page out of the compressor's reach. The probes use the page's
// last byte, which no case uses. `readable`: the kernel must still be able to
// read the page (watched, not yet compressed) -- sendfile, sendmmsg and
// recvmsg read their out-parameter's page before they write it.
template <typename Prep>
unsigned char* armed(bool readable, Prep prep) {
    for (int attempt = 0; attempt < 3; ++attempt) {
        auto* b = static_cast<unsigned char*>(malloc(kBlock));
        if (!b) return nullptr;
        std::memset(b, 0x11, kBlock);
        prep(b);
        unsigned char* probe = b + g_ps - 1;
        for (int i = 0; i < 160; ++i) {
            if (::syscall(SYS_pread64, g_zero, probe, 1, 0) == -1 && errno == EFAULT) {
                if (!readable) return b;
                if (::syscall(SYS_write, g_pipe[1], probe, 1) == 1) {
                    char c;
                    (void)!::read(g_pipe[0], &c, 1);
                    return b;
                }
                break;   // compressed before it was seen watched: a fresh block
            }
            usleep(250 * 1000);
        }
        free(b);
    }
    return nullptr;
}
void nop(unsigned char*) {}

pid_t forkExit(int code) {
    const pid_t p = fork();
    if (p == 0) _exit(code);
    return p;
}
// Waits until the child has exited without reaping it.
bool zombie(pid_t pid) {
    siginfo_t si{};
    return waitid(P_PID, static_cast<id_t>(pid), &si, WEXITED | WNOWAIT) == 0 && si.si_pid == pid;
}
bool gone(pid_t pid) { return kill(pid, 0) == -1 && errno == ESRCH; }

bool inLibsmash(void* fn) {
    Dl_info info{};
    return fn && dladdr(fn, &info) && info.dli_fname && std::strstr(info.dli_fname, "libsmash");
}

sockaddr_in loopback() {
    sockaddr_in a{};
    a.sin_family = AF_INET;
    a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    return a;
}
// A non-blocking listener on loopback with one connection queued; *cli is the
// connecting socket.
int listenerWithConnection(int* cli) {
    const int l = socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK, 0);
    sockaddr_in a = loopback();
    socklen_t al = sizeof(a);
    if (l < 0 || bind(l, reinterpret_cast<sockaddr*>(&a), sizeof(a)) != 0 ||
        getsockname(l, reinterpret_cast<sockaddr*>(&a), &al) != 0 || listen(l, 4) != 0)
        return -1;
    *cli = socket(AF_INET, SOCK_STREAM, 0);
    if (*cli < 0 || connect(*cli, reinterpret_cast<sockaddr*>(&a), sizeof(a)) != 0) return -1;
    return l;
}
// A connected TCP pair; *srv is non-blocking.
bool tcpPair(int* srv, int* cli) {
    const int l = listenerWithConnection(cli);
    if (l < 0) return false;
    for (int i = 0; i < 100 && (*srv = accept(l, nullptr, nullptr)) < 0; ++i) usleep(10 * 1000);
    close(l);
    return *srv >= 0 && fcntl(*srv, F_SETFL, O_NONBLOCK) == 0;
}
// A UDP receiver bound to loopback and a sender connected to it.
bool udpPair(int* rcv, int* snd) {
    *rcv = socket(AF_INET, SOCK_DGRAM, 0);
    *snd = socket(AF_INET, SOCK_DGRAM, 0);
    sockaddr_in a = loopback();
    socklen_t al = sizeof(a);
    return *rcv >= 0 && *snd >= 0 && bind(*rcv, reinterpret_cast<sockaddr*>(&a), sizeof(a)) == 0 &&
           getsockname(*rcv, reinterpret_cast<sockaddr*>(&a), &al) == 0 &&
           connect(*snd, reinterpret_cast<sockaddr*>(&a), sizeof(a)) == 0;
}
unsigned short localPort(int fd) {
    sockaddr_in a{};
    socklen_t al = sizeof(a);
    getsockname(fd, reinterpret_cast<sockaddr*>(&a), &al);
    return a.sin_port;
}

void onSignal(int) {}
// Sends SIGUSR1 to `target` after 200 ms.
void* interrupter(void* arg) {
    usleep(200 * 1000);
    pthread_kill(*static_cast<pthread_t*>(arg), SIGUSR1);
    return nullptr;
}
double since(const timespec& t0) {
    timespec t1;
    clock_gettime(CLOCK_MONOTONIC, &t1);
    return static_cast<double>(t1.tv_sec - t0.tv_sec) + (t1.tv_nsec - t0.tv_nsec) / 1e9;
}

#define ARMED(name, readable, prep)                                                         \
    unsigned char* b = armed(readable, prep);                                               \
    if (!b) { FAIL("%s: the block never became unwritable to the kernel "                   \
                   "(not under libsmash, or not SMASH_SOFTDIRTY=0?)", name); return; }

// ── The cases ────────────────────────────────────────────────────────────

void rawPrecondition() {
    ARMED("precondition", false, nop);
    const pid_t pid = forkExit(41);
    zombie(pid);
    const long r = ::syscall(SYS_wait4, pid, reinterpret_cast<int*>(b), 0, nullptr);
    const int e = errno;
    if (r == -1 && e == EFAULT && gone(pid))
        PASS("precondition: a raw wait4 into the block reaped the child and returned EFAULT");
    else
        FAIL("precondition: raw wait4 returned %ld (errno %d), child %s", r, e, gone(pid) ? "reaped" : "not reaped");
    free(b);
}

template <typename Wait>
void waitCase(const char* name, int code, Wait w) {
    ARMED(name, false, nop);
    const pid_t pid = forkExit(code);
    zombie(pid);
    errno = 0;
    const pid_t r = w(pid, b);
    const int e = errno, st = *reinterpret_cast<int*>(b);
    if (r != pid || !WIFEXITED(st) || WEXITSTATUS(st) != code)
        FAIL("%s: returned %d (errno %d), status %#x: the exit of child %d was lost", name, r, e, st, pid);
    else
        PASS("%s: reaped child %d, exit status %d", name, pid, code);
    free(b);
}

void waitidCase() {
    ARMED("waitid", false, nop);
    const pid_t pid = forkExit(43);
    zombie(pid);
    errno = 0;
    auto* si = reinterpret_cast<siginfo_t*>(b);
    const int r = waitid(P_PID, static_cast<id_t>(pid), si, WEXITED);
    const int e = errno;
    if (r != 0 || si->si_pid != pid || si->si_code != CLD_EXITED || si->si_status != 43)
        FAIL("waitid: returned %d (errno %d), si_pid %d si_code %d si_status %d", r, e,
             si->si_pid, si->si_code, si->si_status);
    else if (!gone(pid))
        FAIL("waitid: child %d was not reaped", pid);
    else
        PASS("waitid: reaped child %d, exit status 43", pid);
    free(b);
}

template <typename Accept>
void acceptCase(const char* name, Accept acc, bool cloexec) {
    int cli = -1;
    const int l = listenerWithConnection(&cli);
    if (l < 0) { FAIL("%s: socket setup", name); return; }
    usleep(50 * 1000);
    ARMED(name, false, [](unsigned char* p) { *reinterpret_cast<socklen_t*>(p) = sizeof(sockaddr_in); });
    auto* len = reinterpret_cast<socklen_t*>(b);
    auto* peer = reinterpret_cast<sockaddr_in*>(b + 64);
    errno = 0;
    const int fd = acc(l, reinterpret_cast<sockaddr*>(peer), len);
    const int e = errno;
    char c = 0;
    const bool alive = fd >= 0 && write(fd, "x", 1) == 1 && read(cli, &c, 1) == 1 && c == 'x';
    const int extra = accept(l, nullptr, nullptr);
    if (fd < 0)
        FAIL("%s: returned %d (errno %d): the queued connection was dropped", name, fd, e);
    else if (*len != sizeof(sockaddr_in) || peer->sin_port != localPort(cli))
        FAIL("%s: peer address wrong (len %u, port %u, client port %u)", name, *len,
             ntohs(peer->sin_port), ntohs(localPort(cli)));
    else if (!alive)
        FAIL("%s: the accepted connection does not reach the client", name);
    else if (cloexec && !(fcntl(fd, F_GETFD) & FD_CLOEXEC))
        FAIL("%s: SOCK_CLOEXEC not applied", name);
    else if (extra >= 0)
        FAIL("%s: a second connection was queued", name);
    else
        PASS("%s: the queued connection, its peer address", name);
    if (fd >= 0) close(fd);
    if (extra >= 0) close(extra);
    close(l);
    close(cli);
    free(b);
}

// recvfrom / __recvfrom_chk on TCP with the source address in the block: the
// data consumed must be returned, the rest left queued.
template <typename Recv>
void tcpRecvfromCase(const char* name, Recv rf) {
    int srv = -1, cli = -1, srv2 = -1, cli2 = -1;
    if (!tcpPair(&srv, &cli) || !tcpPair(&srv2, &cli2)) { FAIL("%s: socket setup", name); return; }
    if (write(cli, "AAAA", 4) != 4 || write(cli, "BBBB", 4) != 4 || write(cli2, "AAAA", 4) != 4) {
        FAIL("%s: send", name);
        return;
    }
    usleep(50 * 1000);
    // What the kernel reports for the same call into stack memory.
    char cb[8];
    sockaddr_storage cfrom{};
    socklen_t cfromlen = sizeof(sockaddr_in);
    const ssize_t cr = ::syscall(SYS_recvfrom, srv2, cb, 4, 0, &cfrom, &cfromlen);
    ARMED(name, false, [](unsigned char* p) { *reinterpret_cast<socklen_t*>(p) = sizeof(sockaddr_in); });
    char buf[8] = {};
    errno = 0;
    const ssize_t r = rf(srv, buf, 4, reinterpret_cast<sockaddr*>(b + 64), reinterpret_cast<socklen_t*>(b));
    const int e = errno;
    char next[8] = {};
    const ssize_t r2 = recv(srv, next, sizeof(next), MSG_DONTWAIT);
    if (r != 4 || std::memcmp(buf, "AAAA", 4) != 0)
        FAIL("%s: returned %zd (errno %d) '%.4s': the first chunk was lost", name, r, e, buf);
    else if (r2 != 4 || std::memcmp(next, "BBBB", 4) != 0)
        FAIL("%s: the next receive got %zd bytes, not BBBB", name, r2);
    else if (cr != 4 || *reinterpret_cast<socklen_t*>(b) != cfromlen)
        FAIL("%s: fromlen %u, the kernel reports %u", name, *reinterpret_cast<socklen_t*>(b), cfromlen);
    else
        PASS("%s (tcp): AAAA then BBBB, fromlen %u as the kernel reports", name, cfromlen);
    close(srv); close(cli); close(srv2); close(cli2);
    free(b);
}

// UDP with the data buffer on the stack and only the address in the block:
// that takes the plain path, not the peek.
template <typename Recv>
void udpAddrCase(const char* name, Recv rv) {
    int rcv = -1, snd = -1;
    if (!udpPair(&rcv, &snd) || send(snd, "one", 3, 0) != 3 || send(snd, "two", 3, 0) != 3) {
        FAIL("%s: socket setup", name);
        return;
    }
    usleep(50 * 1000);
    ARMED(name, false, [](unsigned char* p) { *reinterpret_cast<socklen_t*>(p) = sizeof(sockaddr_in); });
    char buf[8] = {};
    errno = 0;
    const ssize_t r = rv(rcv, buf, b);
    const int e = errno;
    auto* from = reinterpret_cast<sockaddr_in*>(b + 64);
    char next[8] = {};
    const ssize_t r2 = recv(rcv, next, sizeof(next), MSG_DONTWAIT);
    if (r != 3 || std::memcmp(buf, "one", 3) != 0)
        FAIL("%s: returned %zd (errno %d) '%.3s': datagram 'one' was lost", name, r, e, buf);
    else if (r2 != 3 || std::memcmp(next, "two", 3) != 0)
        FAIL("%s: the next receive got %zd bytes, not 'two'", name, r2);
    else if (*reinterpret_cast<socklen_t*>(b) != sizeof(sockaddr_in) || from->sin_port != localPort(snd))
        FAIL("%s: source address wrong (len %u, port %u)", name, *reinterpret_cast<socklen_t*>(b),
             ntohs(from->sin_port));
    else
        PASS("%s (udp): 'one' then 'two', source address right", name);
    close(rcv); close(snd);
    free(b);
}

char g_tcp_buf[8];
iovec g_tcp_iov{g_tcp_buf, 4};

// recvmsg with the msghdr itself in the block: the kernel reads it, consumes
// the data, then writes msg_flags.
void recvmsgHeaderCase() {
    int srv = -1, cli = -1;
    if (!tcpPair(&srv, &cli) || write(cli, "AAAA", 4) != 4 || write(cli, "BBBB", 4) != 4) {
        FAIL("recvmsg (msghdr in the block): socket setup");
        return;
    }
    usleep(50 * 1000);
    ARMED("recvmsg (msghdr in the block)", true, [](unsigned char* p) {
        auto* m = reinterpret_cast<msghdr*>(p);
        *m = msghdr{};
        m->msg_iov = &g_tcp_iov;
        m->msg_iovlen = 1;
        m->msg_flags = 0x5a5a;
    });
    auto* m = reinterpret_cast<msghdr*>(b);
    errno = 0;
    const ssize_t r = recvmsg(srv, m, MSG_DONTWAIT);
    const int e = errno;
    char next[8] = {};
    const ssize_t r2 = recv(srv, next, sizeof(next), MSG_DONTWAIT);
    if (r != 4 || std::memcmp(g_tcp_buf, "AAAA", 4) != 0)
        FAIL("recvmsg (msghdr in the block): returned %zd (errno %d): the first chunk was lost", r, e);
    else if (r2 != 4 || std::memcmp(next, "BBBB", 4) != 0)
        FAIL("recvmsg (msghdr in the block): the next receive got %zd bytes, not BBBB", r2);
    else if (m->msg_flags != 0 || m->msg_controllen != 0)
        FAIL("recvmsg (msghdr in the block): msg_flags %#x msg_controllen %zu, want 0 and 0",
             m->msg_flags, static_cast<size_t>(m->msg_controllen));
    else
        PASS("recvmsg (msghdr in the block): AAAA then BBBB, msg_flags written");
    close(srv); close(cli);
    free(b);
}

// SCM_RIGHTS over an AF_UNIX stream with the control buffer in the block.
void scmRightsCase() {
    int sp[2], pp[2];
    if (socketpair(AF_UNIX, SOCK_STREAM, 0, sp) != 0 || pipe(pp) != 0) { FAIL("scm_rights: setup"); return; }
    alignas(cmsghdr) char ctl[CMSG_SPACE(sizeof(int))] = {};
    iovec sv{const_cast<char*>("x"), 1};
    msghdr sm{};
    sm.msg_iov = &sv;
    sm.msg_iovlen = 1;
    sm.msg_control = ctl;
    sm.msg_controllen = sizeof(ctl);
    cmsghdr* c = CMSG_FIRSTHDR(&sm);
    c->cmsg_level = SOL_SOCKET;
    c->cmsg_type = SCM_RIGHTS;
    c->cmsg_len = CMSG_LEN(sizeof(int));
    std::memcpy(CMSG_DATA(c), &pp[0], sizeof(int));
    if (sendmsg(sp[1], &sm, 0) != 1) { FAIL("scm_rights: sendmsg"); return; }
    ARMED("scm_rights (stream, control in the block)", false, nop);
    char d[4];
    iovec rv{d, sizeof(d)};
    msghdr rm{};
    rm.msg_iov = &rv;
    rm.msg_iovlen = 1;
    rm.msg_control = b + 64;
    rm.msg_controllen = CMSG_SPACE(sizeof(int));
    const ssize_t r = recvmsg(sp[0], &rm, 0);
    cmsghdr* rc = (rm.msg_controllen >= sizeof(cmsghdr)) ? CMSG_FIRSTHDR(&rm) : nullptr;
    int passed = -1;
    if (rc && rc->cmsg_level == SOL_SOCKET && rc->cmsg_type == SCM_RIGHTS)
        std::memcpy(&passed, CMSG_DATA(rc), sizeof(int));
    struct stat a{}, p{};
    const bool same = passed >= 0 && fstat(passed, &a) == 0 && fstat(pp[0], &p) == 0 && a.st_ino == p.st_ino;
    if (r != 1)
        FAIL("scm_rights (stream, control in the block): returned %zd", r);
    else if ((rm.msg_flags & MSG_CTRUNC) || !same)
        FAIL("scm_rights (stream, control in the block): msg_flags %#x, passed fd %d: the descriptor was lost",
             rm.msg_flags, passed);
    else
        PASS("scm_rights (stream, control in the block): the descriptor arrived");
    if (passed >= 0) close(passed);
    close(sp[0]); close(sp[1]); close(pp[0]); close(pp[1]);
    free(b);
}

void sendfileCase() {
    const int f = static_cast<int>(::syscall(SYS_memfd_create, "sendfile_case", 0));
    int p[2];
    if (f < 0 || write(f, "ABCDEFGH", 8) != 8 || pipe(p) != 0 || fcntl(p[0], F_SETFL, O_NONBLOCK) != 0) {
        FAIL("sendfile: setup");
        return;
    }
    ARMED("sendfile", true, [](unsigned char* q) { *reinterpret_cast<off_t*>(q) = 0; });
    auto* off = reinterpret_cast<off_t*>(b);
    errno = 0;
    const ssize_t r = sendfile(p[1], f, off, 8);
    const int e = errno;
    char got[64] = {};
    const ssize_t n = read(p[0], got, sizeof(got));
    if (r != 8 || n != 8 || std::memcmp(got, "ABCDEFGH", 8) != 0)
        FAIL("sendfile: returned %zd (errno %d), the pipe holds %zd bytes '%.16s' (want ABCDEFGH once)",
             r, e, n, got);
    else if (*off != 8)
        FAIL("sendfile: offset %lld after the transfer, want 8", static_cast<long long>(*off));
    else
        PASS("sendfile: ABCDEFGH sent once, offset 8");
    close(f); close(p[0]); close(p[1]);
    free(b);
}

char g_dgram[] = "dgram";
iovec g_dgram_iov{g_dgram, 5};

void sendmmsgCase() {
    int rcv = -1, snd = -1;
    if (!udpPair(&rcv, &snd)) { FAIL("sendmmsg: socket setup"); return; }
    ARMED("sendmmsg", true, [](unsigned char* p) {
        auto* mm = reinterpret_cast<mmsghdr*>(p);
        *mm = mmsghdr{};
        mm->msg_hdr.msg_iov = &g_dgram_iov;
        mm->msg_hdr.msg_iovlen = 1;
    });
    auto* mm = reinterpret_cast<mmsghdr*>(b);
    errno = 0;
    const int r = sendmmsg(snd, mm, 1, 0);
    const int e = errno;
    usleep(50 * 1000);
    int got = 0;
    char d[16];
    while (recv(rcv, d, sizeof(d), MSG_DONTWAIT) == 5) ++got;
    if (r != 1 || got != 1)
        FAIL("sendmmsg: returned %d (errno %d), %d datagrams arrived (want 1)", r, e, got);
    else if (mm->msg_len != 5)
        FAIL("sendmmsg: msg_len %u, want 5", mm->msg_len);
    else
        PASS("sendmmsg: one datagram, msg_len 5");
    close(rcv); close(snd);
    free(b);
}

template <typename Mask>
void maskCase(const char* name, Mask fn) {
    sigset_t none, usr2;
    sigemptyset(&none);
    sigemptyset(&usr2);
    sigaddset(&usr2, SIGUSR2);
    sigprocmask(SIG_SETMASK, &none, nullptr);
    ARMED(name, false, nop);
    auto* old = reinterpret_cast<sigset_t*>(b);
    errno = 0;
    const int r = fn(SIG_BLOCK, &usr2, old);
    sigset_t now;
    sigprocmask(SIG_SETMASK, nullptr, &now);
    if (r != 0)
        FAIL("%s: returned %d (errno %d)", name, r, errno);
    else if (sigismember(old, SIGUSR2))
        FAIL("%s: reported the new mask as the old one", name);
    else if (!sigismember(&now, SIGUSR2))
        FAIL("%s: SIGUSR2 not blocked", name);
    else
        PASS("%s: SIGUSR2 blocked, the old mask reported", name);
    sigprocmask(SIG_SETMASK, &none, nullptr);
    free(b);
}

void prlimitCase() {
    using fn_t = int (*)(pid_t, __rlimit_resource, const rlimit64*, rlimit64*);
    auto fn = reinterpret_cast<fn_t>(dlsym(RTLD_DEFAULT, "prlimit64"));
    if (!inLibsmash(reinterpret_cast<void*>(fn))) { FAIL("prlimit64: dlsym does not resolve into libsmash"); return; }
    rlimit64 cur{};
    fn(0, RLIMIT_NOFILE, nullptr, &cur);
    rlimit64 lower = cur;
    lower.rlim_cur = cur.rlim_cur - 1;
    ARMED("prlimit64", false, nop);
    auto* old = reinterpret_cast<rlimit64*>(b);
    errno = 0;
    const int r = fn(0, RLIMIT_NOFILE, &lower, old);
    rlimit64 now{};
    fn(0, RLIMIT_NOFILE, nullptr, &now);
    if (r != 0)
        FAIL("prlimit64: returned %d (errno %d)", r, errno);
    else if (old->rlim_cur != cur.rlim_cur)
        FAIL("prlimit64: old limit %llu reported, want %llu (the new one is %llu)",
             static_cast<unsigned long long>(old->rlim_cur), static_cast<unsigned long long>(cur.rlim_cur),
             static_cast<unsigned long long>(lower.rlim_cur));
    else if (now.rlim_cur != lower.rlim_cur)
        FAIL("prlimit64: the new limit was not applied");
    else
        PASS("prlimit64: limit lowered, the old one reported");
    fn(0, RLIMIT_NOFILE, &cur, nullptr);
    free(b);
}

// A 2 s sleep interrupted after 200 ms, rem in the block: EINTR well before
// 2 s and the remaining time reported.
template <typename Sleep>
void sleepCase(const char* name, Sleep sl) {
    ARMED(name, false, [](unsigned char* p) { *reinterpret_cast<timespec*>(p) = timespec{}; });
    auto* rem = reinterpret_cast<timespec*>(b);
    pthread_t self = pthread_self(), t;
    timespec t0;
    clock_gettime(CLOCK_MONOTONIC, &t0);
    pthread_create(&t, nullptr, interrupter, &self);
    const timespec req{2, 0};
    const int err = sl(&req, rem);
    const double el = since(t0);
    pthread_join(t, nullptr);
    if (err != EINTR || el > 1.5)
        FAIL("%s: error %d after %.2f s (want EINTR after ~0.2 s): the interrupted sleep was slept again",
             name, err, el);
    else if (rem->tv_sec < 1 || rem->tv_sec >= 2)
        FAIL("%s: rem %ld.%09ld, want about 1.8 s", name, static_cast<long>(rem->tv_sec), rem->tv_nsec);
    else
        PASS("%s: EINTR after %.2f s, %ld.%03ld s remaining", name, el, static_cast<long>(rem->tv_sec),
             rem->tv_nsec / 1000000);
    free(b);
}

}  // namespace

int main() {
    g_ps = static_cast<size_t>(sysconf(_SC_PAGESIZE));
    g_zero = open("/dev/zero", O_RDONLY);
    if (g_zero < 0 || pipe(g_pipe) != 0) { std::printf("FAIL: setup\n"); return 1; }
    struct sigaction sa{};
    sa.sa_handler = onSignal;       // no SA_RESTART: an interrupted sleep returns EINTR
    sigaction(SIGUSR1, &sa, nullptr);

    rawPrecondition();

    waitCase("wait4", 42, [](pid_t pid, unsigned char* b) {
        return wait4(pid, reinterpret_cast<int*>(b), 0, reinterpret_cast<struct rusage*>(b + 64));
    });
    waitidCase();
    waitCase("waitpid", 44, [](pid_t pid, unsigned char* b) {
        return waitpid(pid, reinterpret_cast<int*>(b), 0);
    });
    waitCase("wait", 45, [](pid_t, unsigned char* b) { return wait(reinterpret_cast<int*>(b)); });
    waitCase("wait3", 46, [](pid_t, unsigned char* b) {
        return wait3(reinterpret_cast<int*>(b), 0, reinterpret_cast<struct rusage*>(b + 64));
    });

    acceptCase("accept", [](int l, sockaddr* a, socklen_t* al) { return accept(l, a, al); }, false);
    {
        using fn_t = int (*)(int, sockaddr*, socklen_t*, int);
        auto fn = reinterpret_cast<fn_t>(dlsym(RTLD_DEFAULT, "accept4"));
        if (!inLibsmash(reinterpret_cast<void*>(fn))) FAIL("accept4: dlsym does not resolve into libsmash");
        else acceptCase("accept4", [fn](int l, sockaddr* a, socklen_t* al) { return fn(l, a, al, SOCK_CLOEXEC); }, true);
    }

    tcpRecvfromCase("recvfrom", [](int s, char* buf, size_t n, sockaddr* f, socklen_t* fl) {
        return recvfrom(s, buf, n, 0, f, fl);
    });
    tcpRecvfromCase("__recvfrom_chk", [](int s, char* buf, size_t n, sockaddr* f, socklen_t* fl) {
        return __recvfrom_chk(s, buf, n, 8, 0, f, fl);
    });
    udpAddrCase("recvfrom", [](int s, char* buf, unsigned char* b) {
        return recvfrom(s, buf, 8, MSG_DONTWAIT, reinterpret_cast<sockaddr*>(b + 64),
                        reinterpret_cast<socklen_t*>(b));
    });
    udpAddrCase("recvmsg", [](int s, char* buf, unsigned char* b) {
        iovec iov{buf, 8};
        msghdr m{};
        m.msg_name = b + 64;
        m.msg_namelen = *reinterpret_cast<socklen_t*>(b);
        m.msg_iov = &iov;
        m.msg_iovlen = 1;
        const ssize_t r = recvmsg(s, &m, MSG_DONTWAIT);
        *reinterpret_cast<socklen_t*>(b) = m.msg_namelen;   // where the checks look for it
        return r;
    });
    recvmsgHeaderCase();
    scmRightsCase();

    sendfileCase();
    sendmmsgCase();

    maskCase("sigprocmask", [](int how, const sigset_t* s, sigset_t* o) { return sigprocmask(how, s, o); });
    maskCase("pthread_sigmask", [](int how, const sigset_t* s, sigset_t* o) { return pthread_sigmask(how, s, o); });
    prlimitCase();

    sleepCase("nanosleep", [](const timespec* req, timespec* rem) {
        return nanosleep(req, rem) == 0 ? 0 : errno;
    });
    sleepCase("clock_nanosleep", [](const timespec* req, timespec* rem) {
        return clock_nanosleep(CLOCK_MONOTONIC, 0, req, rem);
    });

    if (g_failures) { std::printf("syscall_side_effect_test: %d FAILURES\n", g_failures); return 1; }
    std::printf("syscall_side_effect_test: all checks passed\n");
    return 0;
}
