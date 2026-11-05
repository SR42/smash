// symbol_version_test.cpp - every function libsmash replaces must be exported
// at the version callers bind, or the replacement never runs.
//
// A caller binds a versioned symbol: the default version the system library
// defines when the caller was linked (getrandom@GLIBC_2.25, accept4@GLIBC_2.10,
// ...). ld.so binds that reference only to a definition of the same version,
// so a wrapper libsmash exports at the base version alone is skipped and the
// system's function runs instead -- for a syscall wrapper, without the EFAULT
// handling it exists for. That was fixed one symbol at a time (recvmmsg,
// sendmmsg, aligned_alloc, the __*_chk family, clock_nanosleep, ...) and never
// swept: 20 functions were still exported at the wrong version (17 syscall
// wrappers, reallocarray, two aligned nothrow operator deletes).
//
// So this test sweeps. In-process, from the dynamic symbol and version tables
// of every loaded object: for each function libsmash defines that another
// loaded library (libc, libstdc++, libm, ...) also defines with a default
// version, libsmash must define it at that version too. Then the practical
// half: functions this binary references at a version above the base must
// resolve into libsmash (dladdr), and so must the same names looked up with
// dlvsym at the version libc defines them.
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <elf.h>
#include <link.h>
#include <map>
#include <poll.h>
#include <set>
#include <spawn.h>
#include <string>
#include <sys/epoll.h>
#include <sys/random.h>
#include <sys/resource.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/xattr.h>
#include <unistd.h>
#include <vector>

namespace {

int g_failures = 0;
#define FAIL(...) do { std::printf("FAIL: " __VA_ARGS__); std::printf("\n"); ++g_failures; } while (0)

struct Version {
    bool dflt;       // the default version (name@@ver), not a hidden one (name@ver)
    bool exported;   // global or weak; a local entry is a definition the
                     // version script hid
};
struct Object {
    std::string path;
    std::map<std::string, std::map<std::string, Version>> funcs;   // name -> version -> flags
};

// A d_ptr in a loaded object's dynamic section: ld.so relocates most of them
// in place, but not all and not on every architecture.
template <typename T>
const T* dynPtr(const dl_phdr_info* info, ElfW(Addr) p) {
    return reinterpret_cast<const T*>(p >= info->dlpi_addr ? p : p + info->dlpi_addr);
}

size_t symbolCount(const dl_phdr_info* info, const ElfW(Word)* hash, const uint32_t* gnu) {
    if (hash) return hash[1];
    if (!gnu) return 0;
    const uint32_t nbuckets = gnu[0], symoffset = gnu[1], bloom_size = gnu[2];
    const auto* bloom = reinterpret_cast<const ElfW(Addr)*>(gnu + 4);
    const auto* buckets = reinterpret_cast<const uint32_t*>(bloom + bloom_size);
    const uint32_t* chain = buckets + nbuckets;
    uint32_t last = 0;
    for (uint32_t b = 0; b < nbuckets; ++b) if (buckets[b] > last) last = buckets[b];
    if (last < symoffset) return symoffset;
    while (!(chain[last - symoffset] & 1)) ++last;
    (void)info;
    return last + 1;
}

int collect(dl_phdr_info* info, size_t, void* out) {
    auto* objs = static_cast<std::vector<Object>*>(out);
    const ElfW(Dyn)* dyn = nullptr;
    for (int i = 0; i < info->dlpi_phnum; ++i)
        if (info->dlpi_phdr[i].p_type == PT_DYNAMIC)
            dyn = reinterpret_cast<const ElfW(Dyn)*>(info->dlpi_addr + info->dlpi_phdr[i].p_vaddr);
    if (!dyn) return 0;
    const ElfW(Sym)* symtab = nullptr;
    const char* strtab = nullptr;
    const ElfW(Half)* versym = nullptr;
    const ElfW(Verdef)* verdef = nullptr;
    const ElfW(Word)* hash = nullptr;
    const uint32_t* gnu = nullptr;
    for (const ElfW(Dyn)* d = dyn; d->d_tag != DT_NULL; ++d) {
        switch (d->d_tag) {
            case DT_SYMTAB: symtab = dynPtr<ElfW(Sym)>(info, d->d_un.d_ptr); break;
            case DT_STRTAB: strtab = dynPtr<char>(info, d->d_un.d_ptr); break;
            case DT_VERSYM: versym = dynPtr<ElfW(Half)>(info, d->d_un.d_ptr); break;
            case DT_VERDEF: verdef = dynPtr<ElfW(Verdef)>(info, d->d_un.d_ptr); break;
            case DT_HASH: hash = dynPtr<ElfW(Word)>(info, d->d_un.d_ptr); break;
            case DT_GNU_HASH: gnu = dynPtr<uint32_t>(info, d->d_un.d_ptr); break;
        }
    }
    if (!symtab || !strtab || !versym || !verdef) return 0;   // no versioned exports
    std::map<unsigned, std::string> vernames;
    for (const ElfW(Verdef)* v = verdef;; v = reinterpret_cast<const ElfW(Verdef)*>(
                                              reinterpret_cast<const char*>(v) + v->vd_next)) {
        const auto* aux = reinterpret_cast<const ElfW(Verdaux)*>(reinterpret_cast<const char*>(v) + v->vd_aux);
        if (!(v->vd_flags & VER_FLG_BASE)) vernames[v->vd_ndx] = strtab + aux->vda_name;
        if (!v->vd_next) break;
    }
    Object o;
    o.path = info->dlpi_name && *info->dlpi_name ? info->dlpi_name : "(main)";
    const size_t n = symbolCount(info, hash, gnu);
    for (size_t i = 1; i < n; ++i) {
        const ElfW(Sym)& s = symtab[i];
        const unsigned type = ELF64_ST_TYPE(s.st_info), bind = ELF64_ST_BIND(s.st_info);
        if (s.st_shndx == SHN_UNDEF || (type != STT_FUNC && type != STT_GNU_IFUNC)) continue;
        const auto it = vernames.find(versym[i] & 0x7fff);
        if (it == vernames.end()) continue;
        o.funcs[strtab + s.st_name][it->second] =
            Version{!(versym[i] & 0x8000), bind == STB_GLOBAL || bind == STB_WEAK};
    }
    objs->push_back(std::move(o));
    return 0;
}

bool inLibsmash(const void* fn) {
    Dl_info info{};
    return fn && dladdr(fn, &info) && info.dli_fname && std::strstr(info.dli_fname, "libsmash");
}

}  // namespace

#if defined(__x86_64__)
// Not declared by current headers; exported by x86-64 glibc at GLIBC_2.4.
extern "C" int __fxstatat(int, int, const char*, struct stat*, int);
#endif

int main() {
    std::vector<Object> objs;
    dl_iterate_phdr(collect, &objs);
    const Object* smash = nullptr;
    for (const Object& o : objs)
        if (o.path.find("libsmash") != std::string::npos) smash = &o;
    if (!smash) { std::printf("FAIL: libsmash is not loaded (run under LD_PRELOAD)\n"); return 1; }

    // The sweep: every function libsmash defines -- exported, or hidden by
    // the version script (alloc8's aligned nothrow operator deletes were
    // defined and never exported) -- that a system library exports at a
    // default version must be exported by libsmash at that version. The vDSO
    // is not a library programs link against (glibc calls it internally).
    int compared = 0;
    std::set<std::string> libs;
    for (const auto& [name, vers] : smash->funcs) {
        for (const Object& o : objs) {
            if (&o == smash || o.path == "(main)" || o.path.find("vdso") != std::string::npos) continue;
            const auto it = o.funcs.find(name);
            if (it == o.funcs.end()) continue;
            for (const auto& [ver, flags] : it->second) {
                if (!flags.dflt || !flags.exported) continue;
                ++compared;
                libs.insert(o.path.substr(o.path.rfind('/') + 1));
                const auto mine = vers.find(ver);
                if (mine == vers.end() || !mine->second.exported) {
                    std::string have;
                    for (const auto& [v, f] : vers)
                        if (f.exported) have += (have.empty() ? "" : ", ") + v;
                    FAIL("%s: callers bind %s@%s (%s), libsmash exports it at %s", name.c_str(), name.c_str(),
                         ver.c_str(), o.path.c_str(), have.empty() ? "no version (defined, hidden)" : have.c_str());
                }
            }
        }
    }
    std::string libnames;
    for (const auto& l : libs) libnames += (libnames.empty() ? "" : ", ") + l;
    std::printf("sweep: %d functions libsmash replaces in %s, compared at their default versions\n",
                compared, libnames.c_str());
    if (compared < 100) FAIL("only %d functions compared: the symbol tables were not read", compared);

    // What this binary binds, at versions above x86-64's base.
    struct Bound { const char* name; const void* fn; };
    const Bound bound[] = {
        {"getrandom", reinterpret_cast<const void*>(&getrandom)},
        {"accept4", reinterpret_cast<const void*>(&accept4)},
        {"ppoll", reinterpret_cast<const void*>(&ppoll)},
        {"epoll_pwait", reinterpret_cast<const void*>(&epoll_pwait)},
        {"prlimit64", reinterpret_cast<const void*>(&prlimit64)},
        {"readlinkat", reinterpret_cast<const void*>(&readlinkat)},
        {"fgetxattr", reinterpret_cast<const void*>(&fgetxattr)},
        {"posix_spawn", reinterpret_cast<const void*>(&posix_spawn)},
        {"posix_spawnp", reinterpret_cast<const void*>(&posix_spawnp)},
        {"execvpe", reinterpret_cast<const void*>(&execvpe)},
        {"reallocarray", reinterpret_cast<const void*>(&reallocarray)},
#if defined(__x86_64__)
        {"__fxstatat", reinterpret_cast<const void*>(&__fxstatat)},
#endif
    };
    int bound_ok = 0;
    for (const Bound& b : bound) {
        if (!inLibsmash(b.fn)) FAIL("%s as this binary binds it resolves outside libsmash", b.name);
        else ++bound_ok;
    }
    std::printf("bound: %d of %zu references above the base version resolve into libsmash\n", bound_ok,
                sizeof(bound) / sizeof(bound[0]));

    // And looked up at libc's own default version.
    int looked = 0;
    for (const Object& o : objs) {
        if (o.path.find("libc.so") == std::string::npos) continue;
        for (const Bound& b : bound) {
            const auto it = o.funcs.find(b.name);
            if (it == o.funcs.end()) continue;
            for (const auto& [ver, flags] : it->second) {
                if (!flags.dflt || !flags.exported) continue;
                void* f = dlvsym(RTLD_DEFAULT, b.name, ver.c_str());
                if (!inLibsmash(f)) FAIL("dlvsym(%s, %s) resolves outside libsmash", b.name, ver.c_str());
                else ++looked;
            }
        }
    }
    std::printf("dlvsym: %d lookups at libc's default version resolve into libsmash\n", looked);

    if (g_failures) { std::printf("symbol_version_test: %d FAILURES\n", g_failures); return 1; }
    std::printf("symbol_version_test: all checks passed\n");
    return 0;
}
