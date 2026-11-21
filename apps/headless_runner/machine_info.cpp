#include "machine_info.hpp"

#include <sys/utsname.h>

#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <thread>

#ifdef __APPLE__
#include <sys/sysctl.h>
#endif

namespace {

#ifdef __APPLE__
// Reads a string sysctl, or "unknown".
std::string sysctl_string(const char* name) {
    size_t len = 0;
    if (sysctlbyname(name, nullptr, &len, nullptr, 0) != 0 || len == 0) return "unknown";
    std::string s(len, '\0');
    if (sysctlbyname(name, s.data(), &len, nullptr, 0) != 0) return "unknown";
    s.resize(s.find('\0') == std::string::npos ? len : s.find('\0'));
    return s;
}

// Reads an integer sysctl (32- or 64-bit), or "unknown".
std::string sysctl_int(const char* name) {
    std::int64_t v = 0;
    size_t len = sizeof(v);
    if (sysctlbyname(name, &v, &len, nullptr, 0) != 0) return "unknown";
    if (len == sizeof(std::int32_t)) {
        std::int32_t v32 = 0;
        len = sizeof(v32);
        if (sysctlbyname(name, &v32, &len, nullptr, 0) != 0) return "unknown";
        return std::to_string(v32);
    }
    return std::to_string(v);
}
#endif

}  // namespace

std::vector<std::pair<std::string, std::string>> collect_machine_info() {
    const char* label = std::getenv("HP_MACHINE_LABEL");
    std::string os = "unknown";
    std::string arch = "unknown";
    utsname u{};
    if (uname(&u) == 0) {
        os = std::string(u.sysname) + " " + u.release;
        arch = u.machine;
    }
    std::string chip = "unknown", cores = "unknown", perf = "unknown", eff = "unknown",
                mem_gb = "unknown", os_build = "unknown";
#ifdef __APPLE__
    // The chip name and core split are only available through sysctl on macOS.
    chip = sysctl_string("machdep.cpu.brand_string");
    cores = sysctl_int("hw.physicalcpu");
    perf = sysctl_int("hw.perflevel0.physicalcpu");
    eff = sysctl_int("hw.perflevel1.physicalcpu");
    const std::string mem = sysctl_int("hw.memsize");
    if (mem != "unknown") mem_gb = std::to_string(std::stoll(mem) / (1024LL * 1024 * 1024));
    const std::string product = sysctl_string("kern.osproductversion");
    if (product != "unknown") os = "macOS " + product;
    os_build = sysctl_string("kern.osversion");
#endif
    return {
        {"label", label != nullptr ? label : "unlabeled"},
        {"chip", chip},
        {"arch", arch},
        {"cores_total", cores},
        {"cores_performance", perf},
        {"cores_efficiency", eff},
        {"logical_cores", std::to_string(std::thread::hardware_concurrency())},
        {"memory_gb", mem_gb},
        {"os", os},
        {"os_build", os_build},
        {"compiler", __VERSION__},
    };
}

std::string utc_timestamp() {
    const std::time_t now = std::time(nullptr);
    std::tm tm{};
    gmtime_r(&now, &tm);
    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &tm);
    return buf;
}
