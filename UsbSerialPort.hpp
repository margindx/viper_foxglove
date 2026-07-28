//
// USB serial-port discovery by Vendor/Product ID.
//
// Resolves the OS serial-port name for a device with a given USB VID/PID by
// reading device descriptors from the OS — WITHOUT opening any port — so it
// cannot reset or interfere with devices other applications are using. Backends
// exist for Windows (SetupAPI) and Linux (sysfs); any other platform (including
// macOS) returns std::nullopt. See issue #54.
//

#ifndef VIPER_USBSERIALPORT_HPP
#define VIPER_USBSERIALPORT_HPP

#include <cstdint>
#include <optional>
#include <string>

#if defined(_WIN32)
#include <windows.h>
#include <setupapi.h>
#include <cctype>
#include <cstdio>
#include <vector>
// MSVC picks up the import library here; MinGW/other toolchains link it via
// CMake (target_link_libraries(... setupapi)).
#pragma comment(lib, "setupapi.lib")
#elif defined(__linux__)
#include <cstdio>
#include <fstream>
#include <filesystem>
#include <system_error>
#include <vector>
#endif

namespace mdx {
namespace usb {

#if defined(_WIN32)
namespace detail {
    // Case-insensitive substring test (USB hardware IDs are ASCII).
    inline bool containsIgnoreCase(const char *haystack, const char *needle) {
        for (const char *h = haystack; *h; ++h) {
            const char *a = h;
            const char *b = needle;
            while (*a && *b &&
                   std::toupper((unsigned char)*a) == std::toupper((unsigned char)*b)) {
                ++a;
                ++b;
            }
            if (!*b) return true;   // reached end of needle -> matched
        }
        return false;
    }
}  // namespace detail
#endif

// Returns the serial-port name (e.g. "COM5", "/dev/ttyACM0", "/dev/cu.usbmodemXXXX")
// of the single present device whose USB descriptors report the given VID/PID.
// Returns std::nullopt if zero or MORE THAN ONE device matches (fail-safe: never
// guess when ambiguous), or if the platform has no backend. Never opens a port.
inline std::optional<std::string> findPortByUsbId(std::uint16_t vid, std::uint16_t pid) {
#if defined(_WIN32)
    // ---- Windows: SetupAPI ----
    // GUID_DEVCLASS_PORTS, hardcoded to avoid a devguid.h / uuid.lib dependency.
    static const GUID kGuidDevClassPorts = {
        0x4d36e978, 0xe325, 0x11ce,
        {0xbf, 0xc1, 0x08, 0x00, 0x2b, 0xe1, 0x03, 0x18}};

    char idFragment[32];
    std::snprintf(idFragment, sizeof(idFragment), "VID_%04X&PID_%04X", vid, pid);

    HDEVINFO devInfo =
        SetupDiGetClassDevsA(&kGuidDevClassPorts, nullptr, nullptr, DIGCF_PRESENT);
    if (devInfo == INVALID_HANDLE_VALUE) {
        return std::nullopt;
    }

    std::vector<std::string> matches;
    SP_DEVINFO_DATA devData;
    devData.cbSize = sizeof(devData);

    for (DWORD i = 0; SetupDiEnumDeviceInfo(devInfo, i, &devData); ++i) {
        char hardwareId[512] = {0};
        if (!SetupDiGetDeviceRegistryPropertyA(
                devInfo, &devData, SPDRP_HARDWAREID, nullptr,
                reinterpret_cast<PBYTE>(hardwareId), sizeof(hardwareId) - 1, nullptr)) {
            continue;
        }
        if (!detail::containsIgnoreCase(hardwareId, idFragment)) {
            continue;
        }
        HKEY key = SetupDiOpenDevRegKey(devInfo, &devData, DICS_FLAG_GLOBAL, 0,
                                        DIREG_DEV, KEY_READ);
        if (key == INVALID_HANDLE_VALUE) {
            continue;
        }
        char portName[16] = {0};
        DWORD type = 0;
        DWORD size = sizeof(portName) - 1;
        LONG status = RegQueryValueExA(key, "PortName", nullptr, &type,
                                       reinterpret_cast<LPBYTE>(portName), &size);
        RegCloseKey(key);
        if (status == ERROR_SUCCESS && type == REG_SZ && portName[0] != '\0') {
            matches.emplace_back(portName);
        }
    }
    SetupDiDestroyDeviceInfoList(devInfo);

    if (matches.size() == 1) return matches.front();
    return std::nullopt;

#elif defined(__linux__)
    // ---- Linux: sysfs ----
    namespace fs = std::filesystem;

    char buf[8];
    std::snprintf(buf, sizeof(buf), "%04x", vid);   // sysfs idVendor is lowercase 4-hex
    const std::string wantVid = buf;
    std::snprintf(buf, sizeof(buf), "%04x", pid);
    const std::string wantPid = buf;

    auto readId = [](const fs::path &f) -> std::string {
        std::ifstream in(f);
        std::string s;
        std::getline(in, s);
        while (!s.empty() && (s.back() == '\n' || s.back() == '\r' ||
                              s.back() == ' ' || s.back() == '\t')) {
            s.pop_back();
        }
        return s;
    };

    std::vector<std::string> matches;
    std::error_code ec;
    for (const auto &entry : fs::directory_iterator("/sys/class/tty", ec)) {
        const std::string name = entry.path().filename().string();   // e.g. ttyACM0
        std::error_code lec;
        const fs::path dev = fs::canonical(entry.path() / "device", lec);
        if (lec) continue;   // no backing device (e.g. ttyS*, virtual ttys)

        // Walk up to the USB device node that carries idVendor/idProduct.
        std::string gotVid, gotPid;
        for (fs::path p = dev; !p.empty() && p != p.root_path(); p = p.parent_path()) {
            std::error_code e2;
            if (fs::exists(p / "idVendor", e2) && fs::exists(p / "idProduct", e2)) {
                gotVid = readId(p / "idVendor");
                gotPid = readId(p / "idProduct");
                break;
            }
        }
        if (gotVid == wantVid && gotPid == wantPid) {
            matches.push_back("/dev/" + name);
        }
    }

    if (matches.size() == 1) return matches.front();
    return std::nullopt;

#else
    (void)vid;
    (void)pid;
    return std::nullopt;   // no backend on this platform (incl. macOS)
#endif
}

}  // namespace usb
}  // namespace mdx

#endif  // VIPER_USBSERIALPORT_HPP
