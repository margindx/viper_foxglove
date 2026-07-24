//
// USB serial-port discovery by Vendor/Product ID.
//
// Resolves the OS serial-port name for a device with a given USB VID/PID by
// reading device descriptors from the OS — WITHOUT opening any port — so it
// cannot reset or interfere with devices other applications are using. Only
// Windows is implemented today (SetupAPI); other platforms return std::nullopt
// so callers fall back to an explicitly-configured port. See issue #52.
//

#ifndef VIPER_USBSERIALPORT_HPP
#define VIPER_USBSERIALPORT_HPP

#include <cstdint>
#include <optional>
#include <string>

#ifdef _WIN32
#include <windows.h>
#include <setupapi.h>
#include <cctype>
#include <cstdio>
#include <vector>
// MSVC picks up the import library here; MinGW/other toolchains link it via
// CMake (target_link_libraries(... setupapi)).
#pragma comment(lib, "setupapi.lib")
#endif

namespace mdx {
namespace usb {

#ifdef _WIN32
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

// Returns the serial-port name (e.g. "COM5") of the single present device whose
// USB hardware ID contains the given VID/PID. Returns std::nullopt if zero or
// MORE THAN ONE device matches (fail-safe: never guess when ambiguous), or if
// the platform has no backend. Never opens a port.
inline std::optional<std::string> findPortByUsbId(std::uint16_t vid, std::uint16_t pid) {
#ifdef _WIN32
    // GUID_DEVCLASS_PORTS, hardcoded to avoid a devguid.h / uuid.lib dependency.
    static const GUID kGuidDevClassPorts = {
        0x4d36e978, 0xe325, 0x11ce,
        {0xbf, 0xc1, 0x08, 0x00, 0x2b, 0xe1, 0x03, 0x18}};

    // The fragment we expect in the hardware ID, e.g. "VID_2886&PID_8064".
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
        // Hardware IDs are a REG_MULTI_SZ; the first (and here, sufficient)
        // entry carries the USB VID/PID.
        char hardwareId[512] = {0};
        if (!SetupDiGetDeviceRegistryPropertyA(
                devInfo, &devData, SPDRP_HARDWAREID, nullptr,
                reinterpret_cast<PBYTE>(hardwareId), sizeof(hardwareId) - 1, nullptr)) {
            continue;
        }
        if (!detail::containsIgnoreCase(hardwareId, idFragment)) {
            continue;
        }

        // The assigned port name lives in the device's Device Parameters key.
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

    if (matches.size() == 1) {
        return matches.front();
    }
    return std::nullopt;  // 0 or >1 matches -> abstain rather than guess
#else
    (void)vid;
    (void)pid;
    return std::nullopt;  // no backend yet on this platform (macOS/Linux: TODO)
#endif
}

}  // namespace usb
}  // namespace mdx

#endif  // VIPER_USBSERIALPORT_HPP
