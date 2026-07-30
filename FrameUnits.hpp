//
// Units reported in the Viper PNO frame.
//
// Every SENFRAMEDATA carries the position and orientation units the SEU is
// currently configured for (SFINFO.bfPosUnits / bfOriUnits). Those settings are
// persistent on the device (CFG_PERSIST), and the factory defaults are inches
// and Euler degrees -- neither of which is what the rest of this program
// assumes. Nothing here sends CMD_UNITS, so the only safe move is to read what
// the device reports and refuse to run on anything we would misinterpret:
//
//   * positions are treated as meters throughout (probe_profiles tip_offset_m,
//     the Foxglove frames, the point clouds). A device left on inches would
//     rescale everything by 39.37 with no visible symptom.
//   * orientations are read as a quaternion (w, x, y, z). In either Euler mode
//     ori[3] is unused and the first three values are azimuth/elevation/roll,
//     so the same four floats would silently become a meaningless rotation.
//
// Takes raw bitfield values rather than a SENFRAMEDATA so it stays free of the
// Viper SDK headers and can be unit-tested without hardware.
//

#ifndef VIPER_FRAMEUNITS_HPP
#define VIPER_FRAMEUNITS_HPP

#include <cstdint>
#include <string>

namespace mdx {

/// Mirrors eViperPosUnits, plus a catch-all for values outside the enum.
enum class PositionUnits { Inch, Foot, Centimeter, Meter, Unknown };

/// Mirrors eViperOriUnits, plus a catch-all for values outside the enum.
enum class OrientationUnits { EulerDegree, EulerRadian, Quaternion, Unknown };

struct FrameUnits {
    PositionUnits position{PositionUnits::Unknown};
    OrientationUnits orientation{OrientationUnits::Unknown};

    /// True only for the combination this program can interpret: meters and
    /// quaternions.
    bool isSupported() const;
};

/// Decode the two SFINFO bitfields. Values outside the known enums map to
/// Unknown rather than being guessed at.
FrameUnits decodeFrameUnits(std::uint32_t positionBits, std::uint32_t orientationBits);

std::string describe(PositionUnits units);
std::string describe(OrientationUnits units);

/// Human-readable summary, e.g. "meters, quaternion".
std::string describe(const FrameUnits &units);

/// Actionable message naming what was reported, what is required, and how to
/// fix it. Only meaningful when isSupported() is false.
std::string unsupportedUnitsMessage(const FrameUnits &units);

} // namespace mdx

#endif //VIPER_FRAMEUNITS_HPP
