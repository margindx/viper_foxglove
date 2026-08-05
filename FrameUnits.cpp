//
// See FrameUnits.hpp.
//

#include "FrameUnits.hpp"

namespace mdx {

FrameUnits decodeFrameUnits(std::uint32_t positionBits, std::uint32_t orientationBits) {
    FrameUnits units;

    // Values mirror eViperPosUnits / eViperOriUnits. The fields are 2 bits
    // wide, so orientation has one encodable value with no enumerator.
    switch (positionBits) {
        case 0: units.position = PositionUnits::Inch; break;
        case 1: units.position = PositionUnits::Foot; break;
        case 2: units.position = PositionUnits::Centimeter; break;
        case 3: units.position = PositionUnits::Meter; break;
        default: units.position = PositionUnits::Unknown; break;
    }

    switch (orientationBits) {
        case 0: units.orientation = OrientationUnits::EulerDegree; break;
        case 1: units.orientation = OrientationUnits::EulerRadian; break;
        case 2: units.orientation = OrientationUnits::Quaternion; break;
        default: units.orientation = OrientationUnits::Unknown; break;
    }

    return units;
}

bool FrameUnits::isSupported() const {
    return position == PositionUnits::Meter && orientation == OrientationUnits::Quaternion;
}

std::string describe(PositionUnits units) {
    switch (units) {
        case PositionUnits::Inch: return "inches";
        case PositionUnits::Foot: return "feet";
        case PositionUnits::Centimeter: return "centimeters";
        case PositionUnits::Meter: return "meters";
        case PositionUnits::Unknown: break;
    }

    return "an unrecognized unit";
}

std::string describe(OrientationUnits units) {
    switch (units) {
        case OrientationUnits::EulerDegree: return "Euler degrees";
        case OrientationUnits::EulerRadian: return "Euler radians";
        case OrientationUnits::Quaternion: return "quaternion";
        case OrientationUnits::Unknown: break;
    }

    return "an unrecognized unit";
}

std::string describe(const FrameUnits &units) {
    return describe(units.position) + ", " + describe(units.orientation);
}

std::string unsupportedUnitsMessage(const FrameUnits &units) {
    std::string message = "Viper is reporting " + describe(units) +
                          "; this program requires meters and quaternion.";

    if (units.position != PositionUnits::Meter) {
        message += " Position units other than meters would rescale every "
                   "published position and the configured tip offset.";
    }

    if (units.orientation != OrientationUnits::Quaternion) {
        message += " In Euler mode the four orientation floats are "
                   "azimuth/elevation/roll plus an unused value, which would be "
                   "misread as a quaternion.";
    }

    message += " These are persistent device settings: set the SEU to meters and "
               "quaternion output (CMD_UNITS) and restart.";

    return message;
}

} // namespace mdx
