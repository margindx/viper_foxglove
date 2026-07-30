//
// See DeviceState.hpp.
//

#include "DeviceState.hpp"

#include <cmath>
#include <sstream>

namespace mdx {

namespace {

bool near(double value, double target, double tolerance) {
    return std::abs(value - target) <= tolerance;
}

std::string joinParams(const double (&params)[4]) {
    std::ostringstream ss;
    ss << "[" << params[0] << ", " << params[1] << ", " << params[2] << ", " << params[3] << "]";
    return ss.str();
}

} // namespace

bool DeviceRotation::isNeutral(double tolerance) const {
    for (const double value : params) {
        if (!std::isfinite(value))
            return false;
    }

    const bool allZero = near(params[0], 0.0, tolerance) && near(params[1], 0.0, tolerance) &&
                         near(params[2], 0.0, tolerance) && near(params[3], 0.0, tolerance);

    // An identity quaternion in either component order. The SDK does not say
    // which it uses, so both are accepted as "no rotation".
    const bool identityLeading = near(std::abs(params[0]), 1.0, tolerance) &&
                                 near(params[1], 0.0, tolerance) &&
                                 near(params[2], 0.0, tolerance) &&
                                 near(params[3], 0.0, tolerance);

    const bool identityTrailing = near(params[0], 0.0, tolerance) &&
                                  near(params[1], 0.0, tolerance) &&
                                  near(params[2], 0.0, tolerance) &&
                                  near(std::abs(params[3]), 1.0, tolerance);

    return allZero || identityLeading || identityTrailing;
}

std::string DeviceRotation::describe() const {
    return joinParams(params);
}

bool isDefaultSensorOrigin(std::uint32_t mode) {
    return mode == 0;   // SNS_ORIG_SRC1
}

std::string sensorOriginLabel(std::uint32_t mode) {
    switch (mode) {
        case 0: return "source 1 (default)";
        case 1: return "source 2";
        case 2: return "source 3";
        case 3: return "source 4";
        case 4: return "common";
        default: break;
    }

    return "unrecognised origin " + std::to_string(mode);
}

std::string frameRateLabel(std::uint32_t code) {
    switch (code) {
        case 0: return "30 Hz";
        case 1: return "60 Hz";
        case 2: return "120 Hz";
        case 3: return "240 Hz";
        case 4: return "480 Hz";
        case 5: return "960 Hz";
        default: break;
    }

    return "unrecognised frame rate code " + std::to_string(code);
}

std::string boresightMessage(int sensor, const DeviceRotation &rotation) {
    std::ostringstream ss;
    ss << "The Viper has a boresight of " << rotation.describe() << " set on sensor " << sensor
       << ". Every orientation it reports is therefore already rotated, and the probe_profiles "
          "tip offset is applied along that rotated frame -- so the tip lands somewhere that "
          "matches none of the sensor's physical axes. This is the rotational twin of a stale "
          "tip offset. Clear it on the device (CMD_BORESIGHT reset) so the SEU reports the "
          "sensor's own orientation.";

    return ss.str();
}

std::string sourceRotationMessage(int source, const DeviceRotation &rotation) {
    std::ostringstream ss;
    ss << "The Viper has a source rotation of " << rotation.describe() << " set on source "
       << source
       << ". That rotates the whole tracker frame, so positions and orientations are reported in "
          "an axis system other than the source's own and every downstream frame -- /tf/viper, "
          "the point clouds, the tip pose -- means something different from what it says. Clear "
          "it (CMD_SRC_ROTATION reset).";

    return ss.str();
}

std::string sensorOriginMessage(int sensor, std::uint32_t mode) {
    std::ostringstream ss;
    ss << "Sensor " << sensor << " is configured to report about " << sensorOriginLabel(mode)
       << " rather than its default origin. Positions from it are then relative to a different "
          "reference than the other sensors and than the published frames assume. Clear it "
          "(CMD_SNS_ORIGIN reset).";

    return ss.str();
}

std::string describeFilter(const FilterSettings &settings) {
    std::ostringstream ss;
    ss << "level " << settings.level << ", params " << joinParams(settings.params);

    if (settings.level == 0)
        ss << " (off)";

    return ss.str();
}

std::string describePredictiveFilter(const PredictiveFilterSettings &settings) {
    if (!settings.quaternion && !settings.position)
        return "off";

    std::ostringstream ss;
    ss << "on for ";
    if (settings.quaternion && settings.position)
        ss << "orientation and position";
    else if (settings.quaternion)
        ss << "orientation";
    else
        ss << "position";

    ss << ", predicting " << settings.predictionSeconds * 1000.0 << " ms ahead";

    return ss.str();
}

std::string describeIncrement(const IncrementSettings &settings) {
    if (!settings.enabled)
        return "off";

    std::ostringstream ss;
    ss << "ON -- the device reports only after movement exceeding " << settings.positionThreshold
       << " (position) or " << settings.orientationThreshold
       << " (orientation), so the stream is deliberately irregular rather than dropping frames";

    return ss.str();
}

std::string distortionMessage(std::uint32_t level, int sensor, std::uint64_t framesAffected) {
    std::ostringstream ss;
    ss << "Viper reports EM distortion of " << level << "/255 on sensor " << sensor << " ("
       << framesAffected << " frame(s) above " << kDistortionWarnLevel
       << " so far). Positions and orientations are degraded while this persists -- move away "
          "from metal, motors and displays, or expect centimeter-scale error.";

    return ss.str();
}

} // namespace mdx
