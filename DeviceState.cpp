//
// See DeviceState.hpp.
//

#include "DeviceState.hpp"

#include <cmath>
#include <iomanip>
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

std::string sensorOriginLabel(std::uint32_t mode) {
    switch (mode) {
        case 0: return "source 1 (default)";
        case 1: return "source 2";
        case 2: return "source 3";
        case 3: return "source 4";
        case 4: return "common";
        default: break;
    }

    return "unrecognized origin " + std::to_string(mode);
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

    return "unrecognized frame rate code " + std::to_string(code);
}

std::optional<int> frameRateHzFromCode(std::uint32_t code) {
    switch (code) {
        case 0: return 30;
        case 1: return 60;
        case 2: return 120;
        case 3: return 240;
        case 4: return 480;
        case 5: return 960;
        default: break;
    }
    return std::nullopt;
}

bool isSupportedFrameRateHz(int hz) {
    return hz == 30 || hz == 60 || hz == 120 || hz == 240 || hz == 480 || hz == 960;
}

std::string supportedFrameRateList() {
    return "30, 60, 120, 240, 480 or 960";
}

bool deliveredRateOutOfBand(int expectedHz, double deliveredHz) {
    if (expectedHz <= 0)
        return true;
    const double ratio = deliveredHz / static_cast<double>(expectedHz);
    return std::abs(ratio - 1.0) > kDeliveredRateTolerance;
}

std::string frameRateMismatchMessage(int expectedHz, std::uint32_t reportedCode) {
    std::ostringstream ss;
    ss << "The SEU is running at " << frameRateLabel(reportedCode) << " but the config expects "
       << expectedHz << " Hz. Sample density, latency and the meaning of every recorded "
          "timestamp all follow from the rate, so a recording made at one rate is not "
          "comparable with one made at another. Either set the SEU to " << expectedHz
       << " Hz or update \"expected_frame_rate_hz\" to match the rate you intend to use.";
    return ss.str();
}

std::string frameRateUnreadableMessage(int expectedHz) {
    std::ostringstream ss;
    ss << "The SEU did not report its frame rate, so the configured expectation of " << expectedHz
       << " Hz could not be confirmed. Failing to read a setting is not evidence that it is "
          "correct, and this is the one setting the delivered-rate check needs to compare "
          "against.";
    return ss.str();
}

std::string deliveredFrameRateMessage(int expectedHz, double deliveredHz, std::uint64_t frames,
                                      double seconds) {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(1);
    ss << "Frames are arriving at " << deliveredHz << " Hz, but the config expects " << expectedHz
       << " Hz (" << frames << " frames over " << std::setprecision(2) << seconds << " s).";

    if (deliveredHz < expectedHz) {
        ss << " The SEU reports the configured rate, so the shortfall is between it and this "
              "program: frames are being dropped, or the link cannot carry the rate. Check the "
              "dropped-frame counts in the log, and try a lower rate to confirm.";
    } else {
        ss << " That is faster than configured, which should not happen and suggests the rate "
              "reported by the SEU is not the rate it is running at.";
    }

    return ss.str();
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

double DistortionSummary::mean() const {
    return frames > 0 ? static_cast<double>(sum) / static_cast<double>(frames) : 0.0;
}

bool DistortionSummary::exceededThreshold() const {
    return peak >= kDistortionWarnLevel;
}

std::string describeDistortionBrief(const DistortionSummary &summary) {
    std::ostringstream ss;
    ss << "dist " << summary.current << " (peak " << summary.peak << ")";
    return ss.str();
}

std::string describeDistortion(const DistortionSummary &summary) {
    if (summary.frames == 0)
        return "no frames seen";

    std::ostringstream ss;
    ss << "peak " << summary.peak << "/255, mean " << std::fixed << std::setprecision(1)
       << summary.mean() << " over " << summary.frames << " frames";

    if (summary.exceededThreshold())
        ss << "  <-- above " << kDistortionWarnLevel;

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
