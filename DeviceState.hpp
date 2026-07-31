//
// Interpretation of the Viper's own configuration.
//
// The SEU carries persistent per-sensor and per-SEU settings that silently
// change what the reported poses mean. We found one the hard way: a stale
// CMD_TIP_OFFSET was displacing every position by 143 mm, and nothing in the
// data said so. This module decides which of those settings we can live with,
// and phrases the refusals.
//
// Split by consequence:
//
//   * Geometry transforms -- boresight, sensor origin, source rotation -- move
//     or rotate what the device reports before we see it. probe_profiles then
//     applies its own transform on top, so the result is wrong in a way that
//     looks like a mounting problem. These stop the run.
//
//   * Quality settings -- filtering, prediction, frame rate, increment mode --
//     change latency, smoothing and sample cadence but not geometry. These are
//     logged so a recording carries the configuration it was made under, and
//     nothing more.
//
// Takes plain values rather than the SDK structs, so it stays free of the Viper
// headers and is unit-testable without hardware.
//

#ifndef VIPER_DEVICESTATE_HPP
#define VIPER_DEVICESTATE_HPP

#include <cstdint>
#include <string>

namespace mdx {

/// The four floats the Viper returns for a boresight or a source rotation.
///
/// The SDK documents these only as `float params[4]`, without naming the
/// convention. Both plausible spellings of "no rotation" -- an identity
/// quaternion and all zeros -- are therefore treated as neutral, and anything
/// else as deliberately set. That errs towards refusing to run, which is the
/// safe direction for a transform we would otherwise double-apply.
struct DeviceRotation {
    double params[4]{0.0, 0.0, 0.0, 0.0};

    bool isNeutral(double tolerance = 1e-6) const;
    std::string describe() const;
};

/// "source 1 (default)", "common", or the raw code when it is outside
/// eSensorOriginMode. Reported rather than gated: which source a sensor
/// references is a deliberate choice on rigs that have a reason to change it.
std::string sensorOriginLabel(std::uint32_t mode);

/// "240 Hz", or the raw code when it is outside eViperFrameRate.
std::string frameRateLabel(std::uint32_t code);

/// Refusal messages. Each names what was found, why it breaks the tip pose, and
/// how to clear it.
std::string boresightMessage(int sensor, const DeviceRotation &rotation);
std::string sourceRotationMessage(int source, const DeviceRotation &rotation);

struct FilterSettings {
    std::uint32_t level{0};
    double params[4]{0.0, 0.0, 0.0, 0.0};
};

struct PredictiveFilterSettings {
    bool quaternion{false};
    bool position{false};
    double predictionSeconds{0.0};
};

struct IncrementSettings {
    bool enabled{false};
    double positionThreshold{0.0};
    double orientationThreshold{0.0};
};

std::string describeFilter(const FilterSettings &settings);
std::string describePredictiveFilter(const PredictiveFilterSettings &settings);

/// Increment mode makes the device report only after the sensor has moved past
/// a threshold, which presents as an irregular stream rather than an error.
std::string describeIncrement(const IncrementSettings &settings);

/// Distortion above this is called out. A guess, not a measured limit: the SDK
/// documents the field only as 0-255, so the number wants tuning against a rig
/// known to be clean.
constexpr std::uint32_t kDistortionWarnLevel = 32;

std::string distortionMessage(std::uint32_t level, int sensor, std::uint64_t framesAffected);

/// Distortion seen over some window of frames -- one calibration step, say.
///
/// Kept because the capture gates measure geometry only: spread, conditioning
/// and sample count say nothing about signal quality, so a capture taken wholly
/// inside a distorted field passes every check and yields a confident wrong
/// answer. This is what lets the operator see that while it is happening, and
/// what puts it next to the residuals afterwards.
struct DistortionSummary {
    /// Worst level across sensors in the most recent frame.
    std::uint32_t current{0};
    std::uint32_t peak{0};
    std::uint64_t sum{0};
    std::uint64_t frames{0};

    double mean() const;
    bool exceededThreshold() const;
};

/// Compact form for the live capture line, e.g. "dist 3 (peak 12)".
std::string describeDistortionBrief(const DistortionSummary &summary);

/// Fuller form for the result block, e.g. "peak 41/255, mean 12.3 over 1820 frames".
std::string describeDistortion(const DistortionSummary &summary);

} // namespace mdx

#endif //VIPER_DEVICESTATE_HPP
