//
// Is the sensor moving inside the probe?
//
// The tip offset is only meaningful if the sensor is rigidly fixed in the probe
// body. A sensor that slides or rotates in its lumen -- pulled by its own cable,
// say -- makes the offset a quantity that genuinely varies, and no amount of
// care during calibration will pin it down. Two runs disagreeing by 12 mm looks
// identical whether the cause is a moving sensor or a poor capture.
//
// This module measures the two things that tell them apart.
//
//   Excursion from a baseline. Rest the probe untouched and load only its
//   cable. The body is stationary by construction, so any pose change is the
//   sensor moving inside it. Holding position fixed also freezes the field, so
//   distortion is not free to vary the way it is in any moving test. Baselines
//   are re-armable, and events can be marked, because the discriminating signal
//   is a step that repeats with each load and reverses on release -- drift does
//   not care where the operator's hand is.
//
//   Relative pose between two sensors. Two sensors on one rigid body hold a
//   constant transform. Any variation is relative movement, established with no
//   pivot, no tip offset and no table. This is the decisive form of the test,
//   and it needs a second sensor clamped to the probe; a multi-sensor probe can
//   stand in as a known-rigid reference to show what a good result looks like in
//   a given field.
//
// Orientation carries more of the tip error than position does. At a 194 mm
// offset one degree is 3.4 mm, so a rotation far too small to see or feel
// dominates a position shift that would be obvious.
//
// Pure: no hardware, no I/O, no Foxglove types. Depends only on Eigen under
// dep/, and is unit-testable against synthetic poses.
//

#ifndef VIPER_STABILITYMONITOR_HPP
#define VIPER_STABILITYMONITOR_HPP

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include "Eigen/Dense"
#include "Eigen/Geometry"

#include "ProbeProfile.hpp"

namespace mdx {

/// What a sensor did while it was supposed to be holding still. Accumulated
/// over the baseline window and then frozen, so later excursions are measured
/// against a fixed reference rather than a rolling one -- a rolling baseline
/// would absorb exactly the slow movement being looked for.
struct StabilityBaseline {
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};

    /// Spread within the baseline window itself: the noise floor. An excursion
    /// only means something relative to this.
    double positionSigmaM{0.0};
    double orientationSigmaDeg{0.0};

    std::size_t samples{0};
    bool valid{false};
};

/// Accumulates poses and produces a baseline. Kept separate from the excursion
/// tracking so a run can re-arm mid-flight without discarding its history.
class BaselineAccumulator {
public:
    void add(const Pose &pose);
    void clear();

    std::size_t samples() const { return samples_; }

    /// Empty until at least two samples have arrived: a single sample has no
    /// spread, and a noise floor of zero would make every later excursion look
    /// infinitely significant.
    std::optional<StabilityBaseline> result() const;

private:
    std::vector<Pose> poses_;
    std::size_t samples_{0};
};

/// How far a sensor has moved from its baseline, now and at its worst.
///
/// Peak is held because the informative moment is usually transient -- a tug on
/// the cable is over before the operator can look up -- and a live number alone
/// would miss it.
struct Excursion {
    double positionM{0.0};
    double orientationDeg{0.0};
    double peakPositionM{0.0};
    double peakOrientationDeg{0.0};

    /// Largest of the two expressed as tip displacement, given a lever arm.
    /// Position and orientation contribute in unknown directions, so this is
    /// deliberately the worse of the two rather than a sum: a bound to act on,
    /// not an estimate to quote.
    double tipEquivalentM(double leverArmM) const;

    /// Excursion relative to the baseline's own noise, position and orientation
    /// taken together, worst-of. Below about 3 there is nothing to see.
    double significance(const StabilityBaseline &baseline) const;
};

/// Tracks one sensor against a frozen baseline.
class ExcursionTracker {
public:
    explicit ExcursionTracker(StabilityBaseline baseline) : baseline_(baseline) {}

    void add(const Pose &pose);

    /// Drops the peaks but keeps the baseline, so a run can be re-zeroed
    /// between load cycles without re-measuring the noise floor.
    void clearPeaks();

    const StabilityBaseline &baseline() const { return baseline_; }
    const Excursion &excursion() const { return excursion_; }

private:
    StabilityBaseline baseline_;
    Excursion excursion_{};
};

/// The transform from one sensor to another, as a separation and a rotation.
///
/// Split this way because the two fail differently: a sensor sliding along its
/// lumen changes the separation, while one rotating in place changes only the
/// relative rotation, and reporting a single 6-DOF number would blur them.
struct RelativePose {
    /// Vector from the first sensor to the second, in the first one's own
    /// frame. The vector rather than its length: a shift perpendicular to the
    /// line between two sensors barely changes the distance between them, so a
    /// scalar separation is blind to it to first order.
    Eigen::Vector3d translation{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond rotation{Eigen::Quaterniond::Identity()};

    double separationM() const { return translation.norm(); }
};

RelativePose relativePose(const Pose &a, const Pose &b);

/// Deviation of a sensor pair from the rigid-body transform they started with.
struct PairDeviation {
    std::size_t first{0};
    std::size_t second{0};

    double baselineSeparationM{0.0};

    double separationDeviationM{0.0};
    double rotationDeviationDeg{0.0};
    double peakSeparationDeviationM{0.0};
    double peakRotationDeviationDeg{0.0};

    std::size_t samples{0};
};

/// Watches every pair of sensors for departure from rigidity.
///
/// All pairs rather than consecutive ones: with three sensors, a single one
/// working loose shows in two pairs and not the third, which localizes it. That
/// is n(n-1)/2 pairs, trivial at the sensor counts this device supports.
class RigidityWitness {
public:
    /// Records the pair transforms as they stand. Needs at least two sensors;
    /// returns false and stays disarmed otherwise.
    bool arm(const std::vector<Pose> &poses);

    /// Ignored unless armed and the sensor count still matches -- a sensor
    /// appearing or dropping out mid-run invalidates the pairing, and silently
    /// re-indexing would compare different sensors to each other.
    void add(const std::vector<Pose> &poses);

    void clearPeaks();

    bool armed() const { return armed_; }
    const std::vector<PairDeviation> &pairs() const { return pairs_; }

    /// Worst rotation deviation across pairs, as tip displacement over a lever
    /// arm. The headline number: it is what relative movement of this size
    /// would do to the calibrated tip.
    double worstTipEquivalentM(double leverArmM) const;

private:
    bool armed_{false};
    std::size_t sensorCount_{0};
    std::vector<RelativePose> baseline_;
    std::vector<PairDeviation> pairs_;
};

/// One operator-stamped event, so a pose step can be tied to what the hand was
/// doing. Correlation with the marks is the evidence; the excursion alone is
/// not, since drift produces excursions too.
struct MonitorEvent {
    double timeSeconds{0.0};
    std::string label;
};

/// Verdict wording for a finished run. Deliberately describes what was measured
/// rather than declaring the sensor loose: the threshold below which movement is
/// invisible is set by the noise floor, and above it the cause is still the
/// operator's to attribute.
std::string describeStability(const Excursion &excursion, const StabilityBaseline &baseline,
                              std::optional<double> leverArmM);

/// Formats a pair line, e.g. "sensors 0-1: separation 84.31 mm, drift 0.12 mm / 0.31 deg".
std::string describePair(const PairDeviation &pair);

} // namespace mdx

#endif //VIPER_STABILITYMONITOR_HPP
