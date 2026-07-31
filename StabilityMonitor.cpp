//
// See StabilityMonitor.hpp.
//

#include "StabilityMonitor.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace mdx {

namespace {

constexpr double kRadToDeg = 57.29577951308232;

/// Below this a sigma is treated as unmeasured rather than zero. A perfectly
/// still synthetic baseline really does have zero spread, and dividing by it
/// would report every excursion as infinitely significant.
constexpr double kSigmaFloor = 1e-9;

/// Geodesic angle between two orientations, in degrees. Via the relative
/// quaternion's scalar part, with the sign folded away: q and -q are the same
/// rotation, so the absolute value is what carries the angle.
double angleBetweenDeg(const Eigen::Quaterniond &a, const Eigen::Quaterniond &b) {
    const Eigen::Quaterniond delta = a.normalized().conjugate() * b.normalized();
    const double w = std::min(1.0, std::abs(delta.w()));
    return 2.0 * std::acos(w) * kRadToDeg;
}

} // namespace

void BaselineAccumulator::add(const Pose &pose) {
    poses_.push_back(pose);
    samples_++;
}

void BaselineAccumulator::clear() {
    poses_.clear();
    samples_ = 0;
}

std::optional<StabilityBaseline> BaselineAccumulator::result() const {
    if (poses_.size() < 2) {
        return std::nullopt;
    }

    // fusePoses already averages positions and handles the quaternion
    // hemisphere problem, and is tested for it. Averaging repeated samples of
    // one sensor is the same operation as averaging several sensors, so it is
    // reused rather than reimplemented -- and it rejects non-finite input,
    // which a baseline must not be built from.
    const auto mean = fusePoses(poses_);
    if (!mean.has_value()) {
        return std::nullopt;
    }

    StabilityBaseline baseline;
    baseline.position = mean->position;
    baseline.orientation = mean->orientation.normalized();
    baseline.samples = poses_.size();

    double positionSumSq = 0.0;
    double orientationSumSq = 0.0;
    for (const auto &pose : poses_) {
        const double d = (pose.position - baseline.position).norm();
        positionSumSq += d * d;

        const double a = angleBetweenDeg(baseline.orientation, pose.orientation);
        orientationSumSq += a * a;
    }

    const double n = static_cast<double>(poses_.size());
    baseline.positionSigmaM = std::sqrt(positionSumSq / n);
    baseline.orientationSigmaDeg = std::sqrt(orientationSumSq / n);
    baseline.valid = true;

    return baseline;
}

double Excursion::tipEquivalentM(double leverArmM) const {
    const double fromRotation = std::abs(leverArmM) * std::sin(peakOrientationDeg / kRadToDeg);
    return std::max(peakPositionM, fromRotation);
}

double Excursion::significance(const StabilityBaseline &baseline) const {
    if (!baseline.valid) {
        return 0.0;
    }

    const double positionRatio = peakPositionM / std::max(baseline.positionSigmaM, kSigmaFloor);
    const double orientationRatio =
            peakOrientationDeg / std::max(baseline.orientationSigmaDeg, kSigmaFloor);

    return std::max(positionRatio, orientationRatio);
}

void ExcursionTracker::add(const Pose &pose) {
    if (!baseline_.valid) {
        return;
    }

    excursion_.positionM = (pose.position - baseline_.position).norm();
    excursion_.orientationDeg = angleBetweenDeg(baseline_.orientation, pose.orientation);

    excursion_.peakPositionM = std::max(excursion_.peakPositionM, excursion_.positionM);
    excursion_.peakOrientationDeg =
            std::max(excursion_.peakOrientationDeg, excursion_.orientationDeg);
}

void ExcursionTracker::clearPeaks() {
    excursion_.peakPositionM = excursion_.positionM;
    excursion_.peakOrientationDeg = excursion_.orientationDeg;
}

RelativePose relativePose(const Pose &a, const Pose &b) {
    const Eigen::Quaterniond qa = a.orientation.normalized();

    RelativePose relative;
    // Both expressed in a's frame, so the pair is invariant to how the probe as
    // a whole is moved. That is the point: the operator can pick the probe up
    // without disturbing the measurement.
    relative.translation = qa.conjugate() * (b.position - a.position);
    relative.rotation = qa.conjugate() * b.orientation.normalized();

    return relative;
}

bool RigidityWitness::arm(const std::vector<Pose> &poses) {
    armed_ = false;
    baseline_.clear();
    pairs_.clear();

    if (poses.size() < 2) {
        return false;
    }

    sensorCount_ = poses.size();

    for (std::size_t i = 0; i < poses.size(); i++) {
        for (std::size_t j = i + 1; j < poses.size(); j++) {
            const auto relative = relativePose(poses[i], poses[j]);
            baseline_.push_back(relative);

            PairDeviation pair;
            pair.first = i;
            pair.second = j;
            pair.baselineSeparationM = relative.separationM();
            pairs_.push_back(pair);
        }
    }

    armed_ = true;
    return true;
}

void RigidityWitness::add(const std::vector<Pose> &poses) {
    if (!armed_ || poses.size() != sensorCount_) {
        return;
    }

    std::size_t index = 0;
    for (std::size_t i = 0; i < poses.size(); i++) {
        for (std::size_t j = i + 1; j < poses.size(); j++, index++) {
            const auto relative = relativePose(poses[i], poses[j]);

            auto &pair = pairs_[index];
            pair.separationDeviationM = (relative.translation - baseline_[index].translation).norm();
            pair.rotationDeviationDeg =
                    angleBetweenDeg(baseline_[index].rotation, relative.rotation);

            pair.peakSeparationDeviationM =
                    std::max(pair.peakSeparationDeviationM, pair.separationDeviationM);
            pair.peakRotationDeviationDeg =
                    std::max(pair.peakRotationDeviationDeg, pair.rotationDeviationDeg);
            pair.samples++;
        }
    }
}

void RigidityWitness::clearPeaks() {
    for (auto &pair : pairs_) {
        pair.peakSeparationDeviationM = pair.separationDeviationM;
        pair.peakRotationDeviationDeg = pair.rotationDeviationDeg;
    }
}

double RigidityWitness::worstTipEquivalentM(double leverArmM) const {
    double worst = 0.0;
    for (const auto &pair : pairs_) {
        const double fromRotation =
                std::abs(leverArmM) * std::sin(pair.peakRotationDeviationDeg / kRadToDeg);
        worst = std::max({worst, pair.peakSeparationDeviationM, fromRotation});
    }
    return worst;
}

std::string describeStability(const Excursion &excursion, const StabilityBaseline &baseline,
                              std::optional<double> leverArmM) {
    std::ostringstream ss;
    ss << std::fixed;

    if (!baseline.valid) {
        return "No baseline was captured, so there is nothing to measure against.";
    }

    ss << std::setprecision(3)
       << "Noise floor  " << baseline.positionSigmaM * 1000.0 << " mm, "
       << baseline.orientationSigmaDeg << " deg over " << baseline.samples << " samples\n"
       << "Peak         " << excursion.peakPositionM * 1000.0 << " mm, "
       << excursion.peakOrientationDeg << " deg";

    if (leverArmM.has_value()) {
        ss << "  ->  " << std::setprecision(2) << excursion.tipEquivalentM(*leverArmM) * 1000.0
           << " mm at the tip (lever " << leverArmM.value() * 1000.0 << " mm)";
    }
    ss << "\n";

    const double significance = excursion.significance(baseline);
    ss << std::setprecision(1) << "Peak is " << significance << "x the noise floor. ";

    // Three bands, and the wording stops at what was measured. Whether an
    // excursion is the sensor shifting, the probe having been nudged or the
    // field drifting is not something the numbers can settle -- that is what
    // the event marks and the repeat-on-reload pattern are for.
    if (significance < 3.0) {
        ss << "Nothing above the noise: no movement of a size this run could detect.";
    } else if (significance < 10.0) {
        ss << "Above the noise, but small. Repeat the load and release several times -- "
              "movement inside the probe steps with each load and reverses on release, "
              "while drift does neither.";
    } else {
        ss << "Well above the noise. If it tracks the marked events, something moved; "
              "check that the probe body itself was not nudged before concluding it was "
              "the sensor.";
    }

    return ss.str();
}

std::string describePair(const PairDeviation &pair) {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(2) << "sensors " << pair.first << "-" << pair.second
       << ": separation " << pair.baselineSeparationM * 1000.0 << " mm, drift "
       << std::setprecision(3) << pair.peakSeparationDeviationM * 1000.0 << " mm / "
       << pair.peakRotationDeviationDeg << " deg";
    return ss.str();
}

} // namespace mdx
