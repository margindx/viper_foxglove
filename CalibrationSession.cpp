//
// See CalibrationSession.hpp.
//

#include "CalibrationSession.hpp"

#include <cmath>

namespace mdx {

namespace {

constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;

/// Angle between two orientations, degrees.
double angleBetweenDeg(const Eigen::Quaterniond &a, const Eigen::Quaterniond &b) {
    const double dot = std::abs(a.normalized().coeffs().dot(b.normalized().coeffs()));

    return 2.0 * std::acos(std::min(1.0, dot)) * kRadToDeg;
}

bool isUsable(const Pose &pose) {
    if (!pose.position.allFinite() || !pose.orientation.coeffs().allFinite())
        return false;

    return std::abs(pose.orientation.norm() - 1.0) <= 0.1;
}

} // namespace

StepInfo describeStep(CalibrationStep step) {
    switch (step) {
        case CalibrationStep::RockingPivot:
            return {step, "1 of 2: tip pivot",
                    "Rest the probe tip on a flat surface and hold that spot. Rock and rotate "
                    "the probe through as wide a range of angles as you can without letting the "
                    "tip slide. Vary the direction you tilt in, not just how far.\n"
                    "\n"
                    "       \\      |      /\n"
                    "        \\     |     /\n"
                    "         \\    |    /     probe body\n"
                    "          \\   |   /\n"
                    "           \\  |  /\n"
                    "            \\ | /\n"
                    "       ------o------  surface\n"
                    "            tip stays on one spot"};

        case CalibrationStep::BodyFlat:
            return {step, "2 of 2: body flat",
                    "Lay the probe down on a flat of its housing -- the flattened side of the "
                    "body, not the lens end -- so it rests stably on the surface. Then rotate it "
                    "on the surface, like turning a clock hand, and re-place it at many different "
                    "angles. Keep the same flat in contact throughout and keep it flat.\n"
                    "\n"
                    "       viewed from above:\n"
                    "\n"
                    "         ,------------------.\n"
                    "         | S            lens|      rotate on the surface\n"
                    "         `------------------'      and re-place  (clock hand)\n"
                    "\n"
                    "       use the same surface as step 1"};

        case CalibrationStep::Done:
            return {step, "Complete", "All captures gathered."};
    }

    return {step, "Unknown", ""};
}

CalibrationSession::CalibrationSession(CaptureCriteria pivotCriteria,
                                       DirectionCriteria directionCriteria,
                                       DecimationCriteria decimation)
    : pivotCriteria_(pivotCriteria), directionCriteria_(directionCriteria), decimation_(decimation) {}

const std::vector<CalibrationSample> &CalibrationSession::samplesFor(CalibrationStep step) const {
    switch (step) {
        case CalibrationStep::RockingPivot: return pivotSamples_;
        case CalibrationStep::BodyFlat: return flatSamples_;
        case CalibrationStep::Done: break;
    }

    return flatSamples_;
}

std::vector<CalibrationSample> &CalibrationSession::samplesFor(CalibrationStep step) {
    return const_cast<std::vector<CalibrationSample> &>(
            static_cast<const CalibrationSession *>(this)->samplesFor(step));
}

std::size_t CalibrationSession::sampleCount(CalibrationStep step) const {
    if (step == CalibrationStep::Done)
        return 0;

    return samplesFor(step).size();
}

bool CalibrationSession::addSample(const Pose &pose) {
    if (step_ == CalibrationStep::Done || !isUsable(pose))
        return false;

    auto &samples = samplesFor(step_);

    // Frames arrive far faster than the probe moves. Keeping near-duplicates
    // would inflate the sample count -- which the capture gate reads as
    // progress -- without adding any information to the solve.
    if (!samples.empty()) {
        const auto &last = samples.back();
        const bool movedEnough =
                (pose.position - last.position).norm() >= decimation_.minDistanceM;
        const bool turnedEnough =
                angleBetweenDeg(pose.orientation, last.orientation) >= decimation_.minAngleDeg;

        if (!movedEnough && !turnedEnough)
            return false;
    }

    samples.push_back(pose);

    return true;
}

DiversityMetrics CalibrationSession::metrics() const {
    switch (step_) {
        case CalibrationStep::RockingPivot:
            return assessCapture(pivotSamples_, pivotCriteria_);

        case CalibrationStep::BodyFlat:
            return assessDirectionCapture(flatSamples_, directionCriteria_);

        case CalibrationStep::Done:
            break;
    }

    DiversityMetrics done;
    done.sufficient = true;

    return done;
}

bool CalibrationSession::advance() {
    if (!readyToAdvance())
        return false;

    switch (step_) {
        case CalibrationStep::RockingPivot: step_ = CalibrationStep::BodyFlat; break;
        case CalibrationStep::BodyFlat: step_ = CalibrationStep::Done; break;
        case CalibrationStep::Done: break;
    }

    return true;
}

void CalibrationSession::restartStep() {
    if (step_ == CalibrationStep::Done)
        return;

    samplesFor(step_).clear();
}

std::optional<CalibrationOutcome> CalibrationSession::solve(double bodyFlatRollDeg) const {
    const auto pivot = solvePointPivot(pivotSamples_);
    if (!pivot.has_value())
        return std::nullopt;

    const auto bodyFlat = solveCommonDirection(flatSamples_);
    if (!bodyFlat.has_value())
        return std::nullopt;

    CalibrationOutcome outcome;
    outcome.pivot = *pivot;
    outcome.bodyFlat = *bodyFlat;
    outcome.tipOffset = pivot->tipOffset;

    // The probe axis comes free from step 1: the probe is straight and the tip
    // lies on its axis, so the sensor-to-tip vector is that axis. Nothing needs
    // to be captured for it, which is why standing the probe on its lens is no
    // longer part of the procedure.
    if (!(pivot->tipOffset.norm() > 1e-9))
        return std::nullopt;

    const Eigen::Vector3d probeAxis = pivot->tipOffset.normalized();

    outcome.tipRotation =
            tipRotationFromAxes(probeAxis, bodyFlat->sensorDirection, bodyFlatRollDeg);
    if (outcome.tipRotation.has_value())
        outcome.rotationFromIdentityDeg = rotationAngleDeg(*outcome.tipRotation);

    // Second opinion: the plane constraint on the rocking samples, using the
    // surface normal the flat placements established. A different error model,
    // so agreement is meaningful.
    // The world direction from step 2 is the surface normal, since the flat was
    // laid on the same surface the tip was pivoted on.
    outcome.planeCheck = solvePlaneTranslation(pivotSamples_, bodyFlat->worldDirection);
    if (outcome.planeCheck.has_value()) {
        outcome.offsetDisagreementM =
                (outcome.planeCheck->tipOffset - outcome.tipOffset).norm();
    }

    return outcome;
}

} // namespace mdx
