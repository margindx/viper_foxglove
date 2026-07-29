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
            return {step, "1 of 3: tip pivot",
                    "Rest the probe tip on a flat surface and hold that spot. Rock and rotate "
                    "the probe through as wide a range of angles as you can without letting the "
                    "tip slide. Vary the direction you tilt in, not just how far."};

        case CalibrationStep::FlatPlacements:
            return {step, "2 of 3: flat placements",
                    "Lay the probe's imaging face flat against the surface. Lift it, rotate it "
                    "about its own axis, and set it down flat again somewhere else. Repeat at "
                    "many different rotations. Keep the face flat -- do not tilt it."};

        case CalibrationStep::SecondFlat:
            return {step, "3 of 3: second flat",
                    "Same action as step 2, on a different face. Pick a flat on the probe housing "
                    "that is NOT the imaging face, lay it flat against the surface, then lift, "
                    "rotate the probe about that face's normal, and set it down flat again. "
                    "Repeat at many different rotations. Use the same flat every time, and keep "
                    "it flat -- do not tilt."};

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
        case CalibrationStep::FlatPlacements: return flatSamples_;
        case CalibrationStep::SecondFlat: return edgeSamples_;
        case CalibrationStep::Done: break;
    }

    return edgeSamples_;
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

        case CalibrationStep::FlatPlacements:
            return assessDirectionCapture(flatSamples_, directionCriteria_);

        case CalibrationStep::SecondFlat:
            return assessDirectionCapture(edgeSamples_, directionCriteria_);

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
        case CalibrationStep::RockingPivot: step_ = CalibrationStep::FlatPlacements; break;
        case CalibrationStep::FlatPlacements: step_ = CalibrationStep::SecondFlat; break;
        case CalibrationStep::SecondFlat: step_ = CalibrationStep::Done; break;
        case CalibrationStep::Done: break;
    }

    return true;
}

void CalibrationSession::restartStep() {
    if (step_ == CalibrationStep::Done)
        return;

    samplesFor(step_).clear();
}

std::optional<CalibrationOutcome> CalibrationSession::solve(double secondFlatRollDeg) const {
    const auto pivot = solvePointPivot(pivotSamples_);
    if (!pivot.has_value())
        return std::nullopt;

    const auto faceNormal = solveCommonDirection(flatSamples_);
    if (!faceNormal.has_value())
        return std::nullopt;

    const auto secondFlat = solveCommonDirection(edgeSamples_);
    if (!secondFlat.has_value())
        return std::nullopt;

    CalibrationOutcome outcome;
    outcome.pivot = *pivot;
    outcome.faceNormal = *faceNormal;
    outcome.secondFlat = *secondFlat;
    outcome.tipOffset = pivot->tipOffset;

    // The face normal is only defined up to sign by the direction solve. Pick
    // the sense pointing from the sensor towards the tip, which is the
    // direction the pivot already established.
    Eigen::Vector3d face = faceNormal->sensorDirection;
    if (face.dot(pivot->tipOffset) < 0.0)
        face = -face;

    outcome.tipRotation =
            tipRotationFromAxes(face, secondFlat->sensorDirection, secondFlatRollDeg);
    if (outcome.tipRotation.has_value())
        outcome.rotationFromIdentityDeg = rotationAngleDeg(*outcome.tipRotation);

    // Second opinion: the plane constraint on the rocking samples, using the
    // surface normal the flat placements established. A different error model,
    // so agreement is meaningful.
    outcome.planeCheck = solvePlaneTranslation(pivotSamples_, faceNormal->worldDirection);
    if (outcome.planeCheck.has_value()) {
        outcome.offsetDisagreementM =
                (outcome.planeCheck->tipOffset - outcome.tipOffset).norm();
    }

    return outcome;
}

} // namespace mdx
