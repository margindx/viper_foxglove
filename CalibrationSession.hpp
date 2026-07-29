//
// The guided calibration state machine.
//
// Sequences the three capture motions, decides when each has gathered enough to
// be trustworthy, and solves for the tip transform at the end. Deliberately
// free of console I/O so the transitions and gating can be tested by feeding it
// synthetic poses; the CLI in Calibrate.cpp is a thin shell around this.
//

#ifndef VIPER_CALIBRATIONSESSION_HPP
#define VIPER_CALIBRATIONSESSION_HPP

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include "PivotSolve.hpp"

namespace mdx {

enum class CalibrationStep {
    /// Motion A: tip on the surface, probe rocked through orientations.
    RockingPivot,
    /// Motion B1: face laid flat on the surface at varied spins and positions.
    FlatPlacements,
    /// Motion B2: a second flat of the housing laid on the same surface.
    SecondFlat,
    /// All captures complete; ready to solve.
    Done,
};

/// What to tell the operator for a given step.
struct StepInfo {
    CalibrationStep step{CalibrationStep::RockingPivot};
    std::string title;
    std::string instructions;
};

StepInfo describeStep(CalibrationStep step);

/// How much a pose must differ from the last retained one to be kept. Raw
/// frames arrive far faster than the probe actually moves; keeping every one
/// would bloat the solve with near-duplicates that add no information.
struct DecimationCriteria {
    double minAngleDeg{0.5};
    double minDistanceM{0.0005};
};

struct CalibrationOutcome {
    /// The offset written to probe_profiles, taken from the rocking pivot.
    Eigen::Vector3d tipOffset{Eigen::Vector3d::Zero()};
    /// Present when both direction steps solved. Absent means the rotation
    /// could not be determined and identity should be kept.
    std::optional<Eigen::Quaterniond> tipRotation;

    PivotResult pivot;
    /// Imaging face normal, in the sensor frame, from step 2.
    DirectionResult faceNormal;
    /// The direction step 3 pinned down -- the second housing flat's normal.
    DirectionResult secondFlat;

    /// Independent offset estimate from the plane constraint applied to the
    /// rocking samples. Absent when that system was too poorly conditioned.
    std::optional<PlaneTranslationResult> planeCheck;
    /// Distance between the pivot and plane-constraint offsets, metres. Large
    /// values mean the two error models disagree and neither should be trusted
    /// without investigation.
    double offsetDisagreementM{0.0};

    /// How far the solved rotation sits from identity, degrees. Small values
    /// are more likely noise than a real mounting angle.
    double rotationFromIdentityDeg{0.0};
};

class CalibrationSession {
public:
    explicit CalibrationSession(CaptureCriteria pivotCriteria = {},
                                DirectionCriteria directionCriteria = {},
                                DecimationCriteria decimation = {});

    CalibrationStep step() const { return step_; }
    StepInfo currentStep() const { return describeStep(step_); }

    /// Offer a fused sensor pose to the current step. Returns true if it was
    /// retained rather than discarded as a near-duplicate.
    bool addSample(const Pose &pose);

    /// Live assessment of the current step, using whichever criteria suit that
    /// motion.
    DiversityMetrics metrics() const;

    bool readyToAdvance() const { return step_ != CalibrationStep::Done && metrics().sufficient; }

    /// Move to the next step. Returns false (and does nothing) when the current
    /// step has not gathered enough.
    bool advance();

    /// Discard the current step's samples and capture it again.
    void restartStep();

    std::size_t sampleCount(CalibrationStep step) const;

    /// Solve the whole calibration. Only meaningful once every step is
    /// captured; returns nullopt if any required solve fails.
    ///
    /// `secondFlatRollDeg` relates the direction recovered by the third step to
    /// the footprint's long axis, measured about the probe axis. Step 3 laying a
    /// second housing flat on the table recovers that flat's normal, which is
    /// not the footprint direction; the angle between them is a property of the
    /// probe's design and comes from CAD, not from the capture. Zero means the
    /// captured direction already is the footprint axis.
    std::optional<CalibrationOutcome> solve(double secondFlatRollDeg = 0.0) const;

private:
    const std::vector<CalibrationSample> &samplesFor(CalibrationStep step) const;
    std::vector<CalibrationSample> &samplesFor(CalibrationStep step);

    CaptureCriteria pivotCriteria_;
    DirectionCriteria directionCriteria_;
    DecimationCriteria decimation_;

    CalibrationStep step_{CalibrationStep::RockingPivot};

    std::vector<CalibrationSample> pivotSamples_;
    std::vector<CalibrationSample> flatSamples_;
    std::vector<CalibrationSample> edgeSamples_;
};

} // namespace mdx

#endif //VIPER_CALIBRATIONSESSION_HPP
