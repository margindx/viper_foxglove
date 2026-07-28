//
// Probe tip calibration solvers.
//
// Three independent estimates are produced from three captured motions:
//
//   A. Rocking pivot -- the tip is held against a flat surface while the probe
//      is swept through orientations. Solves p_pivot = p_i + R_i*t for the tip
//      offset t and the (constant) pivot point. Note the tip here is the centre
//      of a 10 x 1 mm face, not a point: as the probe tilts the contact point
//      migrates across that face, biasing the solve by up to half the footprint
//      length. The bias is strongly anisotropic, so residuals are reported per
//      axis in the probe frame rather than as a single RMS.
//
//   B1. Flat placements -- the face is laid flat on the surface at several
//      positions and spins. Every placement puts the same sensor-frame
//      direction (the face normal) onto the same world direction (the surface
//      normal), which solveCommonDirection recovers.
//
//      Note what this motion cannot do. Because the face is flat every time,
//      R_i^T n is the same vector for every placement, so the tip offset enters
//      only through the constant v.t -- which is indistinguishable from the
//      unknown position of the surface. Only (d - v.t) is observable, never
//      v.t. Flat placements therefore say nothing whatsoever about the
//      translation; they determine orientation only.
//
//   B2. Straightedge -- the 10 mm footprint edge is butted against a straight
//      reference at several positions, putting the footprint long axis onto a
//      fixed world direction. Same solver as B1, and together the two
//      directions determine the full tip rotation.
//
// Everything here is pure: no hardware, no I/O, no Foxglove types. Depends only
// on the vendored Eigen headers under dep/.
//

#ifndef VIPER_PIVOTSOLVE_HPP
#define VIPER_PIVOTSOLVE_HPP

#include <cstddef>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include "Eigen/Dense"
#include "Eigen/Geometry"

#include "ProbeProfile.hpp"

namespace mdx {

/// One captured fused-sensor pose. Positions are metres (see FrameUnits).
using CalibrationSample = Pose;

/// Thresholds deciding when a capture is good enough to solve from. Defaults
/// are starting points to be tuned against hardware, not measured constants.
struct CaptureCriteria {
    /// Samples retained after decimation, not raw frames.
    std::size_t minSamples{40};
    /// Required angular spread of the probe axis, as a half-cone about the mean
    /// direction. A capture swept through a narrow cone is ill-conditioned no
    /// matter how many samples it contains.
    double minConeHalfAngleDeg{20.0};
    /// Upper bound on the least-squares condition number. Above this the solve
    /// is numerically unreliable even if the other two criteria pass.
    double maxConditionNumber{100.0};
};

/// Thresholds for the direction-finding motions (B1 and B2), which need
/// different diversity from the pivot: the probe axis is deliberately held
/// constant while the probe is spun and repositioned, so cone spread is the
/// wrong thing to measure.
struct DirectionCriteria {
    std::size_t minSamples{20};
    /// Normalised gap between the first two singular values of the summed
    /// rotations. Zero means every placement was identical and the direction is
    /// undetermined; larger means the spin genuinely varied.
    double minSeparation{0.15};
};

/// Live feedback while capturing, so the operator can be told what is missing
/// rather than being failed at the end.
struct DiversityMetrics {
    std::size_t sampleCount{0};
    /// Half-angle of the cone enclosing the observed probe-axis directions.
    /// Meaningful for the pivot motion only.
    double coneHalfAngleDeg{0.0};
    /// Condition number of the stacked pivot system; infinity when degenerate.
    /// Pivot motion only.
    double conditionNumber{std::numeric_limits<double>::infinity()};
    /// Singular-value separation of the summed rotations. Direction motions
    /// only.
    double directionSeparation{0.0};
    bool sufficient{false};

    /// What still needs to happen, phrased for an operator. Empty when
    /// sufficient.
    std::string guidance;
};

/// Assess a rocking-pivot capture (motion A).
DiversityMetrics assessCapture(const std::vector<CalibrationSample> &samples,
                               const CaptureCriteria &criteria = {});

/// Assess a direction-finding capture (motions B1 and B2). Uses spin diversity
/// rather than cone spread, because a correct flat-placement capture holds the
/// probe axis fixed by design -- judging it by cone spread would tell the
/// operator to tilt, which is exactly what they must not do.
DiversityMetrics assessDirectionCapture(const std::vector<CalibrationSample> &samples,
                                        const DirectionCriteria &criteria = {});

/// Result of the rocking-pivot solve (motion A).
struct PivotResult {
    /// Sensor-frame offset from the fused sensor origin to the tip.
    Eigen::Vector3d tipOffset{Eigen::Vector3d::Zero()};
    /// The stationary point the tip was held at, in tracker coordinates.
    Eigen::Vector3d pivotPoint{Eigen::Vector3d::Zero()};
    /// Per-axis RMS residual expressed in the probe frame, metres. The
    /// footprint long axis should dominate; a large short-axis value means
    /// something other than contact migration is wrong.
    Eigen::Vector3d residualRmsProbeFrame{Eigen::Vector3d::Zero()};
    /// Overall RMS residual, metres.
    double residualRms{0.0};
    double conditionNumber{0.0};
    std::size_t sampleCount{0};
};

std::optional<PivotResult> solvePointPivot(const std::vector<CalibrationSample> &samples);

/// A sensor-frame direction that maps onto a fixed world direction across all
/// samples (motions B1 and B2).
struct DirectionResult {
    /// The direction in the sensor frame -- the face normal for B1, the
    /// footprint long axis for B2.
    Eigen::Vector3d sensorDirection{Eigen::Vector3d::UnitX()};
    /// The corresponding fixed direction in tracker coordinates.
    Eigen::Vector3d worldDirection{Eigen::Vector3d::UnitX()};
    /// RMS angle between R_i*sensorDirection and worldDirection, degrees.
    double residualDeg{0.0};
    /// Gap between the first two singular values of the rotation sum. Near zero
    /// means the placements were too alike to determine the direction.
    double separation{0.0};
    std::size_t sampleCount{0};
};

std::optional<DirectionResult> solveCommonDirection(const std::vector<CalibrationSample> &samples);

/// Result of the plane-constrained translation solve: a second opinion on the
/// tip offset, independent of the point pivot.
struct PlaneTranslationResult {
    Eigen::Vector3d tipOffset{Eigen::Vector3d::Zero()};
    /// Signed distance from the tracker origin to the surface, metres.
    double planeOffset{0.0};
    /// RMS distance of the solved tip from the surface, metres.
    double residualRms{0.0};
    double conditionNumber{0.0};
    std::size_t sampleCount{0};
};

/// Solve n . (p_i + R_i*t) = d for t and d, with the surface normal n taken
/// from the flat placements. Linear once n is fixed, which is why B1 is solved
/// first.
///
/// Feed this the *rocking* samples, not the flat ones. It needs R_i^T n to vary
/// across samples to separate t from d, which only the rocking motion provides;
/// on flat placements the system is rank-deficient and this returns nullopt
/// rather than a plausible-looking wrong answer.
///
/// This is a genuinely independent estimate rather than a better one. It
/// assumes the tip centre lies on the surface, which is violated as the probe
/// tilts onto an edge, so it carries its own bias -- a different one from the
/// pivot's contact migration. Agreement between the two is evidence; a large
/// disagreement means at least one model is being strained.
std::optional<PlaneTranslationResult> solvePlaneTranslation(
        const std::vector<CalibrationSample> &samples, const Eigen::Vector3d &planeNormal);

/// Build the sensor-to-tip rotation from the two calibrated directions.
///
/// The tip frame follows the convention used by tip_offset_m and by the
/// downstream probe geometry: +x along the probe (the face normal), +y along
/// the 10 mm footprint edge, +z completing the right-handed set. The long axis
/// is orthogonalised against the face normal, so it need not be measured
/// perfectly perpendicular.
///
/// Returns nullopt if the two directions are parallel enough that the third
/// axis is undetermined.
std::optional<Eigen::Quaterniond> tipRotationFromAxes(const Eigen::Vector3d &faceNormalSensor,
                                                      const Eigen::Vector3d &longAxisSensor);

/// Angle of a rotation, degrees -- used to report how far a solved tip rotation
/// sits from identity, so a correction that is really just noise can be seen
/// for what it is.
double rotationAngleDeg(const Eigen::Quaterniond &q);

/// Convert a rotation to the Z-Y-X Euler triple (degrees) that probe_profiles
/// stores, i.e. the inverse of quaternionFromZyxDegrees.
Eigen::Vector3d zyxDegreesFromQuaternion(const Eigen::Quaterniond &q);

} // namespace mdx

#endif //VIPER_PIVOTSOLVE_HPP
