//
// Probe tip calibration solvers.
//
// Three independent estimates are produced from three captured motions:
//
//   A. Rocking pivot -- the tip is held against a flat surface while the probe
//      is swept through orientations. Solves p_pivot = p_i + R_i*t for the tip
//      offset t and the (constant) pivot point. Note the tip here is the center
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
//   B2. Second flat -- a flat of the housing that is not the imaging face is
//      laid on the same surface, at several spins. Structurally identical to
//      B1, just a different face, and together the two directions determine the
//      full tip rotation. The direction it recovers is that flat's normal, not
//      the footprint axis; relating the two is a design constant supplied to
//      tipRotationFromAxes as a roll offset.
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

/// One captured fused-sensor pose. Positions are meters (see FrameUnits).
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
    /// Upper bound on the worst-determined direction of the tip offset, in
    /// meters -- the gate that actually decides when step 1 is done.
    ///
    /// The condition number below cannot do this job. It is one scalar over six
    /// unknowns, so it averages away the thing that matters: a capture can be
    /// well conditioned overall while one direction of the offset is several
    /// times less determined than the others. At a condition of 20 the worst
    /// direction is still uncertain by around 2.6 mm while the best is under
    /// 1 mm, and that anisotropy is visible on the bench as a tip that tracks
    /// well when the probe is rocked one way and wanders when it is rocked
    /// across -- because an offset error along the rocking axis is invariant
    /// under that rock and only shows up under the perpendicular one.
    ///
    /// 1.2 mm needs roughly 35-40 degrees of variation in tilt direction. At
    /// the residual of a real bench capture (8.43 mm RMS) the floor -- what no
    /// amount of further motion improves on -- is about 0.91 mm, so this is
    /// reachable without being generous. A cleaner capture reaches it sooner,
    /// which is the right behaviour: less motion is needed when the data is
    /// better.
    double maxOffsetUncertaintyM{0.0012};
    /// Upper bound on the least-squares condition number. Above this the solve
    /// is numerically unreliable even if the other two criteria pass.
    ///
    /// This is the criterion that catches rocking in a single plane, which the
    /// cone spread cannot see -- and 100 was far too loose to do it. Measured on
    /// synthetic captures at 0.5 mm / 0.2 deg sensor noise: rocking at one
    /// heading conditions at ~57 and misplaces the tip by up to 3.6 mm, while
    /// two or more headings condition at ~6 and land within 0.5 mm. Real bench
    /// captures have reported 7.0 and 8.2. 20 sits well clear of a good capture
    /// and well below a planar one.
    double maxConditionNumber{20.0};
};

/// Turn angle, in degrees, at which a uniform sweep reaches `separation`.
///
/// Separation is 1 - |sinc(delta/2)| for a sweep of delta, so this inverts that
/// numerically. Used to phrase the gate as an angle the operator can act on;
/// the reported progress is measured, not derived from this.
double turnAngleForSeparationDeg(double separation);

/// Thresholds for the direction-finding motions (B1 and B2), which need
/// different diversity from the pivot: the probe axis is deliberately held
/// constant while the probe is spun and repositioned, so cone spread is the
/// wrong thing to measure.
struct DirectionCriteria {
    std::size_t minSamples{20};
    /// Normalized gap between the first two singular values of the summed
    /// rotations. Zero means every placement was identical and the direction is
    /// undetermined; larger means the spin genuinely varied.
    double minSeparation{0.15};
};

/// Spread of the tip offset the capture can support, as an uncertainty
/// ellipsoid in the sensor frame.
///
/// Reported per direction rather than as one number because that is how the
/// error behaves: rocking about a single axis leaves the offset undetermined
/// along that same axis, so the uncertainty is genuinely anisotropic and a
/// scalar summary hides the failure.
struct OffsetUncertainty {
    /// One standard deviation along the best- and worst-determined directions.
    double bestM{0.0};
    double worstM{std::numeric_limits<double>::infinity()};
    /// The worst-determined direction, in the sensor frame. Rocking about an
    /// axis perpendicular to this is what improves it.
    Eigen::Vector3d worstDirection{Eigen::Vector3d::Zero()};
    bool valid{false};

    /// worstM / bestM. Above about 2 the capture is lopsided, which is what
    /// puts a directional error into the tip.
    double anisotropy() const;
};

/// Propagate the fit residual through the pivot system to get the offset's
/// uncertainty ellipsoid.
///
/// Validated against the error actually made, over 400 synthetic captures per
/// geometry: predicted-to-actual came out at 1.04, 1.02 and 1.01 for captures
/// whose worst direction sat at 6.7, 2.6 and 1.3 mm.
OffsetUncertainty offsetUncertainty(const std::vector<CalibrationSample> &samples);

/// Live feedback while capturing, so the operator can be told what is missing
/// rather than being failed at the end.
struct DiversityMetrics {
    std::size_t sampleCount{0};
    /// Half-angle of the cone enclosing the observed probe-axis directions.
    /// Meaningful for the pivot motion only.
    double coneHalfAngleDeg{0.0};
    /// RMS tilt away from the dominant tilt direction, in degrees. The cone
    /// half-angle measures how far the probe was tilted; this measures whether
    /// it was tilted in more than one direction. Rocking at a single heading
    /// yields a wide cone and a near-zero value here.
    double secondarySpreadDeg{0.0};
    /// Condition number of the stacked pivot system; infinity when degenerate.
    /// Pivot motion only. Reported rather than gated on -- see
    /// CaptureCriteria::maxOffsetUncertaintyM.
    double conditionNumber{std::numeric_limits<double>::infinity()};
    /// What the capture so far can pin the offset down to. Pivot motion only.
    OffsetUncertainty offset;
    /// Singular-value separation of the summed rotations. Direction motions
    /// only.
    double directionSeparation{0.0};
    /// Smallest arc, in degrees, containing every heading the probe has been
    /// set to. Direction motions only.
    ///
    /// For the intended continuous turn this is the angle swept. For a few
    /// discrete placements it is their coverage, which can be less than the
    /// path taken between them -- the information is in the headings, not in
    /// how the probe travelled between them, so coverage is the honest figure.
    /// Slightly optimistic in proportion to how much the probe rocks, since the
    /// arc encloses that too.
    ///
    /// Measured rather than inverted from the separation, and shown in place of
    /// it: separation goes as roughly the square of the turn, so it barely
    /// moves over the first 60 degrees and makes a working capture feel stuck.
    double turnRangeDeg{0.0};
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
    /// Per-axis RMS residual expressed in the probe frame, meters. The
    /// footprint long axis should dominate; a large short-axis value means
    /// something other than contact migration is wrong.
    Eigen::Vector3d residualRmsProbeFrame{Eigen::Vector3d::Zero()};
    /// Overall RMS residual, meters.
    double residualRms{0.0};
    double conditionNumber{0.0};
    std::size_t sampleCount{0};
};

std::optional<PivotResult> solvePointPivot(const std::vector<CalibrationSample> &samples);

/// A sensor-frame direction that maps onto a fixed world direction across all
/// samples (motions B1 and B2).
struct DirectionResult {
    /// The direction in the sensor frame -- the imaging face normal for B1, the
    /// second housing flat's normal for B2.
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
    /// Signed distance from the tracker origin to the surface, meters.
    double planeOffset{0.0};
    /// RMS distance of the solved tip from the surface, meters.
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
/// assumes the tip center lies on the surface, which is violated as the probe
/// tilts onto an edge, so it carries its own bias -- a different one from the
/// pivot's contact migration. Agreement between the two is evidence; a large
/// disagreement means at least one model is being strained.
/// How far the plane-constraint offset can move purely because the normal it
/// borrows is uncertain.
///
/// The check takes its surface normal from step 2, which measures that normal
/// with its own RMS error. Tilting the normal by that error and re-solving says
/// how much of any observed disagreement the borrowed normal explains on its
/// own -- and it is usually most of it, because the plane system is weakly
/// conditioned and the offset is a long lever arm.
///
/// Without this the disagreement was being read as evidence about the pivot,
/// which it is not: a well-conditioned pivot next to a large gap was reported as
/// a physical inconsistency even when the normal's own error accounted for the
/// whole thing.
///
/// Returns the largest shift over a ring of tilt directions, relative to
/// `reference`. Zero if the perturbed systems cannot be solved.
double planeCheckNormalSensitivity(const std::vector<CalibrationSample> &samples,
                                   const Eigen::Vector3d &normal, double normalErrorDeg,
                                   const Eigen::Vector3d &reference);

std::optional<PlaneTranslationResult> solvePlaneTranslation(
        const std::vector<CalibrationSample> &samples, const Eigen::Vector3d &planeNormal);

/// Build the sensor-to-tip rotation from the two calibrated directions.
///
/// The tip frame follows the convention used by tip_offset_m and by the
/// downstream probe geometry: +x along the probe (the imaging face normal), +y
/// along the footprint's long axis, +z completing the right-handed set.
///
/// `inPlaneReferenceSensor` is whatever second direction the capture pinned
/// down. It is orthogonalized against the face normal, so it need not be
/// perpendicular to it -- only non-parallel.
///
/// `rollOffsetDeg` rotates the resulting +y about +x, and is how a reference
/// that is not itself the footprint axis gets related to it. Laying a second
/// flat of the housing on the table recovers that flat's normal, not the
/// footprint direction; the angle between them, about the probe axis, is a
/// fixed property of the probe's design and has to come from CAD. Zero means
/// the reference already is the footprint axis.
///
/// Returns nullopt if the two directions are parallel enough that the third
/// axis is undetermined.
std::optional<Eigen::Quaterniond> tipRotationFromAxes(const Eigen::Vector3d &faceNormalSensor,
                                                      const Eigen::Vector3d &inPlaneReferenceSensor,
                                                      double rollOffsetDeg = 0.0);

/// Estimate which way is up, in tracker coordinates, from a pivot capture.
///
/// During the pivot the tip rests on the surface and the probe leans up out of
/// it, so the tip-to-sensor vector -R_i*t points upwards however the probe is
/// tilted. Averaging it over the sweep gives the surface normal's sense.
///
/// This exists to anchor a sign. solveCommonDirection returns its pair of
/// directions from a singular vector, whose sign is arbitrary, so the same
/// capture could yield either (v, n) or (-v, -n). Knowing which way up is turns
/// that into one answer.
///
/// Returns nullopt when the samples are unusable or cancel out.
std::optional<Eigen::Vector3d> estimateUpFromPivot(const std::vector<CalibrationSample> &samples,
                                                   const Eigen::Vector3d &tipOffset);

/// Angle of a rotation, degrees -- used to report how far a solved tip rotation
/// sits from identity, so a correction that is really just noise can be seen
/// for what it is.
double rotationAngleDeg(const Eigen::Quaterniond &q);

/// Convert a rotation to the Z-Y-X Euler triple (degrees) that probe_profiles
/// stores, i.e. the inverse of quaternionFromZyxDegrees.
Eigen::Vector3d zyxDegreesFromQuaternion(const Eigen::Quaterniond &q);

} // namespace mdx

#endif //VIPER_PIVOTSOLVE_HPP
