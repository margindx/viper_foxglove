//
// See PivotSolve.hpp.
//

#include "PivotSolve.hpp"

#include <cmath>
#include <sstream>

namespace mdx {

namespace {

constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

/// The probe axis in the sensor frame before any tip rotation is known. Used
/// only as a proxy for "which way is the probe pointing" when measuring how
/// much the capture actually moved, so the exact choice does not affect any
/// solved quantity.
const Eigen::Vector3d kProbeAxisProxy = Eigen::Vector3d::UnitX();

bool isUsable(const CalibrationSample &sample) {
    if (!sample.position.allFinite())
        return false;

    if (!sample.orientation.coeffs().allFinite())
        return false;

    return std::abs(sample.orientation.norm() - 1.0) <= 0.1;
}

bool allUsable(const std::vector<CalibrationSample> &samples) {
    for (const auto &sample : samples) {
        if (!isUsable(sample))
            return false;
    }

    return true;
}

/// A singular value this far below the largest one means the system is
/// rank-deficient in double precision. Deliberately loose: it separates "the
/// capture did not constrain this at all" from "the capture constrained it
/// poorly". Poor-but-solvable is the capture gate's job, not the solver's.
constexpr double kRankTolerance = 1e-9;

/// Ratio of largest to smallest singular value; infinity when rank-deficient.
///
/// The rank test is relative, not a comparison against zero: a degenerate
/// system produces a smallest singular value around 1e-16 rather than exactly
/// zero, which would otherwise pass as a merely ill-conditioned one and yield a
/// plausible-looking answer that the data does not support.
double conditionFrom(const Eigen::VectorXd &singularValues) {
    if (singularValues.size() == 0)
        return std::numeric_limits<double>::infinity();

    const double largest = singularValues(0);
    const double smallest = singularValues(singularValues.size() - 1);

    if (!std::isfinite(largest) || !(largest > 0.0))
        return std::numeric_limits<double>::infinity();

    if (!(smallest > largest * kRankTolerance))
        return std::numeric_limits<double>::infinity();

    return largest / smallest;
}

} // namespace

double rotationAngleDeg(const Eigen::Quaterniond &q) {
    const Eigen::Quaterniond n = q.normalized();
    // Fold to the shorter of the two equivalent rotations.
    const double w = std::min(1.0, std::abs(n.w()));

    return 2.0 * std::acos(w) * kRadToDeg;
}

Eigen::Vector3d zyxDegreesFromQuaternion(const Eigen::Quaterniond &q) {
    // Extracted from the rotation matrix directly rather than via
    // Eigen::eulerAngles, which is free to return any valid decomposition and
    // near identity picks a branch giving [180, -180, -180]. That is correct as
    // a rotation and useless as a written value: it is going into a config file
    // for a person to read, and "the tip frame is square with the sensor"
    // should not look like three half-turns.
    //
    // For R = Rz(azimuth) * Ry(elevation) * Rx(roll), matching
    // quaternionFromZyxDegrees:
    //
    //     elevation = asin(-R(2,0))
    //     azimuth   = atan2(R(1,0), R(0,0))
    //     roll      = atan2(R(2,1), R(2,2))
    //
    // which puts elevation in [-90, 90] and the other two in (-180, 180], one
    // representation per rotation, and the smallest one.
    const Eigen::Matrix3d r = q.normalized().toRotationMatrix();

    const double sinElevation = std::max(-1.0, std::min(1.0, -r(2, 0)));
    const double elevation = std::asin(sinElevation);

    double azimuth = 0.0;
    double roll = 0.0;

    // At elevation = +/-90 the azimuth and roll axes coincide and only their
    // sum or difference is determined. Pin roll to zero and put the whole
    // rotation into azimuth, rather than splitting it arbitrarily between them.
    constexpr double kGimbalTolerance = 1e-9;
    if (std::abs(r(0, 0)) < kGimbalTolerance && std::abs(r(1, 0)) < kGimbalTolerance) {
        azimuth = std::atan2(-r(0, 1), r(1, 1));
        roll = 0.0;
    } else {
        azimuth = std::atan2(r(1, 0), r(0, 0));
        roll = std::atan2(r(2, 1), r(2, 2));
    }

    return Eigen::Vector3d{azimuth * kRadToDeg, elevation * kRadToDeg, roll * kRadToDeg};
}

DiversityMetrics assessCapture(const std::vector<CalibrationSample> &samples,
                               const CaptureCriteria &criteria) {
    DiversityMetrics metrics;
    metrics.sampleCount = samples.size();

    if (samples.empty() || !allUsable(samples)) {
        metrics.guidance = "Waiting for valid pose data from the Viper.";
        return metrics;
    }

    // How wide a cone the probe axis swept: the half-angle about the mean
    // direction that still contains every observation.
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    std::vector<Eigen::Vector3d> directions;
    directions.reserve(samples.size());

    for (const auto &sample : samples) {
        const Eigen::Vector3d direction = sample.orientation.normalized() * kProbeAxisProxy;
        directions.push_back(direction);
        mean += direction;
    }

    if (mean.norm() > 1e-12) {
        mean.normalize();
        double widest = 0.0;
        for (const auto &direction : directions) {
            const double cosine = std::max(-1.0, std::min(1.0, mean.dot(direction)));
            widest = std::max(widest, std::acos(cosine));
        }
        metrics.coneHalfAngleDeg = widest * kRadToDeg;
    } else {
        // Directions canceled out entirely, which means they are spread over
        // more than a hemisphere -- ample diversity.
        metrics.coneHalfAngleDeg = 180.0;
    }

    // Condition of the stacked pivot system [R_i | -I].
    if (samples.size() >= 2) {
        Eigen::MatrixXd a(3 * samples.size(), 6);
        for (std::size_t i = 0; i < samples.size(); i++) {
            a.block<3, 3>(3 * static_cast<Eigen::Index>(i), 0) =
                    samples[i].orientation.normalized().toRotationMatrix();
            a.block<3, 3>(3 * static_cast<Eigen::Index>(i), 3) = -Eigen::Matrix3d::Identity();
        }

        Eigen::JacobiSVD<Eigen::MatrixXd> svd(a);
        metrics.conditionNumber = conditionFrom(svd.singularValues());
    }

    const bool enoughSamples = metrics.sampleCount >= criteria.minSamples;
    const bool enoughSpread = metrics.coneHalfAngleDeg >= criteria.minConeHalfAngleDeg;
    const bool wellConditioned = metrics.conditionNumber <= criteria.maxConditionNumber;

    metrics.sufficient = enoughSamples && enoughSpread && wellConditioned;

    if (!metrics.sufficient) {
        std::ostringstream guidance;
        if (!enoughSamples) {
            guidance << "Keep going: " << metrics.sampleCount << " of "
                     << criteria.minSamples << " samples. ";
        }
        if (!enoughSpread) {
            guidance << "Tilt the probe through a wider range -- currently "
                     << static_cast<int>(metrics.coneHalfAngleDeg) << " degrees of "
                     << static_cast<int>(criteria.minConeHalfAngleDeg) << " needed. ";
        }
        if (!wellConditioned && enoughSpread) {
            // Spread is adequate but the system is still poorly conditioned,
            // which happens when the motion is confined to a single plane.
            guidance << "Vary the direction of tilt as well as its amount. ";
        }
        metrics.guidance = guidance.str();
    }

    return metrics;
}

DiversityMetrics assessDirectionCapture(const std::vector<CalibrationSample> &samples,
                                        const DirectionCriteria &criteria) {
    DiversityMetrics metrics;
    metrics.sampleCount = samples.size();

    if (samples.empty() || !allUsable(samples)) {
        metrics.guidance = "Waiting for valid pose data from the Viper.";
        return metrics;
    }

    const auto direction = solveCommonDirection(samples);
    metrics.directionSeparation = direction.has_value() ? direction->separation : 0.0;

    const bool enoughSamples = metrics.sampleCount >= criteria.minSamples;
    const bool enoughSpin = metrics.directionSeparation >= criteria.minSeparation;

    metrics.sufficient = enoughSamples && enoughSpin;

    if (!metrics.sufficient) {
        std::ostringstream guidance;
        if (!enoughSamples) {
            guidance << "Keep going: " << metrics.sampleCount << " of " << criteria.minSamples
                     << " placements. ";
        }
        if (!enoughSpin) {
            // Not "rotate about its own axis": with the probe lying on a flat,
            // that reads as rolling it about its long axis, which lifts the
            // flat off the surface. Only the heading may change.
            guidance << "Turn the probe to a different heading between placements, keeping the "
                        "flat on the surface -- the orientations so far are too alike to pin the "
                        "direction down. Sliding it without turning it adds samples but no "
                        "information. ";
        }
        metrics.guidance = guidance.str();
    }

    return metrics;
}

std::optional<PivotResult> solvePointPivot(const std::vector<CalibrationSample> &samples) {
    if (samples.size() < 2 || !allUsable(samples))
        return std::nullopt;

    const auto rows = static_cast<Eigen::Index>(3 * samples.size());
    Eigen::MatrixXd a(rows, 6);
    Eigen::VectorXd b(rows);

    std::vector<Eigen::Matrix3d> rotations;
    rotations.reserve(samples.size());

    for (std::size_t i = 0; i < samples.size(); i++) {
        const Eigen::Matrix3d r = samples[i].orientation.normalized().toRotationMatrix();
        rotations.push_back(r);

        const auto row = 3 * static_cast<Eigen::Index>(i);
        a.block<3, 3>(row, 0) = r;
        a.block<3, 3>(row, 3) = -Eigen::Matrix3d::Identity();
        b.segment<3>(row) = -samples[i].position;
    }

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(a, Eigen::ComputeThinU | Eigen::ComputeThinV);
    const double condition = conditionFrom(svd.singularValues());
    if (!std::isfinite(condition))
        return std::nullopt;

    const Eigen::VectorXd solution = svd.solve(b);
    if (!solution.allFinite())
        return std::nullopt;

    PivotResult result;
    result.tipOffset = solution.head<3>();
    result.pivotPoint = solution.tail<3>();
    result.conditionNumber = condition;
    result.sampleCount = samples.size();

    // Residual is how far each predicted tip lands from the fitted pivot, then
    // rotated into the probe frame so the footprint's long and short axes can
    // be read separately.
    Eigen::Vector3d squared = Eigen::Vector3d::Zero();
    double squaredTotal = 0.0;

    for (std::size_t i = 0; i < samples.size(); i++) {
        const Eigen::Vector3d predicted = samples[i].position + rotations[i] * result.tipOffset;
        const Eigen::Vector3d error = predicted - result.pivotPoint;
        const Eigen::Vector3d probeFrameError = rotations[i].transpose() * error;

        squared += probeFrameError.cwiseProduct(probeFrameError);
        squaredTotal += error.squaredNorm();
    }

    const auto n = static_cast<double>(samples.size());
    result.residualRmsProbeFrame = (squared / n).cwiseSqrt();
    result.residualRms = std::sqrt(squaredTotal / n);

    return result;
}

std::optional<DirectionResult> solveCommonDirection(const std::vector<CalibrationSample> &samples) {
    if (samples.size() < 2 || !allUsable(samples))
        return std::nullopt;

    // Minimizing sum ||R_i v - n||^2 over unit v and unit n reduces to
    // maximizing |S v| where S is the sum of the rotations, so v is S's leading
    // right-singular vector and n is the normalized image of v.
    Eigen::Matrix3d sum = Eigen::Matrix3d::Zero();
    std::vector<Eigen::Matrix3d> rotations;
    rotations.reserve(samples.size());

    for (const auto &sample : samples) {
        const Eigen::Matrix3d r = sample.orientation.normalized().toRotationMatrix();
        rotations.push_back(r);
        sum += r;
    }

    Eigen::JacobiSVD<Eigen::Matrix3d> svd(sum, Eigen::ComputeFullU | Eigen::ComputeFullV);
    const Eigen::Vector3d v = svd.matrixV().col(0);
    const Eigen::Vector3d image = sum * v;

    if (!(image.norm() > 1e-9))
        return std::nullopt;   // every direction fits equally badly

    DirectionResult result;
    result.sensorDirection = v.normalized();
    result.worldDirection = image.normalized();
    result.sampleCount = samples.size();

    // A flat capture makes every rotation identical, so S has three equal
    // singular values and v is arbitrary. The gap between the first two says
    // how far from that degenerate case we are.
    const Eigen::Vector3d singular = svd.singularValues();
    result.separation = (singular(0) - singular(1)) / static_cast<double>(samples.size());

    double squared = 0.0;
    for (const auto &r : rotations) {
        const Eigen::Vector3d mapped = r * result.sensorDirection;
        const double cosine = std::max(-1.0, std::min(1.0, mapped.dot(result.worldDirection)));
        const double angle = std::acos(cosine);
        squared += angle * angle;
    }
    result.residualDeg = std::sqrt(squared / static_cast<double>(samples.size())) * kRadToDeg;

    return result;
}

std::optional<PlaneTranslationResult> solvePlaneTranslation(
        const std::vector<CalibrationSample> &samples, const Eigen::Vector3d &planeNormal) {
    if (samples.size() < 4 || !allUsable(samples))
        return std::nullopt;

    if (!planeNormal.allFinite() || !(planeNormal.norm() > 1e-9))
        return std::nullopt;

    const Eigen::Vector3d n = planeNormal.normalized();

    // With n fixed, n.(p_i + R_i t) = d is linear in (t, d):
    //     [ (R_i^T n)^T   -1 ] [t; d] = -n.p_i
    const auto rows = static_cast<Eigen::Index>(samples.size());
    Eigen::MatrixXd a(rows, 4);
    Eigen::VectorXd b(rows);

    std::vector<Eigen::Matrix3d> rotations;
    rotations.reserve(samples.size());

    for (std::size_t i = 0; i < samples.size(); i++) {
        const Eigen::Matrix3d r = samples[i].orientation.normalized().toRotationMatrix();
        rotations.push_back(r);

        const auto row = static_cast<Eigen::Index>(i);
        a.block<1, 3>(row, 0) = (r.transpose() * n).transpose();
        a(row, 3) = -1.0;
        b(row) = -n.dot(samples[i].position);
    }

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(a, Eigen::ComputeThinU | Eigen::ComputeThinV);
    const double condition = conditionFrom(svd.singularValues());
    if (!std::isfinite(condition))
        return std::nullopt;

    const Eigen::VectorXd solution = svd.solve(b);
    if (!solution.allFinite())
        return std::nullopt;

    PlaneTranslationResult result;
    result.tipOffset = solution.head<3>();
    result.planeOffset = solution(3);
    result.conditionNumber = condition;
    result.sampleCount = samples.size();

    double squared = 0.0;
    for (std::size_t i = 0; i < samples.size(); i++) {
        const Eigen::Vector3d predicted = samples[i].position + rotations[i] * result.tipOffset;
        const double distance = n.dot(predicted) - result.planeOffset;
        squared += distance * distance;
    }
    result.residualRms = std::sqrt(squared / static_cast<double>(samples.size()));

    return result;
}

std::optional<Eigen::Quaterniond> tipRotationFromAxes(const Eigen::Vector3d &faceNormalSensor,
                                                      const Eigen::Vector3d &inPlaneReferenceSensor,
                                                      double rollOffsetDeg) {
    if (!faceNormalSensor.allFinite() || !inPlaneReferenceSensor.allFinite())
        return std::nullopt;

    if (!std::isfinite(rollOffsetDeg))
        return std::nullopt;

    if (!(faceNormalSensor.norm() > 1e-9) || !(inPlaneReferenceSensor.norm() > 1e-9))
        return std::nullopt;

    const Eigen::Vector3d x = faceNormalSensor.normalized();

    // Orthogonalize the reference against the face normal, so the second
    // capture does not have to be perpendicular to the first -- only
    // non-parallel.
    Eigen::Vector3d y = inPlaneReferenceSensor.normalized();
    y -= x * x.dot(y);

    if (!(y.norm() > 1e-6))
        return std::nullopt;   // the two directions are parallel; z undetermined

    y.normalize();

    // Swing the in-plane reference onto the footprint axis. Needed whenever the
    // captured direction is not itself the footprint axis -- a second housing
    // flat, say -- and the offset between them is known from the design.
    if (rollOffsetDeg != 0.0)
        y = Eigen::AngleAxisd(rollOffsetDeg * kDegToRad, x) * y;

    const Eigen::Vector3d z = x.cross(y);

    Eigen::Matrix3d rotation;
    rotation.col(0) = x;
    rotation.col(1) = y;
    rotation.col(2) = z;

    Eigen::Quaterniond q{rotation};
    q.normalize();

    return q;
}

} // namespace mdx
