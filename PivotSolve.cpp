//
// See PivotSolve.hpp.
//

#include "PivotSolve.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace mdx {

namespace {

constexpr double kPi = 3.14159265358979323846;
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

std::optional<Eigen::Vector3d> estimateUpFromPivot(const std::vector<CalibrationSample> &samples,
                                                   const Eigen::Vector3d &tipOffset) {
    if (samples.empty() || !allUsable(samples))
        return std::nullopt;

    if (!tipOffset.allFinite() || !(tipOffset.norm() > 1e-9))
        return std::nullopt;

    const Eigen::Vector3d offsetDirection = tipOffset.normalized();

    // p_tip = p_sensor + R_i*t, so -R_i*t runs from the tip up to the sensor.
    Eigen::Vector3d sum = Eigen::Vector3d::Zero();
    for (const auto &sample : samples)
        sum -= sample.orientation.normalized() * offsetDirection;

    if (!(sum.norm() > 1e-9))
        return std::nullopt;   // the sweep spanned more than a hemisphere

    return sum.normalized();
}

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

        // How much of that cone was swept in more than one direction. Project
        // each observation into the tangent plane at the mean, where its length
        // is the sine of the tilt away from the mean, and take the smaller
        // principal spread of that 2-D scatter. A single-heading rock lays the
        // scatter along one line, leaving this near zero however wide the rock.
        Eigen::Matrix3d scatter = Eigen::Matrix3d::Zero();
        for (const auto &direction : directions) {
            const Eigen::Vector3d tangential = direction - mean.dot(direction) * mean;
            scatter += tangential * tangential.transpose();
        }
        scatter /= static_cast<double>(directions.size());

        // Self-adjoint eigenvalues come out ascending; the smallest is along the
        // mean itself and carries nothing, so the secondary spread is the middle.
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen(scatter);
        const double secondary = std::sqrt(std::max(0.0, eigen.eigenvalues()(1)));
        metrics.secondarySpreadDeg = std::asin(std::min(1.0, secondary)) * kRadToDeg;
    } else {
        // Directions canceled out entirely, which means they are spread over
        // more than a hemisphere -- ample diversity.
        metrics.coneHalfAngleDeg = 180.0;
        metrics.secondarySpreadDeg = 90.0;
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
            // Tilted far enough, but the solve is still weak. Overwhelmingly
            // this is rocking at one heading: the tilts lie in a single plane,
            // leaving the offset along the rocking axis undetermined. Name the
            // motion that fixes it rather than the geometry that broke.
            guidance << "Tilted far enough, but every tilt so far is in the same "
                     << "direction (" << std::fixed << std::setprecision(1)
                     << metrics.secondarySpreadDeg
                     << " deg off-axis). Stand the probe up, turn it to a new "
                     << "heading, and rock again. ";
        }
        metrics.guidance = guidance.str();
    }

    return metrics;
}

namespace {

/// How far the probe has actually been turned about the spin axis.
///
/// The recovered direction is that axis: the probe lies on a flat and only its
/// heading changes, so the surface normal is what stays fixed. Mapping a body
/// vector perpendicular to it into the plane of the surface turns each sample
/// into a heading, and the answer is the smallest arc containing them all --
/// found as 360 minus the widest gap, which is what makes it correct across the
/// wrap-around and for headings visited out of order.
double turnRange(const std::vector<CalibrationSample> &samples, const DirectionResult &direction) {
    if (samples.size() < 2)
        return 0.0;

    const Eigen::Vector3d axis = direction.worldDirection.normalized();
    const Eigen::Vector3d bodyAxis = direction.sensorDirection.normalized();

    // Any body vector off the axis will do; take the least-aligned basis vector
    // so the perpendicular part is never degenerate.
    Eigen::Vector3d seed = Eigen::Vector3d::UnitX();
    if (std::abs(bodyAxis.x()) > std::abs(bodyAxis.y()))
        seed = std::abs(bodyAxis.y()) > std::abs(bodyAxis.z()) ? Eigen::Vector3d::UnitZ()
                                                               : Eigen::Vector3d::UnitY();
    const Eigen::Vector3d bodyRef = (seed - seed.dot(bodyAxis) * bodyAxis).normalized();

    // A fixed 2-D frame in the plane of the surface, to measure headings in.
    Eigen::Vector3d planeX = Eigen::Vector3d::UnitX() - Eigen::Vector3d::UnitX().dot(axis) * axis;
    if (planeX.norm() < 1e-9)
        planeX = Eigen::Vector3d::UnitY() - Eigen::Vector3d::UnitY().dot(axis) * axis;
    planeX.normalize();
    const Eigen::Vector3d planeY = axis.cross(planeX);

    std::vector<double> headings;
    headings.reserve(samples.size());
    for (const auto &sample : samples) {
        const Eigen::Vector3d mapped = sample.orientation.normalized() * bodyRef;
        const Eigen::Vector3d inPlane = mapped - mapped.dot(axis) * axis;
        if (inPlane.norm() < 1e-9)
            continue;   // pointing straight up the axis: no heading to read
        headings.push_back(std::atan2(inPlane.dot(planeY), inPlane.dot(planeX)));
    }

    if (headings.size() < 2)
        return 0.0;

    std::sort(headings.begin(), headings.end());

    double widestGap = 2.0 * kPi - (headings.back() - headings.front());   // across the wrap
    for (std::size_t i = 1; i < headings.size(); i++)
        widestGap = std::max(widestGap, headings[i] - headings[i - 1]);

    return std::max(0.0, (2.0 * kPi - widestGap)) * kRadToDeg;
}

} // namespace

double turnAngleForSeparationDeg(double separation) {
    if (separation <= 0.0)
        return 0.0;
    if (separation >= 1.0)
        return 360.0;

    // 1 - |sinc(d/2)| rises monotonically from 0 to 1 over (0, 2pi], so plain
    // bisection is enough and cannot pick the wrong branch.
    double low = 0.0;
    double high = 2.0 * kPi;
    for (int i = 0; i < 60; i++) {
        const double mid = 0.5 * (low + high);
        const double half = mid / 2.0;
        const double sinc = half < 1e-12 ? 1.0 : std::sin(half) / half;
        if (1.0 - std::abs(sinc) < separation)
            low = mid;
        else
            high = mid;
    }

    return 0.5 * (low + high) * kRadToDeg;
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
    if (direction.has_value())
        metrics.turnRangeDeg = turnRange(samples, *direction);

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
            // In degrees, because the separation itself is a poor progress
            // bar: it goes as roughly the square of the turn, so it barely
            // moves over the first 60 degrees and a capture that is going fine
            // reads as stuck.
            guidance << "Turned " << static_cast<int>(metrics.turnRangeDeg) << " deg of about "
                     << static_cast<int>(turnAngleForSeparationDeg(criteria.minSeparation))
                     << " needed. Turn the probe to a further heading, keeping the flat on the "
                        "surface -- sliding it without turning it adds samples but no "
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

double planeCheckNormalSensitivity(const std::vector<CalibrationSample> &samples,
                                   const Eigen::Vector3d &normal, double normalErrorDeg,
                                   const Eigen::Vector3d &reference) {
    if (samples.empty() || normalErrorDeg <= 0.0 || normal.norm() < 1e-9)
        return 0.0;

    const Eigen::Vector3d axis = normal.normalized();

    // A basis for the tilt directions. Which way the normal is wrong is not
    // known, so a ring is walked and the worst case taken -- the question being
    // answered is whether the normal's error *could* account for the gap, not
    // whether one particular tilt does.
    Eigen::Vector3d u = axis.cross(Eigen::Vector3d::UnitX());
    if (u.norm() < 1e-6)
        u = axis.cross(Eigen::Vector3d::UnitY());
    u.normalize();
    const Eigen::Vector3d v = axis.cross(u);

    const double delta = normalErrorDeg * kDegToRad;
    constexpr int kDirections = 8;

    double worst = 0.0;
    for (int i = 0; i < kDirections; i++) {
        const double theta = 2.0 * kPi * static_cast<double>(i) / kDirections;
        const Eigen::Vector3d tilt = std::cos(theta) * u + std::sin(theta) * v;
        const Eigen::Vector3d perturbed = (axis * std::cos(delta) + tilt * std::sin(delta)).normalized();

        if (const auto solved = solvePlaneTranslation(samples, perturbed))
            worst = std::max(worst, (solved->tipOffset - reference).norm());
    }

    return worst;
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
