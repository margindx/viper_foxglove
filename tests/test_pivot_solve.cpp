//
// Unit tests for the probe tip calibration solvers.
//
// Every case builds synthetic poses from a known ground-truth offset and
// rotation, so the solvers can be checked against an exact answer without
// hardware. The negative cases matter as much as the positive ones: a
// calibration that returns a confident wrong number is worse than one that
// refuses.
//

#include <catch2/catch_test_macros.hpp>

#include "PivotSolve.hpp"

using namespace mdx;

namespace {

constexpr double kDeg = 3.14159265358979323846 / 180.0;

CalibrationSample makeSample(const Eigen::Vector3d &position, const Eigen::Quaterniond &orientation) {
    CalibrationSample s;
    s.position = position;
    s.orientation = orientation.normalized();
    return s;
}

/// Deterministic pseudo-random angles, so failures reproduce exactly.
double angleFor(int i, double scale, int seed) {
    const double x = std::sin(static_cast<double>(i * 7919 + seed * 104729)) * 43758.5453;
    return (x - std::floor(x) - 0.5) * scale;
}

/// Poses of a probe whose tip is held at `pivot` while it is swept through
/// orientations spanning roughly +/- `spreadDeg` about two axes.
std::vector<CalibrationSample> makePivotCapture(const Eigen::Vector3d &tipOffset,
                                                const Eigen::Vector3d &pivot,
                                                int count,
                                                double spreadDeg,
                                                int seed = 1) {
    std::vector<CalibrationSample> samples;
    samples.reserve(count);

    for (int i = 0; i < count; i++) {
        const Eigen::Quaterniond q =
                Eigen::Quaterniond{Eigen::AngleAxisd(angleFor(i, spreadDeg * kDeg, seed),
                                                     Eigen::Vector3d::UnitY())} *
                Eigen::Quaterniond{Eigen::AngleAxisd(angleFor(i, spreadDeg * kDeg, seed + 3),
                                                     Eigen::Vector3d::UnitZ())} *
                Eigen::Quaterniond{Eigen::AngleAxisd(angleFor(i, spreadDeg * kDeg, seed + 7),
                                                     Eigen::Vector3d::UnitX())};

        // The sensor sits wherever it must for the tip to be at the pivot.
        samples.push_back(makeSample(pivot - q.normalized() * tipOffset, q));
    }

    return samples;
}

/// Poses of a probe whose face is laid flat, so `faceNormalSensor` maps onto
/// `surfaceNormal` every time, with a free spin about the surface normal and a
/// free position on the surface.
std::vector<CalibrationSample> makeFlatCapture(const Eigen::Vector3d &faceNormalSensor,
                                               const Eigen::Vector3d &surfaceNormal,
                                               const Eigen::Vector3d &tipOffset,
                                               double planeOffset,
                                               int count,
                                               int seed = 11) {
    const Eigen::Quaterniond align =
            Eigen::Quaterniond::FromTwoVectors(faceNormalSensor.normalized(),
                                               surfaceNormal.normalized());

    // Two directions spanning the surface, for placing the tip around on it.
    Eigen::Vector3d spanA = surfaceNormal.unitOrthogonal();
    Eigen::Vector3d spanB = surfaceNormal.normalized().cross(spanA);

    std::vector<CalibrationSample> samples;
    samples.reserve(count);

    for (int i = 0; i < count; i++) {
        const Eigen::Quaterniond spin{
                Eigen::AngleAxisd(angleFor(i, 360.0 * kDeg, seed), surfaceNormal.normalized())};
        const Eigen::Quaterniond q = (spin * align).normalized();

        // A tip location somewhere on the plane.
        const Eigen::Vector3d tip = surfaceNormal.normalized() * planeOffset +
                                    spanA * angleFor(i, 0.2, seed + 1) +
                                    spanB * angleFor(i, 0.2, seed + 2);

        samples.push_back(makeSample(tip - q * tipOffset, q));
    }

    return samples;
}

bool sameDirectionUpToSign(const Eigen::Vector3d &a, const Eigen::Vector3d &b, double tol = 1e-6) {
    return std::abs(a.normalized().dot(b.normalized())) > 1.0 - tol;
}

} // namespace

TEST_CASE("point pivot recovers a known tip offset", "[pivot]") {
    const Eigen::Vector3d tipOffset{0.157, 0.004, -0.002};
    const Eigen::Vector3d pivot{0.3, -0.1, 0.25};

    const auto samples = makePivotCapture(tipOffset, pivot, 60, 40.0);
    const auto result = solvePointPivot(samples);

    REQUIRE(result.has_value());
    REQUIRE((result->tipOffset - tipOffset).norm() < 1e-9);
    REQUIRE((result->pivotPoint - pivot).norm() < 1e-9);
    REQUIRE(result->residualRms < 1e-9);
    REQUIRE(result->sampleCount == samples.size());
}

TEST_CASE("point pivot residual reflects injected error", "[pivot]") {
    const Eigen::Vector3d tipOffset{0.157, 0.0, 0.0};
    const Eigen::Vector3d pivot{0.1, 0.2, 0.3};

    auto samples = makePivotCapture(tipOffset, pivot, 60, 40.0);

    // Displace every sensor position by 1 mm along a fixed tracker axis. The
    // pivot cannot absorb that, so it must show up in the residual.
    for (auto &sample : samples)
        sample.position += Eigen::Vector3d{0.001, 0.0, 0.0} * (sample.position.x() > 0.0 ? 1.0 : -1.0);

    const auto result = solvePointPivot(samples);

    REQUIRE(result.has_value());
    REQUIRE(result->residualRms > 0.0);
    REQUIRE(result->residualRms < 0.01);
}

TEST_CASE("point pivot refuses a stationary capture", "[pivot][negative]") {
    // Every pose identical: t and the pivot point are perfectly confounded,
    // since any offset can be absorbed by moving the pivot.
    const Eigen::Quaterniond q{Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ())};
    std::vector<CalibrationSample> samples;
    for (int i = 0; i < 50; i++)
        samples.push_back(makeSample(Eigen::Vector3d{0.1, 0.2, 0.3}, q));

    REQUIRE_FALSE(solvePointPivot(samples).has_value());
}

TEST_CASE("point pivot rejects unusable input", "[pivot][negative]") {
    SECTION("too few samples") {
        REQUIRE_FALSE(solvePointPivot({}).has_value());
        REQUIRE_FALSE(solvePointPivot({makeSample({0, 0, 0}, Eigen::Quaterniond::Identity())})
                              .has_value());
    }

    SECTION("a non-finite sample poisons the capture") {
        auto samples = makePivotCapture({0.157, 0, 0}, {0.1, 0.1, 0.1}, 30, 40.0);
        samples[5].position.y() = std::numeric_limits<double>::quiet_NaN();
        REQUIRE_FALSE(solvePointPivot(samples).has_value());
    }
}

TEST_CASE("capture assessment guides an inadequate sweep", "[diversity]") {
    const Eigen::Vector3d tipOffset{0.157, 0, 0};
    const Eigen::Vector3d pivot{0.1, 0.1, 0.1};

    SECTION("a wide, varied sweep is sufficient") {
        const auto samples = makePivotCapture(tipOffset, pivot, 80, 60.0);
        const auto metrics = assessCapture(samples);

        REQUIRE(metrics.sufficient);
        REQUIRE(metrics.guidance.empty());
        REQUIRE(metrics.coneHalfAngleDeg > 20.0);
    }

    SECTION("too few samples is reported as such") {
        const auto samples = makePivotCapture(tipOffset, pivot, 5, 60.0);
        const auto metrics = assessCapture(samples);

        REQUIRE_FALSE(metrics.sufficient);
        REQUIRE(metrics.guidance.find("samples") != std::string::npos);
    }

    SECTION("a narrow sweep asks for more tilt") {
        const auto samples = makePivotCapture(tipOffset, pivot, 80, 2.0);
        const auto metrics = assessCapture(samples);

        REQUIRE_FALSE(metrics.sufficient);
        REQUIRE(metrics.guidance.find("Tilt") != std::string::npos);
    }

    SECTION("no samples at all") {
        const auto metrics = assessCapture({});
        REQUIRE_FALSE(metrics.sufficient);
        REQUIRE(metrics.sampleCount == 0);
    }
}

// The case that reached the bench: an operator rocked the probe back and forth
// at one heading and was told the capture was sufficient while still on that
// first rock. Cone spread cannot see it -- rocking 25 degrees in one plane
// reports the same 25 degrees a proper conical sweep does -- so the condition
// number is the only thing standing between a planar capture and a confident
// wrong answer, and at the original bound of 100 it let one through.
TEST_CASE("rocking at a single heading is refused", "[diversity][negative]") {
    const Eigen::Vector3d tipOffset{0.194, 0, 0};
    const Eigen::Vector3d pivot{0.2, -0.1, 0.4};

    // Tilts about one axis only, with a couple of degrees of hand wobble --
    // without the wobble the system is exactly singular and any bound rejects
    // it. The wobble is what made this pass.
    std::vector<CalibrationSample> planar;
    for (int i = 0; i < 120; i++) {
        const double rock = 25.0 * kDeg * std::sin(i * 0.11);
        const double wobbleA = 2.0 * kDeg * std::sin(i * 0.37 + 1.0);
        const double wobbleB = 2.0 * kDeg * std::sin(i * 0.53 + 2.0);
        const Eigen::Quaterniond q =
                Eigen::Quaterniond{Eigen::AngleAxisd(rock, Eigen::Vector3d::UnitY())} *
                Eigen::Quaterniond{Eigen::AngleAxisd(wobbleA, Eigen::Vector3d::UnitZ())} *
                Eigen::Quaterniond{Eigen::AngleAxisd(wobbleB, Eigen::Vector3d::UnitX())};
        planar.push_back(makeSample(pivot - q.normalized() * tipOffset, q));
    }

    const auto metrics = assessCapture(planar);

    // Everything the operator can see says the capture is going well.
    REQUIRE(metrics.sampleCount > 40);
    REQUIRE(metrics.coneHalfAngleDeg > 20.0);

    REQUIRE_FALSE(metrics.sufficient);
    REQUIRE(metrics.conditionNumber > 20.0);

    // The tilts are wide but all in one direction, and the guidance says to
    // turn to a new heading rather than to tilt further.
    REQUIRE(metrics.secondarySpreadDeg < 5.0);
    REQUIRE(metrics.guidance.find("heading") != std::string::npos);

    // A second heading is enough to fix it, and is what the guidance asks for.
    std::vector<CalibrationSample> varied = planar;
    for (int i = 0; i < 120; i++) {
        const double rock = 25.0 * kDeg * std::sin(i * 0.11);
        const Eigen::Quaterniond q =
                Eigen::Quaterniond{Eigen::AngleAxisd(90.0 * kDeg, Eigen::Vector3d::UnitX())} *
                Eigen::Quaterniond{Eigen::AngleAxisd(rock, Eigen::Vector3d::UnitY())};
        varied.push_back(makeSample(pivot - q.normalized() * tipOffset, q));
    }

    const auto better = assessCapture(varied);
    REQUIRE(better.secondarySpreadDeg > 5.0);
    REQUIRE(better.conditionNumber < metrics.conditionNumber);
    REQUIRE(better.sufficient);
}

TEST_CASE("direction capture is judged on spin, not tilt", "[diversity][direction]") {
    const Eigen::Vector3d faceNormalSensor = Eigen::Vector3d::UnitX();
    const Eigen::Vector3d surfaceNormal = Eigen::Vector3d::UnitZ();

    SECTION("varied spin is sufficient") {
        const auto samples =
                makeFlatCapture(faceNormalSensor, surfaceNormal, {0.157, 0, 0}, 0.5, 40);
        const auto metrics = assessDirectionCapture(samples);

        REQUIRE(metrics.sufficient);
        REQUIRE(metrics.guidance.empty());
    }

    SECTION("a correct flat capture must not be told to tilt") {
        // The probe axis is fixed by design here. Judging this motion by cone
        // spread would demand exactly the wrong action from the operator.
        const auto samples =
                makeFlatCapture(faceNormalSensor, surfaceNormal, {0.157, 0, 0}, 0.5, 40);

        REQUIRE(assessCapture(samples).coneHalfAngleDeg < 1e-6);
        REQUIRE(assessDirectionCapture(samples).guidance.find("Tilt") == std::string::npos);
    }

    SECTION("identical placements ask for rotation") {
        const Eigen::Quaterniond q{Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitX())};
        std::vector<CalibrationSample> samples;
        for (int i = 0; i < 40; i++)
            samples.push_back(makeSample(Eigen::Vector3d{0.1, 0.0, 0.0}, q));

        const auto metrics = assessDirectionCapture(samples);

        REQUIRE_FALSE(metrics.sufficient);
        REQUIRE(metrics.guidance.find("heading") != std::string::npos);

        // Must not read as an instruction to roll the probe about its long
        // axis, which would take the flat off the surface.
        REQUIRE(metrics.guidance.find("own axis") == std::string::npos);
    }
}

TEST_CASE("common direction recovers the face normal", "[direction]") {
    const Eigen::Vector3d faceNormalSensor = Eigen::Vector3d{1.0, 0.1, -0.05}.normalized();
    const Eigen::Vector3d surfaceNormal = Eigen::Vector3d{0.0, 0.0, 1.0};

    const auto samples =
            makeFlatCapture(faceNormalSensor, surfaceNormal, {0.157, 0, 0}, 0.5, 40);
    const auto result = solveCommonDirection(samples);

    REQUIRE(result.has_value());
    REQUIRE(sameDirectionUpToSign(result->sensorDirection, faceNormalSensor));
    REQUIRE(sameDirectionUpToSign(result->worldDirection, surfaceNormal));
    REQUIRE(result->residualDeg < 1e-6);
    REQUIRE(result->separation > 0.1);
}

TEST_CASE("common direction is degenerate without varied spin", "[direction][negative]") {
    // Identical placements: every direction fits equally well, so the solve
    // must not claim to have found one.
    const Eigen::Quaterniond q{Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitX())};
    std::vector<CalibrationSample> samples;
    for (int i = 0; i < 30; i++)
        samples.push_back(makeSample(Eigen::Vector3d{0.1, 0.0, 0.0}, q));

    const auto result = solveCommonDirection(samples);

    REQUIRE(result.has_value());
    REQUIRE(result->separation < 1e-9);   // caller must treat this as unusable
}

TEST_CASE("flat placements cannot determine the translation", "[plane][regression]") {
    // The confounding that rules out using flat placements as a second opinion
    // on the tip offset: with the face flat every time, R_i^T n is constant, so
    // v.t is indistinguishable from the plane's position. The solver must
    // refuse rather than return the minimum-norm answer, which would look
    // plausible and be wrong.
    const Eigen::Vector3d faceNormalSensor = Eigen::Vector3d::UnitX();
    const Eigen::Vector3d surfaceNormal = Eigen::Vector3d::UnitZ();

    const auto samples =
            makeFlatCapture(faceNormalSensor, surfaceNormal, {0.157, 0, 0}, 0.5, 40);

    REQUIRE_FALSE(solvePlaneTranslation(samples, surfaceNormal).has_value());
}

TEST_CASE("plane translation recovers the offset from rocking data", "[plane]") {
    // The rocking motion does vary R_i^T n, which separates t from the plane
    // position. Build a capture where the tip genuinely stays on the plane.
    const Eigen::Vector3d tipOffset{0.157, 0.003, -0.001};
    const Eigen::Vector3d surfaceNormal = Eigen::Vector3d{0.0, 0.0, 1.0};
    const double planeOffset = 0.42;

    std::vector<CalibrationSample> samples;
    for (int i = 0; i < 80; i++) {
        const Eigen::Quaterniond q =
                Eigen::Quaterniond{Eigen::AngleAxisd(angleFor(i, 50.0 * kDeg, 5),
                                                     Eigen::Vector3d::UnitY())} *
                Eigen::Quaterniond{Eigen::AngleAxisd(angleFor(i, 50.0 * kDeg, 9),
                                                     Eigen::Vector3d::UnitX())} *
                Eigen::Quaterniond{Eigen::AngleAxisd(angleFor(i, 360.0 * kDeg, 13),
                                                     Eigen::Vector3d::UnitZ())};

        // Tip somewhere on the plane, sliding around as the operator rocks.
        const Eigen::Vector3d tip{angleFor(i, 0.1, 21), angleFor(i, 0.1, 22), planeOffset};
        samples.push_back(makeSample(tip - q.normalized() * tipOffset, q));
    }

    const auto result = solvePlaneTranslation(samples, surfaceNormal);

    REQUIRE(result.has_value());
    REQUIRE((result->tipOffset - tipOffset).norm() < 1e-8);
    REQUIRE(std::abs(result->planeOffset - planeOffset) < 1e-8);
    REQUIRE(result->residualRms < 1e-9);
}

TEST_CASE("plane translation rejects a degenerate normal", "[plane][negative]") {
    const auto samples = makePivotCapture({0.157, 0, 0}, {0.1, 0.1, 0.1}, 40, 40.0);

    REQUIRE_FALSE(solvePlaneTranslation(samples, Eigen::Vector3d::Zero()).has_value());
    REQUIRE_FALSE(solvePlaneTranslation({}, Eigen::Vector3d::UnitZ()).has_value());
}

TEST_CASE("tip rotation is built from the two calibrated directions", "[rotation]") {
    SECTION("canonical axes give identity") {
        const auto q = tipRotationFromAxes(Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY());

        REQUIRE(q.has_value());
        REQUIRE(rotationAngleDeg(*q) < 1e-9);
    }

    SECTION("the long axis is orthogonalized against the face normal") {
        // A long axis measured 10 degrees out of perpendicular must still give
        // the same rotation as a perfect one.
        const Eigen::Vector3d skewed =
                (Eigen::Vector3d::UnitY() + 0.176 * Eigen::Vector3d::UnitX()).normalized();

        const auto exact = tipRotationFromAxes(Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY());
        const auto skew = tipRotationFromAxes(Eigen::Vector3d::UnitX(), skewed);

        REQUIRE(exact.has_value());
        REQUIRE(skew.has_value());
        REQUIRE(std::abs(exact->coeffs().dot(skew->coeffs())) > 1.0 - 1e-9);
    }

    SECTION("a real mounting rotation round-trips") {
        const Eigen::Quaterniond mount = quaternionFromZyxDegrees(30.0, 15.0, -20.0);
        const Eigen::Vector3d face = mount * Eigen::Vector3d::UnitX();
        const Eigen::Vector3d longAxis = mount * Eigen::Vector3d::UnitY();

        const auto q = tipRotationFromAxes(face, longAxis);

        REQUIRE(q.has_value());
        REQUIRE(std::abs(q->coeffs().dot(mount.coeffs())) > 1.0 - 1e-9);
    }

    SECTION("a roll offset swings the footprint axis about the probe axis") {
        // Step 3 recovers a second housing flat's normal, not the footprint
        // axis; the CAD angle between them is applied here. A 90 degree offset
        // must put +y where +z was.
        const auto plain = tipRotationFromAxes(Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY());
        const auto rolled =
                tipRotationFromAxes(Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY(), 90.0);

        REQUIRE(plain.has_value());
        REQUIRE(rolled.has_value());

        const Eigen::Vector3d yPlain = *plain * Eigen::Vector3d::UnitY();
        const Eigen::Vector3d yRolled = *rolled * Eigen::Vector3d::UnitY();

        REQUIRE(yPlain.isApprox(Eigen::Vector3d::UnitY(), 1e-9));
        REQUIRE(yRolled.isApprox(Eigen::Vector3d::UnitZ(), 1e-9));

        // The probe axis is untouched: roll cannot move the tip direction.
        REQUIRE((*rolled * Eigen::Vector3d::UnitX()).isApprox(Eigen::Vector3d::UnitX(), 1e-9));
    }

    SECTION("a zero roll offset changes nothing") {
        const auto a = tipRotationFromAxes(Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY());
        const auto b = tipRotationFromAxes(Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY(), 0.0);

        REQUIRE(a.has_value());
        REQUIRE(b.has_value());
        REQUIRE(std::abs(a->coeffs().dot(b->coeffs())) > 1.0 - 1e-12);
    }

    SECTION("a non-finite roll offset is refused") {
        REQUIRE_FALSE(tipRotationFromAxes(Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY(),
                                          std::numeric_limits<double>::quiet_NaN())
                              .has_value());
    }

    SECTION("parallel directions leave the third axis undetermined") {
        REQUIRE_FALSE(
                tipRotationFromAxes(Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitX()).has_value());
        REQUIRE_FALSE(tipRotationFromAxes(Eigen::Vector3d::UnitX(),
                                          -Eigen::Vector3d::UnitX())
                              .has_value());
    }

    SECTION("degenerate input") {
        REQUIRE_FALSE(
                tipRotationFromAxes(Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitY()).has_value());
    }
}

TEST_CASE("rotationAngleDeg measures distance from identity", "[rotation]") {
    REQUIRE(rotationAngleDeg(Eigen::Quaterniond::Identity()) < 1e-12);

    const Eigen::Quaterniond ninety{Eigen::AngleAxisd(90.0 * kDeg, Eigen::Vector3d::UnitZ())};
    REQUIRE(std::abs(rotationAngleDeg(ninety) - 90.0) < 1e-9);

    // The antipodal representation is the same rotation, so the same angle.
    const Eigen::Quaterniond negated{-ninety.w(), -ninety.x(), -ninety.y(), -ninety.z()};
    REQUIRE(std::abs(rotationAngleDeg(negated) - 90.0) < 1e-9);
}

TEST_CASE("zyxDegreesFromQuaternion returns the canonical triple", "[rotation][regression]") {
    // The value goes into a config file for a person to read, so a rotation
    // near identity has to be written as near zero. Eigen::eulerAngles is free
    // to return any valid decomposition and near identity chose one reading
    // [180, -180, -180] -- correct, and certain to be "corrected" by whoever
    // found it in their config.
    SECTION("exact identity") {
        const Eigen::Vector3d zyx = zyxDegreesFromQuaternion(Eigen::Quaterniond::Identity());
        REQUIRE(zyx.norm() < 1e-9);
    }

    SECTION("identity reached through the axis construction") {
        // This is the path that produced [180, -180, -180]: a rotation built
        // from cross products, identity only to within rounding.
        const auto built = tipRotationFromAxes(Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY());
        REQUIRE(built.has_value());
        REQUIRE(rotationAngleDeg(*built) < 1e-9);
        REQUIRE(zyxDegreesFromQuaternion(*built).norm() < 1e-6);
    }

    SECTION("the antipodal quaternion gives the same triple") {
        const auto q = quaternionFromZyxDegrees(20.0, -10.0, 35.0);
        const Eigen::Quaterniond negated{-q.w(), -q.x(), -q.y(), -q.z()};

        REQUIRE((zyxDegreesFromQuaternion(q) - zyxDegreesFromQuaternion(negated)).norm() < 1e-9);
    }

    SECTION("elevation stays within +/-90 and the others within +/-180") {
        for (double az : {-170.0, -30.0, 0.0, 45.0, 179.0}) {
            for (double el : {-80.0, -20.0, 0.0, 20.0, 80.0}) {
                for (double roll : {-160.0, 0.0, 120.0}) {
                    const auto zyx =
                            zyxDegreesFromQuaternion(quaternionFromZyxDegrees(az, el, roll));

                    REQUIRE(std::abs(zyx.y()) <= 90.0 + 1e-9);
                    REQUIRE(std::abs(zyx.x()) <= 180.0 + 1e-9);
                    REQUIRE(std::abs(zyx.z()) <= 180.0 + 1e-9);
                }
            }
        }
    }

    SECTION("gimbal lock puts the rotation in azimuth rather than splitting it") {
        const auto q = quaternionFromZyxDegrees(0.0, 90.0, 0.0);
        const auto zyx = zyxDegreesFromQuaternion(q);

        REQUIRE(std::abs(zyx.y() - 90.0) < 1e-6);
        REQUIRE(std::abs(zyx.z()) < 1e-6);   // roll pinned, not shared with azimuth
    }
}

TEST_CASE("zyxDegreesFromQuaternion inverts quaternionFromZyxDegrees", "[rotation]") {
    // Round-trip through the representation probe_profiles actually stores, so
    // what the tool writes reproduces what it solved.
    const std::vector<Eigen::Vector3d> cases{
            {0.0, 0.0, 0.0},
            {30.0, 15.0, -20.0},
            {-95.0, 8.0, 40.0},
            {12.5, -33.0, 87.0},
    };

    for (const auto &zyx : cases) {
        const Eigen::Quaterniond q = quaternionFromZyxDegrees(zyx.x(), zyx.y(), zyx.z());
        const Eigen::Vector3d recovered = zyxDegreesFromQuaternion(q);
        const Eigen::Quaterniond again =
                quaternionFromZyxDegrees(recovered.x(), recovered.y(), recovered.z());

        // Compare rotations, not angle triples: Euler decompositions are not
        // unique, so a different triple denoting the same rotation is correct.
        REQUIRE(std::abs(q.coeffs().dot(again.coeffs())) > 1.0 - 1e-9);
    }
}
