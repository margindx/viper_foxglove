//
// Unit tests for the probe stability monitor.
//
// The question this module answers -- did the sensor move inside the probe --
// is one where a false negative is expensive and a false positive is merely
// annoying, so the cases below weigh the noise floor as heavily as the signal.
// A monitor that calls sensor noise "movement" would send someone to re-mount a
// perfectly good probe.
//

#include <catch2/catch_test_macros.hpp>

#include "StabilityMonitor.hpp"

using namespace mdx;

namespace {

constexpr double kDeg = 3.14159265358979323846 / 180.0;

Pose makePose(const Eigen::Vector3d &position, const Eigen::Quaterniond &orientation) {
    Pose p;
    p.position = position;
    p.orientation = orientation.normalized();
    return p;
}

/// Deterministic wobble, so a failure reproduces exactly.
double jitter(int i, int axis) {
    const double x = std::sin(static_cast<double>(i * 7919 + axis * 104729)) * 43758.5453;
    return (x - std::floor(x)) - 0.5;
}

Pose noisyPose(const Eigen::Vector3d &position, const Eigen::Quaterniond &orientation, int i,
               double positionScale, double angleScaleDeg) {
    const Eigen::Vector3d offset{jitter(i, 1) * positionScale, jitter(i, 2) * positionScale,
                                 jitter(i, 3) * positionScale};
    const Eigen::Vector3d axis =
            Eigen::Vector3d{jitter(i, 4), jitter(i, 5), jitter(i, 6) + 1e-6}.normalized();
    const Eigen::Quaterniond wobble{Eigen::AngleAxisd(jitter(i, 7) * angleScaleDeg * kDeg, axis)};

    return makePose(position + offset, wobble * orientation);
}

} // namespace

TEST_CASE("a baseline needs more than one sample", "[stability]") {
    BaselineAccumulator accumulator;
    REQUIRE_FALSE(accumulator.result().has_value());

    accumulator.add(makePose({0.1, 0.2, 0.3}, Eigen::Quaterniond::Identity()));

    // One sample has no spread, and a noise floor of zero would make every
    // later excursion read as infinitely significant.
    REQUIRE_FALSE(accumulator.result().has_value());

    accumulator.add(makePose({0.1, 0.2, 0.3}, Eigen::Quaterniond::Identity()));
    REQUIRE(accumulator.result().has_value());
}

TEST_CASE("the baseline measures its own noise floor", "[stability]") {
    const Eigen::Vector3d truePosition{0.1, -0.2, 0.35};
    const Eigen::Quaterniond trueOrientation{Eigen::AngleAxisd(30.0 * kDeg, Eigen::Vector3d::UnitZ())};

    BaselineAccumulator accumulator;
    for (int i = 0; i < 200; i++)
        accumulator.add(noisyPose(truePosition, trueOrientation, i, 0.001, 0.3));

    const auto baseline = accumulator.result();
    REQUIRE(baseline.has_value());
    REQUIRE(baseline->valid);
    REQUIRE(baseline->samples == 200);

    // The mean sits on the truth despite the noise.
    REQUIRE((baseline->position - truePosition).norm() < 0.0002);

    // And the reported sigma reflects the noise that was injected rather than
    // being zero or wild.
    REQUIRE(baseline->positionSigmaM > 0.0001);
    REQUIRE(baseline->positionSigmaM < 0.001);
    REQUIRE(baseline->orientationSigmaDeg > 0.0);
    REQUIRE(baseline->orientationSigmaDeg < 0.3);
}

TEST_CASE("noise alone is not called movement", "[stability][negative]") {
    const Eigen::Vector3d position{0.1, -0.2, 0.35};
    const Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();

    BaselineAccumulator accumulator;
    for (int i = 0; i < 200; i++)
        accumulator.add(noisyPose(position, orientation, i, 0.001, 0.3));

    ExcursionTracker tracker{*accumulator.result()};

    // More of exactly the same noise, from a different stretch of the sequence.
    for (int i = 500; i < 700; i++)
        tracker.add(noisyPose(position, orientation, i, 0.001, 0.3));

    // Peak-of-noise against RMS-of-noise is a few sigma by construction, so the
    // bar is that it stays out of the band where the summary claims something
    // moved -- not that it reads as zero.
    const double significance = tracker.excursion().significance(tracker.baseline());
    REQUIRE(significance < 10.0);

    const auto text = describeStability(tracker.excursion(), tracker.baseline(), std::nullopt);
    REQUIRE(text.find("Well above the noise") == std::string::npos);
}

TEST_CASE("a step in the pose shows up against the floor", "[stability]") {
    const Eigen::Vector3d position{0.1, -0.2, 0.35};
    const Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();

    BaselineAccumulator accumulator;
    for (int i = 0; i < 200; i++)
        accumulator.add(noisyPose(position, orientation, i, 0.0002, 0.05));

    ExcursionTracker tracker{*accumulator.result()};

    // The sensor slides 3 mm and rotates 1 degree, both far too small to see.
    const Eigen::Vector3d moved = position + Eigen::Vector3d{0.003, 0.0, 0.0};
    const Eigen::Quaterniond rotated =
            Eigen::Quaterniond{Eigen::AngleAxisd(1.0 * kDeg, Eigen::Vector3d::UnitY())} * orientation;

    for (int i = 500; i < 600; i++)
        tracker.add(noisyPose(moved, rotated, i, 0.0002, 0.05));

    const auto &excursion = tracker.excursion();
    REQUIRE(excursion.peakPositionM > 0.0025);
    REQUIRE(excursion.peakOrientationDeg > 0.9);
    REQUIRE(excursion.significance(tracker.baseline()) > 10.0);

    // At a 194 mm lever arm, 1 degree dominates the 3 mm slide -- which is the
    // reason orientation is reported at all.
    const double tip = excursion.tipEquivalentM(0.194);
    REQUIRE(tip > 0.003);

    const auto text = describeStability(excursion, tracker.baseline(), 0.194);
    REQUIRE(text.find("Well above the noise") != std::string::npos);
    REQUIRE(text.find("at the tip") != std::string::npos);
}

TEST_CASE("peaks are held and can be cleared", "[stability]") {
    BaselineAccumulator accumulator;
    for (int i = 0; i < 50; i++)
        accumulator.add(noisyPose({0.0, 0.0, 0.0}, Eigen::Quaterniond::Identity(), i, 0.0001, 0.01));

    ExcursionTracker tracker{*accumulator.result()};

    // A transient: one displaced sample, then back to rest.
    tracker.add(makePose({0.01, 0.0, 0.0}, Eigen::Quaterniond::Identity()));
    tracker.add(makePose({0.0, 0.0, 0.0}, Eigen::Quaterniond::Identity()));

    // The live number has returned, but the peak remembers -- a tug on a cable
    // is over before anyone can look up.
    REQUIRE(tracker.excursion().positionM < 0.001);
    REQUIRE(tracker.excursion().peakPositionM > 0.009);

    tracker.clearPeaks();
    REQUIRE(tracker.excursion().peakPositionM < 0.001);
}

TEST_CASE("the rigidity witness holds steady on a rigid body", "[stability][witness]") {
    // Two sensors bolted to one body, 84 mm apart and rotated relative to each
    // other, carried through a large arbitrary motion.
    const Eigen::Vector3d offsetInBody{0.084, 0.0, 0.0};
    const Eigen::Quaterniond relativeRotation{Eigen::AngleAxisd(35.0 * kDeg, Eigen::Vector3d::UnitZ())};

    RigidityWitness witness;

    std::vector<Pose> first;
    {
        const Eigen::Quaterniond body = Eigen::Quaterniond::Identity();
        const Eigen::Vector3d origin{0.1, 0.1, 0.1};
        first.push_back(makePose(origin, body));
        first.push_back(makePose(origin + body * offsetInBody, body * relativeRotation));
    }
    REQUIRE(witness.arm(first));
    REQUIRE(witness.armed());
    REQUIRE(witness.pairs().size() == 1);
    REQUIRE(witness.pairs()[0].baselineSeparationM > 0.0839);

    for (int i = 0; i < 100; i++) {
        const Eigen::Quaterniond body{
                Eigen::AngleAxisd(i * 3.0 * kDeg, Eigen::Vector3d{0.3, 0.5, 0.8}.normalized())};
        const Eigen::Vector3d origin{0.1 + 0.01 * i, 0.1 - 0.005 * i, 0.1};

        witness.add({makePose(origin, body), makePose(origin + body * offsetInBody,
                                                      body * relativeRotation)});
    }

    // Moving the whole probe must not register: the pair transform is expressed
    // in the first sensor's own frame precisely so the operator can pick the
    // probe up without disturbing the measurement.
    REQUIRE(witness.pairs()[0].peakSeparationDeviationM < 1e-9);
    REQUIRE(witness.pairs()[0].peakRotationDeviationDeg < 1e-9);
    REQUIRE(witness.pairs()[0].samples == 100);
}

TEST_CASE("the rigidity witness catches one sensor working loose", "[stability][witness]") {
    const Eigen::Vector3d offsetInBody{0.084, 0.0, 0.0};

    RigidityWitness witness;
    const Eigen::Quaterniond body = Eigen::Quaterniond::Identity();
    const Eigen::Vector3d origin{0.1, 0.1, 0.1};
    REQUIRE(witness.arm({makePose(origin, body), makePose(origin + offsetInBody, body)}));

    // The second sensor rotates 2 degrees in place and slides 1 mm sideways --
    // sideways, so a scalar separation would barely notice it.
    const Eigen::Quaterniond slipped{Eigen::AngleAxisd(2.0 * kDeg, Eigen::Vector3d::UnitZ())};
    witness.add({makePose(origin, body),
                 makePose(origin + offsetInBody + Eigen::Vector3d{0.0, 0.001, 0.0}, slipped)});

    const auto &pair = witness.pairs()[0];
    REQUIRE(pair.peakRotationDeviationDeg > 1.9);
    REQUIRE(pair.peakSeparationDeviationM > 0.0009);

    // Reported at the tip, 2 degrees over a 194 mm arm is ~6.8 mm.
    REQUIRE(witness.worstTipEquivalentM(0.194) > 0.006);
}

TEST_CASE("the witness refuses what it cannot compare", "[stability][witness][negative]") {
    RigidityWitness witness;

    SECTION("a single sensor gives no pair") {
        REQUIRE_FALSE(witness.arm({makePose({0, 0, 0}, Eigen::Quaterniond::Identity())}));
        REQUIRE_FALSE(witness.armed());
        REQUIRE(witness.pairs().empty());
    }

    SECTION("no sensors at all") {
        REQUIRE_FALSE(witness.arm({}));
        REQUIRE_FALSE(witness.armed());
    }

    SECTION("a changed sensor count is ignored rather than re-indexed") {
        const Eigen::Quaterniond identity = Eigen::Quaterniond::Identity();
        REQUIRE(witness.arm({makePose({0, 0, 0}, identity), makePose({0.084, 0, 0}, identity),
                             makePose({0.168, 0, 0}, identity)}));
        REQUIRE(witness.pairs().size() == 3);   // every pair, not just consecutive ones

        // A sensor drops out. Silently comparing sensor 2 against what used to
        // be sensor 1 would invent movement that never happened.
        witness.add({makePose({0, 0, 0}, identity), makePose({0.168, 0, 0}, identity)});

        for (const auto &pair : witness.pairs())
            REQUIRE(pair.samples == 0);
    }
}

TEST_CASE("the summary describes what was measured", "[stability]") {
    StabilityBaseline baseline;
    baseline.positionSigmaM = 0.0005;
    baseline.orientationSigmaDeg = 0.1;
    baseline.samples = 200;
    baseline.valid = true;

    SECTION("without a baseline there is nothing to say") {
        StabilityBaseline none;
        const auto text = describeStability(Excursion{}, none, std::nullopt);
        REQUIRE(text.find("No baseline") != std::string::npos);
    }

    SECTION("quiet runs say so plainly") {
        Excursion excursion;
        excursion.peakPositionM = 0.0008;
        excursion.peakOrientationDeg = 0.15;

        const auto text = describeStability(excursion, baseline, std::nullopt);
        REQUIRE(text.find("Nothing above the noise") != std::string::npos);
    }

    SECTION("a marginal run asks for repetition rather than declaring a verdict") {
        Excursion excursion;
        excursion.peakPositionM = 0.0025;
        excursion.peakOrientationDeg = 0.5;

        const auto text = describeStability(excursion, baseline, std::nullopt);
        REQUIRE(text.find("Repeat the load") != std::string::npos);
    }

    SECTION("even a loud run stops short of blaming the sensor") {
        Excursion excursion;
        excursion.peakPositionM = 0.02;
        excursion.peakOrientationDeg = 3.0;

        const auto text = describeStability(excursion, baseline, 0.194);
        REQUIRE(text.find("Well above the noise") != std::string::npos);
        // Whether the body was nudged is not something the numbers can settle.
        REQUIRE(text.find("nudged") != std::string::npos);
    }

    SECTION("the lever arm turns rotation into tip displacement") {
        Excursion excursion;
        excursion.peakPositionM = 0.0001;
        excursion.peakOrientationDeg = 1.0;

        // 1 degree over 194 mm is 3.39 mm, which dominates the 0.1 mm shift.
        REQUIRE(excursion.tipEquivalentM(0.194) > 0.0033);
        REQUIRE(excursion.tipEquivalentM(0.194) < 0.0035);
    }
}

TEST_CASE("pair lines name both sensors and both deviations", "[stability][witness]") {
    PairDeviation pair;
    pair.first = 0;
    pair.second = 2;
    pair.baselineSeparationM = 0.08431;
    pair.peakSeparationDeviationM = 0.00012;
    pair.peakRotationDeviationDeg = 0.31;

    const auto text = describePair(pair);
    REQUIRE(text.find("sensors 0-2") != std::string::npos);
    REQUIRE(text.find("84.31") != std::string::npos);
    REQUIRE(text.find("0.31") != std::string::npos);
}
