//
// Unit tests for the probe-profile fusion maths and config parsing.
//
// These cover the parts of the pipeline whose correctness cannot be eyeballed
// on hardware — in particular the quaternion hemisphere alignment, whose whole
// symptom is an occasional wrong orientation that looks like sensor noise.
//

#include <catch2/catch_test_macros.hpp>

#include "ProbeProfile.hpp"

using namespace mdx;

namespace {

/// q and -q are the same rotation, so compare rotations, not components.
bool sameRotation(const Eigen::Quaterniond &a, const Eigen::Quaterniond &b, double tol = 1e-9) {
    return std::abs(a.normalized().coeffs().dot(b.normalized().coeffs())) > 1.0 - tol;
}

Pose makePose(const Eigen::Vector3d &position, const Eigen::Quaterniond &orientation) {
    Pose p;
    p.position = position;
    p.orientation = orientation;
    return p;
}

const Eigen::Quaterniond kIdentity = Eigen::Quaterniond::Identity();

} // namespace

TEST_CASE("fusePoses rejects frames it cannot trust", "[fuse]") {
    SECTION("no sensors") {
        REQUIRE_FALSE(fusePoses({}).has_value());
    }

    SECTION("non-finite position") {
        const auto nan = std::numeric_limits<double>::quiet_NaN();
        REQUIRE_FALSE(fusePoses({makePose({nan, 0, 0}, kIdentity)}).has_value());
    }

    SECTION("non-finite orientation") {
        const auto inf = std::numeric_limits<double>::infinity();
        REQUIRE_FALSE(fusePoses({makePose({0, 0, 0}, Eigen::Quaterniond{inf, 0, 0, 0})}).has_value());
    }

    SECTION("zero quaternion") {
        REQUIRE_FALSE(fusePoses({makePose({0, 0, 0}, Eigen::Quaterniond{0, 0, 0, 0})}).has_value());
    }

    SECTION("one bad sensor poisons the whole frame") {
        const auto nan = std::numeric_limits<double>::quiet_NaN();
        const std::vector<Pose> poses{
                makePose({1, 2, 3}, kIdentity),
                makePose({1, 2, 3}, kIdentity),
                makePose({nan, 2, 3}, kIdentity),
        };
        REQUIRE_FALSE(fusePoses(poses).has_value());
    }
}

TEST_CASE("a single sensor passes through untouched", "[fuse][single]") {
    // The point of the single-sensor configuration: no averaging, no rounding,
    // the published pose is bit-for-bit the sensor's own.
    const auto orientation = quaternionFromZyxDegrees(31.0, -12.0, 77.0);
    const auto pose = makePose({0.1234, -5.6789, 42.0}, orientation);

    const auto fused = fusePoses({pose});

    REQUIRE(fused.has_value());
    REQUIRE(fused->position.x() == pose.position.x());
    REQUIRE(fused->position.y() == pose.position.y());
    REQUIRE(fused->position.z() == pose.position.z());
    REQUIRE(fused->orientation.coeffs() == pose.orientation.coeffs());
}

TEST_CASE("fusePoses averages positions", "[fuse]") {
    const std::vector<Pose> poses{
            makePose({0, 0, 0}, kIdentity),
            makePose({3, 6, 9}, kIdentity),
            makePose({6, 12, 18}, kIdentity),
    };

    const auto fused = fusePoses(poses);

    REQUIRE(fused.has_value());
    REQUIRE(fused->position.isApprox(Eigen::Vector3d{3, 6, 9}));
    REQUIRE(sameRotation(fused->orientation, kIdentity));
}

TEST_CASE("identical orientations fuse to themselves", "[fuse]") {
    const auto q = quaternionFromZyxDegrees(20.0, 10.0, -5.0);
    const std::vector<Pose> poses{
            makePose({0, 0, 0}, q),
            makePose({0, 0, 0}, q),
            makePose({0, 0, 0}, q),
    };

    const auto fused = fusePoses(poses);

    REQUIRE(fused.has_value());
    REQUIRE(sameRotation(fused->orientation, q));
}

TEST_CASE("antipodal quaternions fuse correctly", "[fuse][regression]") {
    // Regression test for the sign bug: q and -q denote the same rotation, so
    // three sensors that agree perfectly must fuse to that rotation even when
    // one of them reports the antipodal representation. The previous
    // componentwise mean canceled to near-zero here and, after normalizing,
    // returned an essentially arbitrary orientation.
    const auto q = quaternionFromZyxDegrees(45.0, 0.0, 0.0);
    const Eigen::Quaterniond negated{-q.w(), -q.x(), -q.y(), -q.z()};

    SECTION("one sensor of three is flipped") {
        const std::vector<Pose> poses{
                makePose({0, 0, 0}, q),
                makePose({0, 0, 0}, negated),
                makePose({0, 0, 0}, q),
        };

        const auto fused = fusePoses(poses);

        REQUIRE(fused.has_value());
        REQUIRE(sameRotation(fused->orientation, q));
    }

    SECTION("the reference sensor is the flipped one") {
        const std::vector<Pose> poses{
                makePose({0, 0, 0}, negated),
                makePose({0, 0, 0}, q),
                makePose({0, 0, 0}, q),
        };

        const auto fused = fusePoses(poses);

        REQUIRE(fused.has_value());
        REQUIRE(sameRotation(fused->orientation, q));
    }

    SECTION("two sensors reporting the same rotation with opposite signs") {
        // The old componentwise mean canceled exactly here and divided by a
        // zero magnitude, publishing a NaN pose.
        const std::vector<Pose> poses{
                makePose({0, 0, 0}, q),
                makePose({0, 0, 0}, negated),
        };

        const auto fused = fusePoses(poses);

        REQUIRE(fused.has_value());
        REQUIRE(std::isfinite(fused->orientation.w()));
        REQUIRE(std::abs(fused->orientation.norm() - 1.0) < 1e-9);
        REQUIRE(sameRotation(fused->orientation, q));
    }
}

TEST_CASE("fusion does not depend on which sign each sensor reports", "[fuse][regression]") {
    // The sharpest statement of the fix, and the case the old code got badly
    // wrong: sensors that genuinely disagree a little. Flipping the sign of any
    // subset of the inputs names the same set of rotations, so it must produce
    // the same fused rotation. Under the old componentwise mean, flipping the
    // middle sensor moved the result from 19.8 degrees to -47.6 degrees.
    const auto a = quaternionFromZyxDegrees(0.0, 0.0, 0.0);
    const auto b = quaternionFromZyxDegrees(60.0, 0.0, 0.0);
    const auto c = quaternionFromZyxDegrees(0.0, 0.0, 0.0);

    auto negate = [](const Eigen::Quaterniond &q) {
        return Eigen::Quaterniond{-q.w(), -q.x(), -q.y(), -q.z()};
    };

    const auto reference = fusePoses({makePose({0, 0, 0}, a),
                                      makePose({0, 0, 0}, b),
                                      makePose({0, 0, 0}, c)});
    REQUIRE(reference.has_value());

    SECTION("middle sensor flipped") {
        const auto fused = fusePoses({makePose({0, 0, 0}, a),
                                      makePose({0, 0, 0}, negate(b)),
                                      makePose({0, 0, 0}, c)});
        REQUIRE(fused.has_value());
        REQUIRE(sameRotation(fused->orientation, reference->orientation));
    }

    SECTION("first sensor flipped, so the alignment reference itself is negated") {
        const auto fused = fusePoses({makePose({0, 0, 0}, negate(a)),
                                      makePose({0, 0, 0}, b),
                                      makePose({0, 0, 0}, c)});
        REQUIRE(fused.has_value());
        REQUIRE(sameRotation(fused->orientation, reference->orientation));
    }

    SECTION("all sensors flipped") {
        const auto fused = fusePoses({makePose({0, 0, 0}, negate(a)),
                                      makePose({0, 0, 0}, negate(b)),
                                      makePose({0, 0, 0}, negate(c))});
        REQUIRE(fused.has_value());
        REQUIRE(sameRotation(fused->orientation, reference->orientation));
    }

    SECTION("the fused rotation is the one that actually sits between them") {
        // Guards against a fix that is merely sign-stable but wrong: three
        // sensors at 0, 60 and 0 degrees must fuse to roughly 20 degrees, not
        // to the -47.6 degrees the unaligned mean produced.
        const auto expected = quaternionFromZyxDegrees(20.0, 0.0, 0.0);
        REQUIRE(std::abs(reference->orientation.normalized().coeffs().dot(
                        expected.normalized().coeffs())) > 0.999);
    }
}

TEST_CASE("fused orientation lies between the inputs", "[fuse]") {
    const auto a = quaternionFromZyxDegrees(0.0, 0.0, 0.0);
    const auto b = quaternionFromZyxDegrees(60.0, 0.0, 0.0);

    const auto fused = fusePoses({makePose({0, 0, 0}, a), makePose({0, 0, 0}, b)});

    REQUIRE(fused.has_value());
    REQUIRE(sameRotation(fused->orientation, quaternionFromZyxDegrees(30.0, 0.0, 0.0), 1e-9));
}

TEST_CASE("quaternionFromZyxDegrees follows the Viper convention", "[euler]") {
    SECTION("zero is identity") {
        REQUIRE(sameRotation(quaternionFromZyxDegrees(0, 0, 0), kIdentity));
    }

    SECTION("azimuth rotates about Z: +X maps to +Y") {
        const auto q = quaternionFromZyxDegrees(90.0, 0.0, 0.0);
        REQUIRE((q * Eigen::Vector3d::UnitX()).isApprox(Eigen::Vector3d::UnitY(), 1e-9));
    }

    SECTION("elevation rotates about Y: +X maps to -Z") {
        const auto q = quaternionFromZyxDegrees(0.0, 90.0, 0.0);
        REQUIRE((q * Eigen::Vector3d::UnitX()).isApprox(-Eigen::Vector3d::UnitZ(), 1e-9));
    }

    SECTION("roll rotates about X: +Y maps to +Z") {
        const auto q = quaternionFromZyxDegrees(0.0, 0.0, 90.0);
        REQUIRE((q * Eigen::Vector3d::UnitY()).isApprox(Eigen::Vector3d::UnitZ(), 1e-9));
    }
}

TEST_CASE("applyTipTransform places the tip", "[tip]") {
    TipTransform tip;
    tip.translation = Eigen::Vector3d{0.15, 0, 0};

    SECTION("an identity transform changes nothing") {
        const auto pose = makePose({1, 2, 3}, quaternionFromZyxDegrees(10, 20, 30));
        const auto out = applyTipTransform(pose, TipTransform{});

        REQUIRE(out.position.isApprox(pose.position));
        REQUIRE(sameRotation(out.orientation, pose.orientation));
    }

    SECTION("with identity orientation the offset adds directly") {
        const auto out = applyTipTransform(makePose({1, 2, 3}, kIdentity), tip);
        REQUIRE(out.position.isApprox(Eigen::Vector3d{1.15, 2, 3}));
    }

    SECTION("the offset is expressed in the sensor frame") {
        // Yawed 90 degrees, the along-probe +X offset must push the tip along +Y.
        const auto pose = makePose({1, 2, 3}, quaternionFromZyxDegrees(90.0, 0.0, 0.0));
        const auto out = applyTipTransform(pose, tip);

        REQUIRE(out.position.isApprox(Eigen::Vector3d{1, 2.15, 3}, 1e-9));
    }

    SECTION("a tip rotation composes into the published orientation") {
        TipTransform rotated;
        rotated.rotation = quaternionFromZyxDegrees(90.0, 0.0, 0.0);

        const auto pose = makePose({0, 0, 0}, quaternionFromZyxDegrees(90.0, 0.0, 0.0));
        const auto out = applyTipTransform(pose, rotated);

        REQUIRE(sameRotation(out.orientation, quaternionFromZyxDegrees(180.0, 0.0, 0.0)));
    }

    SECTION("the published orientation stays unit norm") {
        TipTransform rotated;
        rotated.rotation = quaternionFromZyxDegrees(33.0, 44.0, 55.0);
        const auto pose = makePose({0, 0, 0}, quaternionFromZyxDegrees(11.0, 22.0, 33.0));

        const auto out = applyTipTransform(pose, rotated);

        REQUIRE(std::abs(out.orientation.norm() - 1.0) < 1e-12);
    }
}

TEST_CASE("selectProfile matches on sensor count", "[profile]") {
    std::vector<ProbeProfile> profiles;
    ProbeProfile single;
    single.sensorCount = 1;
    single.label = "mid-size single";
    ProbeProfile triple;
    triple.sensorCount = 3;
    triple.label = "legacy triple";
    profiles.push_back(single);
    profiles.push_back(triple);

    REQUIRE(selectProfile(profiles, 1)->label == "mid-size single");
    REQUIRE(selectProfile(profiles, 3)->label == "legacy triple");
    REQUIRE(selectProfile(profiles, 2) == nullptr);
    REQUIRE(selectProfile(profiles, 0) == nullptr);
    REQUIRE(selectProfile({}, 1) == nullptr);
}

TEST_CASE("parseProbeProfiles accepts a well-formed config", "[parse]") {
    const auto settings = nlohmann::json::parse(R"({
        "probe_profiles": [
            {"sensor_count": 3, "label": "legacy triple",
             "tip_offset_m": [0.157, 0.0, 0.0], "tip_rotation_zyx_deg": [0.0, 0.0, 0.0]},
            {"sensor_count": 1, "label": "mid-size single",
             "tip_offset_m": [0.150, 0.0, 0.0]}
        ]
    })");

    const auto profiles = parseProbeProfiles(settings);

    REQUIRE(profiles.size() == 2);
    REQUIRE(profiles[0].sensorCount == 3);
    REQUIRE(profiles[0].label == "legacy triple");
    REQUIRE(profiles[0].tip.translation.isApprox(Eigen::Vector3d{0.157, 0, 0}));

    // An omitted rotation must mean identity, so existing behavior is preserved.
    REQUIRE(sameRotation(profiles[1].tip.rotation, kIdentity));
    REQUIRE(profiles[1].sensorCount == 1);
}

TEST_CASE("parseProbeProfiles rejects malformed configs", "[parse]") {
    auto parseText = [](const char *text) {
        return parseProbeProfiles(nlohmann::json::parse(text));
    };

    SECTION("a legacy flat config is refused rather than guessed at") {
        REQUIRE_THROWS_AS(parseText(R"({"offset_x": 0.157, "offset_y": 0.0, "offset_z": 0.0})"),
                          std::runtime_error);
    }

    SECTION("empty array") {
        REQUIRE_THROWS_AS(parseText(R"({"probe_profiles": []})"), std::runtime_error);
    }

    SECTION("not an array") {
        REQUIRE_THROWS_AS(parseText(R"({"probe_profiles": {"1": {}}})"), std::runtime_error);
    }

    SECTION("missing sensor_count") {
        REQUIRE_THROWS_AS(parseText(R"({"probe_profiles": [{"tip_offset_m": [0,0,0]}]})"),
                          std::runtime_error);
    }

    SECTION("sensor_count below one") {
        REQUIRE_THROWS_AS(parseText(R"({"probe_profiles": [{"sensor_count": 0, "tip_offset_m": [0,0,0]}]})"),
                          std::runtime_error);
    }

    SECTION("duplicate sensor_count") {
        REQUIRE_THROWS_AS(parseText(R"({"probe_profiles": [
                              {"sensor_count": 1, "tip_offset_m": [0,0,0]},
                              {"sensor_count": 1, "tip_offset_m": [1,0,0]}]})"),
                          std::runtime_error);
    }

    SECTION("missing tip_offset_m") {
        REQUIRE_THROWS_AS(parseText(R"({"probe_profiles": [{"sensor_count": 1}]})"),
                          std::runtime_error);
    }

    SECTION("tip_offset_m of the wrong length") {
        REQUIRE_THROWS_AS(parseText(R"({"probe_profiles": [{"sensor_count": 1, "tip_offset_m": [0,0]}]})"),
                          std::runtime_error);
    }

    SECTION("tip_offset_m containing a non-number") {
        REQUIRE_THROWS_AS(parseText(R"({"probe_profiles": [{"sensor_count": 1, "tip_offset_m": [0,"a",0]}]})"),
                          std::runtime_error);
    }

    SECTION("tip_rotation_zyx_deg of the wrong length") {
        REQUIRE_THROWS_AS(parseText(R"({"probe_profiles": [
                              {"sensor_count": 1, "tip_offset_m": [0,0,0],
                               "tip_rotation_zyx_deg": [0,0]}]})"),
                          std::runtime_error);
    }

    SECTION("non-string label") {
        REQUIRE_THROWS_AS(parseText(R"({"probe_profiles": [
                              {"sensor_count": 1, "tip_offset_m": [0,0,0], "label": 7}]})"),
                          std::runtime_error);
    }
}
