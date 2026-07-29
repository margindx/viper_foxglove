//
// Unit tests for the calibration state machine and the config write-back.
//
// The session is fed synthetic captures built from a known tip transform, so
// the whole pipeline -- gating, step sequencing, solve, write -- can be
// exercised end to end without a Viper attached.
//

#include <catch2/catch_test_macros.hpp>

#include <cstdio>
#include <fstream>

#include <nlohmann/json.hpp>

#include "CalibrationSession.hpp"
#include "ProbeConfigIo.hpp"

using namespace mdx;

namespace {

constexpr double kDeg = 3.14159265358979323846 / 180.0;

double angleFor(int i, double scale, int seed) {
    const double x = std::sin(static_cast<double>(i * 7919 + seed * 104729)) * 43758.5453;
    return (x - std::floor(x) - 0.5) * scale;
}

Pose makePose(const Eigen::Vector3d &position, const Eigen::Quaterniond &orientation) {
    Pose p;
    p.position = position;
    p.orientation = orientation.normalized();
    return p;
}

/// Ground truth used by every case below.
const Eigen::Vector3d kTipOffset{0.157, 0.0, 0.0};
/// The housing flat's normal is perpendicular to the probe axis, which the
/// pivot recovers as the tip offset direction (+x here).
const Eigen::Vector3d kBodyFlatSensor = Eigen::Vector3d::UnitY();
const Eigen::Vector3d kSurfaceNormal = Eigen::Vector3d::UnitZ();

void feedRockingCapture(CalibrationSession &session, int count = 120) {
    const Eigen::Vector3d pivot{0.2, -0.1, 0.4};

    for (int i = 0; i < count; i++) {
        const Eigen::Quaterniond q =
                Eigen::Quaterniond{Eigen::AngleAxisd(angleFor(i, 70.0 * kDeg, 1),
                                                     Eigen::Vector3d::UnitY())} *
                Eigen::Quaterniond{Eigen::AngleAxisd(angleFor(i, 70.0 * kDeg, 4),
                                                     Eigen::Vector3d::UnitZ())} *
                Eigen::Quaterniond{Eigen::AngleAxisd(angleFor(i, 70.0 * kDeg, 8),
                                                     Eigen::Vector3d::UnitX())};

        session.addSample(makePose(pivot - q.normalized() * kTipOffset, q));
    }
}

/// Placements where `sensorDirection` is laid onto `worldDirection`, spun
/// freely about it.
void feedDirectionCapture(CalibrationSession &session, const Eigen::Vector3d &sensorDirection,
                          const Eigen::Vector3d &worldDirection, int seed, int count = 60) {
    const Eigen::Quaterniond align =
            Eigen::Quaterniond::FromTwoVectors(sensorDirection, worldDirection);

    for (int i = 0; i < count; i++) {
        const Eigen::Quaterniond spin{
                Eigen::AngleAxisd(angleFor(i, 360.0 * kDeg, seed), worldDirection.normalized())};
        const Eigen::Quaterniond q = (spin * align).normalized();
        const Eigen::Vector3d position{angleFor(i, 0.3, seed + 1), angleFor(i, 0.3, seed + 2),
                                       angleFor(i, 0.3, seed + 3)};

        session.addSample(makePose(position, q));
    }
}

/// Drive a session through both captures.
CalibrationSession completeSession() {
    CalibrationSession session;

    feedRockingCapture(session);
    session.advance();

    feedDirectionCapture(session, kBodyFlatSensor, kSurfaceNormal, 21);
    session.advance();

    return session;
}

std::filesystem::path tempConfigPath(const std::string &name) {
    return std::filesystem::temp_directory_path() / ("viper-calib-test-" + name + ".json");
}

void writeFile(const std::filesystem::path &path, const std::string &contents) {
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    out << contents;
}

std::string readFile(const std::filesystem::path &path) {
    std::ifstream in(path, std::ios::binary);
    return std::string{std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

const char *kSampleConfig = R"({
    "probe_profiles": [
        {
            "sensor_count": 3,
            "label": "legacy triple",
            "tip_offset_m": [0.157, 0.0, 0.0],
            "tip_rotation_zyx_deg": [0.0, 0.0, 0.0]
        }
    ],
    "minimum_contact_force": 0.35,
    "use_hardware_contact": true,
    "_comment": {
        "probe_profiles": "REQUIRED.",
        "minimum_contact_force": "the force threshold"
    }
})";

} // namespace

TEST_CASE("the session starts on the pivot step", "[session]") {
    CalibrationSession session;

    REQUIRE(session.step() == CalibrationStep::RockingPivot);
    REQUIRE_FALSE(session.readyToAdvance());
    REQUIRE(session.currentStep().title.find("1 of 2") != std::string::npos);
}

TEST_CASE("near-duplicate poses are discarded", "[session][decimation]") {
    CalibrationSession session;
    const Eigen::Quaterniond q = Eigen::Quaterniond::Identity();

    REQUIRE(session.addSample(makePose({0, 0, 0}, q)));

    // A pose the probe has not meaningfully moved to adds nothing to the solve,
    // and counting it would make the capture gate read stillness as progress.
    REQUIRE_FALSE(session.addSample(makePose({0, 0, 1e-9}, q)));
    REQUIRE(session.sampleCount(CalibrationStep::RockingPivot) == 1);

    SECTION("moving far enough is kept") {
        REQUIRE(session.addSample(makePose({0.01, 0, 0}, q)));
    }

    SECTION("turning far enough is kept") {
        const Eigen::Quaterniond turned{Eigen::AngleAxisd(5.0 * kDeg, Eigen::Vector3d::UnitZ())};
        REQUIRE(session.addSample(makePose({0, 0, 0}, turned)));
    }
}

TEST_CASE("unusable poses are never retained", "[session]") {
    CalibrationSession session;

    Pose bad;
    bad.position = Eigen::Vector3d{std::numeric_limits<double>::quiet_NaN(), 0, 0};
    REQUIRE_FALSE(session.addSample(bad));

    Pose notUnit;
    notUnit.position = Eigen::Vector3d::Zero();
    notUnit.orientation = Eigen::Quaterniond{5.0, 0.0, 0.0, 0.0};
    REQUIRE_FALSE(session.addSample(notUnit));

    REQUIRE(session.sampleCount(CalibrationStep::RockingPivot) == 0);
}

TEST_CASE("a step cannot be skipped before it is ready", "[session][gating]") {
    CalibrationSession session;

    REQUIRE_FALSE(session.advance());
    REQUIRE(session.step() == CalibrationStep::RockingPivot);

    // A stationary capture never becomes ready however long it runs.
    for (int i = 0; i < 500; i++)
        session.addSample(makePose({0.0, 0.0, static_cast<double>(i) * 0.01},
                                   Eigen::Quaterniond::Identity()));

    REQUIRE_FALSE(session.readyToAdvance());
    REQUIRE_FALSE(session.advance());
}

TEST_CASE("the session walks both steps", "[session]") {
    CalibrationSession session;

    feedRockingCapture(session);
    REQUIRE(session.readyToAdvance());
    REQUIRE(session.advance());
    REQUIRE(session.step() == CalibrationStep::BodyFlat);

    feedDirectionCapture(session, kBodyFlatSensor, kSurfaceNormal, 21);
    REQUIRE(session.readyToAdvance());
    REQUIRE(session.advance());
    REQUIRE(session.step() == CalibrationStep::Done);
}

TEST_CASE("restarting a step discards only that step", "[session]") {
    CalibrationSession session;

    feedRockingCapture(session);
    const auto captured = session.sampleCount(CalibrationStep::RockingPivot);
    REQUIRE(captured > 0);

    session.advance();
    feedDirectionCapture(session, kBodyFlatSensor, kSurfaceNormal, 21);
    session.restartStep();

    REQUIRE(session.sampleCount(CalibrationStep::BodyFlat) == 0);
    REQUIRE(session.sampleCount(CalibrationStep::RockingPivot) == captured);
}

TEST_CASE("a completed session recovers the known transform", "[session][solve]") {
    const auto session = completeSession();
    REQUIRE(session.step() == CalibrationStep::Done);

    const auto outcome = session.solve();
    REQUIRE(outcome.has_value());

    SECTION("the tip offset matches") {
        REQUIRE((outcome->tipOffset - kTipOffset).norm() < 1e-8);
    }

    SECTION("the rotation is recovered from the pivot axis and the body flat") {
        REQUIRE(outcome->tipRotation.has_value());
        REQUIRE(outcome->rotationFromIdentityDeg < 1e-6);
    }

    SECTION("the plane constraint agrees with the pivot") {
        REQUIRE(outcome->planeCheck.has_value());
        REQUIRE(outcome->offsetDisagreementM < 1e-6);
    }
}

TEST_CASE("an incomplete session does not solve", "[session][solve]") {
    CalibrationSession session;
    feedRockingCapture(session);

    // The pivot alone is captured; the direction steps have nothing.
    REQUIRE_FALSE(session.solve().has_value());
}

TEST_CASE("writing a profile preserves everything else", "[config]") {
    const auto path = tempConfigPath("preserve");
    writeFile(path, kSampleConfig);

    ProfileUpdate update;
    update.sensorCount = 3;
    update.tipOffsetM = Eigen::Vector3d{0.1601, 0.0012, -0.0003};

    const auto report = writeProfile(path, update, "20260728T041500");

    REQUIRE_FALSE(report.created);
    REQUIRE(std::filesystem::exists(report.backupPath));

    const auto written = nlohmann::ordered_json::parse(readFile(path));

    SECTION("the offset was updated") {
        const auto &entry = written["probe_profiles"][0];
        REQUIRE(entry["tip_offset_m"][0].get<double>() == 0.1601);
        REQUIRE(entry["label"].get<std::string>() == "legacy triple");
    }

    SECTION("unrelated keys survive") {
        REQUIRE(written["minimum_contact_force"].get<double>() == 0.35);
        REQUIRE(written["use_hardware_contact"].get<bool>() == true);
        REQUIRE(written["_comment"].contains("minimum_contact_force"));
    }

    SECTION("key order survives, so _comment still lines up") {
        std::vector<std::string> keys;
        for (auto it = written.begin(); it != written.end(); ++it)
            keys.push_back(it.key());

        const std::vector<std::string> expected{"probe_profiles", "minimum_contact_force",
                                                "use_hardware_contact", "_comment"};
        REQUIRE(keys == expected);
    }

    SECTION("the backup holds the original") {
        const auto backup = nlohmann::ordered_json::parse(readFile(report.backupPath));
        REQUIRE(backup["probe_profiles"][0]["tip_offset_m"][0].get<double>() == 0.157);
    }

    std::filesystem::remove(path);
    std::filesystem::remove(report.backupPath);
}

TEST_CASE("writing a new sensor count appends an entry", "[config]") {
    const auto path = tempConfigPath("append");
    writeFile(path, kSampleConfig);

    ProfileUpdate update;
    update.sensorCount = 1;
    update.tipOffsetM = Eigen::Vector3d{0.150, 0.0, 0.0};
    update.tipRotationZyxDeg = Eigen::Vector3d{0.0, 90.0, 0.0};
    update.label = "mid-size single";

    const auto report = writeProfile(path, update, "ts");

    REQUIRE(report.created);

    const auto written = nlohmann::ordered_json::parse(readFile(path));
    REQUIRE(written["probe_profiles"].size() == 2);

    const auto &added = written["probe_profiles"][1];
    REQUIRE(added["sensor_count"].get<int>() == 1);
    REQUIRE(added["label"].get<std::string>() == "mid-size single");
    REQUIRE(added["tip_rotation_zyx_deg"][1].get<double>() == 90.0);

    std::filesystem::remove(path);
    std::filesystem::remove(report.backupPath);
}

TEST_CASE("an absent rotation leaves an existing one alone", "[config]") {
    // Not solving a rotation this run must not silently undo a previous
    // calibration's rotation.
    const auto path = tempConfigPath("rotation");
    writeFile(path, R"({"probe_profiles":[{"sensor_count":1,"tip_offset_m":[0.1,0,0],
                        "tip_rotation_zyx_deg":[10.0,20.0,30.0]}]})");

    ProfileUpdate update;
    update.sensorCount = 1;
    update.tipOffsetM = Eigen::Vector3d{0.2, 0.0, 0.0};

    const auto report = writeProfile(path, update, "ts");

    const auto written = nlohmann::ordered_json::parse(readFile(path));
    const auto &entry = written["probe_profiles"][0];

    REQUIRE(entry["tip_offset_m"][0].get<double>() == 0.2);
    REQUIRE(entry["tip_rotation_zyx_deg"][0].get<double>() == 10.0);

    std::filesystem::remove(path);
    std::filesystem::remove(report.backupPath);
}

TEST_CASE("reading reports what is about to change", "[config]") {
    const auto path = tempConfigPath("read");
    writeFile(path, kSampleConfig);

    const auto existing = readProfile(path, 3);
    REQUIRE(existing.has_value());
    REQUIRE(existing->tipOffsetM.x() == 0.157);
    REQUIRE(existing->label == "legacy triple");
    REQUIRE(existing->tipRotationZyxDeg.has_value());

    REQUIRE_FALSE(readProfile(path, 1).has_value());

    std::filesystem::remove(path);
}

TEST_CASE("config write failures are reported, not silent", "[config][negative]") {
    SECTION("missing file") {
        ProfileUpdate update;
        update.sensorCount = 1;
        REQUIRE_THROWS_AS(writeProfile(tempConfigPath("does-not-exist"), update, "ts"),
                          std::runtime_error);
    }

    SECTION("unparseable file") {
        const auto path = tempConfigPath("broken");
        writeFile(path, "{ not json");

        ProfileUpdate update;
        update.sensorCount = 1;
        REQUIRE_THROWS_AS(writeProfile(path, update, "ts"), std::runtime_error);

        std::filesystem::remove(path);
    }

    SECTION("nonsensical sensor count") {
        const auto path = tempConfigPath("count");
        writeFile(path, kSampleConfig);

        ProfileUpdate update;
        update.sensorCount = 0;
        REQUIRE_THROWS_AS(writeProfile(path, update, "ts"), std::runtime_error);

        std::filesystem::remove(path);
    }
}

TEST_CASE("a written profile parses back through the normal loader", "[config][roundtrip]") {
    // The whole point of the write: the next normal run must accept it.
    const auto path = tempConfigPath("roundtrip");
    writeFile(path, kSampleConfig);

    ProfileUpdate update;
    update.sensorCount = 1;
    update.tipOffsetM = Eigen::Vector3d{0.1499, 0.0021, -0.0007};
    update.tipRotationZyxDeg = Eigen::Vector3d{1.5, -2.5, 0.25};
    update.label = "mid-size single";

    const auto report = writeProfile(path, update, "ts");

    const auto document = nlohmann::json::parse(readFile(path));
    const auto profiles = parseProbeProfiles(document);

    REQUIRE(profiles.size() == 2);

    const auto *single = selectProfile(profiles, 1);
    REQUIRE(single != nullptr);
    REQUIRE((single->tip.translation - update.tipOffsetM).norm() < 1e-12);

    const auto expected = quaternionFromZyxDegrees(1.5, -2.5, 0.25);
    REQUIRE(std::abs(single->tip.rotation.coeffs().dot(expected.coeffs())) > 1.0 - 1e-12);

    std::filesystem::remove(path);
    std::filesystem::remove(report.backupPath);
}
