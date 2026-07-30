//
// Unit tests for interpreting the Viper's own configuration.
//
// The values here are the raw payloads the device returns, so the decisions can
// be checked without a device attached.
//

#include <catch2/catch_test_macros.hpp>

#include "DeviceState.hpp"

using namespace mdx;

namespace {

DeviceRotation rotation(double a, double b, double c, double d) {
    DeviceRotation r;
    r.params[0] = a;
    r.params[1] = b;
    r.params[2] = c;
    r.params[3] = d;
    return r;
}

} // namespace

TEST_CASE("a neutral device rotation is recognized in either convention", "[device]") {
    SECTION("all zeros") {
        REQUIRE(rotation(0, 0, 0, 0).isNeutral());
    }

    SECTION("identity quaternion, scalar first") {
        REQUIRE(rotation(1, 0, 0, 0).isNeutral());
        REQUIRE(rotation(-1, 0, 0, 0).isNeutral());   // same rotation
    }

    SECTION("identity quaternion, scalar last") {
        REQUIRE(rotation(0, 0, 0, 1).isNeutral());
        REQUIRE(rotation(0, 0, 0, -1).isNeutral());
    }
}

TEST_CASE("any real rotation is refused", "[device]") {
    // 90 degrees about z, either component order.
    REQUIRE_FALSE(rotation(0.7071, 0, 0, 0.7071).isNeutral());
    REQUIRE_FALSE(rotation(0, 0, 0.7071, 0.7071).isNeutral());

    // A small but deliberate boresight must not slip through.
    REQUIRE_FALSE(rotation(0.9999, 0.01, 0, 0).isNeutral());
}

TEST_CASE("a non-finite rotation is never neutral", "[device][negative]") {
    // Unreadable is not the same as unset, and must not be treated as safe.
    REQUIRE_FALSE(rotation(std::numeric_limits<double>::quiet_NaN(), 0, 0, 0).isNeutral());
    REQUIRE_FALSE(rotation(std::numeric_limits<double>::infinity(), 0, 0, 0).isNeutral());
}

TEST_CASE("sensor origin defaults to source 1", "[device]") {
    REQUIRE(isDefaultSensorOrigin(0));
    REQUIRE_FALSE(isDefaultSensorOrigin(1));
    REQUIRE_FALSE(isDefaultSensorOrigin(4));

    REQUIRE(sensorOriginLabel(0).find("default") != std::string::npos);
    REQUIRE(sensorOriginLabel(4) == "common");
    REQUIRE(sensorOriginLabel(99).find("unrecognized") != std::string::npos);
}

TEST_CASE("frame rate codes map to their rates", "[device]") {
    REQUIRE(frameRateLabel(0) == "30 Hz");
    REQUIRE(frameRateLabel(3) == "240 Hz");
    REQUIRE(frameRateLabel(5) == "960 Hz");
    REQUIRE(frameRateLabel(7).find("unrecognized") != std::string::npos);
}

TEST_CASE("refusal messages say what was found and how to clear it", "[device]") {
    SECTION("boresight") {
        const auto message = boresightMessage(0, rotation(0.7071, 0, 0, 0.7071));

        REQUIRE(message.find("boresight") != std::string::npos);
        REQUIRE(message.find("0.7071") != std::string::npos);
        REQUIRE(message.find("CMD_BORESIGHT") != std::string::npos);
        // The point the operator needs: it looks like a mounting fault.
        REQUIRE(message.find("physical axes") != std::string::npos);
    }

    SECTION("source rotation") {
        const auto message = sourceRotationMessage(0, rotation(0.5, 0.5, 0.5, 0.5));

        REQUIRE(message.find("CMD_SRC_ROTATION") != std::string::npos);
        REQUIRE(message.find("tracker frame") != std::string::npos);
    }

    SECTION("sensor origin") {
        const auto message = sensorOriginMessage(2, 4);

        REQUIRE(message.find("common") != std::string::npos);
        REQUIRE(message.find("CMD_SNS_ORIGIN") != std::string::npos);
    }
}

TEST_CASE("quality settings describe themselves for the log", "[device]") {
    SECTION("filter off is called out") {
        FilterSettings settings;
        REQUIRE(describeFilter(settings).find("off") != std::string::npos);
    }

    SECTION("predictive filtering names what it predicts") {
        PredictiveFilterSettings none;
        REQUIRE(describePredictiveFilter(none) == "off");

        PredictiveFilterSettings both;
        both.quaternion = true;
        both.position = true;
        both.predictionSeconds = 0.02;

        const auto described = describePredictiveFilter(both);
        REQUIRE(described.find("orientation and position") != std::string::npos);
        REQUIRE(described.find("20") != std::string::npos);   // ms

        PredictiveFilterSettings positionOnly;
        positionOnly.position = true;
        REQUIRE(describePredictiveFilter(positionOnly).find("position") != std::string::npos);
    }

    SECTION("increment mode explains an irregular stream") {
        IncrementSettings off;
        REQUIRE(describeIncrement(off) == "off");

        IncrementSettings on;
        on.enabled = true;
        on.positionThreshold = 0.001;
        on.orientationThreshold = 0.5;

        const auto described = describeIncrement(on);
        REQUIRE(described.find("ON") != std::string::npos);
        // Worth saying, because it otherwise reads as dropped frames.
        REQUIRE(described.find("irregular") != std::string::npos);
    }
}

TEST_CASE("the distortion message carries the level and the count", "[device]") {
    const auto message = distortionMessage(97, 1, 250);

    REQUIRE(message.find("97") != std::string::npos);
    REQUIRE(message.find("250") != std::string::npos);
    REQUIRE(message.find("metal") != std::string::npos);
}
