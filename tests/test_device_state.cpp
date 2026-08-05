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

// Labelled for the log, not gated: a rig may have a reason to reference a
// different source, and refusing to run over it was not this program's call.
TEST_CASE("sensor origin is named for the record", "[device]") {
    REQUIRE(sensorOriginLabel(0).find("default") != std::string::npos);
    REQUIRE(sensorOriginLabel(4) == "common");
    REQUIRE(sensorOriginLabel(99).find("unrecognized") != std::string::npos);
}

TEST_CASE("the expected frame rate is checked two ways", "[device][framerate]") {
    SECTION("codes map to rates, and unknown codes to nothing") {
        REQUIRE(frameRateHzFromCode(0) == 30);
        REQUIRE(frameRateHzFromCode(3) == 240);
        REQUIRE(frameRateHzFromCode(5) == 960);
        REQUIRE_FALSE(frameRateHzFromCode(6).has_value());
        REQUIRE_FALSE(frameRateHzFromCode(99).has_value());
    }

    SECTION("only rates the SEU can produce are accepted") {
        REQUIRE(isSupportedFrameRateHz(30));
        REQUIRE(isSupportedFrameRateHz(960));
        // 100 Hz is a perfectly reasonable number that this device cannot
        // reach, so it could never match and is a config error.
        REQUIRE_FALSE(isSupportedFrameRateHz(100));
        REQUIRE_FALSE(isSupportedFrameRateHz(0));
        REQUIRE_FALSE(isSupportedFrameRateHz(-240));
    }

    SECTION("the delivered band is symmetric and 10% wide") {
        REQUIRE_FALSE(deliveredRateOutOfBand(240, 240.0));
        REQUIRE_FALSE(deliveredRateOutOfBand(240, 220.0));   // -8.3%
        REQUIRE_FALSE(deliveredRateOutOfBand(240, 260.0));   // +8.3%
        REQUIRE(deliveredRateOutOfBand(240, 210.0));         // -12.5%
        REQUIRE(deliveredRateOutOfBand(240, 300.0));         // +25%

        // The measurement is quantization-limited, so the worst realistic case
        // -- 30 Hz counted over 2 s, one frame either way -- sits well inside.
        REQUIRE_FALSE(deliveredRateOutOfBand(30, 30.5));
        REQUIRE_FALSE(deliveredRateOutOfBand(30, 29.5));

        // A dead stream is a mismatch, not a pass.
        REQUIRE(deliveredRateOutOfBand(240, 0.0));
        REQUIRE(deliveredRateOutOfBand(0, 240.0));
    }

    SECTION("refusals name both numbers and how to reconcile them") {
        const auto mismatch = frameRateMismatchMessage(240, 1);   // SEU at 60 Hz
        REQUIRE(mismatch.find("60 Hz") != std::string::npos);
        REQUIRE(mismatch.find("240") != std::string::npos);
        // Either side may be the one that is wrong, so both fixes are offered.
        REQUIRE(mismatch.find("expected_frame_rate_hz") != std::string::npos);

        const auto unreadable = frameRateUnreadableMessage(240);
        REQUIRE(unreadable.find("240") != std::string::npos);
        REQUIRE(unreadable.find("not evidence") != std::string::npos);
    }

    SECTION("a shortfall is attributed to the link, not to the SEU") {
        // The SEU already passed the configured check, so frames going missing
        // between it and us is the only thing left.
        const auto slow = deliveredFrameRateMessage(240, 61.0, 122, 2.0);
        REQUIRE(slow.find("61.0") != std::string::npos);
        REQUIRE(slow.find("dropped") != std::string::npos);

        const auto fast = deliveredFrameRateMessage(240, 300.0, 600, 2.0);
        REQUIRE(fast.find("faster than configured") != std::string::npos);
    }
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

TEST_CASE("a distortion summary reports what the capture saw", "[device][distortion]") {
    SECTION("nothing captured") {
        DistortionSummary empty;
        REQUIRE(empty.mean() == 0.0);
        REQUIRE_FALSE(empty.exceededThreshold());
        REQUIRE(describeDistortion(empty) == "no frames seen");
    }

    SECTION("a clean capture") {
        DistortionSummary clean;
        clean.current = 2;
        clean.peak = 5;
        clean.sum = 300;
        clean.frames = 200;

        REQUIRE(clean.mean() == 1.5);
        REQUIRE_FALSE(clean.exceededThreshold());

        const auto described = describeDistortion(clean);
        REQUIRE(described.find("peak 5/255") != std::string::npos);
        REQUIRE(described.find("200 frames") != std::string::npos);
        REQUIRE(described.find("above") == std::string::npos);
    }

    SECTION("a capture that went past the threshold is called out") {
        // The point of tracking it: every capture gate measures geometry, so a
        // distorted capture passes them all and this is the only signal.
        DistortionSummary noisy;
        noisy.peak = kDistortionWarnLevel + 20;
        noisy.sum = 1000;
        noisy.frames = 100;

        REQUIRE(noisy.exceededThreshold());
        REQUIRE(describeDistortion(noisy).find("above") != std::string::npos);
    }

    SECTION("exactly at the threshold counts as exceeded") {
        DistortionSummary edge;
        edge.peak = kDistortionWarnLevel;
        REQUIRE(edge.exceededThreshold());
    }

    SECTION("the brief form carries current and peak") {
        DistortionSummary summary;
        summary.current = 7;
        summary.peak = 19;

        const auto brief = describeDistortionBrief(summary);
        REQUIRE(brief.find("7") != std::string::npos);
        REQUIRE(brief.find("19") != std::string::npos);
    }
}

TEST_CASE("the distortion message carries the level and the count", "[device]") {
    const auto message = distortionMessage(97, 1, 250);

    REQUIRE(message.find("97") != std::string::npos);
    REQUIRE(message.find("250") != std::string::npos);
    REQUIRE(message.find("metal") != std::string::npos);
}
