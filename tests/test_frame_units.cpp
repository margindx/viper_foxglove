//
// Unit tests for decoding the units the Viper reports in each PNO frame.
//
// The values here are the raw 2-bit SFINFO fields, matching eViperPosUnits and
// eViperOriUnits in ViperInterface.h.
//

#include <catch2/catch_test_macros.hpp>

#include "FrameUnits.hpp"

using namespace mdx;

TEST_CASE("position units decode to the Viper enum", "[units]") {
    REQUIRE(decodeFrameUnits(0, 2).position == PositionUnits::Inch);
    REQUIRE(decodeFrameUnits(1, 2).position == PositionUnits::Foot);
    REQUIRE(decodeFrameUnits(2, 2).position == PositionUnits::Centimeter);
    REQUIRE(decodeFrameUnits(3, 2).position == PositionUnits::Meter);
}

TEST_CASE("orientation units decode to the Viper enum", "[units]") {
    REQUIRE(decodeFrameUnits(3, 0).orientation == OrientationUnits::EulerDegree);
    REQUIRE(decodeFrameUnits(3, 1).orientation == OrientationUnits::EulerRadian);
    REQUIRE(decodeFrameUnits(3, 2).orientation == OrientationUnits::Quaternion);
}

TEST_CASE("values outside the enums are not guessed at", "[units]") {
    // The orientation field is 2 bits but only has three enumerators, so 3 is
    // encodable and undefined. Anything wider would be a decode error.
    REQUIRE(decodeFrameUnits(3, 3).orientation == OrientationUnits::Unknown);
    REQUIRE(decodeFrameUnits(4, 2).position == PositionUnits::Unknown);
    REQUIRE_FALSE(decodeFrameUnits(3, 3).isSupported());
    REQUIRE_FALSE(decodeFrameUnits(4, 2).isSupported());
}

TEST_CASE("only metres plus quaternion is supported", "[units]") {
    SECTION("the one good combination") {
        REQUIRE(decodeFrameUnits(3, 2).isSupported());
    }

    SECTION("the factory defaults are not") {
        // POS_INCH and ORI_EULER_DEGREE are both zero, i.e. what an unconfigured
        // or factory-reset SEU reports. This is the case the guard exists for.
        REQUIRE_FALSE(decodeFrameUnits(0, 0).isSupported());
    }

    SECTION("right position units, wrong orientation units") {
        REQUIRE_FALSE(decodeFrameUnits(3, 0).isSupported());
        REQUIRE_FALSE(decodeFrameUnits(3, 1).isSupported());
    }

    SECTION("right orientation units, wrong position units") {
        REQUIRE_FALSE(decodeFrameUnits(0, 2).isSupported());
        REQUIRE_FALSE(decodeFrameUnits(1, 2).isSupported());
        REQUIRE_FALSE(decodeFrameUnits(2, 2).isSupported());
    }
}

TEST_CASE("the failure message names the actual problem", "[units]") {
    SECTION("inches and Euler degrees") {
        const auto message = unsupportedUnitsMessage(decodeFrameUnits(0, 0));

        REQUIRE(message.find("inches") != std::string::npos);
        REQUIRE(message.find("Euler degrees") != std::string::npos);
        REQUIRE(message.find("rescale") != std::string::npos);
        REQUIRE(message.find("CMD_UNITS") != std::string::npos);
    }

    SECTION("only the orientation is wrong") {
        const auto message = unsupportedUnitsMessage(decodeFrameUnits(3, 0));

        // Should not warn about rescaling when the position units are fine.
        REQUIRE(message.find("rescale") == std::string::npos);
        REQUIRE(message.find("Euler") != std::string::npos);
    }

    SECTION("only the position is wrong") {
        const auto message = unsupportedUnitsMessage(decodeFrameUnits(2, 2));

        REQUIRE(message.find("centimetres") != std::string::npos);
        REQUIRE(message.find("rescale") != std::string::npos);
        REQUIRE(message.find("misread as a quaternion") == std::string::npos);
    }
}

TEST_CASE("describe is human-readable", "[units]") {
    REQUIRE(describe(decodeFrameUnits(3, 2)) == "metres, quaternion");
    REQUIRE(describe(decodeFrameUnits(0, 0)) == "inches, Euler degrees");
}
