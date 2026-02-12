#include <iostream>
#include "viper_ui.h"
#include "Viper.hpp"
#include "SerialForce.hpp"
#include "FoxgloveInterface.hpp"

#include "foxglove/foxglove.hpp"

#include <chrono>
using namespace std::literals::chrono_literals;

#include <csignal>
#include <functional>
#include <filesystem>

void launchFoxglove() {
    foxglove::setLogLevel(foxglove::LogLevel::Debug);

    static std::function<void()> sigint_handler;

    std::signal(SIGTERM, [](int) {
        if (sigint_handler) {
            sigint_handler();
        }
    });

    std::signal(SIGINT, [](int) {
        if (sigint_handler) {
            sigint_handler();
        }
    });

    std::filesystem::path mcapPath = "viper.mcap";
    std::filesystem::remove(mcapPath);

    auto fgInterface = FoxgloveInterface{"viper.mcap"};
    std::this_thread::sleep_for(1000ms);

    Viper viper{&fgInterface, 10, 100};
    viper.setOffset(0, 0, 0);

    // SerialForce serialForce{fgInterface};
    // serialForce.setContactCallback(
    //     [](mdx::RawForce &force) {
    //         return force.f4 > 0.35;
    //     }
    // );
    // serialForce.init("/dev/cu.usbmodem101");

    std::atomic_bool done = false;
    sigint_handler = [&]
    {
        done = true;
    };

//    std::this_thread::sleep_for(1000ms);

    viper.initTransforms();

    long long counter = 1;

    while (!done) {
        if (counter % 300 == 0) {
            viper.initTransforms();
            fgInterface.publishPointClouds();
            // These functions below are broken for now
            // Refer to https://github.com/helkebir/cavitary instead.
//            fgInterface.publishMesh();
//            fgInterface.publishMeshModel();
            fgInterface.logDebug("Updated point clouds");
        }

        counter++;
        std::this_thread::sleep_for(33ms);
    }
}


int main() {
    launchFoxglove();

    return 0;
}
