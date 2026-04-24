#include <iostream>
#include "viper_ui.h"
#include "Viper.hpp"
#include "SerialForce.hpp"
#include "FoxgloveInterface.hpp"

#include "foxglove/foxglove.hpp"

#include <chrono>
using namespace std::literals::chrono_literals;

#include <nlohmann/json.hpp>
using json = nlohmann::json;

#include <csignal>
#include <functional>
#include <filesystem>

using namespace std;

void launchFoxglove(std::string config_filename) {
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
   
    float offset_x = 0.150;
    float offset_y = 0.0;
    float offset_z = 0.0;
    float min_contact_force = 0.35;
    std::string &&pressure_port = "/dev/ttyACM0";

    if (std::filesystem::exists(config_filename))
    {
        cout << "Parsing config file: " << config_filename << endl;
        std::ifstream f(config_filename);
        json settings = json::parse(f);
        if (settings.contains("offset_x"))
        {
            offset_x = settings["offset_x"];
        }
        if (settings.contains("offset_y"))
        {
            offset_y = settings["offset_y"];
        }
        if (settings.contains("offset_z"))
        {
            offset_z = settings["offset_z"];
        }
        if (settings.contains("minimum_contact_force"))
        {
            // min_contact_force = settings["minimum_contact_force"];
            cout << "    Warning: minimum_contact_force not used at present\n";
        }
        if (settings.contains("pressure_device_port"))
        {
            pressure_port = settings["pressure_device_port"];
        }

        cout << "    Done parsing.\n";
    }
    else
    {
        cout << "No config file found. using default values\n";
    }

    cout << "\nRuntime settings:\n";
    cout << "    offset_x:" << offset_x << endl;
    cout << "    offset_y:" << offset_y << endl;
    cout << "    offset_z:" << offset_z << endl;
    // cout << "    minimum_contact_force:" << min_contact_force << endl;
    cout << "    pressure_device_port:" << pressure_port << endl;

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


int main(int argc, char** argv) {

    std::string config_filename = "viper-config.json";
    if (argc > 1){
        config_filename = argv[1];
    }
    launchFoxglove(config_filename);

    return 0;
}
