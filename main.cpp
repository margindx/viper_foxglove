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
#include <cstdint>

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

    // ---- Parsing runtime config options ---- //
    float offset_x = 0.150;
    float offset_y = 0.0;
    float offset_z = 0.0;
    float min_contact_force = 0.35;
    bool use_hardware_contact = true;
    bool contact_require_f1 = true;
    bool contact_require_f2 = true;
    bool contact_require_f3 = true;
    bool contact_require_f4 = true;
    std::string pressure_usb_id = "";   // "VID:PID" hex; required, no default

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
            min_contact_force = settings["minimum_contact_force"];            
        }
        if (settings.contains("pressure_usb_id"))
        {
            pressure_usb_id = settings["pressure_usb_id"];
        }
        if (settings.contains("use_hardware_contact"))
        {
            use_hardware_contact = settings["use_hardware_contact"];
        }
        if (settings.contains("contact_require_f1"))
        {
            contact_require_f1 = settings["contact_require_f1"];
        }
        if (settings.contains("contact_require_f2"))
        {
            contact_require_f2 = settings["contact_require_f2"];
        }
        if (settings.contains("contact_require_f3"))
        {
            contact_require_f3 = settings["contact_require_f3"];
        }
        if (settings.contains("contact_require_f4"))
        {
            contact_require_f4 = settings["contact_require_f4"];
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
    cout << "    pressure_usb_id:" << pressure_usb_id << endl;
    cout << "    minimum_contact_force:" << min_contact_force << endl;
    cout << "    use_hardware_contact:" << use_hardware_contact << endl;
    cout << "    contact_require_f1:" << contact_require_f1 << endl;
    cout << "    contact_require_f2:" << contact_require_f2 << endl;
    cout << "    contact_require_f3:" << contact_require_f3 << endl;
    cout << "    contact_require_f4:" << contact_require_f4 << endl;
    // ---- End of runtime config parsing ---- //

    std::filesystem::path mcapPath = "viper.mcap";
    std::filesystem::remove(mcapPath);

    auto fgInterface = FoxgloveInterface{"viper.mcap"};
    std::this_thread::sleep_for(1000ms);

    Viper viper{&fgInterface, 10, 100};

    std::atomic_bool done = false;
    sigint_handler = [&]
    {
        done = true;
    };

    // initialize viper with all settings
    viper.setOffset(offset_x, offset_y, offset_z); // slim: (0.150, 0, 0); YOP: (0.157, 0, 0)
    viper.initTransforms();

    SerialForce serialForce{fgInterface};

    // Parse pressure_usb_id ("VID:PID", hex). Required — there is no fallback
    // port. If missing or malformed, the force sensor stays disabled (degraded).
    auto parseUsbId = [](const std::string &s, std::uint16_t &vid, std::uint16_t &pid) -> bool {
        const auto colon = s.find(':');
        if (colon == std::string::npos || colon == 0 || colon + 1 >= s.size()) return false;
        try {
            size_t n1 = 0, n2 = 0;
            unsigned long v = std::stoul(s.substr(0, colon), &n1, 16);
            unsigned long p = std::stoul(s.substr(colon + 1), &n2, 16);
            if (n1 != colon || n2 != s.size() - colon - 1) return false;   // trailing junk
            if (v > 0xFFFF || p > 0xFFFF) return false;
            vid = static_cast<std::uint16_t>(v);
            pid = static_cast<std::uint16_t>(p);
            return true;
        } catch (...) {
            return false;
        }
    };

    std::uint16_t pressure_vid = 0, pressure_pid = 0;
    if (parseUsbId(pressure_usb_id, pressure_vid, pressure_pid)) {
        serialForce.init(pressure_vid, pressure_pid, min_contact_force);
        serialForce.setUseHardwareContact(use_hardware_contact);
        serialForce.setRequireSensor(contact_require_f1, contact_require_f2, contact_require_f3, contact_require_f4);
    } else {
        fgInterface.logError(
            "pressure_usb_id (\"" + pressure_usb_id + "\") is missing or malformed "
            "(expected hex \"VID:PID\", e.g. \"2886:8064\"); force sensor disabled");
    }
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
