#include <iostream>
#include "viper_ui.h"
#include "ProbeProfile.hpp"
#include "Calibrate.hpp"
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
#include <fstream>
#include <cstdint>

using namespace std;

int launchFoxglove(std::string config_filename) {
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
    std::vector<mdx::ProbeProfile> probe_profiles;
    float min_contact_force = 0.35;
    bool use_hardware_contact = true;
    bool contact_require_f1 = true;
    bool contact_require_f2 = true;
    bool contact_require_f3 = true;
    bool contact_require_f4 = true;
    bool generate_geometry = true;
    std::string pressure_usb_id = "";   // "VID:PID" hex; required, no default

    // The probe profiles are required: the tip offset depends on which probe is
    // fitted, and there is no safe default to fall back on. Running without
    // them would misplace the tip silently, so a missing or malformed config is
    // fatal rather than defaulted.
    if (!std::filesystem::exists(config_filename))
    {
        cerr << "Config file not found: " << config_filename << "\n"
             << "A config file is now required because it carries the \"probe_profiles\" "
                "block that maps the connected EM sensor count to a probe tip offset.\n"
             << "See the \"Probe profiles\" section of README.md.\n";
        return 1;
    }

    {
        cout << "Parsing config file: " << config_filename << endl;
        std::ifstream f(config_filename);
        json settings;
        try
        {
            settings = json::parse(f);
        }
        catch (const std::exception &e)
        {
            cerr << "Could not parse config file " << config_filename << ": " << e.what() << "\n";
            return 1;
        }

        try
        {
            probe_profiles = mdx::parseProbeProfiles(settings);
        }
        catch (const std::exception &e)
        {
            cerr << "Invalid probe configuration in " << config_filename << ": " << e.what() << "\n";
            return 1;
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
        if (settings.contains("generate_geometry"))
        {
            generate_geometry = settings["generate_geometry"];
        }

        cout << "    Done parsing.\n";
    }

    cout << "\nRuntime settings:\n";
    cout << "    probe_profiles:" << endl;
    for (const auto &profile : probe_profiles)
    {
        cout << "        " << mdx::describeProfile(profile) << endl;
    }
    cout << "    pressure_usb_id:" << pressure_usb_id << endl;

    cout << "    minimum_contact_force:" << min_contact_force << endl;
    cout << "    use_hardware_contact:" << use_hardware_contact << endl;
    cout << "    contact_require_f1:" << contact_require_f1 << endl;
    cout << "    contact_require_f2:" << contact_require_f2 << endl;
    cout << "    contact_require_f3:" << contact_require_f3 << endl;
    cout << "    contact_require_f4:" << contact_require_f4 << endl;
    cout << "    generate_geometry:" << generate_geometry << endl;
    // ---- End of runtime config parsing ---- //

    std::filesystem::path mcapPath = "viper.mcap";
    std::filesystem::remove(mcapPath);

    auto fgInterface = FoxgloveInterface{"viper.mcap"};
    // Must be set before the Viper is constructed: its constructor starts
    // streaming, and publishPose (geometry accumulation) can fire immediately.
    fgInterface.setGenerateGeometry(generate_geometry);
    std::this_thread::sleep_for(1000ms);

    // The profiles go in through the constructor: it starts the read threads,
    // so anything set afterwards would miss the first frames.
    Viper viper{&fgInterface, probe_profiles, 10, 100};

    std::atomic_bool done = false;
    sigint_handler = [&]
    {
        done = true;
    };

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
        // Raised when the run cannot continue safely -- currently only the
        // device reporting units this program would misinterpret. Stopping is
        // the point: carrying on would publish plausible, wrongly-scaled poses.
        if (viper.hasFatalError()) {
            cerr << "Stopping: " << viper.fatalErrorMessage() << "\n";
            return 1;
        }

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

    return 0;
}


int main(int argc, char** argv) {

    std::string config_filename = "viper-config.json";
    bool calibrate = false;

    for (int i = 1; i < argc; i++) {
        const std::string arg = argv[i];

        if (arg == "--calibrate") {
            calibrate = true;
        } else if (arg == "--help" || arg == "-h") {
            cout << "Usage: viper [--calibrate] [config-file]\n\n"
                 << "  --calibrate   Run the guided probe tip calibration and update the\n"
                 << "                config. Unlike a normal run this does not require an\n"
                 << "                existing probe_profiles entry for the connected sensor\n"
                 << "                count, so a probe can be calibrated for the first time.\n"
                 << "  config-file   Defaults to viper-config.json in the working directory.\n";
            return 0;
        } else {
            config_filename = arg;
        }
    }

    // Calibration deliberately bypasses the probe_profiles requirement: the
    // whole point is to produce that entry, and the normal path refuses to
    // start without it.
    if (calibrate) {
        return mdx::runCalibration(config_filename);
    }

    return launchFoxglove(config_filename);
}
