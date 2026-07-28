#include <iostream>
#include "viper_ui.h"
#include "ProbeProfile.hpp"
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
    std::string &&pressure_port = "/dev/ttyACM0";

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
        if (settings.contains("pressure_device_port"))
        {
            pressure_port = settings["pressure_device_port"];
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

    cout << "\nRuntime settings:\n";
    cout << "    probe_profiles:" << endl;
    for (const auto &profile : probe_profiles)
    {
        cout << "        " << mdx::describeProfile(profile) << endl;
    }
    cout << "    pressure_device_port:" << pressure_port << endl;
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
    serialForce.init(std::move(pressure_port), min_contact_force);
    serialForce.setUseHardwareContact(use_hardware_contact);
    serialForce.setRequireSensor(contact_require_f1, contact_require_f2, contact_require_f3, contact_require_f4);
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
    if (argc > 1){
        config_filename = argv[1];
    }

    return launchFoxglove(config_filename);
}
