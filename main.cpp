#include <iostream>
#include "viper_ui.h"
#include "ZMQServer.hpp"
#include "PointCloudGUI.hpp"
#include "Viper.hpp"
#include "SerialForce.hpp"
#include "FoxgloveInterface.hpp"

#include "foxglove/foxglove.hpp"

#include <chrono>
using namespace std::literals::chrono_literals;

#include <csignal>
#include <functional>
#include <filesystem>
#include <pybind11/embed.h>
namespace py = pybind11;
using namespace py::literals;

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

//    std::signal(SIGKILL, [](int) {
//        if (sigint_handler) {
//            sigint_handler();
//        }
//    });

    std::filesystem::path mcapPath = "viper.mcap";
    std::filesystem::remove(mcapPath);

    auto fgInterface = FoxgloveInterface{"viper.mcap"};
    std::this_thread::sleep_for(1000ms);

    Viper viper{&fgInterface, 10, 100};
    viper.setOffset(0.1, 0.01, 0);

//    SerialForce serialForce{fgInterface};
//    serialForce.init("/dev/cu.usbmodem1101");

    std::atomic_bool done = false;
    sigint_handler = [&]
    {
        done = true;
//        viper.close();
//        serialForce.close();
    };

//    std::this_thread::sleep_for(1000ms);

    long long counter = 1;

    while (!done) {
        if (counter % 150 == 0) {
            fgInterface.publishPointClouds();
            fgInterface.logDebug("Updated point clouds");
        }

        counter++;
        std::this_thread::sleep_for(33ms);
    }
}

void pythonTest() {
    py::scoped_interpreter guard{};
    /**
     * from pipython.pidevice.gcscommands import GCSCommands
     * from pipython.pidevice.gcsmessages import GCSMessages
     * from pipython.pidevice.interfaces.pisocket import PISocket
     * from pipython.pidevice.interfaces.piserial import PISerial
     */

    py::object GCSCommands = py::module_::import("pipython.pidevice.gcscommands").attr("GCSCommands");
    py::object GCSMessages = py::module_::import("pipython.pidevice.gcsmessages").attr("GCSMessages");
    py::object PISerial = py::module_::import("pipython.pidevice.interfaces.piserial").attr("PISerial");

    const std::string PI_SERIAL_PORT = "/dev/tty.usbserial-FT514Q6U";

    py::object pidevice; // Declare pidevice here to expose it

    py::object gateway = PISerial(py::arg("port") = PI_SERIAL_PORT, py::arg("baudrate") = 115200);

    double speed = 10.0; // mm/s
    double scan_x_length = 20.0;
    double scan_y_length = 20.0;
    double scan_stride = 0.35;

    py::list axes;
    axes.append("X");
    axes.append("Y");
    axes.append("Z");

    py::list servo_states;
    servo_states.append(1);
    servo_states.append(1);
    servo_states.append(1);

    // Equivalent to 'with PISerial(...) as gateway:'
    gateway.attr("__enter__")();
    try {
        py::object messages = GCSMessages(gateway);
        py::object gcs_commands = GCSCommands(messages);

        // Equivalent to 'with GCSCommands(...) as pidevice:'
        gcs_commands.attr("__enter__")();
        try {
            pidevice = gcs_commands; // Assign gcs_commands to pidevice
            py::print(pidevice.attr("qIDN")());

            // Set servo states
            pidevice.attr("SVO")(axes, servo_states);

            // Set speed
            py::str speed_str = py::str("VLS {:.1f}").format(speed);
            pidevice.attr("gcscommands").attr("send")(speed_str);

            // Start raster scan
            py::str scan_command_str = py::str("FSC X {:.1f} Y {:.1f} S {:.2f}").format(
                    scan_x_length, scan_y_length, scan_stride
            );
            pidevice.attr("gcscommands").attr("send")(scan_command_str);

        } catch (const py::error_already_set &e) {
            PyErr_Print();
            gcs_commands.attr("__exit__")(nullptr, nullptr, nullptr);
            throw;
        }
        gcs_commands.attr("__exit__")(nullptr, nullptr, nullptr);
    } catch (const py::error_already_set &e) {
        PyErr_Print();
        gateway.attr("__exit__")(nullptr, nullptr, nullptr);
        throw;
    }
    gateway.attr("__exit__")(nullptr, nullptr, nullptr);
}

int main() {
    launchFoxglove();
//    pythonTest();

    return 0;
}


/** Obsolete code
* //    ZMQServer::init(5555, ZMQServer::Publisher);
//    ZMQServer::serve();

//    ZMQServer::init(5555, ZMQServer::Subscriber);
//    ZMQServer::subscribeToSensor(1);
//    ZMQServer::subscribeToSensor(2);
//    ZMQServer::subscribeToSensor(3);
//    ZMQServer::subscribeToSensor(4);
//    ZMQServer::consume();

//    ZMQServer::verbose = false;
//    ZMQServer::init(5555, ZMQServer::Subscriber);
//    ZMQServer::subscribeToSensor(1);
//    ZMQServer::subscribeToSensor(2);
//    ZMQServer::subscribeToSensor(3);
//    ZMQServer::subscribeToSensor(4);
//    ZMQServer::log();

//    ZMQServer::init(5555, ZMQServer::Subscriber);
//    ZMQServer::subscribeToSensor(1);
//    ZMQServer::subscribeToSensor(2);
////    ZMQServer::subscribeToSensor(3);
////    ZMQServer::subscribeToSensor(4);
//    PointCloudGUI::launchRealTime();
*/