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

#include "MeshReconstruction.hpp"

void visualization3D() {
    // 1. --- Create the main scene geometry ---
    // The box is centered at (0,0,0) with side length 1.0.
    // It extends from -0.5 to +0.5 on all axes.
    auto mesh_ptr = open3d::geometry::TriangleMesh::CreateBox(2, 1, 5);
    mesh_ptr->ComputeVertexNormals();
    mesh_ptr->PaintUniformColor({0.7, 0.5, 0.9}); // Purple

    // 2. --- Perform Ray Picking and create visualization geometries ---
    open3d::utility::LogInfo("Performing ray picking tests...");

    // A list to hold all geometries for visualization
    std::vector<std::shared_ptr<const open3d::geometry::Geometry>> geometries;
    geometries.push_back(mesh_ptr);

    // Test case 1: A ray that should HIT the cube
    Eigen::Vector3d origin1(-2.0, 0.5, 0.0);
    Eigen::Vector3d dir1(sqrt(2)/2, 0.0, sqrt(2)/2);
    auto intersection1 = mdx::geometry::RayPick(mesh_ptr, origin1, dir1);

    // Create a line to visualize the ray using the correct constructor
    std::vector<Eigen::Vector3d> points1 = {origin1, origin1 + dir1 * 3.0};
    std::vector<Eigen::Vector2i> lines1  = {{0, 1}};
    auto ray_line1 = std::make_shared<open3d::geometry::LineSet>(points1, lines1);
    ray_line1->PaintUniformColor({0.0, 0.0, 1.0}); // Blue for hit-test
    geometries.push_back(ray_line1);

    if (intersection1) {
        open3d::utility::LogInfo("Ray 1 HIT at ({}, {}, {})", intersection1->x(), intersection1->y(), intersection1->z());
        // Create a small sphere to mark the intersection point
        auto hit_sphere = open3d::geometry::TriangleMesh::CreateSphere(0.05);
        hit_sphere->Translate(*intersection1);
        hit_sphere->PaintUniformColor({0.0, 1.0, 0.0}); // Green
        geometries.push_back(hit_sphere);
    } else {
        open3d::utility::LogWarning("Ray 1 MISSED");
    }

    // Test case 2: A ray that should MISS the cube
    Eigen::Vector3d origin2(0.0, 2.0, 0.0);
    Eigen::Vector3d dir2(1.0, 0.0, 0.0);
    auto intersection2 = mdx::geometry::RayPick(mesh_ptr, origin2, dir2);

    // Create a line to visualize the ray using the correct constructor
    std::vector<Eigen::Vector3d> points2 = {origin2, origin2 + dir2 * 3.0};
    std::vector<Eigen::Vector2i> lines2  = {{0, 1}};
    auto ray_line2 = std::make_shared<open3d::geometry::LineSet>(points2, lines2);
    ray_line2->PaintUniformColor({1.0, 0.0, 0.0}); // Red for miss
    geometries.push_back(ray_line2);

    if (intersection2) {
        open3d::utility::LogWarning("Ray 2 HIT (unexpectedly)");
    } else {
        open3d::utility::LogInfo("Ray 2 MISSED (as expected)");
    }


    // 3. --- Visualize the entire scene ---
    open3d::utility::LogInfo("Starting visualization...");
    open3d::visualization::Visualizer visualizer;
    visualizer.CreateVisualizerWindow("Ray Picking Visualization", 1024, 768);

    for (const auto& geom : geometries) {
        visualizer.AddGeometry(geom);
    }

    // Set a nice camera view
    visualizer.GetViewControl().SetFront({-0.5, -0.8, 0.2});
    visualizer.GetViewControl().SetLookat({0, 0, 0});
    visualizer.GetViewControl().SetUp({0, 0, 1});
    visualizer.GetViewControl().SetZoom(0.8);

    visualizer.Run();
    visualizer.DestroyVisualizerWindow();
}

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
    viper.setOffset(0.157, 0, 0);

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

    viper.initTransforms();

    long long counter = 1;

    while (!done) {
        if (counter % 150 == 0) {
            viper.initTransforms();
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
//    visualization3D();

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