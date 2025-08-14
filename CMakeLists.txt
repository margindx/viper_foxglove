cmake_minimum_required(VERSION 3.28)
project(viper LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# TODO: Change this to a Python interpreter that has PIPython installed.
set(PYTHON_EXECUTABLE "/opt/homebrew/Caskroom/miniconda/base/bin/python")

find_package(pybind11 CONFIG REQUIRED)
find_package(Open3D REQUIRED)

# Fetch Foxglove SDK
include(FetchContent)
FetchContent_Declare(
        foxglove
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE
        # TODO: Copy the URL and SHA for the appropriate .zip file from here:
        # https://github.com/foxglove/foxglove-sdk/releases?q=sdk%2F&expanded=true
        URL https://github.com/foxglove/foxglove-sdk/releases/download/sdk%2Fv0.10.0/foxglove-v0.10.0-cpp-aarch64-apple-darwin.zip
        URL_HASH SHA256=14ded1c5c0e8386cd8c8b9371ae99852131883f69f62500ec6c987daf23f0724
)
FetchContent_MakeAvailable(foxglove)

include_directories(dep)

find_package(PkgConfig REQUIRED)
pkg_search_module(LIBUSB REQUIRED libusb-1.0)

pkg_search_module(RAYLIB libraylib)

pkg_search_module(ZEROMQ zeromq)


# Add the Homebrew library path if not found automatically
if (LIBUSB_FOUND)
    message(STATUS "libusb-1.0 found: ${LIBUSB_LIBRARIES}")
else()
    message(WARNING "libusb-1.0 not found, adding fallback paths")
    set(LIBUSB_INCLUDE_DIRS "/opt/homebrew/Cellar/libusb/1.0.27/include/libusb-1.0")
    set(LIBUSB_LIBRARIES "/opt/homebrew/Cellar/libusb/1.0.27/lib/libusb-1.0.dylib")
endif()

# Add the Homebrew library path if not found automatically
if (RAYLIB_FOUND)
    message(STATUS "raylib found: ${RAYLIB_LIBRARIES}")
else()
    message(WARNING "raylib not found, adding fallback paths")
    set(RAYLIB_INCLUDE_DIRS "/opt/homebrew/opt/raylib/include")
    set(RAYLIB_LIBRARIES "/opt/homebrew/opt/raylib/lib/libraylib.5.5.0.dylib")
endif()

# Add the Homebrew library path if not found automatically
if (ZEROMQ_FOUND)
    message(STATUS "zeromq found: ${ZEROMQ_LIBRARIES}")
else()
    message(WARNING "zeromq not found, adding fallback paths")
    set(ZEROMQ_INCLUDE_DIRS "/opt/homebrew/opt/zeromq/include")
    set(ZEROMQ_LIBRARIES "/opt/homebrew/opt/zeromq/lib/libzmq.5.dylib")
endif()

set(CMAKE_CXX_STANDARD 17)
#set(LIBUSB_INCLUDE_DIR /opt/homebrew/Cellar/libusb/1.0.27/include)
#set(LIBUSB_LIB_DIR /opt/homebrew/Cellar/libusb/1.0.27/lib)
#
link_directories(/opt/homebrew/Cellar/libusb/1.0.27/lib)
include_directories(/opt/homebrew/Cellar/libusb/1.0.27/include/libusb-1.0)


link_directories("/opt/homebrew/opt/zeromq/lib")
include_directories("/opt/homebrew/opt/zeromq/include")

link_directories("/opt/homebrew/opt/raylib/lib")
include_directories("/opt/homebrew/opt/raylib/include")


add_executable(viper main.cpp viper_ui.h viper_ui.cpp viper_usb.h viper_usb.cpp ViperInterface.h viper_queue.h viper_queue.cpp
        dep/serialib/serialib.cpp
        ZMQServer.cpp
        ZMQServer.hpp
        SensorData.cpp
        SensorData.hpp
        PointCloudGUI.cpp
        PointCloudGUI.hpp
        Viper.cpp
        Viper.hpp
        FoxgloveInterface.cpp
        FoxgloveInterface.hpp
        SerialForce.cpp
        SerialForce.hpp
        MeshReconstruction.cpp
        MeshReconstruction.hpp
        FoxgloveUtility.cpp
        FoxgloveUtility.hpp)
# VPif.h VPif.cpp VPcmdDefs.h

# Include libusb headers
target_include_directories(viper PRIVATE ${LIBUSB_INCLUDE_DIRS} ${ZEROMQ_INCLUDE_DIRS} ${RAYLIB_INCLUDE_DIRS})

# Link libusb libraries
target_link_libraries(viper PRIVATE ${LIBUSB_LIBRARIES} pthread ${ZEROMQ_LIBRARIES} ${RAYLIB_LIBRARIES})

# Add include directory for Foxglove SDK
target_include_directories(viper PRIVATE ${foxglove_SOURCE_DIR}/include)
target_include_directories(viper PRIVATE ${foxglove_SOURCE_DIR}/include/foxglove)

# Find all Foxglove SDK source files
file(GLOB FOXGLOVE_SOURCES CONFIGURE_DEPENDS
        "${foxglove_SOURCE_DIR}/src/*.cpp"
        "${foxglove_SOURCE_DIR}/src/server/*.cpp"
)

# Add Foxglove SDK source files
target_sources(viper PRIVATE ${FOXGLOVE_SOURCES})

# Pass additional compiler options if necessary
target_compile_options(viper PRIVATE ${LIBUSB_CFLAGS_OTHER} ${RAYLIB_CFLAGS_OTHER})

# Link against libfoxglove.a
target_link_libraries(viper PRIVATE ${foxglove_SOURCE_DIR}/lib/libfoxglove.a)

# Link against pybind11
target_link_libraries(viper PRIVATE pybind11::embed)

target_link_libraries(viper PRIVATE Open3D::Open3D)