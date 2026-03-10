cmake_minimum_required(VERSION 3.28)
project(viper LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(Open3D REQUIRED)
find_package(Protobuf REQUIRED CONFIG)

# Fetch Foxglove SDK - automatically select based on OS and architecture
include(FetchContent)

# Detect platform and set appropriate Foxglove SDK URL and hash
set(FOXGLOVE_VERSION "0.10.0")
set(FOXGLOVE_BASE_URL "https://github.com/foxglove/foxglove-sdk/releases/download/sdk/v${FOXGLOVE_VERSION}")

# Normalize architecture names
set(FOXGLOVE_ARCH ${CMAKE_SYSTEM_PROCESSOR})
if(FOXGLOVE_ARCH STREQUAL "arm64" OR FOXGLOVE_ARCH STREQUAL "ARM64")
    set(FOXGLOVE_ARCH "aarch64")
elseif(FOXGLOVE_ARCH MATCHES "^(x86_64|amd64|AMD64)$")
    set(FOXGLOVE_ARCH "x86_64")
endif()

# Determine platform string and SHA256 hash
if(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
    # macOS
    if(FOXGLOVE_ARCH STREQUAL "aarch64")
        set(FOXGLOVE_PLATFORM "aarch64-apple-darwin")
        set(FOXGLOVE_SHA256 "14ded1c5c0e8386cd8c8b9371ae99852131883f69f62500ec6c987daf23f0724")
    elseif(FOXGLOVE_ARCH STREQUAL "x86_64")
        set(FOXGLOVE_PLATFORM "x86_64-apple-darwin")
        set(FOXGLOVE_SHA256 "REPLACE_WITH_ACTUAL_SHA256_FOR_MACOS_X86_64")
    else()
        message(FATAL_ERROR "Unsupported macOS architecture: ${CMAKE_SYSTEM_PROCESSOR}")
    endif()
elseif(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    # Linux
    if(FOXGLOVE_ARCH STREQUAL "x86_64")
        set(FOXGLOVE_PLATFORM "x86_64-unknown-linux-gnu")
        set(FOXGLOVE_SHA256 "209d34a8703d44a129c559493e1a3fb215888b4d5973622750ac28b1c345946c")
    elseif(FOXGLOVE_ARCH STREQUAL "aarch64")
        set(FOXGLOVE_PLATFORM "aarch64-unknown-linux-gnu")
        set(FOXGLOVE_SHA256 "REPLACE_WITH_ACTUAL_SHA256_FOR_LINUX_AARCH64")
    else()
        message(FATAL_ERROR "Unsupported Linux architecture: ${CMAKE_SYSTEM_PROCESSOR}")
    endif()
elseif(CMAKE_SYSTEM_NAME STREQUAL "Windows")
    # Windows
    if(FOXGLOVE_ARCH STREQUAL "x86_64")
        set(FOXGLOVE_PLATFORM "x86_64-pc-windows-msvc")
        set(FOXGLOVE_SHA256 "REPLACE_WITH_ACTUAL_SHA256_FOR_WINDOWS_X86_64")
    elseif(FOXGLOVE_ARCH STREQUAL "aarch64")
        set(FOXGLOVE_PLATFORM "aarch64-pc-windows-msvc")
        set(FOXGLOVE_SHA256 "REPLACE_WITH_ACTUAL_SHA256_FOR_WINDOWS_AARCH64")
    else()
        message(FATAL_ERROR "Unsupported Windows architecture: ${CMAKE_SYSTEM_PROCESSOR}")
    endif()
else()
    message(FATAL_ERROR "Unsupported operating system: ${CMAKE_SYSTEM_NAME}")
endif()

# Construct full URL
set(FOXGLOVE_URL "${FOXGLOVE_BASE_URL}/foxglove-v${FOXGLOVE_VERSION}-cpp-${FOXGLOVE_PLATFORM}.zip")

message(STATUS "Detected platform: ${CMAKE_SYSTEM_NAME} ${CMAKE_SYSTEM_PROCESSOR}")
message(STATUS "Downloading Foxglove SDK from: ${FOXGLOVE_URL}")

# Check if SHA256 needs to be updated
if(FOXGLOVE_SHA256 MATCHES "^REPLACE_WITH_ACTUAL_SHA256")
    # Allow build to continue without hash verification (not recommended for production)
    message(WARNING "SHA256 hash not set for ${FOXGLOVE_PLATFORM}. Please download manually and update CMakeLists.txt.")
    message(WARNING "Download from: ${FOXGLOVE_URL}")
    message(WARNING "Get SHA256 from: https://github.com/foxglove/foxglove-sdk/releases?q=sdk%2F&expanded=true")
    FetchContent_Declare(
        foxglove
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE
        URL ${FOXGLOVE_URL}
    )
else()
    FetchContent_Declare(
        foxglove
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE
        URL ${FOXGLOVE_URL}
        URL_HASH SHA256=${FOXGLOVE_SHA256}
    )
endif()

FetchContent_MakeAvailable(foxglove)

include_directories(dep)

find_package(PkgConfig REQUIRED)
pkg_search_module(LIBUSB REQUIRED libusb-1.0)


# TODO: Make libusb finding automated.
# Add the Homebrew library path if not found automatically
if (LIBUSB_FOUND)
    message(STATUS "libusb-1.0 found: ${LIBUSB_LIBRARIES}")
else()
    message(WARNING "libusb-1.0 not found, adding fallback paths")
    set(LIBUSB_INCLUDE_DIRS "/opt/homebrew/Cellar/libusb/1.0.27/include/libusb-1.0")
    set(LIBUSB_LIBRARIES "/opt/homebrew/Cellar/libusb/1.0.27/lib/libusb-1.0.dylib")
endif()

link_directories(/opt/homebrew/Cellar/libusb/1.0.27/lib)
include_directories(/opt/homebrew/Cellar/libusb/1.0.27/include/libusb-1.0)

set(CMAKE_CXX_STANDARD 17)

add_executable(viper main.cpp viper_ui.h viper_ui.cpp viper_usb.h viper_usb.cpp ViperInterface.h viper_queue.h viper_queue.cpp
        dep/serialib/serialib.cpp
        SensorData.cpp
        SensorData.hpp
        Viper.cpp
        Viper.hpp
        FoxgloveInterface.cpp
        FoxgloveInterface.hpp
        SerialForce.cpp
        SerialForce.hpp
        MeshReconstruction.cpp
        MeshReconstruction.hpp
        FoxgloveUtility.cpp
        FoxgloveUtility.hpp
        FoxgloveMacros.hpp
        DynamicProtobufDecoder.cpp
        DynamicProtobufDecoder.hpp)
# VPif.h VPif.cpp VPcmdDefs.h

# Include libusb headers
target_include_directories(viper PRIVATE ${LIBUSB_INCLUDE_DIRS})

# Link libusb libraries
target_link_libraries(viper PRIVATE ${LIBUSB_LIBRARIES} pthread)

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
target_compile_options(viper PRIVATE ${LIBUSB_CFLAGS_OTHER})

# Link against libfoxglove.a
target_link_libraries(viper PRIVATE ${foxglove_SOURCE_DIR}/lib/libfoxglove.a)

target_link_libraries(viper PRIVATE Open3D::Open3D)

target_link_libraries(viper PRIVATE protobuf::libprotobuf)
