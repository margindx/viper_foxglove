//
// Created by Hamza El-Kebir on 7/21/25.
//

#ifndef VIPER_VIPER_HPP
#define VIPER_VIPER_HPP

#include <stdexcept>
#include <thread>
#include <optional>
#include <mutex>
#include <atomic>
#include "viper_usb.h"
#include "viper_queue.h"
#include "ViperInterface.h"
#include "SensorData.hpp"
#include "ProbeProfile.hpp"
#include "FrameUnits.hpp"
#include "DeviceState.hpp"
#include "FoxgloveInterface.hpp"

#include "schema/foxglove/Time_generated.h"
#include "schema/foxglove/PoseInFrame_generated.h"
#include "schema/foxglove/PosesInFrame_generated.h"
#include "flatbuffers/flatbuffers.h"
#include "Eigen/Dense"
#include "Eigen/Geometry"

constexpr uint32_t CRC_SIZE=sizeof(uint32_t);
constexpr uint32_t CMD_DELAY=200;

class Viper {
protected:
    bool keepReading = true;
    bool isContinuous = false;
    viper_usb viperUsb;
    viper_queue pnoQueue_;
    viper_queue cmdQueue_;

    std::vector<float> contactPointBuffer_{};
    std::mutex contactPointBufferMtx_;

    static uint16_t crcTable_[256];

    static uint32_t calculateCrc16(uint8_t *b, uint32_t len);

    /// Probe profiles from the config, one per supported sensor count.
    std::vector<mdx::ProbeProfile> profiles_;
    /// The profile chosen from the first frame that carried sensors, and the
    /// count it was chosen for. Latched: the tip offset must not change under
    /// the operator mid-run just because a connector went intermittent.
    /// Aliases into profiles_, which is never mutated after construction.
    const mdx::ProbeProfile *activeProfile_ = nullptr;
    int latchedSensorCount_ = -1;

    /// Sensor count from the most recent frame, independent of whether a
    /// profile was ever latched. Calibration needs this precisely in the case
    /// where no profile exists to latch.
    std::atomic<int> lastSensorCount_{-1};

    /// Rate-limiting counters for the repeating fault paths.
    uint64_t noProfileFrames_ = 0;
    uint64_t countMismatchFrames_ = 0;
    uint64_t unusableFrames_ = 0;

    /// The units the device reports are only knowable once a frame arrives, so
    /// they are checked on the first one rather than at construction.
    bool unitsChecked_ = false;

    /// Likewise the SEU's own tip offset, which is per-sensor and so can only
    /// be interrogated once the sensor count is known.
    bool deviceTipOffsetsChecked_ = false;

    /// Send a configuration GET and copy the payload out. False when the device
    /// did not answer in a form we can read, which is treated as "unknown"
    /// rather than as a default value.
    bool queryConfig(uint32_t cmd, uint32_t arg1, void *payload, uint32_t payloadSize);

    /// Read the device's configuration once, before trusting any of its data.
    /// Settings that silently transform the geometry stop the run; the rest are
    /// logged so a recording carries the configuration it was made under.
    void readDeviceState(uint32_t nSensors);

    /// Per-frame checks on fields the SEU sends but we would otherwise discard.
    void monitorFrame(SENFRAMEDATA *pfd_all, uint32_t nSensors, uint32_t frameCounter);

    /// Frame-counter continuity, so dropped frames are visible rather than
    /// showing up only as an unexplained rate.
    bool haveFrameCounter_ = false;
    uint32_t lastFrameCounter_ = 0;
    uint64_t frameGapEvents_ = 0;
    uint64_t framesDropped_ = 0;

    /// Frames seen above the distortion warning level.
    uint64_t distortedFrames_ = 0;

    /// Calibration runs before a probe has a profile, so the usual "no profile"
    /// complaint is expected there rather than a fault.
    bool calibrationMode_ = false;

    /// Most recent fused sensor pose, before any tip transform. This is what
    /// calibration must solve against: the pose the offset gets added to.
    std::mutex fusedPoseMtx_;
    std::optional<mdx::Pose> latestFusedPose_;

    /// Set when the run cannot continue safely. The publish thread stops and
    /// main is expected to notice and exit non-zero.
    std::atomic_bool fatalError_{false};
    std::mutex fatalErrorMtx_;
    std::string fatalErrorMessage_;

    void raiseFatalError(const std::string &message) {
        {
            std::lock_guard<std::mutex> guard{fatalErrorMtx_};
            fatalErrorMessage_ = message;
        }
        fgInterface_->logError("Fatal: " + message);
        std::cerr << "Fatal: " << message << std::endl;
        fatalError_ = true;
    }

    flatbuffers::FlatBufferBuilder fbBuilder_;
    FoxgloveInterface *fgInterface_;
    foxglove::schemas::LinePrimitive linePrimitive_;
    foxglove::schemas::ArrowPrimitive arrowPrimitive_;
    foxglove::schemas::PosesInFrame posesInFrame_;
    foxglove::schemas::PoseInFrame hhPose_;

    foxglove::schemas::FrameTransform viperFrameTransform_;
    foxglove::schemas::FrameTransform worldFrameTransform_;

    std::thread continuousPublishThread_;
    std::thread usbReadThread_;

    std::mutex poseMutex_;
    std::vector<Eigen::Vector3f> positions_;
    std::vector<Eigen::Quaternionf> quaternions_;
    std::vector<Eigen::Vector3f> velocities_;
    std::vector<float> distanceTraveled_;
    std::optional<std::chrono::time_point<std::chrono::system_clock>> lastSampleTime_;
public:
    /// The profiles must be supplied here rather than through a setter: the
    /// constructor starts the USB read and continuous-publish threads, so a
    /// profile set afterwards would arrive too late for the first frames.
    /// `calibrationMode` allows construction with no profiles at all, which is
    /// how a probe that has never been calibrated gets bootstrapped: there is
    /// no offset to publish yet, but the fused pose is still needed.
    explicit Viper(FoxgloveInterface* fgInterface, std::vector<mdx::ProbeProfile> profiles,
                   size_t reconnectTries=0, size_t timeOutMs=5, bool calibrationMode=false) :
        fbBuilder_(1024), profiles_(std::move(profiles)), calibrationMode_(calibrationMode),
        fgInterface_(fgInterface), viperUsb{} {
        if (fgInterface == nullptr) {
            throw std::runtime_error("FoxgloveInterface delivered as nullptr");
        } else {
            fgInterface_ = fgInterface;
        }

        if (profiles_.empty() && !calibrationMode_) {
            throw std::runtime_error("Viper constructed with no probe profiles");
        }

        bool connected = false;
        for (size_t i=0; i < reconnectTries+1; i++) {
            try {
                connect();
                connected = true;
            } catch(...) {
                std::cout << "Connection try " << i << " failed..." << std::endl;
                fgInterface_->logWarning("Viper USB connection attempt failed; retrying");
                connected = false;
            }

            if (connected)
                break;

            std::this_thread::sleep_for(std::chrono::milliseconds{timeOutMs});
        }

        for (size_t i=0; i < reconnectTries+1; i++) {
            try {
                startContinuousRead();
                connected = true;
            } catch(...) {
                std::cout << "Starting continuous try " << i << " failed..." << std::endl;
                fgInterface_->logWarning("Viper USB continuous read attempt failed; retrying");
                connected = false;
            }

            if (connected)
                break;

            std::this_thread::sleep_for(std::chrono::milliseconds{timeOutMs});
        }

        if (connected)
            fgInterface_->logInfo("Viper USB connection established");
        else
            fgInterface_->logWarning("Viper USB connection failed after reconnections attempts");

        initTransforms();
        initLinePrimitive();
        initArrowPrimitive();
        initPoseInFrame();
    }

    ~Viper() {
        close();
    }

    void close() {
        keepReading = false;
        isContinuous = false;
        continuousPublishThread_.join();
        // TODO: Add continuous printing shutdown commands for Viper.
        usbReadThread_.join();
    }

    void initTransforms() {
        worldFrameTransform_.parent_frame_id = "world";
        fgInterface_->publishWorldTransform(worldFrameTransform_);

        viperFrameTransform_.parent_frame_id = "world";
        viperFrameTransform_.child_frame_id = "viper";
        viperFrameTransform_.rotation.emplace(foxglove::schemas::Quaternion{
            1, 0, 0, 0
        });
        // TODO: Define translation with respect to robotic arm base

        fgInterface_->publishViperTransform(viperFrameTransform_);
    }

    void initLinePrimitive() {
        linePrimitive_.color = foxglove::schemas::Color{
            172.f/255, 232.f/255, 88.f/255, 1.0
        };
        linePrimitive_.thickness = 0.05;
        linePrimitive_.scale_invariant = false;
        linePrimitive_.type = foxglove::schemas::LinePrimitive::LineType::LINE_STRIP;
    }

    void initArrowPrimitive() {
        arrowPrimitive_.color.emplace(foxglove::schemas::Color{
            172.f/255, 232.f/255, 88.f/255, 1.0
        });
        arrowPrimitive_.pose.emplace(foxglove::schemas::Pose{});
    }

    void initPoseInFrame() {
        hhPose_.frame_id = "viper";
        posesInFrame_.frame_id = "viper";
    }

    /// The most recent fused sensor pose, before any tip transform is applied.
    /// Empty until the first frame that fuses successfully. This is the pose
    /// calibration solves against, so that the offset it produces is the offset
    /// this exact fusion will later have added to it.
    std::optional<mdx::Pose> latestFusedPose() {
        std::lock_guard<std::mutex> guard{fusedPoseMtx_};
        return latestFusedPose_;
    }

    /// Sensor count from the most recent frame, or -1 before any has arrived.
    /// Unlike the latched profile count this is available with no profile
    /// configured, which is the state calibration runs in.
    int lastSensorCount() const { return lastSensorCount_; }

    /// True when the run has hit a condition it cannot continue safely from,
    /// e.g. the device reporting units this program would misinterpret.
    bool hasFatalError() const { return fatalError_; }

    std::string fatalErrorMessage() {
        std::lock_guard<std::mutex> guard{fatalErrorMtx_};
        return fatalErrorMessage_;
    }

    void connect();

    void readUsb(viper_usb *pvpr);

    void startContinuousRead();

    void publishContinuous();

    static bool hasPnoPreamble(uint8_t *respPkg) {
        return *(uint32_t*) respPkg == VIPER_PNO_PREAMBLE;
    }

    static bool validateCrc(const uint32_t crc, const uint8_t *pkg, const uint32_t len) {
        return crc == *(uint32_t*)(pkg + len);
    }

    static bool checkAck(const uint8_t *pkg, const uint32_t offset=16) {
        return *(uint32_t*)(pkg+offset) == CMD_ACTION_ACK;
    }

    static SensorData pnoToSensorData(SENFRAMEDATA *pfd);

    static void pnoToPoseInFrame(SENFRAMEDATA *pfd, flatbuffers::FlatBufferBuilder &builder);
    static void pnoToPosesInFrame(SENFRAMEDATA *pfd, uint32_t nSensors, flatbuffers::FlatBufferBuilder &builder);

    void pnoToPoseInFrame(SENFRAMEDATA *pfd);

    void pnoToPosesInFrame(SENFRAMEDATA* pfd, uint32_t nSensors);

    void pnoToFoxgloveSceneUpdate(SENFRAMEDATA* pfd, uint32_t nSensors);

    SENFRAMEDATA *pnoTransformInBodyFrame(SENFRAMEDATA *pfd);

    SENFRAMEDATA *pnoOffset(SENFRAMEDATA *pfd);

    /// Convert an mdx::Pose into the Foxglove schema type. Note that
    /// foxglove::schemas::Quaternion is ordered (x, y, z, w).
    static foxglove::schemas::Pose toFoxglovePose(const mdx::Pose &pose) {
        foxglove::schemas::Pose out;
        out.position.emplace(foxglove::schemas::Vector3{
            pose.position.x(),
            pose.position.y(),
            pose.position.z()
        });
        out.orientation.emplace(foxglove::schemas::Quaternion{
            pose.orientation.x(),
            pose.orientation.y(),
            pose.orientation.z(),
            pose.orientation.w()
        });

        return out;
    }

    mdx::SwingTwist computeSwingTwist(const foxglove::schemas::Pose &pose) const {
        typedef double T;

        T qx = static_cast<T>(pose.orientation.value().x);
        T qy = static_cast<T>(pose.orientation.value().y);
        T qz = static_cast<T>(pose.orientation.value().z);
        T qw = static_cast<T>(pose.orientation.value().w);

//        Eigen::Vector3<T> nor = {
//            1 - 2*(qy*qy + qz*qz),
//            2*(qx*qy + qw*qz),
//            2*(qx*qz - qw*qy),
//        };

        // NOTE: Computes rotation of y-axis by quaternion.
        Eigen::Vector3<T> nor = {
                2*(qx*qy - qw*qz),
                1 - 2*(qx*qx + qz*qz),
                2*(qy*qz + qx*qw),
        };

        T d = nor.x()*qx + nor.y()*qy + nor.z()*qz;
        T twist = 2*atan2(d, qw);

        return {
            nor.x(), nor.y(), nor.z(), twist
        };
    }
};


#endif //VIPER_VIPER_HPP
