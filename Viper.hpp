//
// Created by Hamza El-Kebir on 7/21/25.
//

#ifndef VIPER_VIPER_HPP
#define VIPER_VIPER_HPP

#include <stdexcept>
#include <thread>
#include <optional>
#include <mutex>
#include "viper_usb.h"
#include "viper_queue.h"
#include "ViperInterface.h"
#include "SensorData.hpp"
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

    Eigen::Vector3f offset_;

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
    explicit Viper(FoxgloveInterface* fgInterface=nullptr, size_t reconnectTries=0, size_t timeOutMs=5) :
        fbBuilder_(1024), offset_(Eigen::Vector3f::Zero()), fgInterface_(fgInterface), viperUsb{} {
        if (fgInterface == nullptr) {
            throw std::runtime_error("FoxgloveInterface delivered as nullptr");
        } else {
            fgInterface_ = fgInterface;
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
        worldFrameTransform_.child_frame_id = "world";
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

    void setOffset(Eigen::Vector3f &offset);

    void setOffset(float x=0, float y=0, float z=0);

    const Eigen::Vector3f& getOffset() const;

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

    void transformPose(foxglove::schemas::Pose &pose) {
        Eigen::Quaternion<double> quat{
            pose.orientation.value().w,
            pose.orientation.value().x,
            pose.orientation.value().y,
            pose.orientation.value().z
        };

        Eigen::Vector3d pos{
            pose.position.value().x,
            pose.position.value().y,
            pose.position.value().z
        };

        Eigen::Quaternion<double> offset{
            0,
            offset_.x(),
            offset_.y(),
            offset_.z()
        };

        auto rotatedOffset = quat * offset * quat.inverse();

        pose.position.value().x = pos.x() + rotatedOffset.x();
        pose.position.value().y = pos.y() + rotatedOffset.y();
        pose.position.value().z = pos.z() + rotatedOffset.z();
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
