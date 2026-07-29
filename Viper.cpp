//
// Created by Hamza El-Kebir on 7/21/25.
//

#include "Viper.hpp"

uint16_t Viper::crcTable_[256] =
{
0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
};

void Viper::connect() {
    if (viperUsb.usb_connect() != 0) {
        fgInterface_->logError("Error connecting to Viper over USB");
        throw std::runtime_error("Error connecting to Viper over USB");
    }

    usbReadThread_ = std::thread{&Viper::readUsb, this, &viperUsb};
}

void Viper::readUsb(viper_usb *pvpr) {
    constexpr uint32_t kRespSize = sizeof(uint32_t)*11+sizeof(SENFRAMEDATA)*16+CRC_SIZE; // max no. of sensors
    // Wide enough that a bulk transfer returns a whole frame (ending on its
    // short-packet boundary) rather than timing out mid-frame; keeps the reads
    // frame-aligned so each transfer starts on a preamble. No sleep: the read
    // blocks here up to the timeout, pacing the loop to the device's rate.
    constexpr unsigned int kReadTimeoutMs = 5;

    uint8_t* respPkg = new uint8_t[kRespSize];
    uint32_t br;
    uint64_t misaligned = 0;   // transfers dropped for not starting on a frame boundary

    while (keepReading) {
        br = pvpr->usb_rec_resp(respPkg, kRespSize, kReadTimeoutMs);

        if (!br)
            continue;   // nothing arrived within the timeout

        uint32_t preamble = *(uint32_t*) respPkg;
        if (preamble == VIPER_PNO_PREAMBLE) {
            pnoQueue_.push(respPkg, br);
        } else if (preamble == VIPER_CMD_PREAMBLE) {
            cmdQueue_.push(respPkg, br);
        } else {
            // Transfer did not start on a known frame boundary (e.g. a fragment
            // left over from a mid-frame timeout). Pushing it would corrupt byte
            // alignment in the queue, so drop it and report periodically.
            if ((++misaligned % 100) == 1) {
                std::stringstream ss;
                ss << "Viper USB read misaligned (" << misaligned
                   << " transfer(s) dropped): no PNO/CMD preamble at start";
                fgInterface_->logWarning(ss.str());
            }
        }
    }

    delete[] respPkg;
}

void Viper::startContinuousRead() {
    uint8_t cmdPkg[32];
    uint8_t respPkg[32];
    uint32_t crc;

    memset(cmdPkg, 0, 32);
    auto *phdr = (SEUCMD_HDR*) cmdPkg;
    phdr->preamble = VIPER_CMD_PREAMBLE;
    phdr->size = 24;
    phdr->seucmd.cmd = CMD_CONTINUOUS_PNO;
    phdr->seucmd.action = CMD_ACTION_SET;

    crc = calculateCrc16(cmdPkg, 28);
    memcpy(cmdPkg+28, &crc, CRC_SIZE);

    viperUsb.usb_send_cmd(cmdPkg, 32);
    std::this_thread::sleep_for(std::chrono::milliseconds(CMD_DELAY));
    cmdQueue_.wait_and_pop(respPkg, 32);

    crc = calculateCrc16(respPkg, 28);
    if (!Viper::validateCrc(crc, respPkg, 28)) {
        fgInterface_->logError("CRC incorrect when starting continuous publishing on Viper");
//        throw std::runtime_error("CRC incorrect when starting continuous publishing");
    }

    if (!Viper::checkAck(respPkg)) {
        std::stringstream ss;
//        throw std::runtime_error("Command acknowledgment not received when starting continuous publishing");
    }

    isContinuous = true;
    fgInterface_->logInfo("Starting continuous publishing thread");
    continuousPublishThread_ = std::thread{&Viper::publishContinuous, this};
}

uint32_t Viper::calculateCrc16(uint8_t *b, uint32_t len) {
    uint32_t crc = 0;
    while (len--)
        crc = crcTable_[(crc ^ *b++) & 0xff] ^ (crc >> 8);

    return crc;
}

void Viper::publishContinuous() {
    viper_usb *pvpr = &viperUsb;

    constexpr uint32_t kHdrEndLoc = 24;
    uint32_t crc, br, nSensors, i, frame;

    constexpr uint32_t respSize = sizeof(uint32_t)*11 + sizeof(SENFRAMEDATA)*16 + CRC_SIZE;
    uint8_t respPkg[respSize];

    SENFRAMEDATA *pfd;

    // Frames that never reach the publish path. Dropping them is correct -- the
    // payload cannot be trusted -- but doing it silently made a stream that
    // publishes nothing indistinguishable from a device that was never sending,
    // with no diagnostic anywhere to tell the two apart.
    uint64_t sizeMismatchedFrames = 0;
    uint64_t crcFailedFrames = 0;

    while (isContinuous && !fatalError_) {
        br = pnoQueue_.wait_and_pop(respPkg, respSize);

        if (!br)
            continue;   // nothing arrived within the queue's wait

        const uint32_t declaredSize = *(uint32_t*)(respPkg + 4) + 8;
        if (br != declaredSize) {
            if ((++sizeMismatchedFrames % 100) == 1) {
                std::stringstream ss;
                ss << "Dropped a PNO frame whose length disagrees with its header ("
                   << sizeMismatchedFrames << " frame(s) so far): received " << br
                   << " bytes, header declares " << declaredSize;
                fgInterface_->logWarning(ss.str());
            }
            continue;
        }

        crc = calculateCrc16(respPkg, br-4);

        if (!validateCrc(crc, respPkg, br-4)) {
            if ((++crcFailedFrames % 100) == 1) {
                std::stringstream ss;
                ss << "Dropped a PNO frame that failed its CRC (" << crcFailedFrames
                   << " frame(s) so far): computed " << crc << ", frame carries "
                   << *(uint32_t*)(respPkg + br - 4);
                fgInterface_->logWarning(ss.str());
            }
            continue;
        }

        nSensors = *(uint32_t*)(respPkg + 20);
        static bool printed = false; if (!printed) { std::cout << "Using " << nSensors << " position sensors" << std::endl; printed = true; }
        pfd = (SENFRAMEDATA*)(respPkg + kHdrEndLoc);
        frame = *(uint32_t*)(respPkg + 12);

        // The device's unit settings are persistent and only observable from a
        // frame, so this is the earliest point they can be checked. Everything
        // downstream assumes metres and quaternions; anything else would be
        // silently misinterpreted rather than failing visibly.
        if (!unitsChecked_ && nSensors > 0) {
            unitsChecked_ = true;
            const auto units = mdx::decodeFrameUnits(pfd->SFinfo.bfPosUnits,
                                                     pfd->SFinfo.bfOriUnits);

            if (!units.isSupported()) {
                raiseFatalError(mdx::unsupportedUnitsMessage(units));
                break;
            }

            fgInterface_->logInfo("Viper reporting " + mdx::describe(units));
            std::cout << "Viper reporting " << mdx::describe(units) << std::endl;
        }

        pnoToFoxgloveSceneUpdate(pfd, nSensors);
    }

    br = pnoQueue_.wait_and_pop(respPkg, respSize);
}

SensorData Viper::pnoToSensorData(SENFRAMEDATA *pfd) {
    SensorData data{};

    uint32_t sens = (pfd->SFinfo.bfSnum&0xff);
    data.sensor_id = (int) sens;
    data.time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

    data.x = pfd->pno.pos[0] * 1000;
    data.y = pfd->pno.pos[1] * 1000;
    data.z = pfd->pno.pos[2] * 1000;

    data.qw = pfd->pno.ori[0];
    data.qx = pfd->pno.ori[1];
    data.qy = pfd->pno.ori[2];
    data.qz = pfd->pno.ori[3];

    return data;
}

void Viper::pnoToPoseInFrame(SENFRAMEDATA *pfd, flatbuffers::FlatBufferBuilder &builder) {
    // Clear flatbuffer builder
    builder.Clear();

    // Get the current time point since the epoch
    auto now = std::chrono::system_clock::now();
    auto duration_since_epoch = now.time_since_epoch();

    auto sec_duration = std::chrono::duration_cast<std::chrono::seconds>(duration_since_epoch);
    long long sec = sec_duration.count();

    auto ns_duration = std::chrono::duration_cast<std::chrono::nanoseconds>(duration_since_epoch);
    long long total_nanoseconds = ns_duration.count();
    long long nsec = total_nanoseconds % 1000000000LL; // 1 second = 1,000,000,000 nanoseconds

    // foxglove timestamp
    auto time = foxglove::Time(sec, nsec);

    // foxglove position
    auto pos = foxglove::CreateVector3(
        builder,
        pfd->pno.pos[0],
        pfd->pno.pos[1],
        pfd->pno.pos[2]
    );

    // foxglove orientation
    auto quat = foxglove::CreateQuaternion(
        builder,
        pfd->pno.ori[0],
        pfd->pno.ori[1],
        pfd->pno.ori[2],
        pfd->pno.ori[3]
    );

    // foxglove pose
    auto pose = foxglove::CreatePose(
        builder,
        pos,
quat
    );

    // create unique name: "viper{sensorID}"
    std::stringstream buf;
    buf << "viper" << (pfd->SFinfo.bfSnum&0xff);
    auto sensorName = builder.CreateString(buf.str());

    // foxglove pose in frame
    auto poseInFrame = foxglove::CreatePoseInFrame(builder, &time, sensorName, pose);

    builder.Finish(poseInFrame);
}

void Viper::pnoToPosesInFrame(SENFRAMEDATA *pfd_all, uint32_t nSensors, flatbuffers::FlatBufferBuilder &builder) {
    // Clear flatbuffer builder
    builder.Clear();

    // Get the current time point since the epoch
    auto now = std::chrono::system_clock::now();
    auto duration_since_epoch = now.time_since_epoch();

    auto sec_duration = std::chrono::duration_cast<std::chrono::seconds>(duration_since_epoch);
    long long sec = sec_duration.count();

    auto ns_duration = std::chrono::duration_cast<std::chrono::nanoseconds>(duration_since_epoch);
    long long total_nanoseconds = ns_duration.count();
    long long nsec = total_nanoseconds % 1000000000LL; // 1 second = 1,000,000,000 nanoseconds

    // foxglove timestamp
    auto time = foxglove::Time(sec, nsec);

    std::vector<flatbuffers::Offset<foxglove::Pose>> posesVec;

    // gather all poses
    for (int i=0; i < nSensors; i++) {
        SENFRAMEDATA *pfd = pfd_all + i;
        // foxglove position
        auto pos = foxglove::CreateVector3(
                builder,
                pfd->pno.pos[0],
                pfd->pno.pos[1],
                pfd->pno.pos[2]
        );

        // foxglove orientation
        auto quat = foxglove::CreateQuaternion(
                builder,
                pfd->pno.ori[0],
                pfd->pno.ori[1],
                pfd->pno.ori[2],
                pfd->pno.ori[3]
        );

        // foxglove pose
        auto pose = foxglove::CreatePose(
                builder,
                pos,
                quat
        );

        posesVec.push_back(pose);
    }

    auto poses = builder.CreateVector(posesVec);

    // foxglove poses in frame
    auto frame = builder.CreateString("viper");
    auto posesInFrame = foxglove::CreatePosesInFrame(builder, &time, frame, poses);

    builder.Finish(posesInFrame);
}

// No-op, kept only because pnoToPosesInFrame (itself unused) calls it. The
// sensor-to-tip transform this once sketched now lives in
// mdx::applyTipTransform, driven by the probe profile, and is applied to the
// fused pose rather than to each sensor's frame data.
SENFRAMEDATA *Viper::pnoTransformInBodyFrame(SENFRAMEDATA *pfd) {
    return pfd;
}

void Viper::pnoToPoseInFrame(SENFRAMEDATA *pfd) {
    pnoToPoseInFrame(pfd, fbBuilder_);
}

void Viper::pnoToPosesInFrame(SENFRAMEDATA *pfd, uint32_t nSensors) {
    for (int i=0; i < nSensors; i++) {
        pnoOffset(pfd + i);
    }

    pnoToPosesInFrame(pfd, nSensors, fbBuilder_);
}

SENFRAMEDATA *Viper::pnoOffset(SENFRAMEDATA *pfd) {
    return pnoTransformInBodyFrame(pfd);
}

void Viper::pnoToFoxgloveSceneUpdate(SENFRAMEDATA *pfd_all, uint32_t nSensors) {
    // Get the current time point since the epoch
    auto now = std::chrono::system_clock::now();
    auto duration_since_epoch = now.time_since_epoch();

    auto sec_duration = std::chrono::duration_cast<std::chrono::seconds>(duration_since_epoch);
    long long sec = sec_duration.count();

    auto ns_duration = std::chrono::duration_cast<std::chrono::nanoseconds>(duration_since_epoch);
    long long total_nanoseconds = ns_duration.count();
    long long nsec = total_nanoseconds % 1000000000LL; // 1 second = 1,000,000,000 nanoseconds

    // foxglove timestamp
    auto time = foxglove::schemas::Timestamp{static_cast<uint32_t>(sec), static_cast<uint32_t>(nsec)};

    lastSensorCount_ = static_cast<int>(nSensors);

    // Pick the probe profile from the number of sensors the SEU is reporting,
    // and latch it. Which probe is fitted is decided by what is plugged in, so
    // the sensor count is the only thing that identifies it.
    if (activeProfile_ == nullptr) {
        if (nSensors == 0)
            return;     // nothing to identify the probe by yet

        activeProfile_ = mdx::selectProfile(profiles_, static_cast<int>(nSensors));

        if (activeProfile_ == nullptr) {
            // Publishing a tip pose here would mean guessing an offset, which
            // misplaces the tip silently. Refuse, and say why. The raw
            // per-sensor poses are still published below: they need no profile,
            // and they are what you need to diagnose this.
            //
            // Silent during calibration, where having no profile yet is the
            // entire premise rather than a fault.
            if (!calibrationMode_ && (++noProfileFrames_ % 100) == 1) {
                std::stringstream ss;
                ss << "No probe profile configured for " << nSensors << " sensor(s); "
                   << "not publishing a tip pose. Configured profiles:";
                for (const auto &profile : profiles_)
                    ss << " " << profile.sensorCount;
                ss << ". Add a matching entry to \"probe_profiles\" in the config file.";
                fgInterface_->logError(ss.str());
            }
        } else {
            latchedSensorCount_ = static_cast<int>(nSensors);
            fgInterface_->logInfo("Probe profile selected: " + mdx::describeProfile(*activeProfile_));
            std::cout << "Probe profile selected: " << mdx::describeProfile(*activeProfile_) << std::endl;
        }
    } else if (static_cast<int>(nSensors) != latchedSensorCount_) {
        // The profile stays latched: the tip offset must not change under the
        // operator mid-run. The tip pose is wrong while this persists, so the
        // message has to be impossible to miss in the log and the MCAP.
        if ((++countMismatchFrames_ % 100) == 1) {
            std::stringstream ss;
            ss << "Sensor count changed from " << latchedSensorCount_ << " to " << nSensors
               << " after the probe profile was latched (" << countMismatchFrames_
               << " frame(s) so far). Still applying the " << latchedSensorCount_
               << "-sensor tip offset, so the published tip pose is NOT trustworthy. "
               << "Check the EM sensor connections and restart.";
            fgInterface_->logError(ss.str());
        }
    }

    // Compute distances; update positions and velocities
//    positions_.resize(nSensors, Eigen::Vector3f::Zero());
//    velocities_.resize(nSensors, Eigen::Vector3f::Zero());
//    quaternions_.resize(nSensors, Eigen::Quaternionf{});
//    distanceTraveled_.resize(nSensors, 0);

//    {
//        std::lock_guard<std::mutex> guard{poseMutex_};
//
//        if (lastSampleTime_.has_value()) {
//            std::chrono::duration<float> dt = now - lastSampleTime_.value();
//
//            for (int j = 0; j < positions_.size(); j++) {
//                SENFRAMEDATA *pfd = pfd_all + j;
//
//                if (j < nSensors) {
//                    auto pos = Eigen::Vector3f{
//                            pfd->pno.pos[0],
//                            pfd->pno.pos[1],
//                            pfd->pno.pos[2]
//                    };
//
//                    auto quat = Eigen::Quaternionf{
//                            pfd->pno.ori[0],
//                            pfd->pno.ori[1],
//                            pfd->pno.ori[2],
//                            pfd->pno.ori[3],
//                    };
//
//                    distanceTraveled_.at(j) += (pos - positions_.at(j)).norm();
//                    velocities_.at(j) = (pos - positions_.at(j)) / (dt.count() + 1e-6);
//                    positions_.at(j) = pos;
//                    quaternions_.at(j) = quat;
//                }
//            }
//        } else {
//            positions_.resize(nSensors, Eigen::Vector3f::Zero());
//            velocities_.resize(nSensors, Eigen::Vector3f::Zero());
//            quaternions_.resize(nSensors, Eigen::Quaternionf{});
//            distanceTraveled_.resize(nSensors, 0);
//
//            for (int j = 0; j < nSensors; j++) {
//                SENFRAMEDATA *pfd = pfd_all + j;
//
//                auto pos = Eigen::Vector3f{
//                        pfd->pno.pos[0],
//                        pfd->pno.pos[1],
//                        pfd->pno.pos[2]
//                };
//
//                auto quat = Eigen::Quaternionf{
//                        pfd->pno.ori[0],
//                        pfd->pno.ori[1],
//                        pfd->pno.ori[2],
//                        pfd->pno.ori[3],
//                };
//
//                positions_.at(j) = pos;
//                quaternions_.at(j) = quat;
//                quaternions_.resize(nSensors, Eigen::Quaternionf{});
//                distanceTraveled_.resize(nSensors, 0);
//            }
//        }
//
//        lastSampleTime_.emplace(now);
//    }


    std::vector<foxglove::schemas::Pose> poses;
    std::vector<mdx::Pose> sensorPoses;
    sensorPoses.reserve(nSensors);

    // gather all poses
    for (int i=0; i < nSensors; i++) {
        SENFRAMEDATA *pfd = pfd_all + i;

        // Same values as the Foxglove pose below, kept in Eigen form for the
        // fusion maths. Quaternion order from the device is (w, x, y, z).
        mdx::Pose sensorPose;
        sensorPose.position = Eigen::Vector3d{
            pfd->pno.pos[0],
            pfd->pno.pos[1],
            pfd->pno.pos[2]
        };
        sensorPose.orientation = Eigen::Quaterniond{
            pfd->pno.ori[0],
            pfd->pno.ori[1],
            pfd->pno.ori[2],
            pfd->pno.ori[3]
        };
        sensorPoses.push_back(sensorPose);

        // foxglove position
        auto pos = foxglove::schemas::Vector3{
            pfd->pno.pos[0],
            pfd->pno.pos[1],
            pfd->pno.pos[2]
        };

        // Log point to line for sensor 0 (skip when downstream generates geometry)
        if (i == 0 && fgInterface_->generateGeometry()) {
            auto point = foxglove::schemas::Point3{
                pos.x,
                pos.y,
                pos.z
            };

            linePrimitive_.points.push_back(point);
            if (fgInterface_->hasContact) {
                std::lock_guard<std::mutex> guard{contactPointBufferMtx_};
                contactPointBuffer_.push_back(pos.x);
                contactPointBuffer_.push_back(pos.y);
                contactPointBuffer_.push_back(pos.z);
            }
        }

        // foxglove orientation
        auto quat = foxglove::schemas::Quaternion{
                pfd->pno.ori[1],
                pfd->pno.ori[2],
                pfd->pno.ori[3],
                pfd->pno.ori[0],
        };

        auto pose = foxglove::schemas::Pose{
            pos,
            quat
        };

//        // Log handheld probe pose
//        if (i == 0) {
//            auto poseInFrame = foxglove::schemas::PoseInFrame{
//                time,
//                "viper",
//                pose
//            };
//
//            arrowPrimitive_.pose.emplace(pose);
////            posesInFrame_.timestamp.emplace(time);
////            posesInFrame_.poses.push_back(pose);
////            fgInterface_->publishHHPoses(posesInFrame_);
//            fgInterface_->publishPose(poseInFrame);
//        }

        poses.push_back(pose);
    }

    // Fuse the sensors into one pose, then map that onto the probe tip. With a
    // single sensor the fusion is a pass-through, so the tip pose is that
    // sensor's own pose with the profile's transform applied.
    // Fuse unconditionally, whether or not a profile is available. Calibration
    // runs precisely when there is no profile yet, and the fused pose is what
    // it has to solve against -- deriving it any other way would calibrate one
    // estimator and deploy another.
    const std::optional<mdx::Pose> fused = mdx::fusePoses(sensorPoses);

    if (fused.has_value()) {
        {
            std::lock_guard<std::mutex> guard{fusedPoseMtx_};
            latestFusedPose_ = fused;
        }

        // Without a profile there is no trustworthy offset, so no tip pose is
        // published. That was reported above; the raw per-sensor poses still go
        // out below either way.
        if (activeProfile_ != nullptr) {
            const auto tipPose = toFoxglovePose(mdx::applyTipTransform(fused.value(), activeProfile_->tip));
            auto swingTwist = computeSwingTwist(tipPose);

            auto poseInFrame = foxglove::schemas::PoseInFrame{
                    time,
                    "viper",
                    tipPose
            };

            fgInterface_->publishPose(poseInFrame);
            fgInterface_->logSwingTwist(swingTwist);
        }
    } else if ((++unusableFrames_ % 100) == 1) {
        // A non-finite or non-unit-norm reading. The raw per-sensor poses are
        // still published below so the bad frames stay visible in the MCAP for
        // diagnosis.
        std::stringstream ss;
        ss << "Dropped a PNO frame with unusable sensor data (" << unusableFrames_
           << " frame(s) so far): no tip pose published for it";
        fgInterface_->logWarning(ss.str());
    }

    auto posesInFrame = foxglove::schemas::PosesInFrame{
        time,
        "viper",
        poses
    };


//    fgInterface_->publishLineToScene(linePrimitive_);
//    fgInterface_->publishArrowToScene(arrowPrimitive_);
    fgInterface_->publishPoses(posesInFrame);
}
