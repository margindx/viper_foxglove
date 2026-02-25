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

    uint8_t* respPkg = new uint8_t[kRespSize];
    uint32_t br;

    uint32_t timeOut = 200;
    viper_queue* pqueue;

    while (keepReading) {
        br = pvpr->usb_rec_resp(respPkg, kRespSize);

        if (br) {
            if (Viper::hasPnoPreamble(respPkg)) {
                pqueue = &pnoQueue_;
                timeOut = 4;
            } else {
                pqueue = &cmdQueue_;
            }

            pqueue->push(respPkg, br);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(timeOut));
    }
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


    while (isContinuous) {
        br = pnoQueue_.wait_and_pop(respPkg, respSize);

        if (br && (br == (*(uint32_t*)(respPkg+4)+8))) {
            crc = calculateCrc16(respPkg, br-4);

            if (validateCrc(crc, respPkg, br-4)) {
	        nSensors = *(uint32_t*)(respPkg + 20);
                static bool printed = false; if (!printed) { std::cout << "Using " << nSensors << " position sensors" << std::endl; printed = true; }
	    	pfd = (SENFRAMEDATA*)(respPkg + kHdrEndLoc);
                frame = *(uint32_t*)(respPkg + 12);

                pnoToFoxgloveSceneUpdate(pfd, nSensors);
            }
        }
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

SENFRAMEDATA *Viper::pnoTransformInBodyFrame(SENFRAMEDATA *pfd) {
//    Eigen::Quaternion<float> quat{
//        pfd->pno.ori[0],
//        pfd->pno.ori[1],
//        pfd->pno.ori[2],
//        pfd->pno.ori[3],
//    };
//
//    Eigen::Vector3f pos{
//        pfd->pno.pos[0]/100.f,
//        pfd->pno.pos[1]/100.f,
//        pfd->pno.pos[2]/100.f
//    };

//    Eigen::Quaternion<float> offset{
//        0,
//        offset_.x(),
//        offset_.y(),
//        offset_.z()
//    };
//
//    auto rotatedOffset = quat * offset * quat.inverse();
//
//    pfd->pno.pos[0] = pos.x() + rotatedOffset.x();
//    pfd->pno.pos[1] = pos.y() + rotatedOffset.y();
//    pfd->pno.pos[2] = pos.z() + rotatedOffset.z();
    pfd->pno.pos[0];
    pfd->pno.pos[1];
    pfd->pno.pos[2];

    return pfd;
}

void Viper::setOffset(Eigen::Vector3f &offset) {
    offset_ = offset;
}

void Viper::setOffset(float x, float y, float z) {
    offset_.x() = x;
    offset_.y() = y;
    offset_.z() = z;
}

const Eigen::Vector3f &Viper::getOffset() const {
    return offset_;
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

    // Apply offset to all sensors and transform to meters
    for (int i=0; i < nSensors; i++) {
        pnoOffset(pfd_all + i);
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

    // gather all poses
    for (int i=0; i < nSensors; i++) {
        SENFRAMEDATA *pfd = pfd_all + i;

        // foxglove position
        auto pos = foxglove::schemas::Vector3{
            pfd->pno.pos[0],
            pfd->pno.pos[1],
            pfd->pno.pos[2]
        };

        // Log point to line for sensor 0
        if (i == 0) {
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

    foxglove::schemas::Pose avgPose;
    double xAvg{0}, yAvg{0}, zAvg{0}, qxAvg{0}, qyAvg{0}, qzAvg{0}, qwAvg{0};

    size_t nPoses = poses.size();
    for (auto &pose : poses) {
        xAvg += pose.position.value().x / nPoses;
        yAvg += pose.position.value().y / nPoses;
        zAvg += pose.position.value().z / nPoses;
        qxAvg += pose.orientation.value().x / nPoses;
        qyAvg += pose.orientation.value().y / nPoses;
        qzAvg += pose.orientation.value().z / nPoses;
        qwAvg += pose.orientation.value().w / nPoses;
    }

    // Normalize quaternion
    auto qMag = sqrt(qxAvg*qxAvg + qyAvg*qyAvg + qzAvg*qzAvg + qwAvg*qwAvg);
    qxAvg /= qMag;
    qyAvg /= qMag;
    qzAvg /= qMag;
    qwAvg /= qMag;

    avgPose.position.emplace(foxglove::schemas::Vector3{xAvg, yAvg, zAvg});
    avgPose.orientation.emplace(foxglove::schemas::Quaternion{qxAvg, qyAvg, qzAvg, qwAvg});
    // Apply offset
    transformPose(avgPose);
    auto swingTwist = computeSwingTwist(avgPose);

    auto poseInFrame = foxglove::schemas::PoseInFrame{
            time,
            "viper",
            avgPose
    };

    fgInterface_->publishPose(poseInFrame);
    fgInterface_->logSwingTwist(swingTwist);

    auto posesInFrame = foxglove::schemas::PosesInFrame{
        time,
        "viper",
        poses
    };


//    fgInterface_->publishLineToScene(linePrimitive_);
//    fgInterface_->publishArrowToScene(arrowPrimitive_);
    fgInterface_->publishPoses(posesInFrame);
}
