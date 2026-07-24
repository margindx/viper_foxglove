//
// Created by Hamza El-Kebir on 7/28/25.
//

#ifndef VIPER_SERIALFORCE_HPP
#define VIPER_SERIALFORCE_HPP

#include "serialib/serialib.h"
#include <mutex>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <thread>
#include <functional>
#include "FoxgloveInterface.hpp"

class SerialForce {
protected:
    float force1_{}, force2_{}, force3_{}, force4_{};
    int contactFlag_{};
    mutable std::mutex mutex1_, mutex2_, mutex3_, mutex4_, contactMutex_;
    serialib serial_;
    char buf_[100]{};

    mdx::Contact contact_;
    std::optional<std::chrono::time_point<std::chrono::system_clock>> lastContactTime_;

    FoxgloveInterface *fgInterface_;
    std::thread continuousPublishThread_;

    std::optional<std::function<bool(mdx::RawForce &)>> hasContactCallback_;
    float _minimum_contact_force = 0.3;
    bool _use_hardware_contact = true;
    bool _require_sensor_1 = true;
    bool _require_sensor_2 = true;
    bool _require_sensor_3 = true;
    bool _require_sensor_4 = true;

    void updateForces_() {
        char latest[sizeof(buf_)];
        while (!closed) {
            // Block for the next complete line.
            if (serial_.readString(buf_, '\n', 100, 5000) <= 0) {
                // Timeout (0) or device read error (<0): nothing complete to parse.
                continue;
            }

            // Advance to the freshest complete line already buffered, dropping any
            // backlog so contact/force stay low-latency when we can't keep up. We
            // drain with readString (1 ms, non-blocking in practice) rather than
            // flushReceiver() so reads stay line-aligned — a blind flush can leave
            // the next read starting mid-line, which the parser would then reject.
            // A short-timeout read that finds no further complete line returns <=0
            // and would clobber its buffer, so drain into a scratch buffer and only
            // promote a successful read.
            while (serial_.readString(latest, '\n', 100, 1) > 0) {
                std::memcpy(buf_, latest, sizeof(buf_));
            }

            std::lock_guard<std::mutex> guard1(mutex1_);
            std::lock_guard<std::mutex> guard2(mutex2_);
            std::lock_guard<std::mutex> guard3(mutex3_);
            std::lock_guard<std::mutex> guard4(mutex4_);
            int contactFlag = 0;
            int result = sscanf(buf_, "%f,%f,%f,%f,%d\n", &force1_, &force2_, &force3_, &force4_, &contactFlag);
            if (result != 5) {
                // Corrupt line — skip it. Reads stay line-aligned, so no flush/resync needed.
                continue;
            }
            contactFlag_ = contactFlag;
            mdx::RawForce rawForce{force1_, force2_, force3_, force4_};

            updateContact_(rawForce);

            fgInterface_->logRawForce(rawForce);
        }
    }

    bool hasContact_(mdx::RawForce &rawForce){
        bool has_contact = true; 

        if (_require_sensor_1){ 
            has_contact = has_contact & (rawForce.f1 > _minimum_contact_force);
        }
        if (_require_sensor_2){ 
            has_contact = has_contact & (rawForce.f2 > _minimum_contact_force);
        }
        if (_require_sensor_3){ 
            has_contact = has_contact & (rawForce.f3 > _minimum_contact_force);
        }
        if (_require_sensor_4){ 
            has_contact = has_contact & (rawForce.f4 > _minimum_contact_force);
        }
        return has_contact;
    }

    void updateContact_(mdx::RawForce &rawForce) {
        auto now = std::chrono::system_clock::now();

        bool contactDetected = _use_hardware_contact ? (contactFlag_ != 0)
                                                      : hasContact_(rawForce);

        if (contactDetected) {
            std::lock_guard<std::mutex> guardContact{contactMutex_};
            if (lastContactTime_.has_value()) {
                auto dt = std::chrono::duration_cast<std::chrono::duration<float>>(now - lastContactTime_.value()).count();
                contact_.contactDuration += dt;
                lastContactTime_.emplace(now);
            } else {
                lastContactTime_.emplace(now);
                contact_.contactDuration = 0;
            }
            contact_.hasContact = true;
        } else {
            std::lock_guard<std::mutex> guardContact{contactMutex_};
            lastContactTime_.reset();
            contact_.contactDuration = 0;
            contact_.hasContact = false;
        }

        fgInterface_->logContact(contact_);
    }
public:
    SerialForce(FoxgloveInterface &fgInterface) : fgInterface_(&fgInterface), contact_{} {

    }

    ~SerialForce() {
        close();
    }


    void init(std::string &&port = "/dev/cu.usbmodem2101", 
              float minimum_contact_force = 0.3) {
        auto serialRes = openSerial(std::move(port));
        if (!serialRes) {
            fgInterface_->logError("Failed to open force sensor serial port");
        }

        _minimum_contact_force = minimum_contact_force;

        startForceUpdate();
    }

    bool closed = false;

    bool openSerial(std::string &&port = "/dev/cu.usbmodem2101") {
        return serial_.openDevice(port.c_str(), 115200) == 1;
    }

    void setUseHardwareContact(bool useHardware) {
        _use_hardware_contact = useHardware;
    }

    void setRequireSensor(bool f1, bool f2, bool f3, bool f4){
        _require_sensor_1 = f1;
        _require_sensor_2 = f2;
        _require_sensor_3 = f3; 
        _require_sensor_4 = f4;
    }

    void startForceUpdate() {
        continuousPublishThread_ = std::thread(&SerialForce::updateForces_, this);
    }

    void close() {
        closed = true;
        continuousPublishThread_.join();
        serial_.closeDevice();
    }

    float getF1() const {
        std::lock_guard<std::mutex> guard(mutex1_);
        return force1_;
    }

    void setF1(double f) {
        std::lock_guard<std::mutex> guard(mutex1_);
        force1_ = f;
    }

    float getF2() const {
        std::lock_guard<std::mutex> guard(mutex2_);
        return force2_;
    }

    void setF2(double f) {
        std::lock_guard<std::mutex> guard(mutex2_);
        force2_ = f;
    }

    float getF3() const {
        std::lock_guard<std::mutex> guard(mutex3_);
        return force3_;
    }

    void setF3(double f) {
        std::lock_guard<std::mutex> guard(mutex3_);
        force3_ = f;
    }

    float getF4() const {
        std::lock_guard<std::mutex> guard(mutex4_);
        return force4_;
    }

    void setF4(double f) {
        std::lock_guard<std::mutex> guard(mutex4_);
        force4_ = f;
    }

    mdx::RawForce getRawForce() {
        return {
            getF1(),
            getF2(),
           getF3(),
           getF4()
        };
    }
};


#endif //VIPER_SERIALFORCE_HPP
