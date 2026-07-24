//
// Created by Hamza El-Kebir on 7/28/25.
//

#ifndef VIPER_SERIALFORCE_HPP
#define VIPER_SERIALFORCE_HPP

#include "serialib/serialib.h"
#include <mutex>
#include <cstdio>
#include <cstring>
#include <string>
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
        // Newline-anchored framing: bytes are accumulated and only text delimited
        // by '\n' on BOTH sides is parsed. This is what guarantees a complete line
        // — the field-count check alone cannot, because a read that starts partway
        // through the first field still yields the right number of commas and the
        // fragment parses as a valid (but wrong) float. See issue #50.
        constexpr size_t kMaxAccumBytes = 1024;   // resync if this many bytes carry no '\n'
        std::string acc;
        char chunk[256];
        bool sawNewline = false;                   // discarded the initial partial fragment yet?

        while (!closed) {
            int n = serial_.readBytes(chunk, sizeof(chunk), 5000);
            if (n <= 0) {
                // Timeout or device read error: nothing new this cycle.
                continue;
            }
            acc.append(chunk, static_cast<size_t>(n));

            // Everything before the first '\n' we ever see may be a partial first
            // field — discard it once so all subsequent parsing starts on a real
            // line boundary. Thereafter acc always begins at a line boundary.
            if (!sawNewline) {
                auto first = acc.find('\n');
                if (first == std::string::npos) {
                    if (acc.size() > kMaxAccumBytes) acc.clear();
                    continue;
                }
                acc.erase(0, first + 1);
                sawNewline = true;
            }

            // Parse only whole lines (bounded by '\n' on both sides); keep any
            // trailing partial for the next read. Drop backlog by parsing just the
            // freshest complete line, preserving the freshest-only policy.
            auto lastNl = acc.rfind('\n');
            if (lastNl == std::string::npos) {
                if (acc.size() > kMaxAccumBytes) { acc.clear(); sawNewline = false; }
                continue;   // only a partial line so far
            }
            std::string complete = acc.substr(0, lastNl);   // one or more whole lines
            acc.erase(0, lastNl + 1);                        // retain the trailing partial

            auto prevNl = complete.rfind('\n');
            std::string line = (prevNl == std::string::npos)
                                   ? complete
                                   : complete.substr(prevNl + 1);

            std::lock_guard<std::mutex> guard1(mutex1_);
            std::lock_guard<std::mutex> guard2(mutex2_);
            std::lock_guard<std::mutex> guard3(mutex3_);
            std::lock_guard<std::mutex> guard4(mutex4_);
            int contactFlag = 0;
            int result = sscanf(line.c_str(), "%f,%f,%f,%f,%d", &force1_, &force2_, &force3_, &force4_, &contactFlag);
            if (result != 5) {
                // Whole line but still malformed (line noise, dropped byte) — skip it.
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
