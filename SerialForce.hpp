//
// Created by Hamza El-Kebir on 7/28/25.
//

#ifndef VIPER_SERIALFORCE_HPP
#define VIPER_SERIALFORCE_HPP

#include "serialib/serialib.h"
#include <mutex>
#include <cstdio>
#include <cstring>
#include <cerrno>
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

    // Serial connection parameters, retained for startup and runtime reconnect.
    static constexpr unsigned int kBaudRate_ = 115200;
    std::string port_ = "/dev/cu.usbmodem2101";
    size_t connectTries_ = 5;               // extra attempts beyond the first
    unsigned int connectTimeoutMs_ = 200;   // backoff between attempts

    // Human-readable description of the last OS-level error from a serial call.
    // serialib uses Win32 on Windows (which reports via GetLastError, NOT errno)
    // and POSIX syscalls elsewhere (which set errno), so the source differs by
    // platform. Call this immediately after the failed serial call, before any
    // other call can overwrite the error state.
    static std::string lastOsError_() {
#ifdef _WIN32
        DWORD code = GetLastError();
        if (code == 0) return "0: no error reported";
        LPSTR buf = nullptr;
        DWORD len = FormatMessageA(
            FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM
                | FORMAT_MESSAGE_IGNORE_INSERTS,
            nullptr, code, MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
            reinterpret_cast<LPSTR>(&buf), 0, nullptr);
        std::string msg = (len && buf) ? std::string(buf, len) : "unknown error";
        if (buf) LocalFree(buf);
        // FormatMessage appends a trailing CRLF; strip it for a clean log line.
        while (!msg.empty() && (msg.back() == '\r' || msg.back() == '\n' || msg.back() == ' '))
            msg.pop_back();
        return std::to_string(code) + ": " + msg;
#else
        int code = errno;
        return std::to_string(code) + ": " + std::strerror(code);
#endif
    }

    // Bounded retry loop used at startup, mirroring Viper's connect loop:
    // connectTries_ extra attempts beyond the first, connectTimeoutMs_ apart.
    // Logs the serialib return code and OS error on each failure for diagnosis.
    bool connect_() {
        for (size_t attempt = 0; attempt <= connectTries_; ++attempt) {
            char rc = serial_.openDevice(port_.c_str(), kBaudRate_);
            if (rc == 1) {
                return true;
            }
            // Capture before anything else touches errno / GetLastError.
            const std::string osErr = lastOsError_();

            fgInterface_->logWarning(
                "Force sensor openDevice(" + port_ + ") failed on attempt "
                + std::to_string(attempt + 1) + "/" + std::to_string(connectTries_ + 1)
                + " (serialib code " + std::to_string(static_cast<int>(rc))
                + ", OS error " + osErr + ")");

            if (attempt < connectTries_) {
                std::this_thread::sleep_for(std::chrono::milliseconds(connectTimeoutMs_));
            }
        }
        return false;
    }

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
            // (Re)connect if the port isn't open — covers a device that was
            // absent at startup or dropped mid-session. One attempt per loop,
            // backing off between tries, until connected or shut down. Failed
            // attempts are not logged individually here to avoid log spam.
            if (!serial_.isDeviceOpen()) {
                if (serial_.openDevice(port_.c_str(), kBaudRate_) != 1) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(connectTimeoutMs_));
                    continue;
                }
                fgInterface_->logInfo("Force sensor serial port connected: " + port_);
                serial_.flushReceiver();
                // A fresh stream starts at an arbitrary byte offset: drop whatever
                // the previous connection left buffered and re-arm the framing so
                // the first partial fragment is discarded again.
                acc.clear();
                sawNewline = false;
            }

            int n = serial_.readBytes(chunk, sizeof(chunk), 5000);
            if (n < 0) {
                // Negative return is a device-level read error (e.g. unplugged);
                // drop the connection so the loop above reconnects.
                fgInterface_->logWarning("Force sensor read failed; will attempt to reconnect");
                serial_.closeDevice();
                continue;
            }
            if (n == 0) {
                // Timeout with no new bytes — nothing to add to the accumulator.
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
              float minimum_contact_force = 0.3,
              size_t connectTries = 5,
              unsigned int connectTimeoutMs = 200) {
        port_ = std::move(port);
        _minimum_contact_force = minimum_contact_force;
        connectTries_ = connectTries;
        connectTimeoutMs_ = connectTimeoutMs;

        if (connect_()) {
            fgInterface_->logInfo("Force sensor serial port opened: " + port_);
        } else {
            fgInterface_->logError(
                "Force sensor serial port failed to open after retries (" + port_
                + "); continuing without force data and retrying in the background");
        }

        // Start the reader regardless of the initial result: if the open failed
        // it keeps retrying, so the application runs degraded rather than dead.
        startForceUpdate();
    }

    bool closed = false;

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
