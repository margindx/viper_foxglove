//
// See Monitor.hpp.
//

#include "Monitor.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <nlohmann/json.hpp>

#include "DeviceState.hpp"
#include "FoxgloveInterface.hpp"
#include "ProbeProfile.hpp"
#include "StabilityMonitor.hpp"
#include "Viper.hpp"

namespace mdx {

namespace {

using namespace std::literals::chrono_literals;

constexpr auto kPollInterval = 20ms;
constexpr auto kReportInterval = 250ms;

/// Baseline window. Long enough to average down per-frame noise and to catch
/// slow wander, short enough that an operator will actually hold still for it.
constexpr auto kBaselineDuration = 5s;

bool prompt(const std::string &question, std::string &answer) {
    std::cout << question << std::flush;

    if (!std::getline(std::cin, answer))
        return false;

    return true;
}

/// Read the required expected frame rate. Unlike the profiles, this is not
/// optional here: it is checked against the device in every mode, and a bench
/// tool that skipped the check would be the one place a misconfigured rig went
/// unnoticed.
std::optional<int> loadExpectedFrameRateHz(const std::string &configPath) {
    try {
        std::ifstream f(configPath);
        return parseExpectedFrameRateHz(nlohmann::json::parse(f));
    } catch (const std::exception &e) {
        std::cerr << "Invalid frame rate configuration in " << configPath << ": " << e.what()
                  << "\n";
        return std::nullopt;
    }
}

std::vector<ProbeProfile> loadProfilesIfPresent(const std::string &configPath) {
    if (!std::filesystem::exists(configPath))
        return {};

    try {
        std::ifstream f(configPath);
        return parseProbeProfiles(nlohmann::json::parse(f));
    } catch (const std::exception &) {
        // A config this program cannot parse is a problem for the normal run,
        // not for this one: the profile is wanted only to scale an excursion
        // into millimeters at the tip, and the measurement stands without it.
        return {};
    }
}

/// Collect poses for a fixed window with a countdown, and build a baseline.
std::optional<StabilityBaseline> captureBaseline(Viper &viper, BaselineAccumulator &accumulator) {
    accumulator.clear();

    const auto start = std::chrono::steady_clock::now();
    auto lastReport = start;

    while (std::chrono::steady_clock::now() - start < kBaselineDuration) {
        if (const auto pose = viper.latestFusedPose())
            accumulator.add(*pose);

        const auto now = std::chrono::steady_clock::now();
        if (now - lastReport >= kReportInterval) {
            lastReport = now;
            const auto left = std::chrono::duration_cast<std::chrono::milliseconds>(
                    kBaselineDuration - (now - start));
            std::cout << "\r  Holding still... " << std::fixed << std::setprecision(1)
                      << static_cast<double>(left.count()) / 1000.0 << "s  ("
                      << accumulator.samples() << " samples)   " << std::flush;
        }

        std::this_thread::sleep_for(kPollInterval);
    }

    std::cout << "\r                                                        \r";
    return accumulator.result();
}

std::string formatMillimeters(double meters) {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(3) << meters * 1000.0 << " mm";
    return ss.str();
}

} // namespace

int runMonitor(const std::string &configPath) {
    std::cout << "\n=== Probe stability monitor ===\n\n"
                 "Checks whether the EM sensor is moving inside the probe body, which makes\n"
                 "the tip offset a quantity that genuinely varies and cannot be calibrated\n"
                 "away.\n\n"
                 "The main test needs no probe motion at all. Rest the probe on the bench and\n"
                 "disturb only its cable -- pull, release, pull again, twist it, let it hang\n"
                 "over the edge. The body is stationary by construction, so anything that\n"
                 "moves is the sensor inside it. Keeping the probe still also holds the field\n"
                 "constant, which no moving test can do.\n\n"
                 "Mark each pull and release as you make it. A step that repeats with every\n"
                 "load and reverses on release is mechanical; drift does not care what your\n"
                 "hand is doing, and correlation with the marks is what separates them.\n\n";

    // Parsed before anything is opened or the device is touched, so a config
    // error fails cleanly rather than leaving a stray recording behind.
    const auto expectedRate = loadExpectedFrameRateHz(configPath);
    if (!expectedRate.has_value())
        return 1;
    const int expectedFrameRateHz = *expectedRate;

    std::optional<FoxgloveInterface> fgInterface;
    try {
        fgInterface.emplace("viper-monitor.mcap");
    } catch (const std::exception &e) {
        std::cerr << "Could not start the Foxglove interface: " << e.what() << "\n";
        return 1;
    }

    std::this_thread::sleep_for(1000ms);

    auto profiles = loadProfilesIfPresent(configPath);
    Viper viper{&fgInterface.value(), profiles, expectedFrameRateHz, 10, 100,
                /*calibrationMode=*/true};

    std::cout << "Waiting for pose data...\n";
    for (int i = 0; i < 250 && !viper.latestFusedPose().has_value(); i++) {
        if (viper.hasFatalError()) {
            std::cerr << "Cannot monitor: " << viper.fatalErrorMessage() << "\n";
            return 1;
        }
        std::this_thread::sleep_for(20ms);
    }

    if (!viper.latestFusedPose().has_value()) {
        std::cerr << "No usable pose data from the Viper. Check the sensors and try again.\n";
        return 1;
    }

    // The delivered-rate window closes a couple of seconds after the stream
    // opens, so the refusal it can raise arrives after the first pose. Wait for
    // the verdict rather than proceeding on a rig that cannot carry its rate.
    std::cout << "Confirming frame rate...\n";
    for (int i = 0; i < 400 && !viper.deliveredRateChecked(); i++) {
        if (viper.hasFatalError()) {
            std::cerr << "Cannot monitor: " << viper.fatalErrorMessage() << "\n";
            return 1;
        }
        std::this_thread::sleep_for(20ms);
    }
    if (viper.hasFatalError()) {
        std::cerr << "Cannot monitor: " << viper.fatalErrorMessage() << "\n";
        return 1;
    }

    const int sensorCount = viper.lastSensorCount();
    if (sensorCount < 1) {
        std::cerr << "The Viper is not reporting any sensors.\n";
        return 1;
    }

    std::cout << "Monitoring " << sensorCount << " connected sensor"
              << (sensorCount == 1 ? "" : "s") << ".\n";

    // Used only to express an excursion as tip displacement. Its own accuracy
    // does not matter much here -- it is a lever arm, not a datum -- so a
    // profile that is merely close enough still makes the number useful.
    std::optional<double> leverArmM;
    if (const auto *profile = selectProfile(profiles, sensorCount)) {
        leverArmM = profile->tip.translation.norm();
        std::cout << "Lever arm from the " << sensorCount << "-sensor profile: "
                  << formatMillimeters(*leverArmM) << ". At that offset 1 degree of sensor\n"
                  << "rotation is " << formatMillimeters(*leverArmM * 0.0174533)
                  << " at the tip, so orientation matters more than position here.\n";
    } else {
        std::cout << "No probe_profiles entry for " << sensorCount
                  << " sensors, so excursions are reported at the sensor only.\n";
    }

    if (sensorCount >= 2) {
        std::cout << "\nTwo or more sensors: the sensor-to-sensor transform is also tracked.\n"
                     "Rigidly mounted sensors hold it constant however the probe is moved, so\n"
                     "any drift there is relative movement -- no bench, no pivot, no offset\n"
                     "involved. A probe whose sensors are known rigid is worth running first,\n"
                     "to see what a good result looks like in this field.\n";
    }

    std::string answer;
    std::cout << "\n";
    if (!prompt("Rest the probe, take your hands off it, then press Enter (q to quit): ", answer))
        return 0;
    if (answer == "q" || answer == "Q")
        return 0;

    BaselineAccumulator accumulator;
    auto baseline = captureBaseline(viper, accumulator);
    if (!baseline.has_value()) {
        std::cerr << "Could not capture a baseline -- no usable pose data arrived.\n";
        return 1;
    }

    std::cout << "Noise floor: " << formatMillimeters(baseline->positionSigmaM) << ", "
              << std::fixed << std::setprecision(3) << baseline->orientationSigmaDeg
              << " deg over " << baseline->samples << " samples.\n";

    ExcursionTracker tracker{*baseline};
    RigidityWitness witness;
    if (sensorCount >= 2)
        witness.arm(viper.latestSensorPoses());

    viper.resetDistortion();

    std::cout << "\nMonitoring. Type a letter then Enter:\n"
                 "  m [label]  mark an event, e.g. \"m pull\" or \"m release\"\n"
                 "  r          re-capture the baseline (probe must be still and unloaded)\n"
                 "  p          clear the peak hold\n"
                 "  q          finish and report\n\n";

    // The poller owns the trackers while it runs; the reader thread below only
    // touches them under this lock, between reports.
    std::mutex trackerMtx;
    std::vector<MonitorEvent> events;
    const auto runStart = std::chrono::steady_clock::now();

    std::atomic_bool running{true};

    // One body for both the initial poll and the restart after a re-arm. Kept
    // as a named lambda because the two drifted apart when they were written
    // out separately -- the restarted one stopped drawing the live line.
    const auto pollLoop = [&] {
        auto lastReport = std::chrono::steady_clock::now();

        while (running) {
            {
                std::lock_guard<std::mutex> guard{trackerMtx};
                if (const auto pose = viper.latestFusedPose())
                    tracker.add(*pose);
                if (witness.armed())
                    witness.add(viper.latestSensorPoses());
            }

            const auto now = std::chrono::steady_clock::now();
            if (now - lastReport >= kReportInterval) {
                lastReport = now;

                std::lock_guard<std::mutex> guard{trackerMtx};
                const auto &excursion = tracker.excursion();

                std::ostringstream line;
                line << "\r  " << std::fixed << std::setprecision(3)
                     << excursion.positionM * 1000.0 << " mm | " << excursion.orientationDeg
                     << " deg | peak " << excursion.peakPositionM * 1000.0 << " mm / "
                     << excursion.peakOrientationDeg << " deg";

                if (leverArmM.has_value())
                    line << " -> " << std::setprecision(2)
                         << excursion.tipEquivalentM(*leverArmM) * 1000.0 << " mm tip";

                line << " | " << std::setprecision(1)
                     << excursion.significance(tracker.baseline()) << "x floor";
                line << " | " << describeDistortionBrief(viper.distortionSummary());

                if (witness.armed()) {
                    // Worst pair only, to keep this to one line. Every pair is
                    // listed in the summary, where localizing a single loose
                    // sensor actually matters.
                    double worstRotation = 0.0;
                    double worstSeparation = 0.0;
                    for (const auto &pair : witness.pairs()) {
                        worstRotation = std::max(worstRotation, pair.peakRotationDeviationDeg);
                        worstSeparation = std::max(worstSeparation, pair.peakSeparationDeviationM);
                    }
                    line << " | pair " << std::setprecision(3) << worstSeparation * 1000.0
                         << " mm / " << worstRotation << " deg";
                }

                line << "      ";
                std::cout << line.str() << std::flush;
            }

            std::this_thread::sleep_for(kPollInterval);
        }
    };

    std::thread poller{pollLoop};

    while (true) {
        if (!prompt("", answer))
            break;

        const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                                     std::chrono::steady_clock::now() - runStart)
                                     .count();
        const double seconds = static_cast<double>(elapsed) / 1000.0;

        if (answer == "q" || answer == "Q")
            break;

        if (!answer.empty() && (answer[0] == 'm' || answer[0] == 'M')) {
            std::string label = answer.size() > 1 ? answer.substr(1) : std::string{};
            while (!label.empty() && label.front() == ' ')
                label.erase(label.begin());
            if (label.empty())
                label = "mark";

            std::lock_guard<std::mutex> guard{trackerMtx};
            events.push_back(MonitorEvent{seconds, label});

            std::ostringstream note;
            note << std::fixed << std::setprecision(1) << "t=" << seconds << "s MARK \"" << label
                 << "\" at " << std::setprecision(3)
                 << tracker.excursion().positionM * 1000.0 << " mm, "
                 << tracker.excursion().orientationDeg << " deg";

            std::cout << "\n  " << note.str() << "\n";
            // Into the recording as well, so the MCAP carries the same
            // annotations the console summary does.
            fgInterface->logInfo("monitor: " + note.str());
            continue;
        }

        if (answer == "r" || answer == "R") {
            std::cout << "\n  Re-capturing the baseline -- hands off the probe.\n";

            // Stop polling first: captureBaseline drives the accumulator from
            // this thread, and the tracker is replaced underneath it.
            running = false;
            poller.join();

            auto refreshed = captureBaseline(viper, accumulator);
            if (refreshed.has_value()) {
                std::lock_guard<std::mutex> guard{trackerMtx};
                baseline = refreshed;
                tracker = ExcursionTracker{*refreshed};
                if (sensorCount >= 2)
                    witness.arm(viper.latestSensorPoses());
                std::cout << "  New noise floor: " << formatMillimeters(refreshed->positionSigmaM)
                          << ", " << std::fixed << std::setprecision(3)
                          << refreshed->orientationSigmaDeg << " deg\n";
            } else {
                std::cout << "  Baseline failed; keeping the previous one.\n";
            }

            running = true;
            poller = std::thread{pollLoop};
            continue;
        }

        if (answer == "p" || answer == "P") {
            std::lock_guard<std::mutex> guard{trackerMtx};
            tracker.clearPeaks();
            witness.clearPeaks();
            std::cout << "\n  Peaks cleared.\n";
            continue;
        }
    }

    running = false;
    if (poller.joinable())
        poller.join();

    std::cout << "\n\n=== Stability summary ===\n\n"
              << describeStability(tracker.excursion(), tracker.baseline(), leverArmM) << "\n";

    if (witness.armed()) {
        std::cout << "\nSensor-to-sensor (rigid mounting holds these at zero):\n";
        for (const auto &pair : witness.pairs())
            std::cout << "  " << describePair(pair) << "\n";

        if (leverArmM.has_value())
            std::cout << "  Worst pair is " << formatMillimeters(witness.worstTipEquivalentM(*leverArmM))
                      << " at the tip.\n";
    }

    const auto distortion = viper.distortionSummary();
    std::cout << "\nEM distortion: " << describeDistortion(distortion) << "\n";
    if (distortion.exceededThreshold())
        std::cout << "  Distortion this high moves the reported pose on its own. Treat any\n"
                     "  excursion above as an upper bound until it is repeated somewhere\n"
                     "  cleaner.\n";

    if (!events.empty()) {
        std::cout << "\nMarked events:\n";
        for (const auto &event : events)
            std::cout << "  " << std::fixed << std::setprecision(1) << event.timeSeconds << "s  "
                      << event.label << "\n";
        std::cout << "\nThe excursion trace and these marks are both in viper-monitor.mcap.\n";
    } else {
        std::cout << "\nNo events were marked, so nothing here ties an excursion to a load.\n"
                     "The peak alone cannot distinguish movement from drift -- mark the pulls\n"
                     "and releases on the next run.\n";
    }

    return 0;
}

} // namespace mdx
