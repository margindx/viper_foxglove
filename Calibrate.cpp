//
// See Calibrate.hpp.
//

#include "Calibrate.hpp"

#include <atomic>
#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <thread>

#include <nlohmann/json.hpp>

#include "CalibrationSession.hpp"
#include "FoxgloveInterface.hpp"
#include "ProbeConfigIo.hpp"
#include "ProbeProfile.hpp"
#include "Viper.hpp"

namespace mdx {

namespace {

using namespace std::literals::chrono_literals;

/// How often the fused pose is sampled. Well below the device rate: the session
/// decimates near-duplicates anyway, so polling faster would only spin.
constexpr auto kPollInterval = 20ms;

/// How often the live capture quality line is redrawn.
constexpr auto kReportInterval = 500ms;

std::string timestampSuffix() {
    const auto now = std::chrono::system_clock::now();
    const std::time_t t = std::chrono::system_clock::to_time_t(now);

    std::tm tm{};
#ifdef _WIN32
    localtime_s(&tm, &t);
#else
    localtime_r(&t, &tm);
#endif

    std::ostringstream ss;
    ss << std::put_time(&tm, "%Y%m%dT%H%M%S");

    return ss.str();
}

std::string formatMetres(const Eigen::Vector3d &v) {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(4) << "[" << v.x() << ", " << v.y() << ", " << v.z()
       << "] m";
    return ss.str();
}

std::string formatMillimetres(const Eigen::Vector3d &v) {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(2) << "[" << v.x() * 1000.0 << ", " << v.y() * 1000.0
       << ", " << v.z() * 1000.0 << "] mm";
    return ss.str();
}

std::string formatDegrees(const Eigen::Vector3d &v) {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(3) << "[" << v.x() << ", " << v.y() << ", " << v.z()
       << "] deg";
    return ss.str();
}

/// Read a line, returning false if stdin closed.
bool prompt(const std::string &question, std::string &answer) {
    std::cout << question << std::flush;

    if (!std::getline(std::cin, answer))
        return false;

    return true;
}

bool confirmed(const std::string &question) {
    std::string answer;
    if (!prompt(question + " [y/N] ", answer))
        return false;

    return answer == "y" || answer == "Y" || answer == "yes" || answer == "Yes";
}

/// Load probe_profiles if the config has any. Unlike the normal startup path
/// this tolerates their absence, since bootstrapping a new probe is the point.
std::vector<ProbeProfile> loadProfilesIfPresent(const std::string &configPath) {
    if (!std::filesystem::exists(configPath))
        return {};

    try {
        std::ifstream f(configPath);
        const auto settings = nlohmann::json::parse(f);
        return parseProbeProfiles(settings);
    } catch (const std::exception &e) {
        std::cout << "  (no usable probe_profiles yet: " << e.what() << ")\n";
        return {};
    }
}

/// Capture one step: poll poses until the operator accepts, showing what the
/// capture still lacks. Returns false if the operator aborted.
///
/// Looped rather than recursive: an operator restarting a difficult step many
/// times should not grow the stack.
bool captureStep(CalibrationSession &session, Viper &viper) {
    const auto info = session.currentStep();

    std::cout << "\n=== " << info.title << " ===\n" << info.instructions << "\n\n";

    while (true) {
        // Nothing is sampled until the operator says to start. Beginning the
        // moment the step opened meant the first samples were of the probe
        // being picked up and positioned -- they polluted the solve and counted
        // towards the capture gate, so a stationary probe looked like progress.
        // This prompt also flushes the instructions above, which otherwise sat
        // in the buffer while acquisition was already running.
        const bool resuming = session.sampleCount(info.step) > 0;

        std::string answer;
        if (!prompt(resuming
                            ? "Reposition, then press Enter to resume capturing (q to abort): "
                            : "Position the probe, then press Enter to start capturing (q to abort): ",
                    answer)) {
            return false;
        }

        if (answer == "q" || answer == "Q")
            return false;

        std::cout << "\nCapturing. Press Enter when the prompt says the capture is sufficient,\n"
                     "or type 'r' then Enter to restart this step, or 'q' to abort.\n\n"
                  << std::flush;

        // Poll on a worker so the operator's Enter is not queued behind a sleep.
        std::atomic_bool capturing{true};
        std::thread poller{[&] {
            auto lastReport = std::chrono::steady_clock::now();

            while (capturing) {
                if (const auto pose = viper.latestFusedPose())
                    session.addSample(*pose);

                const auto now = std::chrono::steady_clock::now();
                if (now - lastReport >= kReportInterval) {
                    lastReport = now;
                    const auto metrics = session.metrics();

                    std::ostringstream line;
                    line << "\r  " << metrics.sampleCount << " samples";
                    if (metrics.coneHalfAngleDeg > 0.0)
                        line << " | spread " << static_cast<int>(metrics.coneHalfAngleDeg)
                             << " deg";
                    if (metrics.directionSeparation > 0.0)
                        line << " | spin " << std::fixed << std::setprecision(2)
                             << metrics.directionSeparation;
                    line << (metrics.sufficient ? " | SUFFICIENT -- press Enter"
                                                : " | " + metrics.guidance);
                    line << "        ";

                    std::cout << line.str() << std::flush;
                }

                std::this_thread::sleep_for(kPollInterval);
            }
        }};

        const bool gotLine = prompt("", answer);

        // Stop and join before touching the session again: everything below
        // reads or mutates state the poller was writing.
        capturing = false;
        poller.join();
        std::cout << "\n";

        if (!gotLine || answer == "q" || answer == "Q")
            return false;

        if (answer == "r" || answer == "R") {
            session.restartStep();
            std::cout << "  Restarted this step.\n";
            continue;
        }

        if (!session.readyToAdvance()) {
            std::cout << "  Not enough yet: " << session.metrics().guidance << "\n";
            continue;
        }

        return true;
    }
}

void reportOutcome(const CalibrationOutcome &outcome, int sensorCount) {
    std::cout << "\n=== Result for " << sensorCount << " sensor"
              << (sensorCount == 1 ? "" : "s") << " ===\n";

    std::cout << "  Tip offset            " << formatMetres(outcome.tipOffset) << "\n";
    std::cout << "  Pivot residual (RMS)  " << std::fixed << std::setprecision(2)
              << outcome.pivot.residualRms * 1000.0 << " mm\n";
    std::cout << "  Residual per axis     " << formatMillimetres(outcome.pivot.residualRmsProbeFrame)
              << "  (probe frame: x along probe, y along the footprint long axis)\n";
    std::cout << "  Samples / condition   " << outcome.pivot.sampleCount << " / " << std::fixed
              << std::setprecision(1) << outcome.pivot.conditionNumber << "\n";

    // The footprint is 10 mm along y and 1 mm along z, so contact migration
    // should show up mostly on y. Anything else means a different problem.
    const auto &r = outcome.pivot.residualRmsProbeFrame;
    if (r.y() > 0.0 && r.z() > r.y())
        std::cout << "  NOTE: the across-footprint residual exceeds the along-footprint one, "
                     "which contact migration alone does not explain.\n";

    if (outcome.planeCheck.has_value()) {
        std::cout << "\n  Independent check (plane constraint):\n";
        std::cout << "    Tip offset          " << formatMetres(outcome.planeCheck->tipOffset)
                  << "\n";
        std::cout << "    Disagreement        " << std::fixed << std::setprecision(2)
                  << outcome.offsetDisagreementM * 1000.0 << " mm\n";

        if (outcome.offsetDisagreementM > 0.005) {
            std::cout << "    WARNING: the two estimates differ by more than 5 mm. They have "
                         "different\n             error models, so a large gap means at least one "
                         "is being strained.\n";
        }
    } else {
        std::cout << "\n  Independent plane-constraint check unavailable for this capture.\n";
    }

    if (outcome.tipRotation.has_value()) {
        const auto zyx = zyxDegreesFromQuaternion(*outcome.tipRotation);
        std::cout << "\n  Tip rotation (Z-Y-X)  " << formatDegrees(zyx) << "\n";
        std::cout << "    From identity       " << std::fixed << std::setprecision(3)
                  << outcome.rotationFromIdentityDeg << " deg\n";
        std::cout << "    Face normal fit     " << std::setprecision(3)
                  << outcome.faceNormal.residualDeg << " deg RMS, separation "
                  << outcome.faceNormal.separation << "\n";
        std::cout << "    Second flat fit     " << std::setprecision(3)
                  << outcome.secondFlat.residualDeg << " deg RMS, separation "
                  << outcome.secondFlat.separation << "\n";

        if (outcome.rotationFromIdentityDeg < 1.0) {
            std::cout << "    NOTE: this is within a degree of identity and is more likely "
                         "measurement noise\n          than a real mounting angle. Consider "
                         "declining and leaving the rotation unset.\n";
        }
    } else {
        std::cout << "\n  Tip rotation could not be determined; the existing value is unchanged.\n";
    }
}

} // namespace

int runCalibration(const std::string &configPath) {
    std::cout << "Viper probe tip calibration\n"
              << "Config: " << configPath << "\n\n";

    if (!std::filesystem::exists(configPath)) {
        std::cerr << "Config file not found: " << configPath
                  << "\nCalibration updates an existing config; create one first "
                     "(see README.md).\n";
        return 1;
    }

    std::cout << "You will need a flat surface, and a probe whose housing has a second flat\n"
                 "face (not the imaging face) that can rest on it.\n";

    // Held by optional so a failure to start (an unwritable directory, a stale
    // recording still held open) reports and exits rather than escaping as an
    // uncaught exception and aborting the process.
    std::optional<FoxgloveInterface> fgInterface;
    try {
        fgInterface.emplace("viper-calibration.mcap");
    } catch (const std::exception &e) {
        std::cerr << "Could not start the Foxglove interface: " << e.what() << "\n";
        return 1;
    }

    std::this_thread::sleep_for(1000ms);

    // Existing profiles are loaded so the connected sensor count can be matched
    // against them for the before/after comparison, but their absence is fine.
    auto profiles = loadProfilesIfPresent(configPath);

    Viper viper{&fgInterface.value(), profiles, 10, 100, /*calibrationMode=*/true};

    std::cout << "Waiting for pose data...\n";
    for (int i = 0; i < 250 && !viper.latestFusedPose().has_value(); i++) {
        if (viper.hasFatalError()) {
            std::cerr << "Cannot calibrate: " << viper.fatalErrorMessage() << "\n";
            return 1;
        }
        std::this_thread::sleep_for(20ms);
    }

    if (!viper.latestFusedPose().has_value()) {
        std::cerr << "No usable pose data from the Viper. Check the sensors and try again.\n";
        return 1;
    }

    const int sensorCount = viper.lastSensorCount();
    if (sensorCount < 1) {
        std::cerr << "The Viper is not reporting any sensors.\n";
        return 1;
    }

    std::cout << "Calibrating for " << sensorCount << " connected sensor"
              << (sensorCount == 1 ? "" : "s") << ".\n";

    const auto existing = readProfile(configPath, sensorCount);
    if (existing.has_value()) {
        std::cout << "Existing profile: tip offset " << formatMetres(existing->tipOffsetM);
        if (!existing->label.empty())
            std::cout << ", label \"" << existing->label << "\"";
        std::cout << "\n";
    } else {
        std::cout << "No existing profile for this sensor count; one will be created.\n";
    }

    CalibrationSession session;

    while (session.step() != CalibrationStep::Done) {
        if (viper.hasFatalError()) {
            std::cerr << "\nAborting: " << viper.fatalErrorMessage() << "\n";
            return 1;
        }

        if (!captureStep(session, viper)) {
            std::cout << "Aborted; nothing was written.\n";
            return 0;
        }

        session.advance();
    }

    // Step 3 recovers the second flat's normal, not the footprint's long axis.
    // Relating the two is a property of the probe's design, so it has to be
    // supplied rather than measured -- and it is asked for here, at run time,
    // so discovering it is 90 rather than 0 costs an answer instead of a
    // rebuild. It only affects roll about the probe axis; the tip position is
    // already fixed by step 1.
    double secondFlatRollDeg = 0.0;
    {
        std::cout << "\nStep 3 measured the second flat's normal. To turn that into the\n"
                     "footprint's orientation I need the angle between them, about the probe\n"
                     "axis, from the probe's design. Often 0 or 90.\n";

        std::string answer;
        if (!prompt("Angle from the second flat's normal to the footprint long axis, in degrees "
                    "[0]: ",
                    answer)) {
            std::cout << "Aborted; nothing was written.\n";
            return 0;
        }

        if (!answer.empty()) {
            try {
                secondFlatRollDeg = std::stod(answer);
            } catch (const std::exception &) {
                std::cerr << "Not a number: \"" << answer << "\". Nothing was written.\n";
                return 1;
            }
        }
    }

    const auto outcome = session.solve(secondFlatRollDeg);
    if (!outcome.has_value()) {
        std::cerr << "The captures could not be solved. Nothing was written.\n";
        return 1;
    }

    reportOutcome(*outcome, sensorCount);

    if (existing.has_value()) {
        const double delta = (outcome->tipOffset - existing->tipOffsetM).norm();
        std::cout << "\n  Change from the existing profile: " << std::fixed << std::setprecision(2)
                  << delta * 1000.0 << " mm\n";
        if (delta > 0.010) {
            std::cout << "  WARNING: that is a large change against a previously trusted value. "
                         "Check the\n           hardware and the capture before accepting it.\n";
        }
    }

    std::cout << "\n";
    if (!confirmed("Write this to " + configPath + "?")) {
        std::cout << "Not written.\n";
        return 0;
    }

    ProfileUpdate update;
    update.sensorCount = sensorCount;
    update.tipOffsetM = outcome->tipOffset;
    update.label = "calibrated " + std::to_string(sensorCount) + "-sensor probe";

    if (outcome->tipRotation.has_value() &&
        confirmed("  Also write the solved tip rotation?")) {
        update.tipRotationZyxDeg = zyxDegreesFromQuaternion(*outcome->tipRotation);
    }

    try {
        const auto report = writeProfile(configPath, update, timestampSuffix());
        std::cout << (report.created ? "Created" : "Updated") << " the "
                  << sensorCount << "-sensor profile in " << configPath << "\n";
        std::cout << "Previous config backed up to " << report.backupPath.string() << "\n";
    } catch (const std::exception &e) {
        std::cerr << "Could not write the config: " << e.what()
                  << "\nThe original is unchanged.\n";
        return 1;
    }

    return 0;
}

} // namespace mdx
