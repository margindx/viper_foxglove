//
// See ProbeProfile.hpp.
//

#include "ProbeProfile.hpp"

#include "DeviceState.hpp"

#include <cmath>
#include <set>
#include <sstream>
#include <stdexcept>

namespace mdx {

namespace {

constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;

/// How far the tip offset may point away from the tip frame's own +x axis.
/// Deliberately generous: this is here to catch a reversed sign or a swapped
/// axis, not to police the few degrees of slop that a real calibration leaves
/// when the tip is not perfectly on the probe's axis.
constexpr double kMaxTipAxisDisagreementDeg = 30.0;

/// A sensor quaternion this far from unit norm is a malformed frame, not a
/// rounding artefact: the Viper streams unit quaternions.
constexpr double kUnitNormTolerance = 0.1;

bool isFinite(const Eigen::Vector3d &v) {
    return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
}

bool isUsable(const Pose &pose) {
    if (!isFinite(pose.position))
        return false;

    const auto &c = pose.orientation.coeffs();
    for (int i = 0; i < 4; i++) {
        if (!std::isfinite(c[i]))
            return false;
    }

    return std::abs(pose.orientation.norm() - 1.0) <= kUnitNormTolerance;
}

/// Read a JSON array of exactly three numbers.
Eigen::Vector3d readVector3(const nlohmann::json &node, const std::string &key, const std::string &where) {
    if (!node.is_array() || node.size() != 3) {
        throw std::runtime_error(where + ": \"" + key + "\" must be an array of 3 numbers");
    }

    Eigen::Vector3d v;
    for (int i = 0; i < 3; i++) {
        if (!node[i].is_number()) {
            throw std::runtime_error(where + ": \"" + key + "\"[" + std::to_string(i) + "] must be a number");
        }
        v[i] = node[i].get<double>();
    }

    if (!isFinite(v)) {
        throw std::runtime_error(where + ": \"" + key + "\" must be finite");
    }

    return v;
}

} // namespace

Eigen::Quaterniond quaternionFromZyxDegrees(double azimuthDeg, double elevationDeg, double rollDeg) {
    const Eigen::Quaterniond q =
            Eigen::AngleAxisd(azimuthDeg * kDegToRad, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(elevationDeg * kDegToRad, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(rollDeg * kDegToRad, Eigen::Vector3d::UnitX());

    return q.normalized();
}

std::optional<Pose> fusePoses(const std::vector<Pose> &poses) {
    if (poses.empty())
        return std::nullopt;

    for (const auto &pose : poses) {
        if (!isUsable(pose))
            return std::nullopt;
    }

    // A single sensor is the identity case: hand back exactly what the device
    // reported rather than pushing it through an average of one.
    if (poses.size() == 1)
        return poses.front();

    const auto n = static_cast<double>(poses.size());
    const Eigen::Quaterniond reference = poses.front().orientation.normalized();

    Eigen::Vector3d positionSum = Eigen::Vector3d::Zero();
    Eigen::Vector4d quaternionSum = Eigen::Vector4d::Zero();

    for (const auto &pose : poses) {
        positionSum += pose.position;

        // q and -q are the same rotation. Without this flip the componentwise
        // sum of antipodal representations cancels, and normalizing the
        // near-zero result yields an arbitrary orientation.
        Eigen::Quaterniond q = pose.orientation.normalized();
        if (q.coeffs().dot(reference.coeffs()) < 0.0)
            q.coeffs() = -q.coeffs();

        quaternionSum += q.coeffs();
    }

    // Every term now has a non-negative dot with the reference, and the
    // reference itself contributes 1, so the norm cannot collapse. Checked
    // anyway: this is the last gate before the pose reaches the robot.
    const double norm = quaternionSum.norm();
    if (!std::isfinite(norm) || norm < 1e-9)
        return std::nullopt;

    Pose fused;
    fused.position = positionSum / n;
    fused.orientation = Eigen::Quaterniond{Eigen::Vector4d{quaternionSum / norm}};

    return fused;
}

Pose applyTipTransform(const Pose &sensorPose, const TipTransform &tip) {
    Pose tipPose;

    // The offset is measured in the sensor's own frame, so rotate it into the
    // tracker frame before adding.
    tipPose.position = sensorPose.position + sensorPose.orientation * tip.translation;
    tipPose.orientation = (sensorPose.orientation * tip.rotation).normalized();

    return tipPose;
}

const ProbeProfile *selectProfile(const std::vector<ProbeProfile> &profiles, int sensorCount) {
    for (const auto &profile : profiles) {
        if (profile.sensorCount == sensorCount)
            return &profile;
    }

    return nullptr;
}

std::vector<ProbeProfile> parseProbeProfiles(const nlohmann::json &settings) {
    if (!settings.contains("probe_profiles")) {
        throw std::runtime_error(
                "config is missing the required \"probe_profiles\" array. The legacy flat "
                "\"offset_x\"/\"offset_y\"/\"offset_z\" keys are no longer supported: the tip offset now "
                "depends on how many EM sensors are connected, so it must be given per sensor count. "
                "See the \"Probe profiles\" section of README.md");
    }

    const auto &node = settings["probe_profiles"];
    if (!node.is_array() || node.empty()) {
        throw std::runtime_error("\"probe_profiles\" must be a non-empty array");
    }

    std::vector<ProbeProfile> profiles;
    std::set<int> seenCounts;

    for (size_t i = 0; i < node.size(); i++) {
        const auto &entry = node[i];
        const std::string where = "probe_profiles[" + std::to_string(i) + "]";

        if (!entry.is_object()) {
            throw std::runtime_error(where + " must be an object");
        }

        ProbeProfile profile;

        if (!entry.contains("sensor_count") || !entry["sensor_count"].is_number_integer()) {
            throw std::runtime_error(where + ": \"sensor_count\" is required and must be an integer");
        }
        profile.sensorCount = entry["sensor_count"].get<int>();
        if (profile.sensorCount < 1) {
            throw std::runtime_error(where + ": \"sensor_count\" must be at least 1");
        }
        if (!seenCounts.insert(profile.sensorCount).second) {
            throw std::runtime_error(
                    where + ": duplicate \"sensor_count\" " + std::to_string(profile.sensorCount) +
                    "; each sensor count may appear at most once");
        }

        if (!entry.contains("tip_offset_m")) {
            throw std::runtime_error(where + ": \"tip_offset_m\" is required");
        }
        profile.tip.translation = readVector3(entry["tip_offset_m"], "tip_offset_m", where);

        if (entry.contains("tip_rotation_zyx_deg")) {
            const auto zyx = readVector3(entry["tip_rotation_zyx_deg"], "tip_rotation_zyx_deg", where);
            profile.tip.rotation = quaternionFromZyxDegrees(zyx.x(), zyx.y(), zyx.z());
        }

        // Both fields answer the same question -- which way along the sensor
        // does the probe point -- and nothing else keeps them in step. The tip
        // frame's +x is the along-probe direction by convention, so the offset
        // has to lie along it. When they disagree the tip lands in the right
        // place while the published orientation faces the other way, which
        // silently reverses the probe geometry and every point-cloud normal
        // drawn from it. That is invisible in the position, so it is caught
        // here instead.
        if (profile.tip.translation.norm() > 1e-9) {
            const Eigen::Vector3d tipAxis = profile.tip.rotation * Eigen::Vector3d::UnitX();
            const Eigen::Vector3d offsetDirection = profile.tip.translation.normalized();
            const double alignment = std::max(-1.0, std::min(1.0, tipAxis.dot(offsetDirection)));
            const double disagreementDeg = std::acos(alignment) * kRadToDeg;

            if (disagreementDeg > kMaxTipAxisDisagreementDeg) {
                std::ostringstream ss;
                ss << where << ": \"tip_offset_m\" and \"tip_rotation_zyx_deg\" disagree by "
                   << static_cast<int>(disagreementDeg)
                   << " degrees about which way the probe points. The tip frame's +x is the "
                      "along-probe direction, so the offset must lie along it, but this offset "
                      "points elsewhere. The tip position would still land correctly while the "
                      "published orientation faced the other way, reversing the drawn probe and "
                      "every point-cloud normal taken from it. If the offset was negated to "
                      "correct for how the sensor is mounted, the rotation has to say so too -- "
                      "a straight reversal is \"tip_rotation_zyx_deg\": [180.0, 0.0, 0.0]. The "
                      "exact rotation, including roll, is what \"viper --calibrate\" solves for.";
                throw std::runtime_error(ss.str());
            }
        }

        if (entry.contains("label")) {
            if (!entry["label"].is_string()) {
                throw std::runtime_error(where + ": \"label\" must be a string");
            }
            profile.label = entry["label"].get<std::string>();
        }

        profiles.push_back(profile);
    }

    return profiles;
}

int parseExpectedFrameRateHz(const nlohmann::json &settings) {
    if (!settings.contains("expected_frame_rate_hz")) {
        throw std::runtime_error(
                "\"expected_frame_rate_hz\" is required. It states the Viper frame rate this "
                "config was written for, and startup refuses to run if the SEU is set to "
                "anything else -- sample density, latency and the meaning of every recorded "
                "timestamp all follow from it. Set it to " + supportedFrameRateList() +
                ". See the \"Expected frame rate\" section of README.md.");
    }

    const auto &value = settings.at("expected_frame_rate_hz");
    if (!value.is_number_integer()) {
        throw std::runtime_error("\"expected_frame_rate_hz\" must be an integer number of Hz (" +
                                 supportedFrameRateList() + ")");
    }

    const int hz = value.get<int>();
    if (!isSupportedFrameRateHz(hz)) {
        throw std::runtime_error("\"expected_frame_rate_hz\" is " + std::to_string(hz) +
                                 ", which the Viper cannot produce. Valid rates are " +
                                 supportedFrameRateList() + ".");
    }

    return hz;
}

std::string describeProfile(const ProbeProfile &profile) {
    std::ostringstream ss;
    ss << profile.sensorCount << " sensor" << (profile.sensorCount == 1 ? "" : "s");
    if (!profile.label.empty())
        ss << " (\"" << profile.label << "\")";
    ss << ", tip offset [" << profile.tip.translation.x() << ", " << profile.tip.translation.y()
       << ", " << profile.tip.translation.z() << "] m";
    if (!profile.tip.rotation.isApprox(Eigen::Quaterniond::Identity()))
        ss << ", with a non-identity tip rotation";

    return ss.str();
}

} // namespace mdx
