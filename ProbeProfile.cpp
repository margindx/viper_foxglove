//
// See ProbeProfile.hpp.
//

#include "ProbeProfile.hpp"

#include <cmath>
#include <set>
#include <sstream>
#include <stdexcept>

namespace mdx {

namespace {

constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

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
        // sum of antipodal representations cancels, and normalising the
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
