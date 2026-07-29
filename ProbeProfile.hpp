//
// Probe profiles: how the EM sensors of a given probe design are fused into a
// single probe-tip pose.
//
// This header is deliberately free of the Foxglove SDK, Open3D, libusb and the
// Viper USB stack: it depends only on the vendored Eigen and nlohmann headers
// under dep/. That keeps the fusion maths unit-testable without hardware.
//

#ifndef VIPER_PROBEPROFILE_HPP
#define VIPER_PROBEPROFILE_HPP

#include <optional>
#include <string>
#include <vector>

#include "Eigen/Dense"
#include "Eigen/Geometry"
#include <nlohmann/json.hpp>

namespace mdx {

/// A position and orientation, in whatever frame and units the caller is using.
struct Pose {
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
};

/// Rigid transform from the (fused) EM sensor frame to the probe tip frame.
struct TipTransform {
    /// Sensor-frame offset from the sensor origin to the tip.
    Eigen::Vector3d translation{Eigen::Vector3d::Zero()};
    /// Rotation from the sensor frame to the tip frame. Identity when the tip
    /// frame is simply a translation of the sensor frame.
    Eigen::Quaterniond rotation{Eigen::Quaterniond::Identity()};
};

/// One probe design: identified by how many EM sensors it presents to the SEU.
struct ProbeProfile {
    /// Number of sensors this profile applies to, matched against the count
    /// reported in the Viper PNO frame.
    int sensorCount{0};
    /// Human-readable name, echoed to the log when the profile is selected.
    std::string label;
    TipTransform tip;
};

/// Z-Y-X intrinsic Euler angles in degrees to a quaternion.
///
/// This is the Viper's own azimuth/elevation/roll convention (see
/// SensorData::convertFromEuler), i.e. R = Rz(azimuth) * Ry(elevation) * Rx(roll).
Eigen::Quaterniond quaternionFromZyxDegrees(double azimuthDeg, double elevationDeg, double rollDeg);

/// Fuse per-sensor poses into a single pose.
///
/// Positions are averaged componentwise. Orientations are averaged
/// componentwise too, but each quaternion is first flipped into the hemisphere
/// of the first one: q and -q represent the same rotation, so summing raw
/// components can cancel to near-zero and produce an arbitrary result after
/// normalisation.
///
/// A single input pose is returned verbatim (no arithmetic, so no rounding):
/// with one sensor the published pose is exactly what the device reported.
///
/// Returns nullopt when there is nothing to fuse, or when any input is
/// unusable — a non-finite component, or a quaternion that is not unit norm
/// within a tolerance. Those indicate a malformed frame, which should be
/// dropped rather than published.
std::optional<Pose> fusePoses(const std::vector<Pose> &poses);

/// Map a fused sensor pose onto the probe tip.
///
/// The offset is expressed in the sensor frame, so it is rotated by the sensor
/// orientation before being added. The returned orientation is the tip frame's,
/// which is the sensor orientation composed with the profile's rotation — with
/// an identity rotation this leaves the orientation untouched.
Pose applyTipTransform(const Pose &sensorPose, const TipTransform &tip);

/// The profile matching `sensorCount`, or nullptr if there is none.
/// The returned pointer aliases `profiles` and is valid for its lifetime.
const ProbeProfile *selectProfile(const std::vector<ProbeProfile> &profiles, int sensorCount);

/// Parse the required "probe_profiles" array out of a parsed config document.
///
/// Throws std::runtime_error with an actionable message if the key is missing
/// or any entry is malformed. There is no fallback to the legacy flat
/// offset_x/offset_y/offset_z keys: running with an offset that does not match
/// the connected hardware silently misplaces the tip, so it is refused instead.
std::vector<ProbeProfile> parseProbeProfiles(const nlohmann::json &settings);

/// One-line summary of a profile, for logging.
std::string describeProfile(const ProbeProfile &profile);

} // namespace mdx

#endif //VIPER_PROBEPROFILE_HPP
