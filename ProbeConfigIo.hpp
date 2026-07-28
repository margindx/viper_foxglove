//
// Reading and updating a single probe_profiles entry in the config file.
//
// Calibration mutates the file that the normal run refuses to start without, so
// the write is deliberately careful: the original is backed up first, the new
// content is written to a temporary and renamed into place so a crash mid-write
// cannot leave a half-file, and every key the program does not own -- including
// the _comment block documenting the settings -- is preserved along with its
// order.
//

#ifndef VIPER_PROBECONFIGIO_HPP
#define VIPER_PROBECONFIGIO_HPP

#include <filesystem>
#include <optional>
#include <string>

#include "Eigen/Dense"

namespace mdx {

/// What calibration wants written for one sensor count.
struct ProfileUpdate {
    int sensorCount{0};
    Eigen::Vector3d tipOffsetM{Eigen::Vector3d::Zero()};
    /// Written only when present. Absent leaves any existing rotation alone
    /// rather than silently resetting it to identity.
    std::optional<Eigen::Vector3d> tipRotationZyxDeg;
    /// Used only when creating a new entry; an existing label is never
    /// overwritten.
    std::string label;
};

/// The values currently in the file for one sensor count, for showing the
/// operator what is about to change.
struct ExistingProfile {
    Eigen::Vector3d tipOffsetM{Eigen::Vector3d::Zero()};
    std::optional<Eigen::Vector3d> tipRotationZyxDeg;
    std::string label;
};

/// Read the profile for `sensorCount`, or nullopt if the file has none.
/// Throws std::runtime_error if the file cannot be read or parsed.
std::optional<ExistingProfile> readProfile(const std::filesystem::path &configPath, int sensorCount);

struct WriteReport {
    /// True when no entry for this sensor count existed before.
    bool created{false};
    std::filesystem::path backupPath;
};

/// Update or insert the probe_profiles entry for `update.sensorCount`.
///
/// `timestampSuffix` names the backup (viper-config.<suffix>.bak); it is passed
/// in rather than read from the clock so the write is deterministic under test.
///
/// Throws std::runtime_error on any failure, having left the original in place.
WriteReport writeProfile(const std::filesystem::path &configPath, const ProfileUpdate &update,
                         const std::string &timestampSuffix);

} // namespace mdx

#endif //VIPER_PROBECONFIGIO_HPP
