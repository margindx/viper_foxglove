//
// See ProbeConfigIo.hpp.
//

#include "ProbeConfigIo.hpp"

#include <fstream>
#include <stdexcept>

#include <nlohmann/json.hpp>

namespace mdx {

namespace {

/// ordered_json, not json: the default type sorts keys alphabetically on
/// serialisation, which would reshuffle the config and break the _comment
/// block's correspondence with the fields it documents.
using ordered = nlohmann::ordered_json;

ordered readDocument(const std::filesystem::path &configPath) {
    std::ifstream in(configPath);
    if (!in)
        throw std::runtime_error("cannot open config file " + configPath.string());

    try {
        return ordered::parse(in);
    } catch (const std::exception &e) {
        throw std::runtime_error("cannot parse config file " + configPath.string() + ": " + e.what());
    }
}

Eigen::Vector3d toVector3(const ordered &node, const std::string &where) {
    if (!node.is_array() || node.size() != 3)
        throw std::runtime_error(where + " must be an array of 3 numbers");

    Eigen::Vector3d v;
    for (int i = 0; i < 3; i++) {
        if (!node[i].is_number())
            throw std::runtime_error(where + " must contain only numbers");
        v[i] = node[i].get<double>();
    }

    return v;
}

ordered toJson(const Eigen::Vector3d &v) {
    return ordered::array({v.x(), v.y(), v.z()});
}

/// Index of the entry for `sensorCount`, or -1.
int findProfileIndex(const ordered &profiles, int sensorCount) {
    for (std::size_t i = 0; i < profiles.size(); i++) {
        const auto &entry = profiles[i];
        if (entry.is_object() && entry.contains("sensor_count") &&
            entry["sensor_count"].is_number_integer() &&
            entry["sensor_count"].get<int>() == sensorCount) {
            return static_cast<int>(i);
        }
    }

    return -1;
}

} // namespace

std::optional<ExistingProfile> readProfile(const std::filesystem::path &configPath, int sensorCount) {
    const ordered document = readDocument(configPath);

    if (!document.contains("probe_profiles") || !document["probe_profiles"].is_array())
        return std::nullopt;

    const auto &profiles = document["probe_profiles"];
    const int index = findProfileIndex(profiles, sensorCount);
    if (index < 0)
        return std::nullopt;

    const auto &entry = profiles[static_cast<std::size_t>(index)];

    ExistingProfile existing;
    if (entry.contains("tip_offset_m"))
        existing.tipOffsetM = toVector3(entry["tip_offset_m"], "tip_offset_m");

    if (entry.contains("tip_rotation_zyx_deg"))
        existing.tipRotationZyxDeg = toVector3(entry["tip_rotation_zyx_deg"], "tip_rotation_zyx_deg");

    if (entry.contains("label") && entry["label"].is_string())
        existing.label = entry["label"].get<std::string>();

    return existing;
}

WriteReport writeProfile(const std::filesystem::path &configPath, const ProfileUpdate &update,
                         const std::string &timestampSuffix) {
    if (update.sensorCount < 1)
        throw std::runtime_error("sensor count must be at least 1");

    ordered document = readDocument(configPath);

    if (!document.contains("probe_profiles"))
        document["probe_profiles"] = ordered::array();

    if (!document["probe_profiles"].is_array())
        throw std::runtime_error("\"probe_profiles\" in " + configPath.string() + " is not an array");

    auto &profiles = document["probe_profiles"];
    const int index = findProfileIndex(profiles, update.sensorCount);

    WriteReport report;
    report.created = index < 0;

    if (report.created) {
        ordered entry;
        entry["sensor_count"] = update.sensorCount;
        if (!update.label.empty())
            entry["label"] = update.label;
        entry["tip_offset_m"] = toJson(update.tipOffsetM);
        if (update.tipRotationZyxDeg.has_value())
            entry["tip_rotation_zyx_deg"] = toJson(*update.tipRotationZyxDeg);

        profiles.push_back(entry);
    } else {
        auto &entry = profiles[static_cast<std::size_t>(index)];
        entry["tip_offset_m"] = toJson(update.tipOffsetM);

        // Leave an existing rotation alone when this run did not solve one:
        // silently resetting it to identity would undo a previous calibration.
        if (update.tipRotationZyxDeg.has_value())
            entry["tip_rotation_zyx_deg"] = toJson(*update.tipRotationZyxDeg);
    }

    // Back up before touching anything, so a bad calibration is one copy from
    // undone.
    const std::filesystem::path backupPath =
            configPath.parent_path() /
            (configPath.stem().string() + "." + timestampSuffix + ".bak");

    std::error_code ec;
    std::filesystem::copy_file(configPath, backupPath,
                               std::filesystem::copy_options::overwrite_existing, ec);
    if (ec)
        throw std::runtime_error("cannot write backup " + backupPath.string() + ": " + ec.message());

    report.backupPath = backupPath;

    // Write to a temporary and rename, so an interrupted write cannot leave a
    // truncated config -- which the normal run would then refuse to start on.
    const std::filesystem::path tempPath = configPath.string() + ".tmp";
    {
        std::ofstream out(tempPath, std::ios::binary | std::ios::trunc);
        if (!out)
            throw std::runtime_error("cannot open " + tempPath.string() + " for writing");

        out << document.dump(4) << "\n";
        if (!out)
            throw std::runtime_error("failed writing " + tempPath.string());
    }

    std::filesystem::rename(tempPath, configPath, ec);
    if (ec) {
        std::filesystem::remove(tempPath);
        throw std::runtime_error("cannot replace " + configPath.string() + ": " + ec.message());
    }

    return report;
}

} // namespace mdx
