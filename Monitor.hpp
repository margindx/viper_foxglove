//
// The interactive stability monitor front end.
//
// A console shell around StabilityMonitor, in the same shape as Calibrate: it
// captures a baseline, polls poses, shows live excursion, and prints a summary.
// The judgment lives in StabilityMonitor, which is tested without hardware.
//

#ifndef VIPER_MONITOR_HPP
#define VIPER_MONITOR_HPP

#include <string>

namespace mdx {

/// Watch a connected probe for the sensor moving inside it.
///
/// Like calibration, this runs without a matching probe_profiles entry: a probe
/// whose stability is in question may well be one that has never calibrated
/// cleanly. A profile is used when present, only to turn an orientation
/// excursion into the tip displacement it implies.
///
/// Returns a process exit code: 0 on a completed or cleanly aborted run,
/// non-zero if the device could not be used.
int runMonitor(const std::string &configPath);

} // namespace mdx

#endif //VIPER_MONITOR_HPP
