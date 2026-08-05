//
// The interactive calibration front end.
//
// A thin console shell around CalibrationSession: it prints the instructions
// for each step, polls the fused pose, shows live capture quality, and writes
// the result back to the config once the operator confirms it. All the
// decision-making lives in CalibrationSession and the solvers, which are tested
// without any of this.
//

#ifndef VIPER_CALIBRATE_HPP
#define VIPER_CALIBRATE_HPP

#include <string>

namespace mdx {

/// Run a guided calibration against a connected Viper and update `configPath`.
///
/// Deliberately tolerant of a config with no matching probe_profiles entry --
/// calibrating a probe that has never been calibrated is the main reason to run
/// this, and the normal startup path refuses to run in that state.
///
/// Returns a process exit code: 0 on success or a clean operator abort,
/// non-zero if the device could not be used or the write failed.
int runCalibration(const std::string &configPath);

} // namespace mdx

#endif //VIPER_CALIBRATE_HPP
