# Troubleshooting: probe tip pose missing / blue dot stuck at the world origin

Written 2026-07-28. Applies to `feat/em-sensor-profiles` (PR #56) and
`feat/pivot-calibration` (PR #58).

Symptom that prompted this: running `viper_foxglove` against brain's
`OCTEmTrackerMultiSensorDebugger.vue`, the blue dot representing viper's
projected tip stayed at the world origin.

---

## 1. What the blue dot is

`avePointFromZMQ` is created at `(0,0,0)` and only ever moves in one handler:

| | location |
| --- | --- |
| created at the origin | `OCTEmTrackerMultiSensorDebugger.vue:586` |
| only update site | `updateAvePointFromZMQ`, line 407 |
| fed by | `OCT_METADATA_ZMQ` socket → `/ws/oct_metadata_zmq` → gateway → `em-pose` ZMQ plane → **`/hh/pose`** |

So "at the origin" means that handler never ran with a usable record. It is not a
rendering problem.

The per-sensor axes come from a **different** socket
(`/ws/oct_metadata_multi_zmq`, fed by **`/viper/poses`**). If the per-sensor axes
move while the blue dot does not, `/viper/poses` is flowing and `/hh/pose` is
not — which is a publisher-side condition, not a transport one.

## 2. The one check that halves the problem

The pressure plot and the contact history are driven by the **same handler, from
the same records** (`updateAvePointFromZMQ`, lines ~392-404, immediately before
the point update).

- **Pressure plot and contact history also dead** → nothing is arriving on
  `/ws/oct_metadata_zmq`. Problem is upstream of the browser → section 3.
- **Pressures/contact updating, dot still at the origin** → records arrive but
  `em_x/em_y/em_z` are zero or missing. Problem is in the record contents →
  section 3, step 4.

## 3. Troubleshooting order

1. **Read viper_foxglove's own output first.** Exactly one of these should
   appear at startup:
   - `Probe profile selected: N sensors ("...")` — profile latched, tip poses
     should be publishing.
   - `No probe profile configured for N sensor(s)` — **this is the cause.** Note
     *which* N it reports; it may not be the N you expect (an intermittent
     connector showing as 2 when the config defines only 1 and 3).
   - `Dropped a PNO frame with unusable sensor data` — the fusion is rejecting
     frames; see section 4.
   - `Sensor count changed from X to Y` — intermittent connector.

   If the TUI launched viper (`VIPER_FOXGLOVE_BINARY`), stdout goes to
   `logs/tui/viper.log`. These also go to the Foxglove log channel.

2. **Isolate viper from the transport.** Open the `viper.mcap` written next to
   the binary, or point Foxglove at `ws://localhost:8765`, and check whether
   `/hh/pose` has any messages. `/viper/poses` populated + `/hh/pose` empty
   confirms the publisher-side gate and rules out the relay and gateway.

3. **If `/hh/pose` has messages**, the break is relay → gateway → websocket.
   Check `foxglove_relay` is running and the `em-pose` plane is subscribed, then
   browser devtools → Network → WS → `/ws/oct_metadata_zmq` for arriving frames.

4. **If frames arrive but the dot is at the origin**, inspect
   `records[0].em_x/em_y/em_z` in a frame. Zeros there point at the gateway's
   `_sensor_to_oct_metadata` mapping rather than at viper.

---

## 4. If the profile loads correctly and `/hh/pose` is still empty

Assume the 1-sensor profile is selected and latched. What in the code can still
suppress `/hh/pose`?

### Useful coupling: which topics fail together

`/hh/pose`, `/hh/combined`, `/hh/swing_twist` and the point clouds are **all
emitted from the same place** — `FoxgloveInterface::publishPose` logs the pose
channel, builds `CombinedSensorData`, and appends to `pointsAll_`, and
`logSwingTwist` is called immediately after in `Viper.cpp`. They are empty
together or populated together.

`/viper/poses` is published independently, further down the same function, and
is **not** gated on any of this.

So:

| observation | implicated |
| --- | --- |
| `/viper/poses` populated, all `/hh/*` empty | group A below |
| everything empty including `/viper/poses` | group B below |
| `/hh/combined` populated but `/hh/pose` empty | impossible — recheck how you are reading them |

### Group A — suppresses `/hh/*` only, `/viper/poses` unaffected

**A1. `fusePoses` rejects the frame. Most likely cause.**

With one sensor, `fusePoses` returns `nullopt` exactly when `isUsable` fails:

- position not finite, or
- any quaternion component not finite, or
- `abs(|q| - 1) > 0.1` — the quaternion is not unit norm within tolerance.

`/viper/poses` performs **no validation** — it publishes the raw floats. So a
malformed orientation silences every `/hh/*` topic while `/viper/poses` looks
perfectly healthy.

**The likely trigger is the device being in Euler orientation mode.**
`ORI_EULER_DEGREE` is the Polhemus factory default. In that mode `ori[0..2]` are
azimuth/elevation/roll in degrees and `ori[3]` is unused, but the code reads the
four floats as `(w, x, y, z)`. The resulting "quaternion" has norm
`sqrt(az^2 + el^2 + roll^2)` — for a probe at 45/10/5 degrees that is about 46,
nowhere near 1. `isUsable` fails on essentially every frame.

Expect this in the log, rate-limited to every 100th frame:

```
Dropped a PNO frame with unusable sensor data (N frame(s) so far): no tip pose published for it
```

**Decisive check:** look at the orientation values on `/viper/poses` in Foxglove.
If the fourth component is always exactly `0` and the others are large
(tens, not fractions of 1), the device is streaming Euler angles.

Degenerate sub-case: a probe sitting at all-zero Euler angles gives norm 0, also
rejected.

This is precisely the hazard the `FrameUnits` guard on #58 was written for. On
#56 there is no units check, so it shows up as this quiet symptom instead of a
clear startup failure. If you are on #58 it aborts at startup with a `Fatal:`
message naming the units instead.

**A2. `nSensors == 0` on every frame.** With a profile already latched, a count
of zero yields an empty `sensorPoses`, so `fusePoses` returns `nullopt` for the
empty-input reason. `/viper/poses` still publishes, with an empty array. The
`Sensor count changed from 1 to 0` error should also be firing.

### Group B — suppresses everything, including `/viper/poses`

**B1. Frames never reach `pnoToFoxgloveSceneUpdate`, silently.** Two gates in
`Viper::publishContinuous` drop frames with **no logging at all**:

```cpp
if (br && (br == (*(uint32_t*)(respPkg+4)+8))) {     // size check -- silent drop
    crc = calculateCrc16(respPkg, br-4);
    if (validateCrc(crc, respPkg, br-4)) {           // CRC check -- silent drop
```

If either fails consistently, nothing is published on any topic and there is no
diagnostic anywhere. The `Viper USB read misaligned` warning in `readUsb` covers
transfers that do not start on a preamble, but not frames that pass that and
then fail size or CRC.

**This is an observability gap worth closing** — a rate-limited counter on both
branches would turn a silent dead stream into a legible one.

**B2. The device never actually started streaming.** `startContinuousRead()` sets
`isContinuous = true` and starts the publish thread, but the CRC-mismatch and
missing-ACK paths only *log* — both `throw`s are commented out. A Viper that
never acknowledged `CMD_CONTINUOUS_PNO` leaves the publish thread running against
a queue that never fills. Both topics stay silent.

**B3. USB read misalignment.** `readUsb` drops transfers with no PNO/CMD
preamble, logging every 100th as `Viper USB read misaligned`.

### Ranked, for a 1-sensor probe whose profile loads correctly

1. **A1** — Euler orientation mode, or otherwise non-unit quaternions. Check the
   log for `Dropped a PNO frame with unusable sensor data`, and the quaternion
   values on `/viper/poses`.
2. **B1** — silent size/CRC drops. Distinguish by whether `/viper/poses` is also
   empty.
3. **A2** — zero sensors reported.
4. **B2 / B3** — streaming never started, or USB framing.
