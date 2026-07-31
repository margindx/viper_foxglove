# MDX Robotics Relay

This code contains two separate programs:

1. A main C++ program that spawns a Foxglove server to capture:
   1. Data from the Polhemus Viper EM tracking system.
   2. Data from the custom nRF52840 microcontroller-based force sensing board
2. A minimal relay script in Python using a custom-developed Foxglove to ZMQ relay library ([`foxglove2zmq`](https://github.com/helkebir/foxglove2zmq)).

## `foxglove2zmq`

The Python code relies on [`foxglove2zmq`](https://github.com/helkebir/foxglove2zmq), which can be installed using
`pip install foxglove2zmq`. It currently spawns a pull server, but can be set to create a pub-sub server.

## C++ viper.exe

The C++ program is set up to try to connect to a Polhemus Viper module. In the `main` function, the force sensing
functionality is currently commented out since it relies on the presence of a force module.

For the probe positioning functionality, the connected EM sensors are fused into a single pose, which is then mapped onto
the probe tip by a rigid transform. Both the number of sensors and the transform come from the config file rather than
being hardcoded, so the same binary supports the legacy 3-sensor probe and a single-sensor probe — see
[Probe profiles](#probe-profiles).


### Requirements
The C++ program has the following dependencies:

- `Foxglove`
- `Protobuf`
- `libusb`
- `Open3D` (**optional** — only needed for mesh reconstruction; see [Open3D (optional)](#open3d-optional))

#### Open3D (optional)

Open3D is only used by the mesh reconstruction endpoints, which are not currently exercised by the main loop.
The program compiles and runs fine without it. Whether Open3D is used is controlled by the `MDX_WITH_OPEN3D`
CMake cache variable:

- `AUTO` (default) — use Open3D if CMake can find it, otherwise build without it.
- `ON` — require Open3D; configuration fails if it is not found.
- `OFF` — never use Open3D, even if it is installed.

Set it at configure time, e.g. `cmake --preset=default -DMDX_WITH_OPEN3D=OFF`. CMake prints whether the mesh
reconstruction endpoints are `ENABLED` or `DISABLED` during configuration.

When Open3D is not compiled in, the mesh reconstruction endpoints (`publishMesh` / `publishMeshModel`) become
no-ops that log a one-time warning the first time they are called, so nothing else in the program is affected.

This is separate from the [`generate_geometry`](#available-settings) runtime setting, which turns off *all*
locally-generated geometry (point clouds and the scene trail as well as the mesh). The two compose: with
`generate_geometry` set to `false` the mesh endpoints return immediately and no Open3D warning is logged,
since no geometry was asked for in the first place.

### Runtime configuration

The C++ program reads its runtime settings from a JSON config file. A sample file, `viper-config.json`, is
included at the repo root and documents each field in its `_comment` block. Treat this top-level file as an
example/template — leave it unedited and copy it to where you actually run the program.

#### Providing a config file

After building, the `viper` executable lives in a build directory (e.g. `build/Release/viper.exe` on Windows).
By default the program looks for a file named `viper-config.json` in the current working directory. The typical
workflow is:

1. Copy the top-level `viper-config.json` into the build directory next to the executable.
2. Edit that copy to set the values you need.
3. Run the program from that directory so it picks up the copied config.

Alternatively, copy `viper-config.json` anywhere you like, edit it, and pass its path as the first
command-line argument:

```
# uses ./viper-config.json from the current directory
./viper            # (Windows: .\viper.exe)

# uses a specific file
./viper /path/to/my-config.json
```

On startup the program prints which file it parsed (or that no file was found) and echoes the full set of
runtime settings it is using, so you can confirm the values took effect.

#### What happens if no config file is found

The program does **not** search for or resolve any alternate config file. It checks exactly one path — either
`viper-config.json` in the working directory, or the path you passed as an argument.

**A config file is required.** If that file does not exist, cannot be parsed, or does not contain a valid
`probe_profiles` block, the program prints an error and exits with status 1 rather than starting. This is
deliberate: `probe_profiles` carries the probe tip offset, and running with the wrong offset misplaces the
tip by centimeters without any visible symptom, so there is no default to fall back on.

Every other key is still parsed field-by-field: any key apart from `probe_profiles` that is omitted falls back
to its built-in default.

#### Available settings

| Key | Type | Default | Description |
| --- | --- | --- | --- |
| `probe_profiles` | array | *(none — required)* | One entry per probe design, keyed by how many EM sensors it presents. See [Probe profiles](#probe-profiles) and [Calibrating the tip offset](#calibrating-the-tip-offset). |
| `minimum_contact_force` | float | `0.35` | Force threshold above which contact is registered (only used when `use_hardware_contact` is `false`). |
| `pressure_usb_id` | string | *(none — required)* | USB `VID:PID` (hex) of the pressure/force sensor, e.g. `"2886:8064"`. The serial port is **auto-detected** by matching this against connected USB devices (**Windows and Linux only**). There is no explicit-port option and no fallback: if the value is missing, malformed, or does not match **exactly one** connected device, the force sensor is disabled. See [Finding the force sensor's VID/PID](#finding-the-force-sensors-vidpid). |
| `use_hardware_contact` | bool | `true` | If `true`, use the 0/1 contact flag reported by the device; if `false`, derive contact from the force thresholds and `contact_require_fN` flags. |
| `contact_require_f1` | bool | `true` | Whether force sensor 1 must exceed `minimum_contact_force` for contact (only used when `use_hardware_contact` is `false`). |
| `contact_require_f2` | bool | `true` | Same as above, for force sensor 2. |
| `contact_require_f3` | bool | `true` | Same as above, for force sensor 3. |
| `contact_require_f4` | bool | `true` | Same as above, for force sensor 4. |
| `generate_geometry` | bool | `true` | Generate point clouds, the mesh, and the scene trail locally. Set to `false` when a downstream component generates them instead; raw poses, TF, and forces are published either way. |

### Required Viper device settings

The SEU must be configured to report **meters** and **quaternions**. These are persistent device settings,
and both Polhemus factory defaults (inches, Euler degrees) are wrong for this program:

- Positions are treated as meters everywhere — `tip_offset_m`, the published frames, the point clouds. A
  device left on inches would rescale all of it by 39.37.
- Orientations are read as `(w, x, y, z)`. In either Euler mode the fourth value is unused and the first
  three are azimuth/elevation/roll, so the same four floats become a meaningless rotation.

Neither mistake announces itself, so the program reads the units from the first frame and **refuses to run**
on anything else, naming what the device reported and what is required. On the good path it logs the detected
units once at startup. Nothing here ever *sets* the units — use `CMD_UNITS` on the device.

### Device configuration readout

On the first frame the program reads the SEU's own configuration and logs it to `/viper/log`, so every MCAP
recording carries the device state it was made under. That question is otherwise unanswerable after the fact,
and it matters: a stale `CMD_TIP_OFFSET` was once displacing every published tip by 143 mm with nothing in the
data to show it.

**Settings that silently transform the geometry stop the run**, because the program applies its own transform
on top of them and the result looks like a mounting fault rather than a configuration one:

| Setting | Effect if set |
| --- | --- |
| `CMD_TIP_OFFSET` | the device has already displaced positions to a tip; `tip_offset_m` is then added again |
| `CMD_BORESIGHT` | orientations are pre-rotated, so the tip offset is applied along a rotated frame |
| `CMD_SNS_ORIGIN` | the sensor reports about a different origin than the published frames assume |
| `CMD_SRC_ROTATION` | the whole tracker frame is rotated |

**Settings that affect latency or cadence are logged, not gated**: `CMD_FILTER`, `CMD_PREDFILTER_CFG` and
`_EXT`, `CMD_FRAMERATE`, `CMD_INCREMENT`, and `CMD_WHOAMI` (device, serial, firmware). Increment mode is worth
noticing — it makes the device report only after a movement threshold, which presents as an irregular stream
rather than as an error.

A setting the device declines to report is logged as **unverified** rather than assumed safe. Failing to read
a setting is not evidence that it is unset.

### Per-frame monitoring

Three fields arrive in every frame and are checked rather than discarded:

- **Distortion** (`SFinfo.bfDistortion`, 0–255) — EM distortion degrades position and orientation directly and
  is otherwise invisible. A rate-limited warning is logged above a threshold. That threshold is a guess, not a
  measured limit, and wants tuning against a rig known to be clean.

  Calibration surfaces it as well, because none of its capture gates can: spread, condition number and sample
  count are all measures of *geometry*, so a capture taken wholly inside a distorted field passes every one of
  them and yields a confident, well-conditioned, wrong answer. The live capture line shows the current and peak
  level, the result block reports peak and mean per step beside the residuals, and a capture whose peak went
  above the threshold needs an extra confirmation before it can be written.
- **The frame counter** — gaps are the direct evidence of dropped data that an unexplained publish rate only
  hints at. Counted and reported, with backwards or very large jumps treated as a counter reset rather than a
  drop.
- **The virtual-sensor flag** (`SFinfo.bfSvirt`) — a sensor the SEU reports without one being physically
  present. Fusing it would average a fabricated pose into the tip, so it stops the run.

### Probe profiles

The probe tip offset depends on which probe is fitted, and each probe design presents a different number of EM
sensors to the Viper SEU. `probe_profiles` maps one to the other:

```json
"probe_profiles": [
    {
        "sensor_count": 3,
        "label": "legacy triple",
        "tip_offset_m": [0.157, 0.0, 0.0],
        "tip_rotation_zyx_deg": [0.0, 0.0, 0.0]
    },
    {
        "sensor_count": 1,
        "label": "mid-size single",
        "tip_offset_m": [0.150, 0.0, 0.0]
    }
]
```

| Field | Type | Required | Description |
| --- | --- | --- | --- |
| `sensor_count` | int ≥ 1 | yes | Number of connected EM sensors this profile applies to. Each count may appear at most once. |
| `tip_offset_m` | `[x, y, z]` | yes | Offset in **meters** from the fused sensor origin to the probe tip, in the sensor frame. `x` is the along-probe direction; `y`/`z` are offsets to the probe center and are normally `0`. |
| `tip_rotation_zyx_deg` | `[az, el, roll]` | no | Rotation from the sensor frame to the tip frame, in degrees, using the Viper's own Z-Y-X (azimuth / elevation / roll) convention. Omit for identity. |
| `label` | string | no | Name echoed to the log when the profile is selected. |

`tip_offset_m` and `tip_rotation_zyx_deg` both say which way along the sensor the probe points, and they are
checked against each other at startup: the tip frame's +x is the along-probe direction by convention, so the
offset must lie along it. A profile where they disagree by more than 30° is a startup error.

This catches a specific and otherwise silent mistake. If a sensor is mounted with its +x pointing away from
the tip, negating `tip_offset_m` puts the tip in the right place — but leaves the published orientation facing
backwards, which reverses the drawn probe geometry and every point-cloud normal taken from it. The position
looks correct throughout, so nothing else would report it. The rotation has to be negated too: a straight
reversal is `"tip_rotation_zyx_deg": [180.0, 0.0, 0.0]`, and the exact value including roll is what
`viper --calibrate` solves for.

The 30° tolerance is deliberately loose. It is there to catch a reversed sign or a swapped axis, not to police
the couple of degrees of slop a real calibration leaves when the tip is not perfectly on the probe's axis.

#### Switching between probes

**Switching probes means changing what is plugged in. Nothing in the config needs editing to switch.**

On startup the program reads the sensor count from the first Viper frame that reports any sensors, selects the
matching profile, and logs which one it picked:

```
Probe profile selected: 3 sensors ("legacy triple"), tip offset [0.157, 0, 0] m
```

The profile is then **latched** for the rest of the run. Keep both profiles in your config and the same file
works for either probe.

Two failure cases are reported rather than guessed at:

- **No profile matches the connected sensor count.** No tip pose is published at all, and an error naming the
  configured counts is logged. The raw per-sensor poses on `/viper/poses` keep flowing.
- **The sensor count changes mid-run** (typically an intermittent connector). The originally latched profile
  stays in effect — the tip offset does not shift under the operator mid-experiment — and a rate-limited error
  is logged saying the published tip pose is no longer trustworthy. Check the connections and restart.

#### Calibrating the tip offset

`tip_offset_m` can be measured from the mechanical design, or solved for from the probe's own motion:

```
./viper --calibrate            # (Windows: .\viper.exe --calibrate)
./viper --calibrate /path/to/my-config.json
```

This is a **bench procedure**, run at the machine the Viper is attached to. Unlike a normal run it does
**not** require an existing `probe_profiles` entry for the connected sensor count — producing that entry is
the point, so a probe can be calibrated for the first time. It writes to the same config file it read.

You need a flat surface. The probe rests on the 4-screw side of its body for the second step. There are
two captures, and the program will not let you leave one until the data can actually support a solve — it
shows live what is still missing. Nothing is sampled until you have read the step and pressed Enter, so you
can get the probe into position first.

```
STEP 1 — lens pivot                   STEP 2 — body side
                                      (viewed from above)
   \      |      /
    \     |     /                        ,------------------.
     \    |    /     probe body          | S            lens|
      \   |   /      sensor to lens      `------------------'
       \  |  /
        \ | /                            lay the flattened side down,
   ------o------  surface                 then rotate it on the surface
        lens stays on one spot             and re-place at many angles
```

1. **Lens pivot.** Rest the probe's lens on the surface and keep it on that one spot throughout — it must not
   slide.

   The lens is a narrow rectangle, so rock it on a definite edge rather than sweeping it around freely; that
   keeps the contact predictable.

   1. Rock back and forth over the long (10 mm) edge, tilting at least 20 degrees each way.
   2. Turn the probe to a new heading, keeping the lens on the same spot, and rock over the long edge again.
      Repeat at several headings.
   3. If the prompt still asks for more spread, rock carefully over the short (1 mm) edge too.

   Rocking on one edge only tilts in a single plane, and a sweep confined to one plane is ill-conditioned
   however long you run it — hence the varied headings. The reason to prefer them over the short edge is
   contact migration: the lens center is the datum being solved for, and it sits 0.5 mm from the long edge but
   5 mm from the short one, so rocking on the short edge displaces the datum ten times as far. Keep it in
   reserve for when the varied headings alone do not satisfy the gate.

   The prompt will not let a single-heading capture through, but it used to. The spread figure alone cannot
   tell the two apart — rocking 25 degrees in one plane reports the same 25 degrees a varied sweep does — so
   the condition number is the only thing separating them, and its bound was loose enough to pass a planar
   capture that misplaces the tip by millimeters. The live line now shows spread as two numbers, `26/9 deg`:
   how far the probe was tilted, and how much of that was in some direction other than the dominant one. The
   second number stays near zero until you turn to a new heading.

   This solves the tip offset. It also gives the probe axis for free — the probe is straight and the lens lies
   on its axis, so the sensor-to-lens vector *is* that axis. Nothing needs to be captured for it, which is why
   there is no step that stands the probe on its lens.

2. **Body side.** The probe body is oval in cross-section, with two opposite sides flatter than the rest. One
   shows **4 screws**, the other 6. Lay the probe down with the **4-screw side** against the surface, and let
   it settle. Then, keeping that side on the surface the whole time, turn the probe slowly through a full
   circle, as though sweeping a clock hand around.

   **Which side goes down decides the roll**, and the two differ by half a turn. Using the 6-screw side, or
   swapping part way through, produces a result that looks equally plausible and is 180° out. Nothing in the
   data can tell the two apart, which is why the side is named rather than left to choice.

   Two cautions, both of which corrupt the result rather than merely slowing it down:

   - **Do not lift it.** Capture is continuous, so poses recorded mid-lift — with the side off the surface —
     go into the solve alongside the good ones.
   - **Do not let it rock.** The section is oval, so uneven pressure tilts it, and that tilt is precisely the
     quantity being measured.

   This solves the one further direction needed to fix roll about the probe axis, and a long line of contact
   along the body is a far more stable angular reference than balancing the probe on its lens.

   Because the recovered direction is that side's normal rather than the footprint's long axis, the program
   then asks for the angle between the two, measured about the probe axis with the 4-screw side down. That is a property of the probe's
   design and comes from CAD, not from the capture — commonly `0` or `90` degrees. It affects only roll; the
   tip position is already fixed by step 1.

Do both steps on the **same surface**: step 2's recovered world direction is used as the surface normal for
the independent check on step 1.

The result is printed with its residuals before anything is written, and you are asked to confirm. The
rotation is confirmed separately, so you can accept a new offset while leaving the orientation alone. The
previous config is copied to `viper-config.<timestamp>.bak` and the new one is written atomically, so an
interrupted write cannot leave a truncated file that the next normal run would refuse to start on.

##### Reading the residuals

The per-axis residual is expressed in the **sensor's own body axes**, not the probe's. The two coincide only
when the tip rotation is identity, so on a probe whose sensor is not mounted squarely an error that is
strongly directional in the probe frame is spread evenly across all three components. The program says so
when it detects that case; an even spread there is expected rather than suspicious.

The overall RMS is the number to judge the capture by. A second, independent estimate is computed by applying
a plane constraint to the same rocking samples, using the surface normal from step 2. It has a *different*
error model, so agreement between the two is evidence — and a disagreement above 5 mm is reported together
with both condition numbers, since the plane check is the weaker solve and fails first.

Note what calibration cannot do: step 2 determines orientation only. With the flat down every time, the
offset along its normal is perfectly confounded with the unknown position of the surface, so those placements
carry no information about the translation at all.

#### How the sensors are fused

Positions are averaged. Orientations are averaged componentwise **after** flipping each quaternion into the
same hemisphere as the first sensor's: `q` and `−q` are the same rotation, so summing the raw components can
cancel to near-zero and yield an arbitrary orientation after normalization. The tip transform is then applied
to the fused pose, and the published orientation is the tip frame's.

With `sensor_count: 1` the fusion is a pass-through: the published pose is exactly the sensor's own pose with
the tip transform applied, with no averaging arithmetic involved.

A frame whose readings are non-finite, or whose quaternions are not unit norm, is dropped without publishing a
tip pose, and a rate-limited warning is logged.

### Finding the force sensor's VID/PID

Every USB device advertises a 16-bit **Vendor ID** and **Product ID** (each written as 4 hex digits). The
program identifies the force sensor by these — not by a fixed port name — and auto-detects whichever serial
port that device currently occupies. The sample `viper-config.json` ships with `"pressure_usb_id": "2886:8064"`,
which is the custom nRF52840 board (Seeed, VID `2886` / PID `8064`); set it to a different `VID:PID` only if
your board reports different IDs.

Enter the value as `"VID:PID"` in hex, e.g. `"2886:8064"` (case-insensitive; a `0x` prefix is also accepted).
Detection is passive — it reads USB descriptors and never opens any port, so it cannot disturb devices other
applications are using — and it is deliberately strict: it uses the port **only** when exactly one connected
device matches, otherwise the sensor stays disabled and the program keeps looking in the background.

**Windows**

1. Open **Device Manager** and find the board (under **Ports (COM & LPT)**, or **Universal Serial Bus devices**).
2. Right-click it → **Properties** → **Details** tab.
3. In the **Property** dropdown choose **Hardware Ids** (or **Device instance path**). You'll see a string like
   `USB\VID_2886&PID_8064&MI_00`. The digits after `VID_` and `PID_` are your VID and PID → `"2886:8064"`.

**Linux**

- Easiest — `lsusb` prints `ID <vid>:<pid>` directly:

  ```
  $ lsusb
  Bus 001 Device 005: ID 2886:8064 Seeed Technology Co., Ltd. ...
  ```

  Here the sensor's `pressure_usb_id` is `"2886:8064"`.
- Or, from a known device node:

  ```
  $ udevadm info --name=/dev/ttyACM0 --attribute-walk | grep -m1 -i idVendor
  $ udevadm info --name=/dev/ttyACM0 --attribute-walk | grep -m1 -i idProduct
  ```

  (equivalently, read `/sys/class/tty/ttyACM0/device/../idVendor` and `.../idProduct`).

> **macOS:** VID/PID auto-detection is not implemented, so the force sensor is unsupported on macOS (it stays
> disabled). Windows and Linux only.

### Checking that the sensor is not moving inside the probe

```
./viper --monitor            # (Windows: .\viper.exe --monitor)
```

Every tip offset assumes the EM sensor is rigidly fixed in the probe body. If it can slide or rotate in its
housing — dragged by its own cable, say — then the offset is not a constant, and no amount of care during
calibration will pin it down. The symptom is calibration runs that each look internally consistent but
disagree with each other, which is indistinguishable from a run of poor captures.

**The main test involves no probe motion at all.** Rest the probe on the bench and disturb only its cable:
pull, release, pull again, twist it, let it hang over the edge. The body is stationary by construction, so
you do not need a second sensor to certify it did not move — anything that moves is the sensor inside it.
Holding the probe still also freezes the field, which no moving test can do: any test that carries the probe
around confounds mechanical change with distortion that varies from place to place.

Mark each pull and release with `m pull` / `m release` as you make it. **Correlation with the marks is the
evidence, not the excursion itself** — a step that repeats with every load and reverses on release is
mechanical, while drift does not care what your hand is doing. A run with no marks cannot tell them apart,
and the summary says so.

The run opens by holding still for five seconds to measure the noise floor, and every later excursion is
reported as a multiple of it. Below about 3× there is nothing a run of this length could see.

| key | effect |
| --- | --- |
| `m [label]` | mark an event, e.g. `m pull` — also written to the MCAP |
| `r` | re-capture the baseline (probe still and unloaded) |
| `p` | clear the peak hold |
| `q` | finish and print the summary |

Peaks are held because the informative moment is transient: a tug is over before you can look up.

**Watch orientation at least as closely as position.** At a 194 mm offset, one degree of sensor rotation is
3.4 mm at the tip, so a rotation far too small to see or feel outweighs a position shift that would be
obvious. When `probe_profiles` has an entry for the connected sensor count, its offset length is used as a
lever arm and the tip-equivalent displacement is shown alongside.

#### With two or more sensors

Two sensors rigidly mounted on one body hold a constant transform between them, however the probe is moved.
The monitor tracks that transform for every sensor pair, so **any drift there is relative movement** —
established with no bench, no pivot and no tip offset involved. This is the decisive form of the test, and it
needs a second sensor clamped (not taped — otherwise you are measuring the tape) to the probe.

Two things make it more useful than it first appears:

- A **multi-sensor probe whose mounting is known good** can be run first, to see what "rigid" looks like in
  your field including distortion. That calibrates the test itself.
- With three sensors, one working loose shows up in two pairs and not the third, which localizes it. All
  pairs are tracked rather than consecutive ones for exactly this reason.

Like `--calibrate`, this does not require an existing `probe_profiles` entry: a probe whose sensor may be
loose is often one that has never calibrated cleanly enough to have one. The full trace and the event marks
are written to `viper-monitor.mcap`.

### Documenting fields inline

Because JSON has no native comment syntax, `viper-config.json` documents each setting inside a `_comment`
object that mirrors the field names. The program ignores any keys it doesn't recognize, so this block travels
with the config and keeps the field descriptions next to the values without affecting parsing. When adding a
new setting, add a matching entry under `_comment` describing it.


## Windows installation and building notes for viper.exe


### 1. Install Build Tools for VS Code

Install [Build Tools for VS Code](https://visualstudio.microsoft.com/downloads/#build-tools-for-visual-studio-2026) (which includes the `vcpkg` C/C++ package manager).  Defaults for a C/C++ workflow are fine. Also install VS Code if it's not there.

(may require a restart)

### 2. find path where `vcpkg` is installed

Find the path where `vcpkg` is installed by running the following command from a powershell


```
Get-ChildItem C:\ -recurse -include "vcpkg.exe"
```


You'll get a lot of permission-denied errors, but you should also see a positive match similar to:


```
Directory: C:\Program Files (x86)\Microsoft Visual Studio\18\BuildTools\VC\vcpkg

Mode LastWriteTime Length Name

---- ------------- ------ ----

-a---- 4/16/2026 10:14 AM 6579232 vcpkg.exe
```


### 3. Install Open3D (optional)

Open3D is only required for the mesh reconstruction endpoints. If you don't need them, skip this step — the
build will automatically proceed without Open3D (or pass `-DMDX_WITH_OPEN3D=OFF` at configure time to be
explicit). See [Open3D (optional)](#open3d-optional) above for details.

To build with Open3D support:

1. Go to https://github.com/isl-org/Open3D/releases , go to the assets of the release you want (we are using 0.19.0 as of 4/16/2026) and download the `open3d-devel-windows-amd64-0.19.0.zip`
2. Unpack that zip and put it somewhere nice.
3. Set a new environment variable `Open3D_DIR` that points to the `CMake` subdirectory of `open3d-devel-windows-amd64-0.19.0`. e.g., on the 3017 pc: `C:\Users\dx\Documents\open3d-devel-windows-amd64-0.19.0\open3d-devel-windows-amd64-0.19.0\CMake`. (In windows, to set an env variable via a GUI: open start/search then search for "Edit the system environment" and open it. Click "Envionrment Variables..." and add a new one. )

### 4. Configure and Build viper_foxglove

#### Source configuration

1. From the `viper_foxglove` git repo, check out the `u/mxk62/win` branch
2. Modify the `VCPKG_ROOT` field in `CMakeUserPresets.json` to point to the `vcpkg` install path (from step 2 above). On the 3017 PC, this file looks like:

```json
{
  "version": 10,
  "configurePresets": [
    {
      "name": "default",
      "inherits": "vcpkg",
      "environment": {
        "VCPKG_ROOT": "C:/Program Files (x86)/Microsoft Visual Studio/18/BuildTools/VC/vcpkg"
      }
    }
  ]
}
```

#### Building

**VERY IMPORTANT**: do not use a plain powershell. After installing the Build Tools, you'll have command prompts available that handle the environment setup needed to access the various tools.

1. Open a  `x64 Native Tools Command Prompt for VS`  (start typing that in the search bar and it should pop up)
2. `cd` to your `viper_foxglove` directory (e.g., `cd  C:\Users\dx\git\viper_foxglove` )
3. run `cmake --preset=default`  (this will fetch a bunch of dependencies )
4. build a release: `cmake --build build --config Release`

#### Running the tests

The probe-profile fusion maths and config parsing are covered by unit tests that need no hardware. They are
built alongside the main target and run with:

```
ctest --test-dir build -C Release --output-on-failure
```

The test target depends only on Catch2 (fetched by vcpkg via `vcpkg.json`) and the vendored headers under
`dep/`, so it builds without Open3D, the Foxglove SDK or libusb. If Catch2 is not present, CMake prints
`Catch2 not found - skipping the viper_tests target` and the main build proceeds; CI runs `ctest` with
`--no-tests=error` so a missing Catch2 there fails the run instead of silently skipping.

### Running

cd to the `buid/Releease` subdirectory of your `viper_foxglove` repo (e.g., `cd  C:\Users\dx\git\viper_foxglove\build\Release` ) and start it with `.\viper.exe` (a plain powershell is OK here)
