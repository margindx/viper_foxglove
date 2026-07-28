# MDX Robotics Relay

This code contains two separate programs:

1. A main C++ program that spawns a Foxglove server to capture:
   1. Data from the Polhemus Viper EM tracking system.
   2. Data from the custom nRF52840 microcontroller-based force sensing board
2. A minimal relay script in Python using a custom-developed Foxglove to ZMQ relay library ([`foxglove2zmq`](https://github.com/helkebir/foxglove2zmq)).

The C++ program is set up to try to connect to a Polhemus Viper module. In the `main` function, the force sensing
functionality is currently commented out since it relies on the presence of a force module.

For the probe positioning functionality, the connected EM sensors are fused into a single pose, which is then mapped onto
the probe tip by a rigid transform. Both the number of sensors and the transform come from the config file rather than
being hardcoded, so the same binary supports the legacy 3-sensor probe and a single-sensor probe — see
[Probe profiles](#probe-profiles).

The C++ program has the following dependencies:

- `Foxglove`
- `Open3D`
- `Protobuf`
- `libusb`

The Python code relies on [`foxglove2zmq`](https://github.com/helkebir/foxglove2zmq), which can be installed using
`pip install foxglove2zmq`. It currently spawns a pull server, but can be set to create a pub-sub server.


## Runtime configuration

The C++ program reads its runtime settings from a JSON config file. A sample file, `viper-config.json`, is
included at the repo root and documents each field in its `_comment` block. Treat this top-level file as an
example/template — leave it unedited and copy it to where you actually run the program.

### Providing a config file

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

### What happens if no config file is found

The program does **not** search for or resolve any alternate config file. It checks exactly one path — either
`viper-config.json` in the working directory, or the path you passed as an argument.

**A config file is required.** If that file does not exist, cannot be parsed, or does not contain a valid
`probe_profiles` block, the program prints an error and exits with status 1 rather than starting. This is
deliberate: `probe_profiles` carries the probe tip offset, and running with the wrong offset misplaces the
tip by centimetres without any visible symptom, so there is no default to fall back on.

Every other key is still parsed field-by-field: any key apart from `probe_profiles` that is omitted falls back
to its built-in default.

### Available settings

| Key | Type | Default | Description |
| --- | --- | --- | --- |
| `probe_profiles` | array | *(none — required)* | One entry per probe design, keyed by how many EM sensors it presents. See [Probe profiles](#probe-profiles). |
| `minimum_contact_force` | float | `0.35` | Force threshold above which contact is registered (only used when `use_hardware_contact` is `false`). |
| `pressure_device_port` | string | `/dev/ttyACM0` | USB device port for the pressure/force sensor. |
| `use_hardware_contact` | bool | `true` | If `true`, use the 0/1 contact flag reported by the device; if `false`, derive contact from the force thresholds and `contact_require_fN` flags. |
| `contact_require_f1` | bool | `true` | Whether force sensor 1 must exceed `minimum_contact_force` for contact (only used when `use_hardware_contact` is `false`). |
| `contact_require_f2` | bool | `true` | Same as above, for force sensor 2. |
| `contact_require_f3` | bool | `true` | Same as above, for force sensor 3. |
| `contact_require_f4` | bool | `true` | Same as above, for force sensor 4. |

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
| `tip_offset_m` | `[x, y, z]` | yes | Offset in **metres** from the fused sensor origin to the probe tip, in the sensor frame. `x` is the along-probe direction; `y`/`z` are offsets to the probe centre and are normally `0`. |
| `tip_rotation_zyx_deg` | `[az, el, roll]` | no | Rotation from the sensor frame to the tip frame, in degrees, using the Viper's own Z-Y-X (azimuth / elevation / roll) convention. Omit for identity. |
| `label` | string | no | Name echoed to the log when the profile is selected. |

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

#### How the sensors are fused

Positions are averaged. Orientations are averaged componentwise **after** flipping each quaternion into the
same hemisphere as the first sensor's: `q` and `−q` are the same rotation, so summing the raw components can
cancel to near-zero and yield an arbitrary orientation after normalisation. The tip transform is then applied
to the fused pose, and the published orientation is the tip frame's.

With `sensor_count: 1` the fusion is a pass-through: the published pose is exactly the sensor's own pose with
the tip transform applied, with no averaging arithmetic involved.

A frame whose readings are non-finite, or whose quaternions are not unit norm, is dropped without publishing a
tip pose, and a rate-limited warning is logged.

### Documenting fields inline

Because JSON has no native comment syntax, `viper-config.json` documents each setting inside a `_comment`
object that mirrors the field names. The program ignores any keys it doesn't recognize, so this block travels
with the config and keeps the field descriptions next to the values without affecting parsing. When adding a
new setting, add a matching entry under `_comment` describing it.


## Windows installation and building notes


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


### 3. Install Open3D 

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
