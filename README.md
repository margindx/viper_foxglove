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

For the probe positioning functionality, a rigid transformation is currently hardcoded to average 3 distinct EM sensors
and to translate the resulting position along the axial direction to get the probe tip position. This is based on the
legacy probe design, and will require a small update once the EM sensor module for the mid-size probe is designed.


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
`viper-config.json` in the working directory, or the path you passed as an argument. If that file does not
exist, it prints `No config file found. using default values` and runs entirely on the built-in defaults
compiled into the program.

Parsing is also field-by-field: any key omitted from the config file falls back to its built-in default, so a
partial config file only overrides the fields it specifies.

#### Available settings

| Key | Type | Default | Description |
| --- | --- | --- | --- |
| `offset_x` | float | `0.150` | Along-probe offset from sensor to tip (meters). |
| `offset_y` | float | `0.0` | Horizontal offset from sensor to probe center (keep at 0). |
| `offset_z` | float | `0.0` | Horizontal offset from sensor to probe center (keep at 0). |
| `minimum_contact_force` | float | `0.35` | Force threshold above which contact is registered (only used when `use_hardware_contact` is `false`). |
| `pressure_usb_id` | string | *(none — required)* | USB `VID:PID` (hex) of the pressure/force sensor, e.g. `"2886:8064"`. The serial port is **auto-detected** by matching this against connected USB devices (**Windows and Linux only**). There is no explicit-port option and no fallback: if the value is missing, malformed, or does not match **exactly one** connected device, the force sensor is disabled. See [Finding the force sensor's VID/PID](#finding-the-force-sensors-vidpid). |
| `use_hardware_contact` | bool | `true` | If `true`, use the 0/1 contact flag reported by the device; if `false`, derive contact from the force thresholds and `contact_require_fN` flags. |
| `contact_require_f1` | bool | `true` | Whether force sensor 1 must exceed `minimum_contact_force` for contact (only used when `use_hardware_contact` is `false`). |
| `contact_require_f2` | bool | `true` | Same as above, for force sensor 2. |
| `contact_require_f3` | bool | `true` | Same as above, for force sensor 3. |
| `contact_require_f4` | bool | `true` | Same as above, for force sensor 4. |
| `generate_geometry` | bool | `true` | Generate point clouds, the mesh, and the scene trail locally. Set to `false` when a downstream component generates them instead; raw poses, TF, and forces are published either way. |

> Note: the built-in default for `offset_x` is `0.150`, while the sample `viper-config.json` ships with
> `0.157`.

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

### Running

cd to the `buid/Releease` subdirectory of your `viper_foxglove` repo (e.g., `cd  C:\Users\dx\git\viper_foxglove\build\Release` ) and start it with `.\viper.exe` (a plain powershell is OK here)
