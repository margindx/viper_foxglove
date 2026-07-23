# MDX Robotics Relay

This code contains two separate programs:

1. A main C++ program that spawns a Foxglove server to capture:
   1. Data from the Polhemus Viper EM tracking system.
   2. Data from the custom nRF52840 microcontroller-based force sensing board
2. A minimal relay script in Python using a custom-developed Foxglove to ZMQ relay library ([`foxglove2zmq`](https://github.com/helkebir/foxglove2zmq)).

The C++ program is set up to try to connect to a Polhemus Viper module. In the `main` function, the force sensing
functionality is currently commented out since it relies on the presence of a force module.

For the probe positioning functionality, a rigid transformation is currently hardcoded to average 3 distinct EM sensors
and to translate the resulting position along the axial direction to get the probe tip position. This is based on the
legacy probe design, and will require a small update once the EM sensor module for the mid-size probe is designed.

The C++ program has the following dependencies:

- `Foxglove`
- `Open3D`
- `Protobuf`
- `libusb`

The Python code relies on [`foxglove2zmq`](https://github.com/helkebir/foxglove2zmq), which can be installed using
`pip install foxglove2zmq`. It currently spawns a pull server, but can be set to create a pub-sub server.


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

### Running

cd to the `buid/Releease` subdirectory of your `viper_foxglove` repo (e.g., `cd  C:\Users\dx\git\viper_foxglove\build\Release` ) and start it with `.\viper.exe` (a plain powershell is OK here)
