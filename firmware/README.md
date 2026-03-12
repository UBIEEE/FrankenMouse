# MicroMouse Firmware

This folder contains the firmware code for the MicroMouse.

The firmware is divided into two components:
1. Platform-independent logic library:
   - Contains all the code for navigating and solving the maze.  
   - Desktop unit tests can be found in the `tests` directory.
2. Platform-specific hardware implementations:
   - Each platform has its own folder in the `platform` directory containing the code for that platform's entry point and hardware implementations.

## Building for the Robot

### Prerequisites

To build for the robot, you must install the [GNU Arm Embedded Toolchain](https://developer.arm.com/Tools%20and%20Software/GNU%20Toolchain).

Windows:
```powershell
winget install --id=Arm.GnuArmEmbeddedToolchain -e
```

macOS
```zsh
brew install --cask gcc-arm-embedded
```

Linux (Ubuntu):
```bash
sudo apt install gcc-arm-none-eabi
```

### Building

By default, CMake will configure the build for the latest version of the MicroMouse hardware. To manually select a specific platform, add `-DBUILD_MOUSE_V2=ON` or `-DBUILD_MOUSE_V3=ON` to the configuration command. 

<details>
  <summary>Note about ROS2 messing things up...</summary>

  > It is recommended that you do not have your ROS2 environment activated when configuring the CMake project to build for the physical robot, since ROS2 may silently add unwanted compiler/linker flags to the build. Often these linker flags are unsupported by the embedded linker being used, causing the build to fail. Other times the build may succeed, although these linker flags may have unforeseen consequences that can be difficult to debug.

</details>

```bash
cmake . -Bbuild -DBUILD_MOUSE_V3=ON
cmake --build build
```

The MicroMouse firmware will be built to `build/mouse-vX-firmware.elf`.

## Building for Desktop (Simulation)

### Prerequisites

To build for simulation, you must install [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html). Newer versions of ROS2 may work, but have not been tested.

### Building

Activate your ROS2 environment:

```bash
source /opt/ros/humble/setup.bash
```

Configure and build

```bash
cmake . -Bbuild -DBUILD_SIMULATION=ON -DBUILD_TESTS=ON
cmake --build build
```

The simulation firmware will be built to `build/simulation`.
Unit tests will be built to `build/tests`.

The built simulation firmware simply publishes and subscribes to ROS2 topics instead of using hardware peripherals, allowing it to interact with the [App](../app/README.md), [CLI](../cli/README.md) and the [3D Godot Simulator](../simulation/mouse_v3/README.md).

### Running Tests

To run all tests, use CTest:
```bash
cd build
ctest
```

## Using Visual Studio Code

### Setup

Install the CMake extension for VS Code.

Open the `.vscode/settings.json` file and edit the `cmake.configureArgs` setting to specify your desired target platform, example:

```json
  ...
  "cmake.configureArgs": [
    "-DBUILD_MOUSE_V3=ON", // Build for V3 hardware
    "-DBUILD_MOUSE_V2=OFF",
    "-DBUILD_SIMULATION=OFF",
    "-DBUILD_TESTS=OFF",
  ],
  ...
```


<details>
  <summary>Building for the Robot</summary>
  
  1. Install the __STM32CubelDE for Visual Studio Code__ extension.
  2. Once installed, you should see the following notification:
     __Would you like to configure discovered Make projects as STM32Cube project(s)?__
  3. Click __Yes__
  4. A __Project Setup__ tab should open. In this tab, enter the __Board/Device__ (STM32WB55CGU6 for V2 and V3), and select the __Project__ from the dropdown: either `mouse-v3-firmware` or `mouse-v2-firmware`. If you don't see either of these options, make sure you have edited the `cmake.configureArgs` setting to specify the correct target platform (see above) and have run `> CMake: Configure`.

</details>

<details>
  <summary>Building for Simulation</summary>

  To build the simulation firmware, your ROS2 environment must be activated before launching VS Code. The simplest way to do this is to launch VS Code from the terminal:

  ```bash
  cd firmware/
  source /opt/ros/humble/setup.bash
  code .
  ```

</details>

### Building

Open the command pallette and run `> CMake: Configure`, then `> CMake: Build`.

### Running Tests

If tests were built, open the command pallette and run `> CMake: Run Tests`. This will run all tests and show the results in the __Test Results__ tab.

> [!NOTE]
> You may need to restart VS Code after first building for it to find the tests.

### Debugging

Switch to the __Run and Debug__ tab on the left sidebar.

To debug the firmware on a connected robot, select the __STM32Cube: STM32 Launch ST-Link GDB Server__ launch configuration.

To debug the simulation firmware, select the __Launch Simulation Firmware__ launch configuration.
