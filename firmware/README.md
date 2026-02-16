# MicroMouse Firmware

This folder contains the firmware code for the MicroMouse.

The firmware is divided into two components:
1. Platform-independent logic library:
   - Contains all the code for navigating and solving the maze.  
   - Desktop unit tests can be found in the `tests` directory.
2. Platform-specific hardware implementations:
   - Each platform has its own folder in the `platform` directory containing the code for that platform's entry point and hardware implementations.

## Building

### Prerequisites

To build for the robot, you must install the [GNU Arm Embedded Toolchain](https://developer.arm.com/Tools%20and%20Software/GNU%20Toolchain).

To build for simulation, you must install [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html). Newer versions of ROS2 may work, but have not been tested.

> [!IMPORTANT]
> To building for simulation, you must [activate your ROS2 environment](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html#environment-setup) before CMake configuration so that the build script can find the required ROS2 libraries on your system.
>
> However, it is important to not have your ROS2 environment activated when building for the physical robot, since ROS2 may silently add unwanted compiler/linker flags to the build. Often these linker flags are unsupported by the linker being used, causing the build to fail. Other times the build may succeed, although these linker flags may have unforeseen consequences that can be difficult to debug.

### Building: Command-Line

By default, CMake will target the latest version of the MicroMouse hardware.
To build for desktop simulation, add `-DBUILD_SIMULATION=ON` to the
configuration command. To build desktop tests, add `-DBUILD_TESTS=ON`. Please see the [Simulation Section](#simulation) for more information about simulation.

```
cmake . -Bbuild
cmake --build build
```

To run all tests, use CTest:
```
cd build
ctest
```
To run a specific test, add `-R <test_name>` to the ctest command, or just run
the test executable directly (e.g. `./build/tests/test_maze`).

### Building: VS Code

Install the CMake extension for VS Code.

By default, CMake will target the latest version of the MicroMouse hardware.
To change this, enter settings and find the `Cmake: Configure Args` setting. To
build for desktop simulation, add `-DBUILD_SIMULATION=ON`, and to build desktop
tests, add `-DBUILD_TESTS=ON`. Please see the [Simulation Section](#simulation) for more information about simulation.

Open the command pallette and run `> CMake: Configure`, then `> CMake: Build`. 

To run all tests, open the command pallette and run `> CMake: Run Tests`.
Switch to the __Test Results__ tab to see a breakdown of the results.

> [!NOTE]
> You may need to restart VS Code after first building for it to find the tests.

## Simulation

When built for simulation, the firmware is compiled as a desktop application. Instead of using hardware peripherals, the firmware instead publishes and subscribes to ROS2 topics. The simulation firmware can interact with the [MicroMouse Dashboard App](../app/README.md) when built for ROS2, and with the [3D Godot Simulator](../simulation/mouse_v3/README.md).
