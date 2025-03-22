# MicroMouse Firmware

This folder contains the firmware code for the MicroMouse.

The firmware is divided into two components:
1. Platform-independent logic library:
   - Contains all the code for navigating and solving the maze.  
   - Desktop unit tests can be found in the `tests` directory.
2. Platform-specific hardware implementations:
   - Each platform has its own folder in the `platform` directory containing the code for that platform's entry point and hardware implementations.

## Building: Command-Line

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

## Building: VS Code

Install the CMake extension for VS Code.

By default, CMake will target the latest version of the MicroMouse hardware.
To change this, enter settings and find the `Cmake: Configure Args` setting. To
build for desktop simulation, add `-DBUILD_SIMULATION=ON`, and to build desktop
tests, add `-DBUILD_TESTS=ON`. Please see the [Simulation Section](#simulation) for more information about simulation.

Open the command pallette and run `> CMake: Configure`, then `> CMake: Build`. 

To run all tests, open the command pallette and run `> CMake: Run Tests`.
Switch to the __Test Results__ tab to see a breakdown of the results.

Note: You may need to restart VS Code after first building for it to find the tests.

## Simulation

When building for simulation, the firmware is compiled as a desktop application. Instead of using hardware peripherals, the firmware instead publishes and subscribes to ROS2 topics. The simulation firmware can interact with the [MicroMouse Dashboard App](../app/README.md) when built for ROS2, and with the [3D Godot Simulator](../simulation/mouse_v3/README.md).