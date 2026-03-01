#include <micromouse/robot/robot.hpp>
#include <micromouse/logging.hpp>

#include <rclcpp/rclcpp.hpp>

#include <simulation/hardware/buttons_impl.hpp>
#include <simulation/hardware/drivetrain_impl.hpp>
#include <simulation/hardware/ir_sensors_impl.hpp>
#include <simulation/hardware/feedback_impl.hpp>

#include <chrono>
#include <thread>

using namespace robot;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  LogInfo(
      "\n"
      "┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓\n"
      "┃ Starting the MicroMouse firmware in simulation mode. Instead of interfacing with ┃\n"
      "┃ real hardware, this firmware program will use ROS2 topics to interface with a    ┃\n"
      "┃ simulated robot.                                                                 ┃\n"
      "┃                                                                                  ┃\n"
      "┃ This program is intended to be used with the accompanying Godot simulation to    ┃\n"
      "┃ test how this firmware behaves in a simulated 3D environment.                    ┃\n"
      "┃                                                                                  ┃\n"
      "┃ Both of the clients (Dashboard app and CLI) are able to be built with ROS2       ┃\n"
      "┃ support so that they can interface with this simulation firmware in the same way ┃\n"
      "┃ that they would interface with the firmware running on the actual robot via BLE. ┃\n"
      "┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛\n");
  LogWarn(
      "DO NOT RUN THIS PROGRAM FROM WITHIN VSCODE IF YOU WANT ROS2 TO BE ABLE TO CONNECT TO OTHER NODES!\n");  // Always forget

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(get_simulation_buttons());
  executor.add_node(get_simulation_drivetrain());
  executor.add_node(get_simulation_ir_sensors());
  executor.add_node(get_simulation_feedback());

  Robot::get().init();

  Robot::get().on_connect();
  Robot::get().publish_status_feedback();

  size_t iter = 0;

  while (rclcpp::ok()) {
    auto start_time_point = std::chrono::high_resolution_clock::now();

    executor.spin_some();

    Robot::get().periodic();

    if (iter++ % (ROBOT_PUBLISH_FEEDBACK_PERIOD_MS / ROBOT_UPDATE_PERIOD_MS) == 0) {
      Robot::get().publish_periodic_feedback();
    }

    auto end_time_point = std::chrono::high_resolution_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time_point - start_time_point);

    auto sleep_duration = std::chrono::milliseconds(ROBOT_UPDATE_PERIOD_MS) - duration;

    if (sleep_duration.count() > 0) {
      std::this_thread::sleep_for(sleep_duration);
    }
  }

  Robot::get().on_disconnect();

  rclcpp::shutdown();

  LogInfo("simulation firmware shutdown successfully");

  return 0;
}
