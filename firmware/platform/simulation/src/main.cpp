#include <micromouse/robot.hpp>

#include <rclcpp/rclcpp.hpp>

#include <simulation/hardware/buttons_impl.hpp>
#include <simulation/hardware/drivetrain_impl.hpp>
#include <simulation/hardware/ir_sensors_impl.hpp>
#include <simulation/hardware/feedback_impl.hpp>

#include <chrono>
#include <thread>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(get_simulation_buttons());
  executor.add_node(get_simulation_drivetrain());
  executor.add_node(get_simulation_ir_sensors());
  executor.add_node(get_simulation_feedback());

  Robot::get().init();

  Robot::get().on_connect();
  Robot::get().publish_extra_feedback();

  size_t iter = 0;

  while (rclcpp::ok()) {
    auto start_time_point = std::chrono::high_resolution_clock::now();

    executor.spin_some();

    Robot::get().periodic();

    if (iter++ % (ROBOT_PUBLISH_FEEDBACK_PERIOD_MS / ROBOT_UPDATE_PERIOD_MS) ==
        0) {
      Robot::get().publish_periodic_feedback();
    }

    auto end_time_point = std::chrono::high_resolution_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
        end_time_point - start_time_point);

    auto sleep_duration =
        std::chrono::milliseconds(ROBOT_UPDATE_PERIOD_MS) - duration;

    if (sleep_duration.count() > 0) {
      std::this_thread::sleep_for(sleep_duration);
    }
  }

  Robot::get().on_disconnect();

  rclcpp::shutdown();
  return 0;
}
