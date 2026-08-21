/**
 *  t21_tracked.hpp
 *  ------------------------------------------------------------------
 *  ROS 2 Control SystemInterface для гусеничной платформы Т-21.
 *  ВНИМАНИЕ: в пакете UDP линейная скорость передаётся в см/с,
 *            угловая — в град/с, положение геометрии — в градусах.
 *  Добавлено логирование вызовов read() в текстовый файл для отладки.
 *  ------------------------------------------------------------------
 */
#pragma once

#include <array>
#include <vector>
#include <fstream>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

#include "udp_tracked.hpp"
#include "visibility_control.h"

namespace t21_hardware
{

class T21TrackedHardware final : public hardware_interface::SystemInterface
{
public:
  T21TrackedHardware();
  RCLCPP_SHARED_PTR_DEFINITIONS(T21TrackedHardware)

  ROS2_CONTROL_T21_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareInfo & info) override;

  ROS2_CONTROL_T21_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  ROS2_CONTROL_T21_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  ROS2_CONTROL_T21_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/) override;

  ROS2_CONTROL_T21_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/) override;

  ROS2_CONTROL_T21_HARDWARE_PUBLIC
  hardware_interface::return_type read(
      const rclcpp::Time & /*time*/,
      const rclcpp::Duration & /*period*/) override;

  ROS2_CONTROL_T21_HARDWARE_PUBLIC
  hardware_interface::return_type write(
      const rclcpp::Time & /*time*/,
      const rclcpp::Duration & /*period*/) override;


private:
  tracked_platform_udp::EthTrackedSocket socket_{};

  // индексы: 0-linVel, 1-angVel, 2-geomPos
  std::array<double, 3> cmd_{0.0, 0.0, 0.0};
  std::array<double, 3> state_{0.0, 0.0, 0.0};

  void send_stop_packet();
  double state_prev_geom_{0.0};

  bool geom_latched_{false};

  // локально интегрируемые позиции виртуальных суставов
  double left_pos_{0.0};
  double right_pos_{0.0};

  rclcpp::Time last_time_;
  rclcpp::Time last_receive_time_;
  bool first_read_{true};
  bool           is_connected_ = false;
  float          last_good_deg_ = 180.0f;
  rclcpp::Time   last_cmd_ts_{0,0,RCL_SYSTEM_TIME};
  rclcpp::Time   last_stop_ts_{0,0,RCL_SYSTEM_TIME};
  float          last_feedback_deg_ = 180.0f;
  rclcpp::Time   write_blocked_until_{0,0,RCL_SYSTEM_TIME};
  // лог-файл для отладки
  std::ofstream log_file_;
};

}  // namespace t21_hardware
