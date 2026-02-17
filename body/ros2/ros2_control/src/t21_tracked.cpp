#include "ros2_control_t21_hardware/t21_tracked.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include <cmath>
#include <algorithm>
#include <iomanip>

namespace t21_hardware
{
constexpr double G = 30.75;
constexpr double R = 0.072;
constexpr double L = 0.374;
constexpr double MIN_G_DEG = 180.0;
constexpr double MAX_G_DEG = 300.0;
constexpr double MIN_G_RAD = MIN_G_DEG * M_PI / 180.0;
constexpr double MAX_G_RAD = MAX_G_DEG * M_PI / 180.0;

static rclcpp::Clock throttle_clock{RCL_SYSTEM_TIME};

T21TrackedHardware::T21TrackedHardware()
  : state_prev_geom_{0.0},
    geom_latched_{false},
    left_pos_{0.0},
    right_pos_{0.0},
    last_time_{0,0,RCL_SYSTEM_TIME},
    last_receive_time_{0,0,RCL_SYSTEM_TIME},
    first_read_{true},
    is_connected_(false),
    last_good_deg_(180.0f),
    last_cmd_ts_{0,0,RCL_SYSTEM_TIME},
    last_stop_ts_{0,0,RCL_SYSTEM_TIME},
    last_feedback_deg_(180.0f),
    write_blocked_until_{0,0,RCL_SYSTEM_TIME} {}

hardware_interface::CallbackReturn
T21TrackedHardware::on_init(const hardware_interface::HardwareInfo &info)
{
  if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    return CallbackReturn::ERROR;

  if (info_.joints.size() != 3) {
    RCLCPP_FATAL(rclcpp::get_logger("T21TrackedHardware"),
                 "URDF должен содержать 3 сустава, найдено %zu", info_.joints.size());
    return CallbackReturn::ERROR;
  }

  const char *exp_cmd[3]   = {"velocity","velocity","position"};
  const char *exp_state[3] = {"velocity","velocity","position"};
  for (size_t i = 0; i < 3; ++i) {
    if (info_.joints[i].command_interfaces[0].name != exp_cmd[i] ||
        info_.joints[i].state_interfaces[0].name   != exp_state[i]) {
      RCLCPP_FATAL(rclcpp::get_logger("T21TrackedHardware"),
                   "Joint %s: ожидались cmd %s, state %s",
                   info_.joints[i].name.c_str(), exp_cmd[i], exp_state[i]);
      return CallbackReturn::ERROR;
    }
  }
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
T21TrackedHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> si;
  si.reserve(5);
  si.emplace_back("wheel_left_joint",  "velocity", &state_[0]);
  si.emplace_back("wheel_left_joint",  "position", &left_pos_);
  si.emplace_back("wheel_right_joint", "velocity", &state_[1]);
  si.emplace_back("wheel_right_joint", "position", &right_pos_);
  si.emplace_back("geom_joint",        "position", &state_[2]);
  return si;
}

std::vector<hardware_interface::CommandInterface>
T21TrackedHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> ci;
  ci.reserve(3);
  ci.emplace_back("wheel_left_joint",  "velocity", &cmd_[0]);
  ci.emplace_back("wheel_right_joint", "velocity", &cmd_[1]);
  ci.emplace_back("geom_joint",        "position", &cmd_[2]);
  return ci;
}

hardware_interface::CallbackReturn
T21TrackedHardware::on_activate(const rclcpp_lifecycle::State &)
{
  first_read_ = true;
  left_pos_ = right_pos_ = 0.0;
  state_prev_geom_ = 0.0;
  last_time_ = rclcpp::Clock().now();

  last_good_deg_ = 180.0f;
  last_cmd_ts_ = rclcpp::Clock{RCL_SYSTEM_TIME}.now();
  last_stop_ts_ = rclcpp::Clock{RCL_SYSTEM_TIME}.now();
  last_feedback_deg_ = 180.0f;
  write_blocked_until_ = rclcpp::Time(0,0,RCL_SYSTEM_TIME);
  is_connected_ = false;

  log_file_.open("t21_debug_log.txt", std::ios::out | std::ios::trunc);
  if (log_file_.is_open()) {
      log_file_ << "SEND,geo_mode,geo_cmd_deg,lin_cm,ang_deg,geo_deg\n";
      log_file_ << "RECV,geo_mode,omega_l,omega_r,geo_deg\n";
  }
  return CallbackReturn::SUCCESS;
}

void T21TrackedHardware::send_stop_packet()
{
  const float geom_deg = last_feedback_deg_; // держим последнее известное положение
  socket_.sendCommand(0.0f, 0.0f, geom_deg, false);
  if (log_file_.is_open()) {
    log_file_ << "SEND, PWM, "
              << geom_deg << ", "
              << 0.0 << ", "
              << 0.0 << "\n";
  }
}

hardware_interface::CallbackReturn
T21TrackedHardware::on_deactivate(const rclcpp_lifecycle::State &)
{
  send_stop_packet();
  if (log_file_.is_open()) log_file_.close();
  return CallbackReturn::SUCCESS;
}

/* ———————————— read ———————————— */
hardware_interface::return_type
T21TrackedHardware::read(const rclcpp::Time &, const rclcpp::Duration &period)
{
  bool   got         = false;
  double raw_linVel  = 0.0;
  double raw_angVel  = 0.0;
  double raw_geomDeg = 0.0;
  double omega_l     = 0.0;
  double omega_r     = 0.0;
  double geom_rad    = 0.0;

  tracked_platform_udp::Packet128 pkt;
  got = socket_.receiveState(pkt);

  if (!got) {
    if (is_connected_) {
      RCLCPP_WARN_THROTTLE(rclcpp::get_logger("T21TrackedHardware"), throttle_clock,
        1000, "UDP не пришло — положение не обновляется, сохраняем старое");
      is_connected_ = false;
    }
    if (log_file_.is_open()) {
      log_file_ << "RECV, ERROR, UDP_LOST\n";
    }
    return hardware_interface::return_type::OK;
  }

  raw_linVel  = pkt.linVel;
  raw_angVel  = pkt.angVel;
  raw_geomDeg = pkt.geomPos;

  // ——— Фильтр выпадения положения geom_joint = 0 ———
  if (std::abs(raw_geomDeg) < 1.0 && std::abs(last_feedback_deg_) > 100.0) {
    RCLCPP_WARN_THROTTLE(rclcpp::get_logger("T21TrackedHardware"), throttle_clock,
      1000, "Dropout geom_joint: geomPos=%.2f° заменено на предыдущее %.2f°", raw_geomDeg, last_feedback_deg_);
    raw_geomDeg = last_feedback_deg_; // пропуск аномалии
  }

  // Clamp RX geom
  if (raw_geomDeg < MIN_G_DEG || raw_geomDeg > MAX_G_DEG) {
    RCLCPP_WARN_THROTTLE(rclcpp::get_logger("T21TrackedHardware"), throttle_clock,
      1000, "Получена geomPos=%.2f° вне диапазона [%.0f;%.0f] — ограничено", raw_geomDeg, MIN_G_DEG, MAX_G_DEG);
    //raw_geomDeg = std::clamp(raw_geomDeg, MIN_G_DEG, MAX_G_DEG);
  }

  is_connected_ = true;
  last_feedback_deg_ = raw_geomDeg; // всегда сохраняем последнее валидное

  const double rpm2rad = 2.0 * M_PI / 60.0;
  omega_l  = raw_linVel  * rpm2rad / G;
  omega_r  = raw_angVel  * rpm2rad / G;
  geom_rad = raw_geomDeg * M_PI / 180.0;

  state_[1] = omega_l;//изменение индексов для левой и правой частей для исправления инвертированного поворота
  state_[0] = omega_r;
  state_[2] = geom_rad;

  // Интеграция позиций
  const double dt = std::min(period.seconds(), 0.1);
  left_pos_  += state_[0] * dt;
  right_pos_ += state_[1] * dt;

  if (log_file_.is_open()) {
    log_file_ << "RECV, n/a, "
              << omega_l << ", "
              << omega_r << ", "
              << raw_geomDeg << "\n";
  }
  return hardware_interface::return_type::OK;
}

/* ———————————— write ———————————— */
hardware_interface::return_type
T21TrackedHardware::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  constexpr double TOL_DEG   = 15.0;
  constexpr double MAX_JUMP  = 90.0;
  constexpr double OBSERVE_S = 0.25;

  const rclcpp::Time now = rclcpp::Clock{RCL_SYSTEM_TIME}.now();

  // ——— Защита: блокировка write после STOP ———
  if (write_blocked_until_ > now) {
    if (log_file_.is_open()) {
      log_file_ << "SEND, BLOCKED, STOP_SAFETY\n";
    }
    return hardware_interface::return_type::OK;
  }

  const double omega_l = cmd_[0];
  const double omega_r = cmd_[1];

  double geom_cmd_rad = std::clamp(cmd_[2], MIN_G_RAD, MAX_G_RAD);
  if (geom_cmd_rad != cmd_[2]) {
    RCLCPP_WARN_THROTTLE(rclcpp::get_logger("T21TrackedHardware"), throttle_clock,
      1000, "geom_joint %.1f° ограничено [%.0f;%.0f]", cmd_[2]*180.0/M_PI, MIN_G_DEG, MAX_G_DEG);
  }
  const float geom_deg = static_cast<float>(geom_cmd_rad * 180.0 / M_PI);

  auto short_diff = [](double a){ return std::remainder(a, 360.0); };

  const float  cur_deg  = static_cast<float>(state_[2] * 180.0 / M_PI);
  const double err_deg  = short_diff(geom_deg - cur_deg);
  const double abs_err  = std::fabs(err_deg);

  const float  prev_deg = static_cast<float>(state_prev_geom_ * 180.0 / M_PI);
  const double err_prev = short_diff(geom_deg - prev_deg);
  const bool   diverging = std::fabs(err_prev) < abs_err;

  const bool big_jump = abs_err > MAX_JUMP;
  const bool runaway  = abs_err > TOL_DEG && diverging && (now - last_cmd_ts_).seconds() > OBSERVE_S;

  // ——— STOP и блокировка на 1с ———
  if ((big_jump || runaway) && (now - last_stop_ts_).seconds() > 0.10) {
    socket_.sendCommand(0.0f, 0.0f, cur_deg, false);
    socket_.sendCommand(0.0f, 0.0f, last_good_deg_, true);

    RCLCPP_ERROR(rclcpp::get_logger("T21TrackedHardware"),
      "STOP-safety: err %.1f° (%s) → возврат к %.1f°, блокировка write на 1с",
      abs_err, big_jump ? "jump" : "diverge", last_good_deg_);

    last_stop_ts_ = now;
    write_blocked_until_ = now + rclcpp::Duration::from_seconds(1.0);

    if (log_file_.is_open()) {
      log_file_ << "SEND, STOP, "
                << cur_deg << ", "
                << "0.0, 0.0\n";
    }
    return hardware_interface::return_type::OK;
  }

  // ——— Обычная отправка ——— //должно ли тут быть ntohs?
  const float lin_cm  = static_cast<float>(0.5 * R * (omega_r + omega_l) * 100.0);
  const float ang_deg = static_cast<float>((R/L) * (omega_r - omega_l) * 180.0 / M_PI);

  socket_.sendCommand(lin_cm, ang_deg, geom_deg, true);

  if (log_file_.is_open()) {
    log_file_ << "SEND, POS, "
              << std::fixed << std::setprecision(2)
              << geom_deg << ", "
              << lin_cm << ", "
              << ang_deg << "\n";
  }
  last_good_deg_ = geom_deg;
  last_cmd_ts_   = now;
  state_prev_geom_ = state_[2];

  return hardware_interface::return_type::OK;
}

} // namespace t21_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(t21_hardware::T21TrackedHardware,
                       hardware_interface::SystemInterface)
