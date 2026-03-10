#include "roboclaw_hardware/roboclaw_hardware.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <cmath>
#include <string>

namespace roboclaw_hardware
{

// ===========================================================================
// Parameter helpers
// ===========================================================================

std::string RoboClawHardware::get_param(
  const hardware_interface::HardwareInfo & info,
  const std::string & name,
  const std::string & default_val) const
{
  auto it = info.hardware_parameters.find(name);
  if (it != info.hardware_parameters.end()) {
    return it->second;
  }
  return default_val;
}

void RoboClawHardware::extract_connection_parameters(
  const hardware_interface::HardwareInfo & info)
{
  tcp_host_ = get_param(info, "tcp_host", tcp_host_);
  tcp_port_ = std::stoi(get_param(info, "tcp_port", std::to_string(tcp_port_)));
  socket_timeout_ = std::stod(
    get_param(info, "socket_timeout", std::to_string(socket_timeout_)));

  auto addr_str = get_param(info, "address", "0x80");
  if (addr_str.rfind("0x", 0) == 0 || addr_str.rfind("0X", 0) == 0) {
    address_ = static_cast<uint8_t>(std::stoul(addr_str, nullptr, 16));
  } else {
    address_ = static_cast<uint8_t>(std::stoul(addr_str));
  }
}

void RoboClawHardware::extract_physical_parameters(
  const hardware_interface::HardwareInfo & info)
{
  wheel_radius_ = std::stod(
    get_param(info, "wheel_radius", std::to_string(wheel_radius_)));
  wheel_separation_ = std::stod(
    get_param(info, "wheel_separation", std::to_string(wheel_separation_)));
  encoder_cpr_ = std::stoi(
    get_param(info, "encoder_counts_per_rev", std::to_string(encoder_cpr_)));
  gear_ratio_ = std::stod(
    get_param(info, "gear_ratio", std::to_string(gear_ratio_)));
}

void RoboClawHardware::extract_motion_parameters(
  const hardware_interface::HardwareInfo & info)
{
  auto strategy_str = get_param(info, "motion_strategy", "speed_accel");
  if (strategy_str == "duty") {
    motion_strategy_ = MotionStrategy::DUTY;
  } else if (strategy_str == "duty_accel") {
    motion_strategy_ = MotionStrategy::DUTY_ACCEL;
  } else if (strategy_str == "speed") {
    motion_strategy_ = MotionStrategy::SPEED;
  } else {
    motion_strategy_ = MotionStrategy::SPEED_ACCEL;
  }

  buffer_depth_ = std::stoi(
    get_param(info, "buffer_depth", std::to_string(buffer_depth_)));
  default_accel_ = static_cast<uint32_t>(std::stoul(
    get_param(info, "default_acceleration", std::to_string(default_accel_))));

  enc_stuck_limit_ = static_cast<uint32_t>(std::stoul(
    get_param(info, "encoder_stuck_limit", std::to_string(enc_stuck_limit_))));
  enc_runaway_limit_ = static_cast<uint32_t>(std::stoul(
    get_param(info, "encoder_runaway_limit", std::to_string(enc_runaway_limit_))));
  enc_comm_fail_limit_ = static_cast<uint32_t>(std::stoul(
    get_param(info, "encoder_comm_fail_limit", std::to_string(enc_comm_fail_limit_))));
  enc_max_speed_rad_s_ = std::stod(
    get_param(info, "encoder_max_speed_rad_s", std::to_string(enc_max_speed_rad_s_)));
}

void RoboClawHardware::extract_servo_parameters(
  const hardware_interface::HardwareInfo & info)
{
  auto home_str = get_param(info, "auto_home_on_startup", "false");
  auto limits_str = get_param(info, "position_limits_enabled", "false");
  auto to_bool = [](const std::string & s) {
    return s == "true" || s == "True" || s == "1";
  };
  auto_home_on_startup_ = to_bool(home_str);
  position_limits_enabled_ = to_bool(limits_str);

  min_pos_left_  = std::stod(get_param(info, "min_position_left",  "-1000000"));
  max_pos_left_  = std::stod(get_param(info, "max_position_left",   "1000000"));
  min_pos_right_ = std::stod(get_param(info, "min_position_right", "-1000000"));
  max_pos_right_ = std::stod(get_param(info, "max_position_right",  "1000000"));
}

bool RoboClawHardware::validate_parameters() const
{
  if (address_ < 0x80 || address_ > 0x87) {
    RCLCPP_ERROR(rclcpp::get_logger("RoboClawHardware"),
      "address must be 0x80-0x87, got 0x%02X", address_);
    return false;
  }
  if (wheel_radius_ <= 0.0) {
    RCLCPP_ERROR(rclcpp::get_logger("RoboClawHardware"),
      "wheel_radius must be positive");
    return false;
  }
  if (wheel_separation_ <= 0.0) {
    RCLCPP_ERROR(rclcpp::get_logger("RoboClawHardware"),
      "wheel_separation must be positive");
    return false;
  }
  if (encoder_cpr_ <= 0) {
    RCLCPP_ERROR(rclcpp::get_logger("RoboClawHardware"),
      "encoder_counts_per_rev must be positive");
    return false;
  }
  if (gear_ratio_ <= 0.0) {
    RCLCPP_ERROR(rclcpp::get_logger("RoboClawHardware"),
      "gear_ratio must be positive");
    return false;
  }
  return true;
}

// ===========================================================================
// Lifecycle
// ===========================================================================

hardware_interface::CallbackReturn RoboClawHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
#pragma GCC diagnostic pop

  auto logger = rclcpp::get_logger("RoboClawHardware");

  try {
    extract_connection_parameters(info);
    extract_physical_parameters(info);
    extract_motion_parameters(info);
    extract_servo_parameters(info);

    if (!validate_parameters()) {
      return hardware_interface::CallbackReturn::ERROR;
    }

    unit_converter_ = std::make_unique<UnitConverter>(
      wheel_radius_, encoder_cpr_, gear_ratio_);

    RCLCPP_INFO(logger,
      "Initialized: tcp=%s:%d addr=0x%02X | %s",
      tcp_host_.c_str(), tcp_port_, address_,
      unit_converter_->to_string().c_str());

  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger, "on_init failed: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RoboClawHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto logger = rclcpp::get_logger("RoboClawHardware");

  try {
    transport_ = std::make_unique<RoboClawTcp>();
    protocol_  = std::make_unique<RoboClawProtocol>(*transport_);

    if (!establish_connection()) {
      return hardware_interface::CallbackReturn::ERROR;
    }

    detect_controller_type();

    if (servo_capable_) {
      configure_servo_parameters();
    }
    if (auto_home_on_startup_ && controller_type_ == ControllerType::ROBOCLAW) {
      perform_auto_homing();
    }

    RCLCPP_INFO(logger, "Configured: controller=%s, servo=%s",
      (controller_type_ == ControllerType::ROBOCLAW ? "RoboClaw" :
       controller_type_ == ControllerType::MCP      ? "MCP" : "Unknown"),
      (servo_capable_ ? "yes" : "no"));

  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger, "on_configure failed: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RoboClawHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto logger = rclcpp::get_logger("RoboClawHardware");

  emergency_stop_active_ = false;
  initialize_state_interfaces();

  // Ensure motors are stopped before the control loop begins.
  // This prevents transient motion from stale RoboClaw state or
  // controller initialization artifacts.
  protocol_->DutyM1M2(address_, 0, 0);

  // Set the RoboClaw's own serial timeout (500 ms).  Note: this timeout
  // is reset by ANY command including reads (GetEncoders), so it only
  // protects against complete communication loss, not missing cmd_vel.
  protocol_->SetTimeout(address_, 500);

  cmd_vel_dirty_ = false;
  prev_cmd_vel_ = {0.0, 0.0};
  enc_health_ = {};
  diag_slot_ = 0;
  diag_cycle_counter_ = 0;

  RCLCPP_INFO(logger, "Activated -- motors stopped, control loop running");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RoboClawHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto logger = rclcpp::get_logger("RoboClawHardware");

  if (protocol_) {
    protocol_->DutyM1M2(address_, 0, 0);
  }

  RCLCPP_INFO(logger, "Deactivated -- motors stopped");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ===========================================================================
// Interface export
// ===========================================================================

std::vector<hardware_interface::StateInterface>
RoboClawHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_POSITION,
      &hw_state_pos_[i]);
    interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_VELOCITY,
      &hw_state_vel_[i]);
  }

  interfaces.emplace_back("diagnostics", "main_battery_v",   &gpio_main_battery_v_);
  interfaces.emplace_back("diagnostics", "temperature_c",    &gpio_temperature_c_);
  interfaces.emplace_back("diagnostics", "error_status",     &gpio_error_status_);
  interfaces.emplace_back("diagnostics", "current_left_a",   &gpio_current_left_a_);
  interfaces.emplace_back("diagnostics", "current_right_a",  &gpio_current_right_a_);
  interfaces.emplace_back("diagnostics", "encoder_health",   &gpio_encoder_health_);

  return interfaces;
}

std::vector<hardware_interface::CommandInterface>
RoboClawHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_VELOCITY,
      &hw_cmd_vel_[i]);
  }

  return interfaces;
}

// ===========================================================================
// Real-time loop
// ===========================================================================

hardware_interface::return_type RoboClawHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  if (!protocol_ || emergency_stop_active_) {
    return hardware_interface::return_type::OK;
  }

  // --- Encoder positions (1 TCP round-trip, ~5-10ms) ---
  auto enc = protocol_->GetEncoders(address_);

  if (!enc.ok) {
    if (++enc_health_.comm_fail_counter >= enc_comm_fail_limit_) {
      RCLCPP_ERROR(rclcpp::get_logger("RoboClawHardware"),
        "Encoder comm failure (%u consecutive) -- emergency stop",
        enc_health_.comm_fail_counter);
      gpio_encoder_health_ = 3.0;
      emergency_stop();
    }
    return hardware_interface::return_type::OK;
  }
  enc_health_.comm_fail_counter = 0;

  hw_state_pos_[0] = unit_converter_->counts_to_radians(enc.enc1);
  hw_state_pos_[1] = unit_converter_->counts_to_radians(enc.enc2);

  double dt = period.seconds();
  if (!first_read_ && dt > 0.0) {
    hw_state_vel_[0] = (hw_state_pos_[0] - prev_state_pos_[0]) / dt;
    hw_state_vel_[1] = (hw_state_pos_[1] - prev_state_pos_[1]) / dt;
  }

  check_encoder_health();

  prev_state_pos_[0] = hw_state_pos_[0];
  prev_state_pos_[1] = hw_state_pos_[1];
  first_read_ = false;

  // --- Rotating diagnostics: 1 TCP read every N-th cycle (1 of 4 slots) --
  // Most cycles: GetEncoders only (~8ms, fits 20ms budget easily).
  // Every 3rd cycle: GetEncoders + 1 diag (~15ms, still fits 20ms).
  // Full refresh of all 4 diagnostics: every 12 cycles = 240ms ≈ 4Hz.
  if (++diag_cycle_counter_ >= kDiagIntervalCycles) {
    diag_cycle_counter_ = 0;
    read_one_diagnostic();
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RoboClawHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!protocol_ || emergency_stop_active_) {
    return hardware_interface::return_type::OK;
  }

  constexpr double kEpsilon = 1e-6;
  bool changed = std::abs(hw_cmd_vel_[0] - prev_cmd_vel_[0]) > kEpsilon ||
                 std::abs(hw_cmd_vel_[1] - prev_cmd_vel_[1]) > kEpsilon;

  if (!changed && !cmd_vel_dirty_) {
    return hardware_interface::return_type::OK;
  }

  auto ret = execute_velocity_command(hw_cmd_vel_[0], hw_cmd_vel_[1]);
  prev_cmd_vel_[0] = hw_cmd_vel_[0];
  prev_cmd_vel_[1] = hw_cmd_vel_[1];
  cmd_vel_dirty_ = false;
  return ret;
}

// ===========================================================================
// Motion strategies
// ===========================================================================

hardware_interface::return_type RoboClawHardware::execute_velocity_command(
  double left_rad_s, double right_rad_s)
{
  try {
    constexpr double kStopThreshold = 1e-6;
    bool is_stop = std::abs(left_rad_s) < kStopThreshold &&
                   std::abs(right_rad_s) < kStopThreshold;

    // For zero velocity, always use DutyM1M2(0,0) — a direct PWM stop.
    // Speed/SpeedAccel modes rely on the RoboClaw's internal PID which
    // reads encoder feedback; noisy or unmounted encoders cause the
    // PID to fight endlessly, preventing the motor from actually stopping.
    if (is_stop) {
      bool ok = protocol_->DutyM1M2(address_, 0, 0);
      return ok ? hardware_interface::return_type::OK
                : hardware_interface::return_type::ERROR;
    }

    bool ok = false;

    switch (motion_strategy_) {
      case MotionStrategy::DUTY: {
        int16_t d1 = unit_converter_->rad_per_sec_to_duty(left_rad_s);
        int16_t d2 = unit_converter_->rad_per_sec_to_duty(right_rad_s);
        ok = protocol_->DutyM1M2(address_, d1, d2);
        break;
      }
      case MotionStrategy::DUTY_ACCEL: {
        int16_t d1 = unit_converter_->rad_per_sec_to_duty(left_rad_s);
        int16_t d2 = unit_converter_->rad_per_sec_to_duty(right_rad_s);
        ok = protocol_->DutyAccelM1M2(address_,
          d1, default_accel_, d2, default_accel_);
        break;
      }
      case MotionStrategy::SPEED: {
        int32_t s1 = unit_converter_->rad_per_sec_to_counts_per_sec(left_rad_s);
        int32_t s2 = unit_converter_->rad_per_sec_to_counts_per_sec(right_rad_s);
        ok = protocol_->SpeedM1M2(address_, s1, s2);
        break;
      }
      case MotionStrategy::SPEED_ACCEL: {
        int32_t s1 = unit_converter_->rad_per_sec_to_counts_per_sec(left_rad_s);
        int32_t s2 = unit_converter_->rad_per_sec_to_counts_per_sec(right_rad_s);
        ok = protocol_->SpeedAccelM1M2(address_, default_accel_, s1, s2);
        break;
      }
    }

    if (!ok) {
      RCLCPP_WARN_THROTTLE(rclcpp::get_logger("RoboClawHardware"),
        *rclcpp::Clock::make_shared(), 2000,
        "Motor command failed -- check connection");
    }

    return hardware_interface::return_type::OK;

  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("RoboClawHardware"),
      "execute_velocity_command exception: %s", e.what());
    return hardware_interface::return_type::ERROR;
  }
}

// ===========================================================================
// Rotating diagnostics (single-threaded, 1 TCP read per cycle)
// ===========================================================================

void RoboClawHardware::read_one_diagnostic()
{
  switch (diag_slot_) {
    case 0: {
      auto volts = protocol_->GetVolts(address_);
      if (volts.ok) {
        gpio_main_battery_v_ = static_cast<double>(volts.main_bat) / 10.0;
      }
      break;
    }
    case 1: {
      auto temps = protocol_->GetTemps(address_);
      if (temps.ok) {
        gpio_temperature_c_ = static_cast<double>(temps.temp1) / 10.0;
      }
      break;
    }
    case 2: {
      auto err = protocol_->ReadError(address_);
      if (err.ok) {
        gpio_error_status_ = static_cast<double>(err.error);
      }
      break;
    }
    case 3: {
      auto cur = protocol_->ReadCurrents(address_);
      if (cur.ok) {
        gpio_current_left_a_  = static_cast<double>(cur.current1) / 100.0;
        gpio_current_right_a_ = static_cast<double>(cur.current2) / 100.0;
      }
      break;
    }
  }

  diag_slot_ = (diag_slot_ + 1) % kDiagSlotCount;
}

// ===========================================================================
// Encoder health monitoring
// ===========================================================================

void RoboClawHardware::check_encoder_health()
{
  if (first_read_) {
    return;
  }

  for (int i = 0; i < 2; ++i) {
    bool commanding = std::abs(hw_cmd_vel_[i]) > 1e-4;
    bool moved = std::abs(hw_state_pos_[i] - prev_state_pos_[i]) > 1e-6;

    if (commanding && !moved) {
      if (++enc_health_.stuck_counter[i] >= enc_stuck_limit_) {
        RCLCPP_ERROR(rclcpp::get_logger("RoboClawHardware"),
          "Encoder %d stuck (%u cycles with command but no motion) -- emergency stop",
          i, enc_health_.stuck_counter[i]);
        gpio_encoder_health_ = 1.0;
        emergency_stop();
        return;
      }
    } else {
      enc_health_.stuck_counter[i] = 0;
    }

    if (std::abs(hw_state_vel_[i]) > enc_max_speed_rad_s_) {
      if (++enc_health_.runaway_counter[i] >= enc_runaway_limit_) {
        RCLCPP_ERROR(rclcpp::get_logger("RoboClawHardware"),
          "Encoder %d runaway (vel=%.1f rad/s > max=%.1f) -- emergency stop",
          i, hw_state_vel_[i], enc_max_speed_rad_s_);
        gpio_encoder_health_ = 2.0;
        emergency_stop();
        return;
      }
    } else {
      enc_health_.runaway_counter[i] = 0;
    }
  }
}

// ===========================================================================
// Safety
// ===========================================================================

bool RoboClawHardware::emergency_stop()
{
  if (!protocol_) {
    return false;
  }
  bool ok = protocol_->DutyM1M2(address_, 0, 0);
  if (ok) {
    emergency_stop_active_ = true;
    RCLCPP_WARN(rclcpp::get_logger("RoboClawHardware"),
      "EMERGENCY STOP activated");
  }
  return ok;
}

bool RoboClawHardware::clear_motion_buffers()
{
  if (!protocol_) {
    return false;
  }
  return protocol_->DutyM1M2(address_, 0, 0);
}

// ===========================================================================
// Connection helpers
// ===========================================================================

bool RoboClawHardware::establish_connection()
{
  auto logger = rclcpp::get_logger("RoboClawHardware");

  if (!transport_->connect(tcp_host_, tcp_port_, socket_timeout_)) {
    RCLCPP_ERROR(logger, "TCP connect failed: %s:%d",
      tcp_host_.c_str(), tcp_port_);
    return false;
  }

  auto ver = protocol_->ReadVersion(address_);
  if (!ver.ok) {
    RCLCPP_ERROR(logger, "ReadVersion failed -- is the RoboClaw powered?");
    transport_->close();
    return false;
  }

  RCLCPP_INFO(logger, "Connected: %s", ver.version.c_str());
  return true;
}

void RoboClawHardware::detect_controller_type()
{
  auto ver = protocol_->ReadVersion(address_);
  if (!ver.ok) {
    controller_type_ = ControllerType::UNKNOWN;
    servo_capable_ = false;
    return;
  }

  std::string lower = ver.version;
  std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);

  if (lower.find("roboclaw") != std::string::npos) {
    controller_type_ = ControllerType::ROBOCLAW;
    servo_capable_ = true;
  } else if (lower.find("mcp") != std::string::npos) {
    controller_type_ = ControllerType::MCP;
    servo_capable_ = true;
  } else {
    controller_type_ = ControllerType::UNKNOWN;
    servo_capable_ = false;
  }
}

void RoboClawHardware::initialize_state_interfaces()
{
  if (!protocol_) {
    return;
  }

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdangling-pointer"
  auto enc = protocol_->GetEncoders(address_);
  if (enc.ok) {
    hw_state_pos_[0] = unit_converter_->counts_to_radians(enc.enc1);
    hw_state_pos_[1] = unit_converter_->counts_to_radians(enc.enc2);
  }
#pragma GCC diagnostic pop

  hw_state_vel_[0] = 0.0;
  hw_state_vel_[1] = 0.0;
  hw_cmd_vel_[0]   = 0.0;
  hw_cmd_vel_[1]   = 0.0;
}

// ===========================================================================
// Diagnostics
// ===========================================================================

DiagnosticData RoboClawHardware::read_comprehensive_diagnostics()
{
  DiagnosticData diag;
  if (!protocol_) {
    return diag;
  }

  auto err = protocol_->ReadError(address_);
  if (err.ok) {
    diag.error_status = err.error;
  }

  auto cur = protocol_->ReadCurrents(address_);
  if (cur.ok) {
    diag.current_m1_a = static_cast<double>(cur.current1) / 100.0;
    diag.current_m2_a = static_cast<double>(cur.current2) / 100.0;
  }

  auto temps = protocol_->GetTemps(address_);
  if (temps.ok) {
    diag.temp1_c = static_cast<double>(temps.temp1) / 10.0;
    diag.temp2_c = static_cast<double>(temps.temp2) / 10.0;
  }

  auto volts = protocol_->GetVolts(address_);
  if (volts.ok) {
    diag.main_battery_v  = static_cast<double>(volts.main_bat)  / 10.0;
    diag.logic_battery_v = static_cast<double>(volts.logic_bat) / 10.0;
  }

  auto se = protocol_->GetSpeedErrors(address_);
  if (se.ok) {
    diag.speed_error_m1 = se.error1;
    diag.speed_error_m2 = se.error2;
  }

  auto pe = protocol_->GetPosErrors(address_);
  if (pe.ok) {
    diag.pos_error_m1 = pe.error1;
    diag.pos_error_m2 = pe.error2;
  }

  auto buf = protocol_->ReadBuffers(address_);
  if (buf.ok) {
    diag.buffer1 = buf.buffer1;
    diag.buffer2 = buf.buffer2;
  }

  diag.valid = true;
  return diag;
}

RoboClawHardware::HealthStatus RoboClawHardware::monitor_system_health()
{
  HealthStatus health;
  health.diagnostics = read_comprehensive_diagnostics();
  const auto & d = health.diagnostics;

  if (!d.valid) {
    health.level = HealthStatus::ERROR;
    health.messages.push_back("Diagnostics read failed");
    return health;
  }

  if (d.error_status != 0) {
    health.level = HealthStatus::ERROR;
    health.messages.push_back("Hardware error flags: 0x" +
      ([](uint32_t v) {
        char buf[12];
        std::snprintf(buf, sizeof(buf), "%08X", v);
        return std::string(buf);
      })(d.error_status));
  }

  if (d.main_battery_v < 10.0) {
    health.messages.push_back("Low main battery: " +
      std::to_string(d.main_battery_v) + "V");
    if (health.level < HealthStatus::WARNING) {
      health.level = HealthStatus::WARNING;
    }
  }

  if (d.temp1_c > 70.0 || d.temp2_c > 70.0) {
    health.messages.push_back("High temperature: " +
      std::to_string(std::max(d.temp1_c, d.temp2_c)) + " C");
    if (health.level < HealthStatus::WARNING) {
      health.level = HealthStatus::WARNING;
    }
  }

  constexpr double kCurrentWarn = 12.0;
  if (std::abs(d.current_m1_a) > kCurrentWarn ||
      std::abs(d.current_m2_a) > kCurrentWarn)
  {
    health.messages.push_back("High current: M1=" +
      std::to_string(d.current_m1_a) + "A, M2=" +
      std::to_string(d.current_m2_a) + "A");
    if (health.level < HealthStatus::WARNING) {
      health.level = HealthStatus::WARNING;
    }
  }

  return health;
}

bool RoboClawHardware::check_error_limits()
{
  if (!protocol_) {
    return false;
  }
  auto err = protocol_->ReadError(address_);
  if (err.ok && err.error != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("RoboClawHardware"),
      "Controller error flags: 0x%08X", err.error);
    return true;
  }
  return false;
}

BufferStatus RoboClawHardware::interpret_buffer_status(uint8_t raw)
{
  BufferStatus bs;
  if (raw == 0xFF) {
    bs.state = BufferStatus::IDLE;
    bs.commands_in_buffer = 0;
    bs.executing = false;
  } else if (raw == 0x00) {
    bs.state = BufferStatus::EXECUTING;
    bs.commands_in_buffer = 0;
    bs.executing = true;
  } else {
    bs.state = BufferStatus::BUFFERED;
    bs.commands_in_buffer = raw;
    bs.executing = false;
  }
  return bs;
}

BufferStatus RoboClawHardware::get_buffer_status()
{
  if (!protocol_) {
    return {};
  }
  auto buf = protocol_->ReadBuffers(address_);
  if (buf.ok) {
    return interpret_buffer_status(buf.buffer1);
  }
  return {};
}

// ===========================================================================
// Position / servo control
// ===========================================================================

bool RoboClawHardware::execute_absolute_position_command(
  double left_pos_rad, double right_pos_rad,
  double max_speed_rad_s, double accel_rad_s2, double decel_rad_s2,
  bool buffer_command)
{
  if (!protocol_ || !unit_converter_) {
    return false;
  }

  int32_t  pos1   = unit_converter_->radians_to_counts(left_pos_rad);
  int32_t  pos2   = unit_converter_->radians_to_counts(right_pos_rad);
  uint32_t speed  = static_cast<uint32_t>(std::abs(
    unit_converter_->rad_per_sec_to_counts_per_sec(max_speed_rad_s)));
  uint32_t accel  = static_cast<uint32_t>(std::abs(
    unit_converter_->rad_per_sec_to_counts_per_sec(accel_rad_s2)));
  uint32_t deccel = static_cast<uint32_t>(std::abs(
    unit_converter_->rad_per_sec_to_counts_per_sec(decel_rad_s2)));

  uint8_t buf_flag = buffer_command ? 1 : 0;

  return protocol_->SpeedAccelDeccelPosM1M2(
    address_,
    accel, speed, deccel, pos1,
    accel, speed, deccel, pos2,
    buf_flag);
}

bool RoboClawHardware::execute_distance_command(
  double left_dist_m, double right_dist_m,
  double speed_m_s, double accel_m_s2,
  bool buffer_command)
{
  if (!protocol_ || !unit_converter_) {
    return false;
  }

  uint32_t dist1 = static_cast<uint32_t>(std::abs(
    unit_converter_->meters_to_counts(left_dist_m)));
  uint32_t dist2 = static_cast<uint32_t>(std::abs(
    unit_converter_->meters_to_counts(right_dist_m)));

  double angular_speed = speed_m_s / wheel_radius_;
  int32_t speed_counts = unit_converter_->rad_per_sec_to_counts_per_sec(angular_speed);
  // Distance commands want positive speed; direction is implicit in the sign
  // relationship between current and target position.
  if (left_dist_m < 0) { speed_counts = -speed_counts; }

  double angular_accel = accel_m_s2 / wheel_radius_;
  uint32_t accel_counts = static_cast<uint32_t>(std::abs(
    unit_converter_->rad_per_sec_to_counts_per_sec(angular_accel)));

  uint8_t buf_flag = buffer_command ? 1 : 0;

  return protocol_->SpeedAccelDistM1M2(
    address_, accel_counts,
    speed_counts, dist1,
    speed_counts, dist2,
    buf_flag);
}

RoboClawHardware::ServoErrors RoboClawHardware::get_servo_errors()
{
  ServoErrors se;
  if (!protocol_ || !unit_converter_) {
    return se;
  }

  auto pe = protocol_->GetPosErrors(address_);
  if (pe.ok) {
    se.pos_error_left_rad  = unit_converter_->counts_to_radians(pe.error1);
    se.pos_error_right_rad = unit_converter_->counts_to_radians(pe.error2);
    constexpr double kPosThresholdRad = 0.6;
    se.position_ok = (std::abs(se.pos_error_left_rad)  < kPosThresholdRad &&
                      std::abs(se.pos_error_right_rad) < kPosThresholdRad);
  }

  auto spe = protocol_->GetSpeedErrors(address_);
  if (spe.ok) {
    se.spd_error_left_rad_s =
      unit_converter_->counts_per_sec_to_rad_per_sec(spe.error1);
    se.spd_error_right_rad_s =
      unit_converter_->counts_per_sec_to_rad_per_sec(spe.error2);
    constexpr double kSpdThresholdRadS = 3.0;
    se.speed_ok = (std::abs(se.spd_error_left_rad_s)  < kSpdThresholdRadS &&
                   std::abs(se.spd_error_right_rad_s) < kSpdThresholdRadS);
  }

  return se;
}

// ===========================================================================
// Stubs (future implementation)
// ===========================================================================

void RoboClawHardware::configure_servo_parameters()
{
  RCLCPP_INFO(rclcpp::get_logger("RoboClawHardware"),
    "configure_servo_parameters: stub -- PID and error limits not yet configured");
}

void RoboClawHardware::perform_auto_homing()
{
  RCLCPP_INFO(rclcpp::get_logger("RoboClawHardware"),
    "perform_auto_homing: stub -- homing sequence not yet implemented");
}

}  // namespace roboclaw_hardware

// ===========================================================================
// Plugin registration
// ===========================================================================

PLUGINLIB_EXPORT_CLASS(
  roboclaw_hardware::RoboClawHardware,
  hardware_interface::SystemInterface)
