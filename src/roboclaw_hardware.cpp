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
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

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

  // Set the RoboClaw's own serial timeout (500 ms).  If the controller
  // receives no command within this window it will stop the motors --
  // an additional safety layer beyond diff_drive_controller's cmd_vel_timeout.
  protocol_->SetTimeout(address_, 500);

  RCLCPP_INFO(logger, "Activated -- control loop running");
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
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!protocol_ || emergency_stop_active_) {
    return hardware_interface::return_type::OK;
  }

  // --- Encoder positions ---
  auto enc = protocol_->GetEncoders(address_);
  if (enc.ok) {
    hw_state_pos_[0] = unit_converter_->counts_to_radians(enc.enc1);
    hw_state_pos_[1] = unit_converter_->counts_to_radians(enc.enc2);
  }

  // --- Motor speeds ---
  auto spd = protocol_->GetSpeeds(address_);
  if (spd.ok) {
    hw_state_vel_[0] = unit_converter_->counts_per_sec_to_rad_per_sec(spd.speed1);
    hw_state_vel_[1] = unit_converter_->counts_per_sec_to_rad_per_sec(spd.speed2);
  }

  // --- Periodic diagnostics (every N cycles, not every cycle) ---
  if (++diag_read_counter_ >= diag_read_interval_) {
    diag_read_counter_ = 0;

    auto volts = protocol_->GetVolts(address_);
    if (volts.ok) {
      gpio_main_battery_v_ = static_cast<double>(volts.main_bat) / 10.0;
    }

    auto temps = protocol_->GetTemps(address_);
    if (temps.ok) {
      gpio_temperature_c_ = static_cast<double>(temps.temp1) / 10.0;
    }

    auto err = protocol_->ReadError(address_);
    if (err.ok) {
      gpio_error_status_ = static_cast<double>(err.error);
    }

    auto cur = protocol_->ReadCurrents(address_);
    if (cur.ok) {
      gpio_current_left_a_  = static_cast<double>(cur.current1) / 100.0;
      gpio_current_right_a_ = static_cast<double>(cur.current2) / 100.0;
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RoboClawHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!protocol_ || emergency_stop_active_) {
    return hardware_interface::return_type::OK;
  }

  return execute_velocity_command(hw_cmd_vel_[0], hw_cmd_vel_[1]);
}

// ===========================================================================
// Motion strategies
// ===========================================================================

hardware_interface::return_type RoboClawHardware::execute_velocity_command(
  double left_rad_s, double right_rad_s)
{
  try {
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

  auto enc = protocol_->GetEncoders(address_);
  if (enc.ok) {
    hw_state_pos_[0] = unit_converter_->counts_to_radians(enc.enc1);
    hw_state_pos_[1] = unit_converter_->counts_to_radians(enc.enc2);
  }

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
