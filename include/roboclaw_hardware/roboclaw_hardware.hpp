#ifndef ROBOCLAW_HARDWARE__ROBOCLAW_HARDWARE_HPP_
#define ROBOCLAW_HARDWARE__ROBOCLAW_HARDWARE_HPP_

#include "roboclaw_hardware/roboclaw_protocol.hpp"
#include "roboclaw_hardware/roboclaw_tcp.hpp"
#include "roboclaw_hardware/unit_converter.hpp"
#include "roboclaw_hardware/visibility_control.h"

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace roboclaw_hardware
{

/// Motion control strategies -- determines which RoboClaw command is used
/// in the write() hot path.
enum class MotionStrategy
{
  DUTY,         ///< Open-loop PWM (DutyM1M2)
  DUTY_ACCEL,   ///< Ramped PWM (DutyAccelM1M2)
  SPEED,        ///< PID speed (SpeedM1M2)
  SPEED_ACCEL   ///< PID speed with acceleration (SpeedAccelM1M2)
};

/// Controller type detected from the firmware version string.
enum class ControllerType { ROBOCLAW, MCP, UNKNOWN };

/// Aggregate for all diagnostic data read from the controller in a
/// single burst.  No heap allocation -- fixed-size fields only.
struct DiagnosticData
{
  bool     valid = false;

  // Error / status
  uint32_t error_status = 0;

  // Electrical
  double   main_battery_v = 0.0;
  double   logic_battery_v = 0.0;
  double   current_m1_a = 0.0;
  double   current_m2_a = 0.0;

  // Thermal
  double   temp1_c = 0.0;
  double   temp2_c = 0.0;

  // Servo errors
  int32_t  speed_error_m1 = 0;
  int32_t  speed_error_m2 = 0;
  int32_t  pos_error_m1 = 0;
  int32_t  pos_error_m2 = 0;

  // Buffer
  uint8_t  buffer1 = 0xFF;
  uint8_t  buffer2 = 0xFF;
};

/// Buffer status interpretation.
struct BufferStatus
{
  enum State { IDLE, EXECUTING, BUFFERED };
  State   state = IDLE;
  int     commands_in_buffer = 0;
  bool    executing = false;
};

// ===========================================================================
// RoboClawHardware  --  ros2_control SystemInterface plugin
// ===========================================================================

class ROBOCLAW_HARDWARE_PUBLIC RoboClawHardware
  : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RoboClawHardware)

  // ---- ros2_control lifecycle ---------------------------------------------

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;

  // ---- Real-time loop (called by controller_manager) ----------------------

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // ---- Public API beyond ros2_control -------------------------------------

  /// Immediate motor stop.  Sets duty to zero and flags emergency state.
  bool emergency_stop();

  /// Read all diagnostics in one burst (called periodically, not every cycle).
  DiagnosticData read_comprehensive_diagnostics();

  /// Aggregate health assessment.
  struct HealthStatus {
    enum Level { OK, WARNING, ERROR };
    Level  level = OK;
    std::vector<std::string> messages;
    DiagnosticData diagnostics;
  };
  HealthStatus monitor_system_health();

  /// Check if the controller's hardware error flag is set.
  bool check_error_limits();

  /// Interpret the raw buffer value.
  static BufferStatus interpret_buffer_status(uint8_t raw);

  /// Read buffer utilization.
  BufferStatus get_buffer_status();

  // ---- Position / servo commands ------------------------------------------

  bool execute_absolute_position_command(
    double left_pos_rad, double right_pos_rad,
    double max_speed_rad_s, double accel_rad_s2, double decel_rad_s2,
    bool buffer_command = false);

  bool execute_distance_command(
    double left_dist_m, double right_dist_m,
    double speed_m_s, double accel_m_s2,
    bool buffer_command = false);

  struct ServoErrors {
    double pos_error_left_rad  = 0.0;
    double pos_error_right_rad = 0.0;
    double spd_error_left_rad_s  = 0.0;
    double spd_error_right_rad_s = 0.0;
    bool   position_ok = true;
    bool   speed_ok = true;
  };
  ServoErrors get_servo_errors();

  /// Clear all motion buffers (sends DutyM1M2(0,0)).
  bool clear_motion_buffers();

  // ---- Stubs (future implementation) --------------------------------------

  void configure_servo_parameters();
  void perform_auto_homing();

private:
  // ---- Parameter extraction (from HardwareInfo) ---------------------------
  void extract_connection_parameters(const hardware_interface::HardwareInfo & info);
  void extract_physical_parameters(const hardware_interface::HardwareInfo & info);
  void extract_motion_parameters(const hardware_interface::HardwareInfo & info);
  void extract_servo_parameters(const hardware_interface::HardwareInfo & info);
  bool validate_parameters() const;

  std::string get_param(const hardware_interface::HardwareInfo & info,
                        const std::string & name,
                        const std::string & default_val) const;

  // ---- Internal helpers ---------------------------------------------------
  bool establish_connection();
  void detect_controller_type();
  void initialize_state_interfaces();
  hardware_interface::return_type execute_velocity_command(
    double left_rad_s, double right_rad_s);

  // ---- Connection ---------------------------------------------------------
  std::string    tcp_host_ = "192.168.68.60";
  int            tcp_port_ = 8234;
  double         socket_timeout_ = RoboClawTcp::kDefaultTimeoutSec;
  uint8_t        address_ = 0x80;

  std::unique_ptr<RoboClawTcp>      transport_;
  std::unique_ptr<RoboClawProtocol> protocol_;

  // ---- Physical parameters ------------------------------------------------
  double wheel_radius_         = 0.2;
  double wheel_separation_     = 0.3;
  int    encoder_cpr_          = 1000;
  double gear_ratio_           = 16.0;

  std::unique_ptr<UnitConverter> unit_converter_;

  // ---- Motion control -----------------------------------------------------
  MotionStrategy motion_strategy_ = MotionStrategy::SPEED_ACCEL;
  int            buffer_depth_    = 4;
  uint32_t       default_accel_   = 1000;

  // ---- Servo parameters ---------------------------------------------------
  bool   auto_home_on_startup_     = false;
  bool   position_limits_enabled_  = false;
  double min_pos_left_  = -1000000.0;
  double max_pos_left_  =  1000000.0;
  double min_pos_right_ = -1000000.0;
  double max_pos_right_ =  1000000.0;

  // ---- Controller state ---------------------------------------------------
  ControllerType controller_type_ = ControllerType::UNKNOWN;
  bool servo_capable_             = false;
  bool emergency_stop_active_     = false;

  // ---- ros2_control state & command arrays --------------------------------
  // Indices: 0 = left, 1 = right
  std::array<double, 2> hw_cmd_vel_  = {0.0, 0.0};
  std::array<double, 2> hw_state_pos_ = {0.0, 0.0};
  std::array<double, 2> hw_state_vel_ = {0.0, 0.0};

  // ---- Write-on-change: skip TCP write when command hasn't changed --------
  std::array<double, 2> prev_cmd_vel_ = {0.0, 0.0};
  bool   cmd_vel_dirty_ = false;

  // ---- Diagnostics GPIO state interfaces ----------------------------------
  double gpio_main_battery_v_  = 0.0;
  double gpio_temperature_c_   = 0.0;
  double gpio_error_status_    = 0.0;
  double gpio_current_left_a_  = 0.0;
  double gpio_current_right_a_ = 0.0;

  // ---- Velocity estimation from encoder deltas ----------------------------
  std::array<double, 2> prev_state_pos_ = {0.0, 0.0};
  bool   first_read_ = true;

  // ---- Diagnostics scheduling ---------------------------------------------
  int    diag_cycle_counter_ = 0;
  static constexpr int kDiagIntervalCycles = 25;  // 4 slots × 25 cycles = 100 cycles = 1Hz full refresh
  int    diag_slot_ = 0;                          // rotates 0..3 across slots
};

}  // namespace roboclaw_hardware

#endif  // ROBOCLAW_HARDWARE__ROBOCLAW_HARDWARE_HPP_
