/**
 * roboclaw_service_node — ROS2 service interface for RoboClaw diagnostics and config.
 *
 * Provides 5 services for operations outside the normal ros2_control write() loop:
 *   ~/stop_motors       — send zero duty to both motors
 *   ~/reset_encoders    — zero encoder counters
 *   ~/get_motor_status  — read voltage, current, temperature, error flags
 *   ~/set_pid_gains     — set velocity PID for M1 or M2
 *   ~/clear_errors      — re-read error state (clears latched warnings)
 *
 * IMPORTANT: This node opens its own TCP connection to the RoboClaw.
 * Do not call services while the ros2_control hardware interface is actively
 * running at full rate — commands will interleave on the serial bus.
 * Intended use: startup configuration, diagnostics, and shutdown operations.
 */

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "roboclaw_hardware/roboclaw_tcp.hpp"
#include "roboclaw_hardware/roboclaw_protocol.hpp"

#include "roboclaw_hardware/srv/stop_motors.hpp"
#include "roboclaw_hardware/srv/reset_encoders.hpp"
#include "roboclaw_hardware/srv/get_motor_status.hpp"
#include "roboclaw_hardware/srv/set_pid_gains.hpp"
#include "roboclaw_hardware/srv/clear_errors.hpp"

namespace roboclaw_hardware
{

using StopMotors    = roboclaw_hardware::srv::StopMotors;
using ResetEncoders = roboclaw_hardware::srv::ResetEncoders;
using GetMotorStatus= roboclaw_hardware::srv::GetMotorStatus;
using SetPIDGains   = roboclaw_hardware::srv::SetPIDGains;
using ClearErrors   = roboclaw_hardware::srv::ClearErrors;

class RoboClawServiceNode : public rclcpp::Node
{
public:
  RoboClawServiceNode()
  : Node("roboclaw_service_node")
  {
    this->declare_parameter("tcp_host", "192.168.68.60");
    this->declare_parameter("tcp_port", 8234);
    this->declare_parameter("address",  0x80);

    tcp_host_ = this->get_parameter("tcp_host").as_string();
    tcp_port_ = static_cast<int>(this->get_parameter("tcp_port").as_int());
    address_  = static_cast<uint8_t>(this->get_parameter("address").as_int());

    tcp_  = std::make_unique<RoboClawTcp>();
    proto_= std::make_unique<RoboClawProtocol>(*tcp_);

    if (!tcp_->connect(tcp_host_, tcp_port_)) {
      RCLCPP_ERROR(get_logger(), "Cannot connect to RoboClaw at %s:%d",
                   tcp_host_.c_str(), tcp_port_);
    } else {
      RCLCPP_INFO(get_logger(), "Connected to RoboClaw at %s:%d (address 0x%02X)",
                  tcp_host_.c_str(), tcp_port_, address_);
    }

    stop_srv_ = create_service<StopMotors>("~/stop_motors",
      std::bind(&RoboClawServiceNode::stop_motors_cb, this,
                std::placeholders::_1, std::placeholders::_2));

    reset_enc_srv_ = create_service<ResetEncoders>("~/reset_encoders",
      std::bind(&RoboClawServiceNode::reset_encoders_cb, this,
                std::placeholders::_1, std::placeholders::_2));

    get_status_srv_ = create_service<GetMotorStatus>("~/get_motor_status",
      std::bind(&RoboClawServiceNode::get_motor_status_cb, this,
                std::placeholders::_1, std::placeholders::_2));

    set_pid_srv_ = create_service<SetPIDGains>("~/set_pid_gains",
      std::bind(&RoboClawServiceNode::set_pid_gains_cb, this,
                std::placeholders::_1, std::placeholders::_2));

    clear_err_srv_ = create_service<ClearErrors>("~/clear_errors",
      std::bind(&RoboClawServiceNode::clear_errors_cb, this,
                std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(get_logger(), "RoboClaw service node ready.");
  }

private:
  bool ensure_connected()
  {
    if (tcp_->is_connected()) { return true; }
    RCLCPP_WARN(get_logger(), "TCP disconnected — reconnecting…");
    return tcp_->reconnect();
  }

  void stop_motors_cb(
    std::shared_ptr<StopMotors::Request> /*req*/,
    std::shared_ptr<StopMotors::Response> res)
  {
    if (!ensure_connected()) { res->success = false; res->message = "TCP connection failed"; return; }
    res->success = proto_->DutyM1M2(address_, 0, 0);
    res->message = res->success ? "Motors stopped" : "DutyM1M2(0,0) failed";
    RCLCPP_INFO(get_logger(), "stop_motors: %s", res->message.c_str());
  }

  void reset_encoders_cb(
    std::shared_ptr<ResetEncoders::Request> /*req*/,
    std::shared_ptr<ResetEncoders::Response> res)
  {
    if (!ensure_connected()) { res->success = false; res->message = "TCP connection failed"; return; }
    res->success = proto_->ResetEncoders(address_);
    res->message = res->success ? "Encoders reset to zero" : "ResetEncoders failed";
    RCLCPP_INFO(get_logger(), "reset_encoders: %s", res->message.c_str());
  }

  void get_motor_status_cb(
    std::shared_ptr<GetMotorStatus::Request> /*req*/,
    std::shared_ptr<GetMotorStatus::Response> res)
  {
    if (!ensure_connected()) { res->success = false; res->message = "TCP connection failed"; return; }

    auto volts    = proto_->GetVolts(address_);
    auto currents = proto_->ReadCurrents(address_);
    auto temps    = proto_->GetTemps(address_);
    auto errors   = proto_->ReadError(address_);

    if (!volts.ok || !currents.ok || !temps.ok || !errors.ok) {
      res->success = false;
      res->message = "One or more reads failed";
      return;
    }

    // RoboClaw returns values scaled: volts×10, amps×100, °C×10
    res->main_battery_v  = volts.main_bat  / 10.0f;
    res->logic_battery_v = volts.logic_bat / 10.0f;
    res->current_left_a  = currents.current1 / 100.0f;
    res->current_right_a = currents.current2 / 100.0f;
    res->temperature_c   = temps.temp1 / 10.0f;
    res->error_status    = errors.error;
    res->success         = true;
    res->message         = "OK";
  }

  void set_pid_gains_cb(
    std::shared_ptr<SetPIDGains::Request> req,
    std::shared_ptr<SetPIDGains::Response> res)
  {
    if (!ensure_connected()) { res->success = false; res->message = "TCP connection failed"; return; }

    bool ok = false;
    if (req->motor == 0) {
      ok = proto_->SetM1PID(address_, req->kp, req->ki, req->kd, req->qpps);
    } else if (req->motor == 1) {
      ok = proto_->SetM2PID(address_, req->kp, req->ki, req->kd, req->qpps);
    } else {
      res->success = false;
      res->message = "Invalid motor id (0=left/M1, 1=right/M2)";
      return;
    }

    res->success = ok;
    res->message = ok
      ? "PID gains set for motor " + std::to_string(req->motor)
      : "SetPID failed for motor " + std::to_string(req->motor);
    RCLCPP_INFO(get_logger(), "set_pid_gains motor%d: %s", req->motor, res->message.c_str());
  }

  void clear_errors_cb(
    std::shared_ptr<ClearErrors::Request> /*req*/,
    std::shared_ptr<ClearErrors::Response> res)
  {
    if (!ensure_connected()) { res->success = false; res->message = "TCP connection failed"; return; }

    auto err = proto_->ReadError(address_);
    if (!err.ok) { res->success = false; res->message = "ReadError failed"; return; }

    res->success = true;
    if (err.error == 0) {
      res->message = "No errors present";
    } else {
      char buf[16];
      snprintf(buf, sizeof(buf), "0x%08X", err.error);
      res->message = std::string("Error flags read (") + buf +
                     ") — hardware errors re-assert if condition persists";
    }
    RCLCPP_INFO(get_logger(), "clear_errors: %s", res->message.c_str());
  }

  std::string tcp_host_;
  int         tcp_port_;
  uint8_t     address_;

  std::unique_ptr<RoboClawTcp>      tcp_;
  std::unique_ptr<RoboClawProtocol> proto_;

  rclcpp::Service<StopMotors>::SharedPtr     stop_srv_;
  rclcpp::Service<ResetEncoders>::SharedPtr  reset_enc_srv_;
  rclcpp::Service<GetMotorStatus>::SharedPtr get_status_srv_;
  rclcpp::Service<SetPIDGains>::SharedPtr    set_pid_srv_;
  rclcpp::Service<ClearErrors>::SharedPtr    clear_err_srv_;
};

}  // namespace roboclaw_hardware

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<roboclaw_hardware::RoboClawServiceNode>());
  rclcpp::shutdown();
  return 0;
}
