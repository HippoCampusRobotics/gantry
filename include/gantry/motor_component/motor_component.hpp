// Copyright (C) 2023 Thies Lennart Alff
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301,
// USA

#pragma once
#include <gantry_msgs/msg/motor_limit_switches.hpp>
#include <gantry_msgs/msg/motor_position.hpp>
#include <gantry_msgs/msg/motor_status.hpp>
#include <gantry_msgs/msg/motor_velocity.hpp>
#include <gantry_msgs/srv/get_float_drive.hpp>
#include <gantry_msgs/srv/set_float_drive.hpp>
#include <gantry_msgs/srv/set_home_position.hpp>
#include <hippo_msgs/msg/float64_stamped.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "gantry/motor/motor_interface.hpp"

namespace gantry {
namespace mode {
enum Mode {
  kPosition = 0,
  kVelocity = 1,
};
}
class MotorNode : public rclcpp::Node {
 public:
  explicit MotorNode(const rclcpp::NodeOptions &options);

 private:
  struct Params {
    std::string type;
    std::string device;
    int baud;
    double timeout;
    int update_period_ms;
    int increments_per_rev;
    int increments_per_length_unit;
    int max_rpm;
    struct Defaults {
      int max_rpm;
      int max_accel;
      int max_decel;
    } defaults;
  };
  struct PositionSetpoint {
    bool relative{false};  /// flag to indicate absolute and relative positions
    int position{0};       /// in motor increments
    bool updated{false};   /// to indicate that the setpoint has been updated
  };

  struct VelocitySetpoint {
    bool updated{false};  /// to indicate that the setpoint has been updated
    int velocity{0};      /// in motor rpm
    rclcpp::Time time;
  };

  double RPM2Speed(int rpm) const {
    const double rpm_per_speed_unit =
        params_.increments_per_length_unit * 60.0 / params_.increments_per_rev;
    return rpm / rpm_per_speed_unit;
  }

  int Speed2RPM(double speed) const {
    const double rpm_per_speed_unit =
        params_.increments_per_length_unit * 60.0 / params_.increments_per_rev;
    return static_cast<int>(speed * rpm_per_speed_unit);
  }

  void InitParams();
  bool CreateMotor();
  bool InitMotor();
  void InitPublishers();
  void InitSubscriptions();
  void InitTimers();
  void InitServices();
  void Run();
  void PublishPosition(int position, const rclcpp::Time &now);
  void PublishVelocity(int velocity, const rclcpp::Time &now);
  void PublishLimitSwitches(bool lower, bool upper, const rclcpp::Time &now);
  void PublishTransmissionErrors(int errors, const rclcpp::Time &now);
  void PublishMotorStatus(const MotorStatus &status, const rclcpp::Time &now);
  bool UpdateMotorData();
  bool SetDefaults();
  std::unique_ptr<Motor> motor_;
  std::mutex mutex_;

  void OnAbsolutePositionSetpoint(
      const gantry_msgs::msg::MotorPosition::SharedPtr msg);
  void OnRelativePositionSetpoint(
      const gantry_msgs::msg::MotorPosition::SharedPtr msg);
  void OnVelocitySetpoint(const gantry_msgs::msg::MotorVelocity::SharedPtr msg);

  void SetPositionSetpoint(const gantry_msgs::msg::MotorPosition::SharedPtr msg,
                           bool relative);

  void ServeStartHoming(
      [[maybe_unused]] const std_srvs::srv::Trigger_Request::SharedPtr,
      std_srvs::srv::Trigger_Response::SharedPtr);

  void ServeSetHomePosition(
      const gantry_msgs::srv::SetHomePosition::Request::SharedPtr,
      gantry_msgs::srv::SetHomePosition::Response::SharedPtr);

  void ServeEnable(const std_srvs::srv::SetBool::Request::SharedPtr,
                   std_srvs::srv::SetBool::Response::SharedPtr);
  void ServeGetMaxAccel(
      const gantry_msgs::srv::GetFloatDrive::Request::SharedPtr,
      gantry_msgs::srv::GetFloatDrive::Response::SharedPtr);
  void ServeGetMaxDecel(
      const gantry_msgs::srv::GetFloatDrive::Request::SharedPtr,
      gantry_msgs::srv::GetFloatDrive::Response::SharedPtr);
  void ServeGetMaxSpeed(
      const gantry_msgs::srv::GetFloatDrive::Request::SharedPtr,
      gantry_msgs::srv::GetFloatDrive::Response::SharedPtr);
  void ServeGetPhysicalRPMLimit(
      const gantry_msgs::srv::GetFloatDrive::Request::SharedPtr,
      gantry_msgs::srv::GetFloatDrive::Response::SharedPtr);
  void ServeSetMaxAccel(
      const gantry_msgs::srv::SetFloatDrive::Request::SharedPtr,
      gantry_msgs::srv::SetFloatDrive::Response::SharedPtr);
  void ServeSetMaxDecel(
      const gantry_msgs::srv::SetFloatDrive::Request::SharedPtr,
      gantry_msgs::srv::SetFloatDrive::Response::SharedPtr);
  void ServeSetMaxSpeed(
      const gantry_msgs::srv::SetFloatDrive::Request::SharedPtr,
      gantry_msgs::srv::SetFloatDrive::Response::SharedPtr);

  bool MoveToPositionSetpoint();
  bool MoveWithVelocitySetpoint();

  rclcpp::Publisher<gantry_msgs::msg::MotorPosition>::SharedPtr position_pub_;
  rclcpp::Publisher<gantry_msgs::msg::MotorVelocity>::SharedPtr velocity_pub_;
  rclcpp::Publisher<gantry_msgs::msg::MotorLimitSwitches>::SharedPtr
      limit_switches_pub_;
  rclcpp::Publisher<hippo_msgs::msg::Float64Stamped>::SharedPtr
      transmission_errors_pub_;
  rclcpp::Publisher<gantry_msgs::msg::MotorStatus>::SharedPtr motor_status_pub_;

  rclcpp::Subscription<gantry_msgs::msg::MotorPosition>::SharedPtr
      absolute_position_sub_;
  rclcpp::Subscription<gantry_msgs::msg::MotorPosition>::SharedPtr
      relative_position_sub_;
  rclcpp::Subscription<gantry_msgs::msg::MotorVelocity>::SharedPtr
      velocity_sub_;

  struct Services {
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_homing;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable;
    rclcpp::Service<gantry_msgs::srv::SetHomePosition>::SharedPtr
        set_home_position;
    rclcpp::Service<gantry_msgs::srv::GetFloatDrive>::SharedPtr get_max_speed;
    rclcpp::Service<gantry_msgs::srv::SetFloatDrive>::SharedPtr set_max_speed;
    rclcpp::Service<gantry_msgs::srv::GetFloatDrive>::SharedPtr get_max_accel;
    rclcpp::Service<gantry_msgs::srv::SetFloatDrive>::SharedPtr set_max_accel;
    rclcpp::Service<gantry_msgs::srv::GetFloatDrive>::SharedPtr get_max_decel;
    rclcpp::Service<gantry_msgs::srv::SetFloatDrive>::SharedPtr set_max_decel;
    rclcpp::Service<gantry_msgs::srv::GetFloatDrive>::SharedPtr
        get_physical_rpm_limit;
  };
  Services services_;

  template <typename T>
  bool MotorOkayForService(std::shared_ptr<T> response) {
    if (!motor_) {
      response->success = false;
      response->message = "Motor object not created!";
      return false;
    }
    if (!motor_->initialized_) {
      response->success = true;
      response->message = "Motor not initialized.";
      return false;
    }
    return true;
  }

  rclcpp::TimerBase::SharedPtr run_timer_;
  int transmission_errors_;

  Params params_;
  PositionSetpoint position_setpoint_;
  VelocitySetpoint velocity_setpoint_;
  bool is_emergency_stopped{false};
  mode::Mode mode_{mode::kPosition};
};
}  // namespace gantry
