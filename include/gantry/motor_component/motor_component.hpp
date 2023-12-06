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
#include <hippo_msgs/msg/float64_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include "gantry/motor/motor_interface.hpp"

namespace gantry {
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
    int increments_per_length_unit;
    int rpm_per_velocity_unit;
    int nominal_rpm;
  };
  struct PositionSetpoint {
    bool relative{false};  /// flag to indicate absolute and relative positions
    int position{0};       /// in motor increments
    bool updated{false};   /// to indicate that the setpoint has been updated
  };

  struct VelocitySetpoint {
    bool updated{false};  /// to indicate that the setpoint has been updated
    int velocity{0};      /// in motor rpm
  };

  void InitParams();
  bool CreateMotor();
  bool InitMotor();
  void InitPublishers();
  void InitSubscriptions();
  void InitTimers();
  void Run();
  void PublishPosition(int position, const rclcpp::Time &now);
  void PublishVelocity(int velocity, const rclcpp::Time &now);
  void PublishLimitSwitches(bool lower, bool upper, const rclcpp::Time &now);
  void PublishTransmissionErrors(int errors, const rclcpp::Time &now);
  void PublishMotorStatus(const MotorStatus &status, const rclcpp::Time &now);
  bool UpdateMotorData();
  std::unique_ptr<Motor> motor_;

  void OnAbsolutePositionSetpoint(
      const gantry_msgs::msg::MotorPosition::SharedPtr msg);
  void OnRelativePositionSetpoint(
      const gantry_msgs::msg::MotorPosition::SharedPtr msg);
  void OnVelocitySetpoint(const gantry_msgs::msg::MotorVelocity::SharedPtr msg);

  void SetPositionSetpoint(const gantry_msgs::msg::MotorPosition::SharedPtr msg,
                           bool relative);

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

  rclcpp::TimerBase::SharedPtr run_timer_;
  int transmission_errors_;

  Params params_;
  PositionSetpoint position_setpoint_;
  VelocitySetpoint velocity_setpoint_;
  bool is_emergency_stopped{false};
};
}  // namespace gantry
