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

#include "gantry/motor_component/motor_component.hpp"

#include "gantry/motor/mcbl2805.hpp"
#include "gantry/motor/mcbl3006.hpp"

namespace gantry {
MotorNode::MotorNode(const rclcpp::NodeOptions &options)
    : Node("motor", options) {
  InitParams();
  if (!CreateMotor()) {
    return;
  }
  if (!InitMotor()) {
    RCLCPP_FATAL(get_logger(), "Node will be inactive.");
    return;
  }
  InitPublishers();
  InitTimers();

  RCLCPP_INFO_STREAM(get_logger(), "Initialized.");
}

void MotorNode::InitPublishers() {
  rclcpp::QoS qos = rclcpp::SensorDataQoS();
  qos.keep_last(1);
  std::string topic;

  using namespace gantry_msgs::msg;

  topic = "~/position";
  position_pub_ = create_publisher<MotorPosition>(topic, qos);

  topic = "~/velocity";
  velocity_pub_ = create_publisher<MotorVelocity>(topic, qos);

  topic = "~/limit_switches";
  limit_switches_pub_ = create_publisher<MotorLimitSwitches>(topic, qos);

  topic = "~/transmission_errors";
  transmission_errors_pub_ =
      create_publisher<hippo_msgs::msg::Float64Stamped>(topic, qos);
}

void MotorNode::InitTimers() {
  run_timer_ = create_timer(std::chrono::milliseconds(20), [this]() { Run(); });
}

void MotorNode::Run() {
  if (!UpdateMotorData()) {
    ++transmission_errors_;
  }
  PublishTransmissionErrors(transmission_errors_, now());
}

bool MotorNode::UpdateMotorData() {
  if (!motor_) {
    RCLCPP_FATAL(get_logger(), "Motor has not been created! Cannot run.");
    return false;
  }

  auto position = motor_->GetPosition();
  auto velocity = motor_->GetVelocity();

  if (!position) {
    RCLCPP_ERROR(get_logger(), "Could not read motor position.");
    return false;
  }
  PublishPosition(*position, now());

  if (!velocity) {
    RCLCPP_ERROR(get_logger(), "Could not read motor velocity");
    return false;
  }
  PublishVelocity(*velocity, now());

  return true;
}

void MotorNode::PublishPosition(int _position, const rclcpp::Time &_now) {
  if (!position_pub_) {
    RCLCPP_ERROR(get_logger(),
                 "Position publisher not initialized. Cannot publish.");
    return;
  }
  gantry_msgs::msg::MotorPosition msg;
  msg.header.stamp = _now;
  msg.increments = _position;
  msg.position =
      static_cast<double>(_position) / params_.increments_per_length_unit;
  position_pub_->publish(msg);
}

void MotorNode::PublishVelocity(int _velocity, const rclcpp::Time &_now) {
  if (!velocity_pub_) {
    RCLCPP_ERROR(get_logger(),
                 "Velocity publisher not initialized. Cannot publish.");
    return;
  }
  gantry_msgs::msg::MotorVelocity msg;
  msg.header.stamp = _now;
  msg.rpm = _velocity;
  msg.velocity = static_cast<double>(_velocity) / params_.rpm_per_velocity_unit;
  velocity_pub_->publish(msg);
}

void MotorNode::PublishLimitSwitches(bool lower, bool upper,
                                     const rclcpp::Time &_now) {
  if (!limit_switches_pub_) {
    RCLCPP_ERROR(get_logger(),
                 "Limit switches publisher not initialized. Cannot publish.");
    return;
  }
  gantry_msgs::msg::MotorLimitSwitches msg;
  msg.header.stamp = _now;
  msg.lower_limit_reached = lower;
  msg.upper_limit_reached = upper;
  limit_switches_pub_->publish(msg);
}

void MotorNode::PublishTransmissionErrors(int _errors,
                                          const rclcpp::Time &_now) {
  hippo_msgs::msg::Float64Stamped msg;
  msg.header.stamp = _now;
  msg.data = _errors;
  if (!transmission_errors_pub_) {
    RCLCPP_ERROR(
        get_logger(),
        "Transmission errors publisher not initialized. Cannot publish.");
    return;
  }
  transmission_errors_pub_->publish(msg);
}

bool MotorNode::InitMotor() {
  if (!motor_) {
    RCLCPP_FATAL(get_logger(),
                 "Motor has not been created! Cannot initialize.");
    return false;
  }
  motor_->InitSerial(params_.device);
  if (!motor_->initialized_) {
    RCLCPP_ERROR(get_logger(),
                 "Failed to initialize motor's serial connection at '%s'",
                 params_.device.c_str());
    return false;
  }
  auto position = motor_->GetPosition();
  if (!position) {
    RCLCPP_FATAL_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Could not read motor position. Are all wires connected "
        "properly? Is the motor powered (relay box)?");
    return false;
  }
  return true;
}

bool MotorNode::CreateMotor() {
  if (params_.type == "mcbl2805") {
    motor_ = std::make_unique<MCBL2805>();
  } else if (params_.type == "mcbl3006") {
    motor_ = std::make_unique<MCBL3006>();
  } else {
    RCLCPP_FATAL(get_logger(),
                 "Unhandled motor type: <%s>. This node will be inactive!",
                 params_.type.c_str());
    return false;
  }
  return true;
}
}  // namespace gantry
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(gantry::MotorNode)
