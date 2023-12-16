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
  InitSubscriptions();
  InitServices();
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

  topic = "~/motor_status";
  motor_status_pub_ =
      create_publisher<gantry_msgs::msg::MotorStatus>(topic, qos);
}

void MotorNode::InitSubscriptions() {
  rclcpp::QoS qos = rclcpp::SensorDataQoS();
  qos.keep_last(1);
  std::string topic;

  using namespace gantry_msgs::msg;
  topic = "~/setpoint/absolute_position";
  absolute_position_sub_ = create_subscription<MotorPosition>(
      topic, qos, [this](const MotorPosition::SharedPtr msg) {
        OnAbsolutePositionSetpoint(msg);
      });

  topic = "~/setpoint/relative_position";
  relative_position_sub_ = create_subscription<MotorPosition>(
      topic, qos, [this](const MotorPosition::SharedPtr msg) {
        OnRelativePositionSetpoint(msg);
      });

  topic = "~/setpoint/velocity";
  velocity_sub_ = create_subscription<MotorVelocity>(
      topic, qos,
      [this](const MotorVelocity::SharedPtr msg) { OnVelocitySetpoint(msg); });
}

void MotorNode::InitTimers() {
  run_timer_ = create_timer(std::chrono::milliseconds(params_.update_period_ms),
                            [this]() { Run(); });
}

void MotorNode::InitServices() {
  std::string name;

  name = "~/start_homing";
  start_homing_service_ = create_service<std_srvs::srv::Trigger>(
      name, [this](const std_srvs::srv::Trigger_Request::SharedPtr request,
                   std_srvs::srv::Trigger_Response::SharedPtr response) {
        ServeStartHoming(request, response);
      });

  name = "~/set_home_position";
  set_home_position_service_ =
      create_service<gantry_msgs::srv::SetHomePosition>(
          name,
          [this](
              const gantry_msgs::srv::SetHomePosition::Request::SharedPtr req,
              gantry_msgs::srv::SetHomePosition::Response::SharedPtr resp) {
            ServeSetHomePosition(req, resp);
          });

  name = "~/enable";
  enable_service_ = create_service<std_srvs::srv::SetBool>(
      name, [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
                   std_srvs::srv::SetBool::Response::SharedPtr resp) {
        ServeEnable(req, resp);
      });
}

void MotorNode::ServeStartHoming(
    const std_srvs::srv::Trigger_Request::SharedPtr,
    std_srvs::srv::Trigger_Response::SharedPtr _response) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!MotorOkayForService(_response)) {
    return;
  }
  _response->success = motor_->StartHoming();
  _response->message = "Homing sequence started.";
}

void MotorNode::ServeSetHomePosition(
    const gantry_msgs::srv::SetHomePosition::Request::SharedPtr _request,
    gantry_msgs::srv::SetHomePosition::Response::SharedPtr _response) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!MotorOkayForService(_response)) {
    return;
  }

  int increments;
  if (_request->position.increments != 0) {
    increments = _request->position.increments;
  } else {
    increments = static_cast<int>(_request->position.position *
                                  params_.increments_per_length_unit);
  }
  _response->success = motor_->SetHome(increments);
}

void MotorNode::ServeEnable(
    const std_srvs::srv::SetBool::Request::SharedPtr _request,
    std_srvs::srv::SetBool::Response::SharedPtr _response) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!MotorOkayForService(_response)) {
    return;
  }
  if (_request->data) {
    _response->success = motor_->Enable();
  } else {
    _response->success = motor_->Disable();
  }
}

void MotorNode::SetPositionSetpoint(
    const gantry_msgs::msg::MotorPosition::SharedPtr _msg, bool relative) {
  position_setpoint_.updated = true;
  position_setpoint_.relative = relative;
  if (_msg->increments != 0) {
    position_setpoint_.position = _msg->increments;
  } else {
    position_setpoint_.position =
        _msg->position * params_.increments_per_length_unit;
  }
}

void MotorNode::OnAbsolutePositionSetpoint(
    const gantry_msgs::msg::MotorPosition::SharedPtr _msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  SetPositionSetpoint(_msg, false);
}

void MotorNode::OnRelativePositionSetpoint(
    const gantry_msgs::msg::MotorPosition::SharedPtr _msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  SetPositionSetpoint(_msg, true);
}

void MotorNode::OnVelocitySetpoint(
    const gantry_msgs::msg::MotorVelocity::SharedPtr _msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  velocity_setpoint_.updated = true;
  if (_msg->rpm != 0) {
    velocity_setpoint_.velocity = _msg->rpm;
  } else {
    velocity_setpoint_.velocity =
        _msg->velocity * params_.rpm_per_velocity_unit;
  }
}

void MotorNode::Run() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!UpdateMotorData()) {
    ++transmission_errors_;
  }
  if (is_emergency_stopped) {
    motor_->SetVelocity(0);
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "Emergency stop active!");
    return;
  }
  if (position_setpoint_.updated) {
    position_setpoint_.updated = false;
    transmission_errors_ += static_cast<int>(!MoveToPositionSetpoint());
  }
  if (velocity_setpoint_.updated) {
    velocity_setpoint_.updated = false;
    transmission_errors_ += static_cast<int>(!MoveWithVelocitySetpoint());
  }
  PublishTransmissionErrors(transmission_errors_, now());
}

bool MotorNode::MoveToPositionSetpoint() {
  bool success;
  if (position_setpoint_.relative) {
    success = motor_->SetRelativePositionTarget(position_setpoint_.position);
  } else {
    success = motor_->SetAbsolutePositionTarget(position_setpoint_.position);
  }
  if (!success) {
    RCLCPP_ERROR(get_logger(), "Failed to set position setpoint");
    return false;
  }
  if (!motor_->MoveToCurrentTargetPosition()) {
    RCLCPP_ERROR(get_logger(), "Failed to move to target position.");
    return false;
  }
  return true;
}

bool MotorNode::MoveWithVelocitySetpoint() {
  bool success;
  success = motor_->SetVelocity(velocity_setpoint_.velocity);
  if (!success) {
    RCLCPP_ERROR(get_logger(), "Failed to set velocity setpoint");
  }
  return success;
}

bool MotorNode::UpdateMotorData() {
  if (!motor_) {
    RCLCPP_FATAL(get_logger(), "Motor has not been created! Cannot run.");
    return false;
  }

  auto position = motor_->GetPosition();
  if (!position) {
    RCLCPP_ERROR(get_logger(), "Could not read motor position.");
    return false;
  }
  PublishPosition(*position, now());

  auto velocity = motor_->GetVelocity();
  if (!velocity) {
    RCLCPP_ERROR(get_logger(), "Could not read motor velocity");
    return false;
  }
  PublishVelocity(*velocity, now());

  auto status = motor_->UpdateStatus();
  if (!status) {
    RCLCPP_ERROR(get_logger(), "Could not update motor status.");
    return false;
  }
  PublishMotorStatus(*status, now());
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

void MotorNode::PublishMotorStatus(const MotorStatus &_status,
                                   const rclcpp::Time &_now) {
  if (!motor_status_pub_) {
    RCLCPP_ERROR(get_logger(),
                 "Motor status publisher not initialized. Cannot publish.");
    return;
  }
  gantry_msgs::msg::MotorStatus msg;
  msg.header.stamp = _now;
  msg.enabled = _status.enabled;
  msg.homing = _status.homing;
  msg.position_reached = _status.position_reached;
  msg.lower_limit_switch_pressed = _status.limit_switches.lower_pressed;
  msg.upper_limit_switch_pressed = _status.limit_switches.upper_pressed;
  motor_status_pub_->publish(msg);
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
