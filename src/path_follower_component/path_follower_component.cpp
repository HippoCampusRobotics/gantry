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

#include "gantry/path_follower_component/path_follower_component.hpp"

#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace gantry {

PathFollowerNode::PathFollowerNode(const rclcpp::NodeOptions &_options)
    : Node("path_follower", _options) {
  InitParams();
  InitPublishers();
  InitSubscriptions();
  InitTimers();
}

void PathFollowerNode::InitTimers() {
  for (std::size_t i = 0; i < kNumAxes; ++i) {
    positions_timed_out_.at(i) = false;
    position_timeouts_.at(i) = create_timer(
        std::chrono::milliseconds(200), [this, i]() { OnPositionTimeout(i); });
  }
}

void PathFollowerNode::InitPublishers() {
  std::string topic;
  rclcpp::QoS qos = rclcpp::SensorDataQoS().keep_last(1);

  for (std::size_t i = 0; i < axis_names_.size(); ++i) {
    std::string axis = axis_names_.at(i);
    topic = "motor_" + axis + "/setpoint/velocity";
    velocity_pubs_.at(i) =
        create_publisher<gantry_msgs::msg::MotorVelocity>(topic, qos);

    topic = "motor_" + axis + "/setpoint/absolute_position";
    position_pubs_.at(i) =
        create_publisher<gantry_msgs::msg::MotorPosition>(topic, qos);
  }
}

void PathFollowerNode::InitSubscriptions() {
  std::string topic;
  rclcpp::QoS qos = rclcpp::SensorDataQoS().keep_last(1);

  for (std::size_t i = 0; i < axis_names_.size(); ++i) {
    std::string axis = axis_names_.at(i);
    topic = "motor_" + axis + "/position";
    position_subs_.at(i) = create_subscription<gantry_msgs::msg::MotorPosition>(
        topic, qos,
        [this, &i](const gantry_msgs::msg::MotorPosition::SharedPtr msg) {
          OnMotorPosition(i, msg);
        });
  }
}
bool PathFollowerNode::IsMotorPositionTimedOut() {
  return std::any_of(positions_timed_out_.begin(), positions_timed_out_.end(),
                     [](bool i) { return i; });
}

std::string PathFollowerNode::GetWaypointsFilePath() {
  if (!params_.path_file.empty()) {
    return params_.path_file;
  }
  std::string file_path;
  const std::string pkg{"gantry"};
  try {
    file_path = ament_index_cpp::get_package_share_directory(pkg);
  } catch (const ament_index_cpp::PackageNotFoundError &) {
    RCLCPP_ERROR(get_logger(),
                 "Failed to load default waypoints because the package [%s] "
                 "could not be found.",
                 pkg.c_str());
    return "";
  }
  file_path += "/config/default_path.yaml";
  return file_path;
}

void PathFollowerNode::LoadDefaultWaypoints() {
  std::string file_path = GetWaypointsFilePath();
  path_ = std::make_shared<path_planning::Path>();
  try {
    path_->LoadFromYAML(file_path);
  } catch (const YAML::ParserException &) {
    RCLCPP_ERROR(get_logger(), "Failed to parse default waypoints at [%s]",
                 file_path.c_str());
    path_ = nullptr;
    return;
  } catch (const YAML::BadFile &) {
    RCLCPP_ERROR(get_logger(),
                 "Failed to load default waypoints at [%s]: bad file.",
                 file_path.c_str());
    path_ = nullptr;
    return;
  }
  RCLCPP_INFO(get_logger(), "Loaded default waypoints at [%s]",
              file_path.c_str());
  path_->SetLookAhead(params_.look_ahead_distance);
}

void PathFollowerNode::PublishVelocitySetpoint(
    const Eigen::Vector3d &_setpoint) {
  gantry_msgs::msg::MotorVelocity msg;
  msg.header.stamp = now();

  for (std::size_t i = 0; i < axis_names_.size(); ++i) {
    msg.velocity = _setpoint[i];
    if (!velocity_pubs_.at(i)) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
                            "Velocity pub for %s-axis not initialized.",
                            axis_names_.at(i).c_str());
      continue;
    }
    velocity_pubs_[i]->publish(msg);
  }
}

void PathFollowerNode::PublishPositionSetpoint(
    const Eigen::Vector3d &_setpoint) {
  gantry_msgs::msg::MotorPosition msg;
  msg.header.stamp = now();

  for (std::size_t i = 0; i < position_pubs_.size(); ++i) {
    msg.position = _setpoint(i);
    if (!position_pubs_.at(i)) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
                            "Position pub for %s-axis not initialized.",
                            axis_names_.at(i).c_str());
      continue;
    }
    position_pubs_[i]->publish(msg);
  }
}

void PathFollowerNode::ServeStart(
    const std_srvs::srv::Trigger_Request::SharedPtr,
    std_srvs::srv::Trigger_Response::SharedPtr _response) {
  std::string text;
  bool success;
  if (running_) {
    text = "Already running";
    success = false;
  } else {
    running_ = true;
    text = "Starting...";
    success = true;
  }
  _response->success = success;
  _response->message = text;
  RCLCPP_INFO_STREAM(get_logger(), text);
}

void PathFollowerNode::ServeStop(
    const std_srvs::srv::Trigger_Request::SharedPtr,
    std_srvs::srv::Trigger_Response::SharedPtr _response) {
  std::string text;
  bool success;
  if (running_) {
    running_ = false;
    success = true;
    text = "Stopping...";
  } else {
    text = "Already stopped.";
    success = false;
  }
  // publish zero velocity anyway. does not hurt even if already stopped.
  PublishVelocitySetpoint(Eigen::Vector3d::Zero());
  _response->success = success;
  _response->message = text;
  RCLCPP_INFO_STREAM(get_logger(), text);
}

void PathFollowerNode::ServeMoveToStart(
    const std_srvs::srv::Trigger_Request::SharedPtr,
    std_srvs::srv::Trigger_Response::SharedPtr _response) {
  std::string text;
  running_ = false;
  std::lock_guard<decltype(mutex_)> guard(mutex_);
  text = "Moving to start position";
  PublishPositionSetpoint(start_position_);
  _response->success = true;
  _response->message = text;
  RCLCPP_INFO_STREAM(get_logger(), text);
}

void PathFollowerNode::UpdateControl() {
  if (!path_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "No path has been set.");
    return;
  }
  if (IsMotorPositionTimedOut()) {
    PublishVelocitySetpoint(Eigen::Vector3d::Zero());
    return;
  }
  if (!running_) {
    PublishVelocitySetpoint(Eigen::Vector3d::Zero());
    return;
  }

  if (!path_->Update(position_)) {
    PublishVelocitySetpoint(Eigen::Vector3d::Zero());
    if (!path_->IsLoop()) {
      running_ = false;
    } else {
      RCLCPP_ERROR(get_logger(), "Failed to update target position");
    }
    return;
  }
  const Eigen::Vector3d heading =
      (path_->TargetPoint() - position_).normalized();
  PublishVelocitySetpoint(params_.speed * heading);
}

void PathFollowerNode::OnMotorPosition(
    std::size_t _index, const gantry_msgs::msg::MotorPosition::SharedPtr _msg) {
  std::lock_guard<decltype(mutex_)> guard(mutex_);
  position_[_index] = _msg->position;
  position_timeouts_.at(_index)->reset();
  positions_timed_out_.at(_index) = false;
}

void PathFollowerNode::OnPositionTimeout(std::size_t _index) {
  if (!positions_timed_out_.at(_index)) {
    RCLCPP_WARN(get_logger(),
                "Motor %s position timed out. Waiting for new data.",
                axis_names_.at(_index).c_str());
  }
  positions_timed_out_.at(_index) = true;
  position_timeouts_.at(_index)->cancel();
  PublishVelocitySetpoint(Eigen::Vector3d::Zero());
}

}  // namespace gantry

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(gantry::PathFollowerNode)
