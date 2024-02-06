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
#include <atomic>
#include <eigen3/Eigen/Dense>
#include <gantry_msgs/msg/motor_position.hpp>
#include <gantry_msgs/msg/motor_velocity.hpp>
#include <mutex>
#include <path_planning/path.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace gantry {
class PathFollowerNode : public rclcpp::Node {
 public:
  explicit PathFollowerNode(const rclcpp::NodeOptions &options);

 private:
  struct Params {
    double look_ahead_distance;
    double speed;
    bool ignore_z_distance;
    std::string path_file;
  };
  static constexpr std::size_t kNumAxes = 3;
  typedef std::array<
      rclcpp::Publisher<gantry_msgs::msg::MotorVelocity>::SharedPtr, kNumAxes>
      VelocityPublishers;
  typedef std::array<
      rclcpp::Publisher<gantry_msgs::msg::MotorPosition>::SharedPtr, kNumAxes>
      PositionPublishers;
  typedef std::array<std::string, kNumAxes> AxisNames;
  typedef std::array<
      rclcpp::Subscription<gantry_msgs::msg::MotorPosition>::SharedPtr,
      kNumAxes>
      PositionSubscriptions;
  typedef std::array<bool, kNumAxes> PositionsTimeoutStatus;
  typedef std::array<rclcpp::TimerBase::SharedPtr, kNumAxes> PositionTimeouts;
  void InitParams();
  void InitPublishers();
  void InitSubscriptions();
  void InitTimers();
  void PublishVelocitySetpoint(const Eigen::Vector3d &);
  void PublishPositionSetpoint(const Eigen::Vector3d &);
  bool IsMotorPositionTimedOut();
  std::string GetWaypointsFilePath();
  void LoadDefaultWaypoints();
  void UpdateControl();

  void ServeStart(const std_srvs::srv::Trigger_Request::SharedPtr,
                  std_srvs::srv::Trigger_Response::SharedPtr);
  void ServeStop(const std_srvs::srv::Trigger_Request::SharedPtr,
                 std_srvs::srv::Trigger_Response::SharedPtr);
  void ServeMoveToStart(const std_srvs::srv::Trigger_Request::SharedPtr,
                        std_srvs::srv::Trigger_Response::SharedPtr);

  void OnMotorPosition(std::size_t index,
                       const gantry_msgs::msg::MotorPosition::SharedPtr);

  void OnPositionTimeout(std::size_t index);

  rcl_interfaces::msg::SetParametersResult OnParameters(
      const std::vector<rclcpp::Parameter> &parameters);

  AxisNames axis_names_ = {"x", "y", "z"};

  PositionSubscriptions position_subs_;

  VelocityPublishers velocity_pubs_;
  PositionPublishers position_pubs_;

  Params params_;
  std::shared_ptr<path_planning::Path> path_;
  std::atomic<bool> running_{false};
  std::mutex mutex_;

  Eigen::Vector3d start_position_{0.0, 0.0, 0.0};
  Eigen::Vector3d position_{0.0, 0.0, 0.0};

  OnSetParametersCallbackHandle::SharedPtr params_cb_handle_;

  PositionsTimeoutStatus positions_timed_out_;
  PositionTimeouts position_timeouts_;
};
}  // namespace gantry
