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
#include <gantry_msgs/msg/motor_position.hpp>
#include <gantry_msgs/msg/motor_velocity.hpp>
#include <path_planning/path.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>

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
  void InitParams();
  void InitPublishers();
  void InitSubscriptions();
  std::string GetWaypointsFilePath();

  rcl_interfaces::msg::SetParametersResult OnParameters(
      const std::vector<rclcpp::Parameter> &parameters);

  rclcpp::Subscription<gantry_msgs::msg::MotorPosition>::SharedPtr
      position_x_sub_;
  rclcpp::Subscription<gantry_msgs::msg::MotorPosition>::SharedPtr
      position_y_sub_;
  rclcpp::Subscription<gantry_msgs::msg::MotorPosition>::SharedPtr
      position_z_sub_;

  rclcpp::Publisher<gantry_msgs::msg::MotorVelocity>::SharedPtr velocity_x_sub_;
  rclcpp::Publisher<gantry_msgs::msg::MotorVelocity>::SharedPtr velocity_y_sub_;
  rclcpp::Publisher<gantry_msgs::msg::MotorVelocity>::SharedPtr velocity_z_sub_;

  Params params_;
  std::shared_ptr<path_planning::Path> path_;

  OnSetParametersCallbackHandle::SharedPtr params_cb_handle_;
};
}  // namespace gantry
