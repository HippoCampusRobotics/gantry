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

#include <hippo_common/param_utils.hpp>

#include "gantry/path_follower_component/path_follower_component.hpp"

namespace gantry {

void PathFollowerNode::InitParams() {
  HIPPO_COMMON_DECLARE_PARAM_NO_DEFAULT(look_ahead_distance);
  HIPPO_COMMON_DECLARE_PARAM_NO_DEFAULT(path_file);
  HIPPO_COMMON_DECLARE_PARAM_NO_DEFAULT(speed);
  HIPPO_COMMON_DECLARE_PARAM_NO_DEFAULT(ignore_z_distance);

  params_cb_handle_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> &params) {
        return OnParameters(params);
      });
}

rcl_interfaces::msg::SetParametersResult PathFollowerNode::OnParameters(
    const std::vector<rclcpp::Parameter> &_parameters) {
  std::string text;
  rcl_interfaces::msg::SetParametersResult result;
  result.reason = "unhandled";
  result.successful = true;
  bool updated{false};

  for (const auto &parameter : _parameters) {
    HIPPO_COMMON_ASSIGN_SIMPLE_LOG(look_ahead_distance, updated, text);
    HIPPO_COMMON_ASSIGN_SIMPLE_LOG(path_file, updated, text);
    HIPPO_COMMON_ASSIGN_SIMPLE_LOG(ignore_z_distance, updated, text);
    HIPPO_COMMON_ASSIGN_SIMPLE_LOG(speed, updated, text);
  }

  if (updated) {
    if (path_) {
      path_->SetLookAhead(params_.look_ahead_distance);
      path_->ignore_z() = params_.ignore_z_distance;
    }
  }
  return result;
}

}  // namespace gantry
