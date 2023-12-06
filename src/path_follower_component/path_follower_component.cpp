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

}  // namespace gantry

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(gantry::PathFollowerNode)
