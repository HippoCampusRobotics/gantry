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

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/experimental/executors/events_executor/events_executor.hpp>
#include <rclcpp/rclcpp.hpp>

#include "gantry/motor_component/motor_component.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  rclcpp::experimental::executors::EventsExecutor exec;
  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true);

  std::string pkg_dir = ament_index_cpp::get_package_share_directory("gantry");
  std::string config_dir = pkg_dir + "/config";
  std::array<std::string, 3> axes{{"x", "y", "z"}};

  for (const auto &axis : axes) {
    std::string name = "motor_" + axis;
    std::string config_file = config_dir + "/" + name + ".yaml";
    std::string name_remap = "__node:=" + name;
    options.arguments(
        {"--ros-args", "--params-file", config_file, "-r", name_remap});
    auto motor_node = std::make_shared<gantry::MotorNode>(options);
    exec.add_node(motor_node);
  }

  exec.spin();

  rclcpp::shutdown();

  return 0;
}
