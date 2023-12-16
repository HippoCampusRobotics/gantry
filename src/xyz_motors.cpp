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
#include <thread>

#include "gantry/motor_component/motor_component.hpp"

std::shared_ptr<std::thread> CreateSpinThread(std::string axis) {
  return std::make_shared<std::thread>([axis]() {
    std::string pkg_dir =
        ament_index_cpp::get_package_share_directory("gantry");
    std::string config_dir = pkg_dir + "/config";
    std::string name = "motor_" + axis;
    std::string config_file = config_dir + "/" + name + ".yaml";
    std::string name_remap = "__node:=" + name;
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);
    options.arguments(
        {"--ros-args", "--params-file", config_file, "-r", name_remap});

    auto node = std::make_shared<gantry::MotorNode>(options);

    rclcpp::experimental::executors::EventsQueue::UniquePtr events_queue =
        std::make_unique<rclcpp::experimental::executors::SimpleEventsQueue>();
    rclcpp::experimental::executors::EventsExecutor exec(
        std::move(events_queue), true);

    exec.add_node(node->get_node_base_interface());
    exec.spin();
    exec.remove_node(node->get_node_base_interface());
  });
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  std::array<std::string, 3> axes{{"x", "y", "z"}};

  std::vector<std::shared_ptr<std::thread>> spin_threads;

  for (const auto &axis : axes) {
    spin_threads.push_back(CreateSpinThread(axis));
    RCLCPP_INFO(rclcpp::get_logger("xyz_motor"), "Created %s axis motor.",
                axis.c_str());
  }
  for (auto &thread : spin_threads) {
    thread->join();
  }
  rclcpp::shutdown();

  return 0;
}
