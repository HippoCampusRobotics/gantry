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

#include <rclcpp/experimental/executors/events_executor/events_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <thread>

#include "gantry/path_follower_component/path_follower_component.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  rclcpp::experimental::executors::EventsExecutor exec;
  rclcpp::experimental::executors::EventsExecutor timer_exec;
  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true);

  auto node = std::make_shared<gantry::PathFollowerNode>(options);

  exec.add_node(node);
  timer_exec.add_callback_group(node->timer_cb_group_,
                                node->get_node_base_interface());
  std::thread node_thread([&exec]() { exec.spin(); });
  std::thread timer_thread([&timer_exec]() { timer_exec.spin(); });

  node_thread.join();
  timer_thread.join();

  rclcpp::shutdown();

  return 0;
}
