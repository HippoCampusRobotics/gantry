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

#include "gantry/motor_component/motor_component.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  rclcpp::experimental::executors::EventsQueue::UniquePtr events_queue =
      std::make_unique<rclcpp::experimental::executors::SimpleEventsQueue>();
  // the separate thread for the timers are necessary. Otherwise the
  // timer callbacks get queued up, if the execution of the callbacks
  // can't keep up. This does not seem to be the case for a separate
  // timer manager thread.
  rclcpp::experimental::executors::EventsExecutor exec(std::move(events_queue),
                                                       true);
  rclcpp::NodeOptions options;

  auto motor_node = std::make_shared<gantry::MotorNode>(options);

  exec.add_node(motor_node);
  exec.spin();

  rclcpp::shutdown();

  return 0;
}
