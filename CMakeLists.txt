# Copyright (C) 2023 Thies Lennart Alff
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301,
# USA

cmake_minimum_required(VERSION 3.5)
project(gantry)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CXX_FLAGS -Wall -Wextra -Wpedantic -Wshadow -std=c++17 -O2 -fPIC)
add_compile_options(${CXX_FLAGS})

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_srvs REQUIRED)
find_package(gantry_msgs REQUIRED)
find_package(hippo_common REQUIRED)
find_package(path_planning REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

################################################################################
# gantry_motor library
################################################################################
set(lib_name gantry_motor)
add_library(${lib_name}
  STATIC
  src/motor/motor_interface.cpp
  src/motor/mcbl2805.cpp
  src/motor/mcbl3006.cpp
)

target_include_directories(${lib_name}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/esc_serial/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${lib_name}
  rclcpp
)

ament_export_libraries(${lib_name}_export HAS_LIBRARY_TARGET)

install(TARGETS ${lib_name}
  EXPORT ${lib_name}_export
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)

################################################################################
# motor component
################################################################################
add_library(motor_component SHARED
  src/motor_component/motor_component.cpp
  src/motor_component/motor_component_params.cpp
)
target_link_libraries(motor_component gantry_motor)
ament_target_dependencies(motor_component
  rclcpp
  rclcpp_components
  hippo_common
  gantry_msgs
  std_srvs
)
rclcpp_components_register_nodes(motor_component PLUGIN "gantry::MotorNode")
install(
  TARGETS motor_component
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

################################################################################
# path_follower component
################################################################################
add_library(path_follower_component SHARED
  src/path_follower_component/path_follower_component.cpp
  src/path_follower_component/path_follower_component_params.cpp
)
ament_target_dependencies(path_follower_component
  rclcpp
  rclcpp_components
  hippo_common
  gantry_msgs
  path_planning
  std_srvs
  Eigen3
  visualization_msgs
)
rclcpp_components_register_nodes(path_follower_component PLUGIN "gantry::PathFollowerNode")
install(
  TARGETS path_follower_component
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
################################################################################
# single_motor node
################################################################################
set(exec_name single_motor)
add_executable(${exec_name}
  src/single_motor.cpp
)
target_link_libraries(${exec_name} motor_component)
ament_target_dependencies(${exec_name}
  rclcpp
)
install(TARGETS ${exec_name}
  DESTINATION lib/${PROJECT_NAME}
)

include_directories(
  include
)

################################################################################
# xyz_motors container node
################################################################################
set(exec_name xyz_motors)
add_executable(${exec_name}
  src/xyz_motors.cpp
)
target_link_libraries(${exec_name} motor_component)
ament_target_dependencies(${exec_name}
  rclcpp
)
install(TARGETS ${exec_name}
  DESTINATION lib/${PROJECT_NAME}
)

include_directories(
  include
)


################################################################################
# path_follower node
################################################################################
set(exec_name path_follower)
add_executable(${exec_name}
  src/path_follower.cpp
)
target_link_libraries(${exec_name} path_follower_component)
ament_target_dependencies(${exec_name}
  rclcpp
)
install(TARGETS ${exec_name}
  DESTINATION lib/${PROJECT_NAME}
)

################################################################################
# install Python Stuff
################################################################################

# Install Python modules
# ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  nodes/grid_position_control.py
  DESTINATION lib/${PROJECT_NAME}
)


################################################################################
# install shared resources
################################################################################
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

