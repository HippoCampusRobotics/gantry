#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from gantry_msgs.msg import MotorPosition, MotorVelocity, MotorStatus
from hippo_msgs.msg import BoolStamped
from std_srvs.srv import Trigger

import yaml
import numpy as np
import datetime

# qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
#                  history=QoSHistoryPolicy.KEEP_LAST,
#                  depth=1)
qos = 1


class GridPositionControl(Node):

    def __init__(self, node_name):
        super().__init__(node_name=node_name)
        self.initialized = False

        self.waypoint_list = self.load_waypoints()

        self.axes = ('x', 'y', 'z')
        self.max_speeds = [0.1, 0.1, 0.1]

        self.running = False

        self.position_reached = [False, False, False]
        self.all_positions_reached = False
        self.last_time_position_reached = self.get_clock().now()

        self.updated_waypoint_index = False
        self.current_waypoint_index = 0

        self.motor_positions = [0.0, 0.0, 0.0]
        self.positions_timed_out = [False, False, False]

        descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE)
        self.declare_parameter('measurement_time', descriptor=descriptor)
        self.wait_time_measurement = self.get_parameter(
            'measurement_time').value
        self.get_logger().info(
            f'Using measurement time of {self.wait_time_measurement}s.')
        self.delay_before_measurement = 0.05  # s

        # services
        self.start_srv = self.create_service(Trigger, '~/start',
                                             self.serve_start)
        self.stop_srv = self.create_service(Trigger, '~/stop', self.serve_stop)

        self.is_meas_time = False

        self.motor_position_setpoint_pubs = []
        self.motor_velocity_setpoint_pubs = []
        self.motor_position_subs = []
        self.motor_status_subs = []
        self.meas_time_pub = self.create_publisher(BoolStamped,
                                                   'measurement_active', qos)
        # init subscribers + publishers
        for i in range(len(self.axes)):
            topic_name = 'motor_' + self.axes[i] + '/setpoint/absolute_position'
            pub = self.create_publisher(MotorPosition, topic_name, qos)
            self.motor_position_setpoint_pubs.append(pub)

            topic_name = 'motor_' + self.axes[i] + '/setpoint/velocity'
            pub = self.create_publisher(MotorVelocity, topic_name, qos)
            self.motor_velocity_setpoint_pubs.append(pub)

            topic_name = 'motor_' + self.axes[i] + '/position'
            sub = self.create_subscription(
                MotorPosition,
                topic_name,
                lambda msg, index=i: self.on_motor_position(msg, index),
                qos)
            self.motor_position_subs.append(sub)

            topic_name = 'motor_' + self.axes[i] + '/motor_status'
            sub = self.create_subscription(
                MotorStatus,
                topic_name,
                lambda msg, index=i: self.on_motor_status(msg, index),
                qos)
            self.motor_status_subs.append(sub)

        # timers
        self.control_timer = self.create_timer(timer_period_sec=(1 / 50),
                                               callback=self.on_control_timer)

        self.timeout_timers = []
        for i in range(len(self.axes)):
            timer = self.create_timer(
                (1 / 2), lambda index=i: self.on_timeout_timer(index))
            self.timeout_timers.append(timer)

        self.estimated_total_time = self.get_estimated_time_to_end(0)
        self.get_logger().info(
            f'Total estimated time: {round(self.estimated_total_time / 60.0)} min '
        )

        self.get_logger().info(f'Initialization finished.')
        self.initialized = True

    def load_waypoints(self):
        descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_STRING)
        self.declare_parameter('waypoint_file', descriptor=descriptor)
        filepath = self.get_parameter('waypoint_file').value

        with open(filepath, 'r') as f:
            data = yaml.safe_load(f)

        waypoint_list = []
        for wp in data['waypoints']:
            waypoint_list.append([wp.get('x'), wp.get('y'), wp.get('z')])
        return waypoint_list

    def on_motor_position(self, msg: MotorPosition, index: int):
        if not self.initialized:
            return

        self.motor_positions[index] = msg.position

        if self.positions_timed_out[index]:
            self.get_logger().info(
                f'Received position for ' +
                f'{self.axes[index]}-axis. Not timed out anymore.')

        self.positions_timed_out[index] = False
        self.timeout_timers[index].reset()

    def on_motor_status(self, msg: MotorStatus, index: int):
        if not self.initialized:
            return

        self.position_reached[index] = msg.position_reached

        if all(self.position_reached):
            if not self.all_positions_reached:
                # first time all motors reached position
                self.get_logger().info(
                    f'First time all motors reached waypoint!')
                self.all_positions_reached = True
                self.last_time_position_reached = self.get_clock().now()
                return
            else:
                # not the first time
                now = self.get_clock().now()
                dt = (now - self.last_time_position_reached).nanoseconds * 1e-9
                if dt < (self.delay_before_measurement +
                         self.wait_time_measurement):
                    # still wait some more time before moving to next waypoint
                    if self.delay_before_measurement < dt < (
                            self.delay_before_measurement +
                            self.wait_time_measurement):
                        self.is_meas_time = True
                        self.get_logger().info(
                            f'Now measuring at waypoint' +
                            f' {self.current_waypoint_index} of {len(self.waypoint_list)}',
                            throttle_duration_sec=self.wait_time_measurement)
                        remaining_time = self.get_estimated_time_to_end(
                            self.current_waypoint_index)
                        self.get_logger().info(
                            f'Total estimated remaining time: {round(remaining_time / 60.0)} min',
                            throttle_duration_sec=self.wait_time_measurement)
                    return
                else:
                    # time's up!
                    self.is_meas_time = False
                    if not self.updated_waypoint_index:
                        # let's start sending the next waypoint
                        self.get_logger().info(
                            f'Finished at waypoint ' +
                            f'{self.waypoint_list[self.current_waypoint_index]}'
                            + f' with index {self.current_waypoint_index}')
                        self.current_waypoint_index += 1
                        self.updated_waypoint_index = True
                        self.get_logger().info(
                            f'Moving on to waypoint ' +
                            f'{self.waypoint_list[self.current_waypoint_index]}'
                            + f' with index {self.current_waypoint_index}')

        else:  # not all motor positions reached
            # reset status
            self.all_positions_reached = False
            self.is_meas_time = False
            self.updated_waypoint_index = False

    def on_control_timer(self):
        if not self.initialized:
            return

        self.publish_measurement_active(self.is_meas_time)

        if self.is_motor_position_timed_out():
            self.publish_velocity_setpoint([0.0, 0.0, 0.0])
            return
        if not self.running:
            self.publish_velocity_setpoint([0.0, 0.0, 0.0])
            return

        # TODO check if waypoints valid?
        self.publish_position_setpoint(
            self.waypoint_list[self.current_waypoint_index])
        # self.get_logger().info(
        #     f'Currently publishing the following setpoint: ' +
        #     f'{self.waypoint_list[self.current_waypoint_index]}',
        #     throttle_duration_sec=2)

    def on_timeout_timer(self, index: int):
        if not self.initialized:
            return

        if not self.positions_timed_out[index]:
            self.get_logger().warning(
                f'Motor position for {self.axes[index]}-axis timed out. ' +
                'Waiting for new data.')

        self.positions_timed_out[index] = True
        self.timeout_timers[index].cancel()

    def is_motor_position_timed_out(self):
        return any(self.positions_timed_out)

    def get_estimated_time_to_end(self, waypoint_index: int) -> float:
        num_waypoints = len(self.waypoint_list) - waypoint_index
        remaining_time = (self.wait_time_measurement +
                          self.delay_before_measurement) * num_waypoints
        for wp in range(waypoint_index, len(self.waypoint_list) - 1):
            max_time = [None] * len(self.axes)
            for i in range(len(self.axes)):
                distance_i = self.waypoint_list[
                    wp + 1][i] - self.waypoint_list[wp][i]
                max_time[i] = distance_i / self.max_speeds[i]
            remaining_time += max(
                max_time
            ) + 0.5  # some extra to account for axis to start moving
        return remaining_time

    def publish_position_setpoint(self, setpoint: list[float]):
        msg = MotorPosition()
        msg.header.stamp = self.get_clock().now().to_msg()

        for i in range(len(self.axes)):
            msg.position = setpoint[i]
            if not self.motor_position_setpoint_pubs[i]:
                self.get_logger().error(
                    f'Position pub for {self.axes[i]}-axis not initialized',
                    throttle_duration_sec=1)
                continue
            self.motor_position_setpoint_pubs[i].publish(msg)

    def publish_velocity_setpoint(self, setpoint: list[float]):
        msg = MotorVelocity()
        msg.header.stamp = self.get_clock().now().to_msg()

        for i in range(len(self.axes)):
            msg.velocity = setpoint[i]
            if not self.motor_velocity_setpoint_pubs[i]:
                self.get_logger().error(
                    f'Velocity pub for {self.axes[i]}-axis not initialized',
                    throttle_duration_sec=1)
                continue
            self.motor_velocity_setpoint_pubs[i].publish(msg)

    def publish_measurement_active(self, is_meas_time: bool):
        msg = BoolStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.data = is_meas_time
        self.meas_time_pub.publish(msg)

    def serve_start(self, request, response):
        if self.running:
            message = 'Already running'
            success = False
        else:
            self.running = True
            message = 'Starting...'
            success = False
        response.message = message
        response.success = success
        self.get_logger().info(message)
        return response

    def serve_stop(self, request, response):
        if self.running:
            self.running = False
            success = True
            message = 'Stopping...'
        else:
            message = 'Already stopped.'
            success = False
        # while zero velocity should be published elsewhere now,
        # it does not hurt to publish it here, too:
        self.publish_velocity_setpoint([0.0, 0.0, 0.0])
        response.message = message
        response.success = success
        self.get_logger().info(message)


def main():
    rclpy.init()
    node = GridPositionControl("gantry_grid_control")
    rclpy.spin(node)


if __name__ == "__main__":
    main()
