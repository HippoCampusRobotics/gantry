#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from gantry_msgs.msg import MotorPosition, MotorVelocity, MotorStatus

import yaml

qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                 history=QoSHistoryPolicy.KEEP_LAST,
                 depth=1)


class GridPositionControl(Node):

    def __init__(self, node_name):
        super().__init__(node_name=node_name)
        self.waypoint_grid = None

        self.axes = ('x', 'y', 'z')

        self.position_reached = [False, False, False]
        self.all_positions_reached = False
        self.last_time_position_reached = self.get_clock().now()

        self.publishing_next_setpoint = False
        self.current_waypoint_index = 0

        self.motor_positions = [0.0, 0.0, 0.0]
        self.positions_timed_out = [False, False, False]

        self.wait_time_measurement = 0.0

        # publishers
        self.motor_setpoint_pubs = []

        # subscribers
        self.motor_position_subs = []
        self.motor_velocity_subs = []
        self.motor_status_subs = []

        # init subscribers + publishers
        for i in range(len(self.axes)):
            topic_name = 'motor' + self.axes[i] + '/setpoint/absolute_position'
            pub = self.create_publisher(
                MotorPosition,
                topic_name,
                qos,
            )
            self.motor_setpoint_pubs.append(pub)

            topic_name = 'motor_' + self.axes[i] + '/position'
            sub1 = self.create_subscription(
                MotorPosition, topic_name,
                lambda msg: self.on_motor_position(i, msg), qos)
            self.motor_position_subs.append(sub1)

            topic_name = 'motor_' + self.axes[i] + '/motor_status'
            sub2 = self.create_subscription(
                MotorVelocity, topic_name,
                lambda msg: self.on_motor_status(i, msg), qos)
            self.motor_velocity_subs.append(sub2)

        # timer
        self.control_timer = self.create_timer(timer_period_sec=(1 / 200),
                                               callback=self.on_control_timer())

        self.timeout_timer = self.create_timer(timer_period_sec=(1 / 200),
                                               callback=self.on_timeout_timer())

    def load_default_waypoints(self):
        descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_STRING)
        self.declare_parameter('waypoint_file', descriptor=descriptor)
        filepath = self.get_parameter('waypoint_file').value

        with open(filepath, 'r') as f:
            data = yaml.safe_load(f)

        return data['waypoints']

    def on_motor_position(self, index: int, msg: MotorPosition):
        self.motor_positions[index] = msg.position

        if self.positions_timed_out[index]:
            self.get_logger().info(
                f'Received position for' +
                f'{self.axes[index]}-axis. Not timed out anymore.')
        self.positions_timed_out[index] = False
        self.timeout_timer.reset()
        



    def on_motor_status(self, index: int, msg: MotorStatus):
        self.position_reached[index] = msg.position_reached

        if all(self.position_reached):
            self.publishing_next_setpoint = False

            if not self.all_positions_reached:
                # first time all motors reached position
                self.all_positions_reached = True
                self.last_time_position_reached = self.get_clock().now()
                return
            else:
                # not the first time
                now = self.get_clock().now()
                dt = (self.last_time_position_reached - now).nanoseconds * 1e-9
                if dt < (0.05 + self.wait_time_measurement):
                    # still wait some more time before moving to next waypoint
                    return
                else:
                    # let's start sending the next waypoint
                    self.publishing_next_setpoint = True

        else:  # not all motor positions reached
            self.all_positions_reached = False

    def on_control_timer(self):
        pass

    def on_timeout_timer(self):
        for i in range(len(self.axes)):
            if self.positions_timed_out[i]:
                self.get_logger().warning(
                    f'Motor position for {self.axes[i]}-axis timed out.' +
                    'Waiting for new data.')

        

    def is_motor_position_timed_out(self, index: int):
        pass

    def publish_position_setpoint(self, index: int, position: float):
        msg = MotorPosition()
        msg.position = position
        msg.header.stamp = self.get_clock().now().to_msg()
        self.motor_setpoint_pubs[index].publish(msg)


def main():
    rclpy.init()
    node = GridPositionControl("gantry_grid_control")
    rclpy.spin(node)


if __name__ == "__main__":
    main()
