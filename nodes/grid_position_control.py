#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from gantry_msgs.msg import MotorPosition, MotorVelocity, MotorStatus
from hippo_msgs.msg import BoolStamped
from std_srvs.srv import Trigger

import yaml

qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                 history=QoSHistoryPolicy.KEEP_LAST,
                 depth=1)


# TODO: - waypoint grid als list/...?
#       - 


class GridPositionControl(Node):

    def __init__(self, node_name):
        super().__init__(node_name=node_name)
        self.initialized = False

        self.waypoint_grid = self.load_waypoints()

        self.axes = ('x', 'y', 'z')

        self.running = False

        self.position_reached = [False, False, False]
        self.all_positions_reached = False
        self.last_time_position_reached = self.get_clock().now()

        self.publishing_next_setpoint = False
        self.current_waypoint_index = 0

        self.motor_positions = [0.0, 0.0, 0.0]
        self.positions_timed_out = [False, False, False]

        self.wait_time_measurement = 0.0

        # services
        self.start_srv = self.create_service(Trigger, '~/start',
                                             self.serve_start)
        self.stop_srv = self.create_service(Trigger, '~/stop', self.serve_stop)

        # publishers
        self.motor_position_setpoint_pubs = []
        self.motor_velocity_setpoint_pubs = []
        self.is_meas_time = False
        self.meas_time_pub = self.create_publisher(BoolStamped,
                                                   'measurement_active', qos)

        # subscribers
        self.motor_position_subs = []
        self.motor_velocity_subs = []
        self.motor_status_subs = []

        # init subscribers + publishers
        for i in range(len(self.axes)):
            topic_name = 'motor_' + self.axes[i] + '/setpoint/absolute_position'
            pub1 = self.create_publisher(MotorPosition, topic_name, qos)
            self.motor_position_setpoint_pubs.append(pub1)

            topic_name = 'motor_' + self.axes[i] + '/setpoint/velocity'
            pub2 = self.create_publisher(MotorVelocity, topic_name, qos)
            self.motor_velocity_setpoint_pubs.append(pub2)

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

        # timers
        self.control_timer = self.create_timer(timer_period_sec=(1 / 50),
                                               callback=self.on_control_timer)

        self.timeout_timer_x = self.create_timer((1 / 2), self.on_timeout_x)
        self.timeout_timer_y = self.create_timer((1 / 2), self.on_timeout_y)
        self.timeout_timer_z = self.create_timer((1 / 2), self.on_timeout_z)

        self.get_logger().info(f'Initialization finished.')
        self.initialized = True

    def load_waypoints(self):
        descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_STRING)
        self.declare_parameter('waypoint_file', descriptor=descriptor)
        filepath = self.get_parameter('waypoint_file').value

        with open(filepath, 'r') as f:
            data = yaml.safe_load(f)

        return data['waypoints']

    def on_motor_position(self, index: int, msg: MotorPosition):
        if not self.initialized:
            self.get_logger().info(f'Not initialized yet.')
            return

        self.motor_positions[index] = msg.position

        if self.positions_timed_out[index]:
            self.get_logger().info(
                f'Received position for ' +
                f'{self.axes[index]}-axis. Not timed out anymore.')

        self.positions_timed_out[index] = False

        if index == 0:
            self.timeout_timer_x.reset()
        elif index == 1:
            self.timeout_timer_y.reset()
        elif index == 2:
            self.timeout_timer_z.reset()

    def on_motor_status(self, index: int, msg: MotorStatus):
        if not self.initialized:
            self.get_logger().info(f'Not initialized yet.')
            return

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
                    if 0.05 < dt < self.wait_time_measurement:
                        self.is_meas_time = True
                    return
                else:
                    # let's start sending the next waypoint
                    self.is_meas_time = False
                    self.current_waypoint_index += self.current_waypoint_index

        else:  # not all motor positions reached
            self.all_positions_reached = False
            self.is_meas_time = False

    def on_control_timer(self):
        if not self.initialized:
            return

        self.publish_measurement_active(self.is_meas_time)

        if self.is_motor_position_timed_out():
            self.publish_velocity_setpoint([0.0, 0.0, 0.0])
            return
        if not self.running:
            self.publish_velocity_setpoint([0.0, 0.0, 0.0])
        # TODO check if waypoints valid?
        self.get_logger().info(f'{self.waypoint_grid[85]}')

    def on_timeout_timer(self, index: int):
        if not self.initialized:
            self.get_logger().info(f'Not initialized yet.')
            return
        self.get_logger().info(
            f'Timer for index {index} / axis {self.axes[index]} called!')

        if not self.positions_timed_out[index]:
            self.get_logger().warning(
                f'Motor position for {self.axes[index]}-axis timed out. ' +
                'Waiting for new data.')

        self.positions_timed_out[index] = True

        if index == 0:
            self.timeout_timer_x.cancel()
        elif index == 1:
            self.timeout_timer_y.cancel()
        elif index == 2:
            self.timeout_timer_z.cancel()

    def on_timeout_x(self):
        if not self.positions_timed_out[0]:
            self.get_logger().warning(
                f'Motor position for {self.axes[0]}-axis timed out. ' +
                'Waiting for new data.')

        self.positions_timed_out[0] = True
        self.timeout_timer_x.cancel()

    def on_timeout_y(self):
        if not self.positions_timed_out[1]:
            self.get_logger().warning(
                f'Motor position for {self.axes[1]}-axis timed out. ' +
                'Waiting for new data.')

        self.positions_timed_out[1] = True
        self.timeout_timer_y.cancel()

    def on_timeout_z(self):
        if not self.positions_timed_out[2]:
            self.get_logger().warning(
                f'Motor position for {self.axes[2]}-axis timed out. ' +
                'Waiting for new data.')

        self.positions_timed_out[2] = True
        self.timeout_timer_z.cancel()

    def is_motor_position_timed_out(self):
        return any(self.positions_timed_out)

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

    def serve_move_to_start(self, request, response):
        pass


def main():
    rclpy.init()
    node = GridPositionControl("gantry_grid_control")
    rclpy.spin(node)


if __name__ == "__main__":
    main()
