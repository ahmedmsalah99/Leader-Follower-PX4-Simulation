#!/usr/bin/env python3
"""Arm every drone, take off and hold (hover) at a given altitude.

Talks to each PX4 instance directly over its /px4_<N>/fmu topics, retries
commands until PX4 confirms the state change, and exits once all drones hover.
"""

import argparse
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from px4_msgs.msg import VehicleStatus, VehicleCommand, VehicleLocalPosition, HomePosition

RETRY_PERIOD = 1.0  # seconds between command retries
ALT_TOLERANCE = 0.3  # meters


class DroneHover:
    """Per-drone state machine: WAIT_STATUS -> ARM -> TAKEOFF -> CLIMB -> DONE"""

    def __init__(self, node, drone_number, altitude):
        self.node = node
        self.drone_number = drone_number
        self.altitude = altitude
        self.namespace = f'/px4_{drone_number}'
        self.state = 'WAIT_STATUS'
        self.status = None
        self.position = None
        self.home = None
        self.last_command_time = None

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        node.create_subscription(VehicleStatus, f'{self.namespace}/fmu/out/vehicle_status_v1',
                                 self._status_callback, qos_profile)
        node.create_subscription(VehicleLocalPosition, f'{self.namespace}/fmu/out/vehicle_local_position_v1',
                                 self._position_callback, qos_profile)
        node.create_subscription(HomePosition, f'{self.namespace}/fmu/out/home_position_v1',
                                 self._home_callback, qos_profile)
        self.command_publisher = node.create_publisher(
            VehicleCommand, f'{self.namespace}/fmu/in/vehicle_command', 10)

    def _status_callback(self, msg):
        self.status = msg

    def _position_callback(self, msg):
        self.position = msg

    def _home_callback(self, msg):
        self.home = msg

    def log(self, text):
        self.node.get_logger().info(f'drone {self.drone_number}: {text}')

    def set_state(self, state):
        self.state = state
        self.last_command_time = None
        self.log(state)

    def due_for_retry(self):
        now = self.node.get_clock().now().nanoseconds / 1e9
        if self.last_command_time is None or now - self.last_command_time >= RETRY_PERIOD:
            self.last_command_time = now
            return True
        return False

    def send_command(self, command, param1=0.0, param5=0.0, param6=0.0, param7=0.0):
        msg = VehicleCommand()
        msg.timestamp = self.node.get_clock().now().nanoseconds // 1000
        msg.command = command
        msg.param1 = param1
        msg.param5 = param5
        msg.param6 = param6
        msg.param7 = param7
        msg.target_system = self.status.system_id
        msg.target_component = 1
        msg.source_system = self.status.system_id
        msg.source_component = 1
        msg.from_external = True
        self.command_publisher.publish(msg)

    def altitude_reached(self):
        return (self.position is not None and self.position.z_valid
                and -self.position.z >= self.altitude - ALT_TOLERANCE)

    def update(self):
        if self.state == 'DONE':
            return
        if self.status is None:
            return

        armed = self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED
        nav_state = self.status.nav_state

        if self.state == 'WAIT_STATUS':
            self.set_state('ARM')

        elif self.state == 'ARM':
            if armed:
                self.set_state('TAKEOFF')
            elif self.due_for_retry():
                if not self.status.pre_flight_checks_pass:
                    self.log('waiting for preflight checks to pass')
                    return
                self.send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)

        elif self.state == 'TAKEOFF':
            if nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF:
                self.set_state('CLIMB')
            elif not armed:
                # PX4 auto-disarms if takeoff doesn't follow arming quickly enough
                self.set_state('ARM')
            elif self.due_for_retry():
                # takeoff altitude is AMSL; NaN lets PX4 fall back to MIS_TAKEOFF_ALT
                takeoff_alt = (self.home.alt + self.altitude
                               if self.home is not None and self.home.valid_alt else math.nan)
                self.send_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF,
                                  param5=math.nan, param6=math.nan, param7=takeoff_alt)

        elif self.state == 'CLIMB':
            # PX4 switches to hold (AUTO_LOITER) when the takeoff altitude is reached
            if nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER and self.altitude_reached():
                self.set_state('DONE')
            elif not armed:
                self.set_state('ARM')


class HoverNode(Node):
    def __init__(self, num_drones, altitude, timeout):
        super().__init__('hover_command')
        self.drones = [DroneHover(self, i, altitude) for i in range(1, num_drones + 1)]
        self.timeout = timeout
        self.start_time = self.get_clock().now()
        self.result = None
        self.create_timer(0.1, self.update)

    def update(self):
        for drone in self.drones:
            drone.update()

        if all(drone.state == 'DONE' for drone in self.drones):
            self.get_logger().info('all drones are hovering')
            self.result = 0
            return

        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed > self.timeout:
            pending = {drone.drone_number: drone.state for drone in self.drones if drone.state != 'DONE'}
            self.get_logger().error(f'timed out after {self.timeout:.0f}s, pending drones: {pending}')
            self.result = 1


def main():
    parser = argparse.ArgumentParser(description='Arm all drones, take off and hover')
    parser.add_argument('-n', '--num-drones', type=int, default=1, help='number of drones (default: 1)')
    parser.add_argument('-a', '--altitude', type=float, default=2.5,
                        help='hover altitude above home in meters (default: 2.5)')
    parser.add_argument('-t', '--timeout', type=float, default=60.0,
                        help='give up after this many seconds (default: 60)')
    args = parser.parse_args()

    rclpy.init()
    node = HoverNode(args.num_drones, args.altitude, args.timeout)
    try:
        while rclpy.ok() and node.result is None:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.result = 1
    result = node.result
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(result)


if __name__ == '__main__':
    main()
