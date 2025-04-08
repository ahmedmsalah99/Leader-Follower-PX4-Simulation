#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from pymavlink import mavutil
import socket
from drones_msgs.msg import SwarmStatus, DroneStatus, Leaderpos
from dataclasses import dataclass
from enum import Enum
from typing import Optional

class MAVStatus(Enum):
    UNINIT = 0
    BOOT = 1
    CALIBRATING = 2
    STANDBY = 3
    ACTIVE = 4
    CRITICAL = 5
    EMERGENCY = 6
    POWEROFF = 7
    FLIGHT_TERMINATION = 8

@dataclass
class DroneStatusFactory:
    system_id: int
    in_swarm: bool
    last_timestamp: int
    system_status: MAVStatus
    
    def is_active(self) -> bool:
        """Calculate if drone is currently active based on timestamp"""
        return not self.is_timed_out()

    def is_timed_out(self) -> bool:
        """Calculate if drone has timed out based on timestamp"""
        if self.last_timestamp == 0:
            return False
        current_time = rclpy.clock.Clock().now().nanoseconds // 1000000000
        return (current_time - self.last_timestamp) > 2
    
    def generate(self):
        return DroneStatus(
            system_id=self.system_id,
            in_swarm=True if self.in_swarm==1 else False,
            last_timestamp=self.last_timestamp,
            system_status=self.system_status,
            is_active=self.is_active(),
            is_timed_out=self.is_timed_out(),
        )




class SwarmManager(Node):
    def __init__(self):
        super().__init__('swarm_manager')

        self.active_drones_sub = self.create_subscription(
            Leaderpos, f'/global/leader_pose', 
            self._leader_pose_callback, 10)
        self.swarm_status_pub = self.create_publisher(SwarmStatus, f'/global/swarm_status', 10)
        self.leader_id = None
        self.drones_status = {}
        self.initialize_comms()
        self.create_timer(0.1, self.update)

    def _leader_pose_callback(self, msg):
        self.leader_id = msg.system_id
        self.get_logger().info(f"leader id: {self.leader_id}",throttle_duration_sec=3)

    def update(self):
        msg = self.udp_in.recv_match(blocking=False)
        self.handle_msg(msg)
        swarm_status_msg = self.create_swarm_status_msg()
        self.swarm_status_pub.publish(swarm_status_msg)
        
    def create_swarm_status_msg(self):
        swarm_status_msg = SwarmStatus()
        swarm_status_msg.leader_id = 0 if self.leader_id is None else self.leader_id
        # self.get_logger().info(f'{len(list(self.drones_status.values()))}')
        swarm_status_msg.drones_status = [factory.generate() for factory in self.drones_status.values()]
        swarm_status_msg.swarm_size = len(swarm_status_msg.drones_status)
        swarm_status_msg.swarm_active_drones_num = len([drone.is_active for drone in swarm_status_msg.drones_status])
        swarm_status_msg.swarm_participants_num = len([drone.in_swarm for drone in swarm_status_msg.drones_status if drone.in_swarm])
        swarm_status_msg.swarm_participants_ids = [drone.system_id for drone in swarm_status_msg.drones_status if drone.in_swarm]
        if len(swarm_status_msg.swarm_participants_ids) > 0:
            # self.get_logger().info(swarm_status_msg.swarm_participants_ids[0])
            swarm_status_msg.swarm_participants_ids = sorted(swarm_status_msg.swarm_participants_ids)
        return swarm_status_msg


    def handle_msg(self, msg):
        if not msg:
            return
        t = msg.get_type()
        if t == "HEARTBEAT":
            self.handle_heartbeat_msg(msg)
        
    def handle_heartbeat_msg(self, msg):
        id = msg.get_srcSystem()
        timestamp = int(msg._timestamp)
        if id not in self.drones_status:
            self.get_logger().info(f'{msg._timestamp}')
            self.drones_status[id] = DroneStatusFactory(id, msg.custom_mode,timestamp, msg.system_status)
        else:
            self.drones_status[id].custom_mode = msg.custom_mode
            self.drones_status[id].last_timestamp = timestamp
            self.drones_status[id].system_status = msg.system_status

        
    def initialize_comms(self):# MAVLink connection
        self.udp_in = mavutil.mavlink_connection('udpin:0.0.0.0:14551')
        self.udp_in.port.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.udp_in.port.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, 
                        socket.inet_aton("239.255.0.7") + socket.inet_aton("0.0.0.0"))


def main(args=None):
    rclpy.init(args=args)
    swarm_manager = SwarmManager()
    rclpy.spin(swarm_manager)
    swarm_manager.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()