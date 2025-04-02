#!/usr/bin/env python3

from time import sleep
import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleStatus, VehicleCommand, VehicleLocalPosition, BatteryStatus, TrajectorySetpoint, OffboardControlMode
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from drones_msgs.msg import Leaderpos
from functools import wraps
from pymavlink import mavutil
import socket


class DroneController(Node):
    def __init__(self):
        super().__init__('drone_controller')
        
        # Declare and get drone number parameter
        self.declare_parameter('drone_number', 1)
        self.drone_number = self.get_parameter('drone_number').value
        if self.drone_number < 1:
            self.get_logger().error('Invalid drone number. Must be >= 1')
            raise ValueError('Drone number must be >= 1')
            
        self.namespace = f'/px4_{self.drone_number}'
        
        # Constants
        self.MIN_BATTERY_LEVEL = 0.2  # percentage
        self.POSITION_THRESHOLD = 1.0  # meters
        self.VELOCITY_THRESHOLD = 0.1  # m/s
        self.TAKEOFF_ALT = 1.5
        self.FOLLOWER_MARGIN = 0.5

        # Initialize state variables
        self.arming_state = 1
        self.nav_state = 0
        self.current_status = None
        self.current_position = None
        self.leader_position = None
        self.battery_status = None
        self.system_id = None
        self.component_id = None
        self.leader_id = None
        self.is_swarm_participant = True
        self.udp_out = None
        # Configure QoS profile
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        # Publishers with namespaced topics
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, f'{self.namespace}/fmu/in/vehicle_command', 10)


        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, f'{self.namespace}/fmu/in/trajectory_setpoint', 10
        )

        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode,
            f'{self.namespace}/fmu/in/offboard_control_mode',
            10
        )

        # Subscribers with QoS settings
        self.status_sub = self.create_subscription(
            VehicleStatus, f'{self.namespace}/fmu/out/vehicle_status_v1', 
            self._status_callback, qos_profile)
            
        self.local_position_sub = self.create_subscription(
            VehicleLocalPosition, f'{self.namespace}/fmu/out/vehicle_local_position',
            self._local_position_callback, qos_profile)
            
        self.battery_status_sub = self.create_subscription(
            BatteryStatus, f'{self.namespace}/fmu/out/battery_status',
            self._battery_status_callback, qos_profile)

        # Command subscribers with drone number in topic name
        self.leader_pose_sub = self.create_subscription(
            Leaderpos, f'/global/leader_pose', 
            self._leader_pose_callback, 10)
        
        self.arm_command_sub = self.create_subscription(
            Bool, f'drone_{self.drone_number}/command/arm',
            self._arm_command_callback, 10)
        
        self.launch_command_sub = self.create_subscription(
            Bool, f'drone_{self.drone_number}/command/launch',
            self._launch_command_callback, 10)
    
        self.full_launch_command_sub = self.create_subscription(
            Bool, f'drone_{self.drone_number}/command/full_launch',
            self._full_launch_command_callback, 10)
        
        self.return_command_sub = self.create_subscription(
            Bool, f'drone_{self.drone_number}/command/return',
            self._return_command_callback, 10)
        
        # Timer for periodic updates
        self.create_timer(0.1, self.update)  # 10Hz update rate
        
        self.get_logger().info(f'Drone controller initialized with number {self.drone_number}')

    def is_leader(self):
        return self.leader_id is not None and self.leader_id == self.system_id
    
    def check_status(func):
        """Wrapper that checks if self.current_status exists before executing the function."""
        @wraps(func)
        def wrapper(self, *args, **kwargs):
            if self.current_status is None:
                self.get_logger().warn('No status updates received yet',throttle_duration_sec=1)
                return None  # or raise an exception if preferred
            return func(self, *args, **kwargs)
        return wrapper
    
    def initialize_comms(self):# MAVLink connection
        self.udp_out = mavutil.mavlink_connection('udpout:239.255.0.7:14551',source_system=self.system_id)

    @check_status
    def update(self):
        """Periodic update function"""
        # Safety checks
        if self.current_status.failsafe:
            self.get_logger().error('Failsafe triggered!',throttle_duration_sec=1)
            return
        if self.udp_out is not None:
            self.udp_out.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_QUADROTOR,
                                                mavutil.mavlink.MAV_AUTOPILOT_PX4, mavutil.mavlink.MAV_MODE_FLAG_AUTO_ENABLED,
                                                  self.is_swarm_participant, mavutil.mavlink.MAV_STATE_ACTIVE)
        
        if self.battery_status is not None and self.battery_status.remaining < self.MIN_BATTERY_LEVEL:
            self.get_logger().warn('Low battery!',throttle_duration_sec=1)
            self._handle_low_battery()

        if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.publish_offboard_control_mode()

        # State machine updates with error checking
        try: 
            if self.is_leader():
                self.perform_leader_work()
            else:
                self.follow_leader()
        
        except Exception as e:
            self.get_logger().error(f'Error in state machine update: {str(e)}',throttle_duration_sec=1)

    def perform_leader_work(self):
        if self.current_position is None:
            self.get_logger().warn('No position available yet',throttle_duration_sec=1)
            return
        
    
    @check_status
    def follow_leader(self):
        """Follow the leader drone"""
        leader_pos = self.leader_position

        # Check if leader position exists and timestamp is not expired
        if (leader_pos is None or 
            leader_pos.timestamp == 0):
            self.get_logger().warn('No valid leader position available',throttle_duration_sec=1)
            return
        
        current_time = self.get_clock().now().nanoseconds // 1000
        is_timed_out = (current_time - leader_pos.timestamp) > 2000000 # 2 second timeout

        if is_timed_out:
            self.get_logger().warn('Leader position timed out',throttle_duration_sec=1)
            return
        
        # If not armed, or the drone is not in the air, don't follow
        if self.arming_state != VehicleStatus.ARMING_STATE_ARMED or (-self.current_position.z < self.POSITION_THRESHOLD and self.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD):
            self.get_logger().warn(f'Drone not armed or not in the air',throttle_duration_sec=1)
            return

        # If the drone is on hold, switch to offboard mode
        if self.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER:
            self.get_logger().info('Switching to offboard mode')
            self.publish_offboard_control_mode()
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE,1.0,6.0)
            
        # Calculate offset based on drone ID
        offset = self.drone_number * self.FOLLOWER_MARGIN

        # Create and publish offboard control message with offset position
        offboard_msg = TrajectorySetpoint()
        offboard_msg.position = [
            leader_pos.x - offset,  # Offset in x direction
            leader_pos.y,           # Same y position as leader
            leader_pos.z            # Same altitude as leader
        ]
        offboard_msg.yaw = leader_pos.heading  # Maintain same heading as leader

        self.trajectory_setpoint_publisher.publish(offboard_msg)

    def publish_offboard_control_mode(self):
        """Publish offboard control mode message"""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = self.get_clock().now().nanoseconds // 1000  # Convert to microseconds
        self.offboard_control_mode_publisher.publish(msg)

    def _status_callback(self, msg):
        """Callback for vehicle status updates"""
        self.current_status = msg
        
        # Update system and component IDs if not set
        if self.system_id is None or self.component_id is None:
            self.system_id = msg.system_id
            self.component_id = msg.component_id
            self.initialize_comms()
            self.get_logger().info(f'Updated IDs - System: {self.system_id}, Component: {self.component_id}')
        
        self.arming_state = msg.arming_state
        self.nav_state = msg.nav_state
    
    def _local_position_callback(self, msg):
        """Callback for local position updates"""
        self.current_position = msg

    def _battery_status_callback(self, msg):
        """Callback for battery status updates"""
        self.battery_status = msg
    
    def _leader_pose_callback(self, msg):
        """Callback for leader pose updates"""
        # if msg.drone_num == self.drone_number:
        #     return
        self.leader_position = msg.pose
        self.leader_id = msg.system_id

    @check_status
    def _arm_command_callback(self, msg):
        """Handle arm/disarm commands"""
        command_type = "arm" if msg.data else "disarm"
        self.get_logger().info(f'Received {command_type} command')
        
        # Safety check for arming
        if msg.data and not self._pre_arm_check():
            self.get_logger().error('Pre-arm check failed')
            return
            
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=1.0 if msg.data else 0.0)

    @check_status
    def _launch_command_callback(self, msg):
        if msg.data and self.arming_state == VehicleStatus.ARMING_STATE_ARMED:
            self.get_logger().info('Received launch command')
            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF,
                param7=self.TAKEOFF_ALT
            )

    @check_status
    def _full_launch_command_callback(self, msg):
        """Handle launch commands"""
        if not msg.data:
            self.get_logger().error('No valid data received')
            return 

        self.get_logger().info('Received full launch command')

        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=1.0 )
        sleep(2)
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF,
            param7=self.TAKEOFF_ALT
        )
        sleep(2)
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=0.0 )
        sleep(2)
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=1.0 )

    @check_status
    def _return_command_callback(self, msg):
        """Handle return commands"""
        if msg.data:
            self.get_logger().info('Received return command')
            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH,
            )

    @check_status
    def publish_vehicle_command(self, command, param1=0.0, param2=0.0,param3=0.0, param4=0.0,param5=0.0, param6=0.0,param7=0.0):
        """Helper method to publish vehicle commands"""

        msg = VehicleCommand()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        msg.param1 = param1
        msg.param2 = param2
        msg.param3 = param3
        msg.param4 = param4
        msg.param5 = param5
        msg.param6 = param6
        msg.param7 = param7
        msg.command = command
        msg.target_system = self.system_id if self.system_id is not None else 1
        msg.target_component = self.component_id if self.component_id is not None else 1
        msg.source_system = self.system_id if self.system_id is not None else 1
        msg.source_component = self.component_id if self.component_id is not None else 1
        msg.from_external = True
        self.vehicle_command_publisher.publish(msg)
        
    def _pre_arm_check(self):
        """Perform pre-arm safety checks"""
        if self.current_status is None:
            self.get_logger().error('Pre-arm check failed: No status available')
            return False
            
        # Check battery
        if self.battery_status is None or self.battery_status.remaining < self.MIN_BATTERY_LEVEL:
            self.get_logger().error('Pre-arm check failed: Low battery')
            return False
            
        # Check if system is ready
        if not self.current_status.system_type:
            self.get_logger().error('Pre-arm check failed: System not ready')
            return False
            
        return True


    def _handle_low_battery(self):
        """Handle low battery conditions"""
        self.get_logger().warn('Low battery - Initiating return to launch',throttle_duration_sec=1)
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH,
            param1=self.TAKEOFF_ALT
        )



def main(args=None):
    try:
        rclpy.init(args=args)
        controller = DroneController()
        rclpy.spin(controller)
        controller.destroy_node()
        rclpy.shutdown()
    except Exception as e:
        print(f'Fatal error: {str(e)}')
        rclpy.shutdown()

if __name__ == '__main__':
    main()
