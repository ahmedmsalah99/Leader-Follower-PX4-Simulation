#!/usr/bin/env python3

from time import sleep
import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleStatus, VehicleCommand, VehicleLocalPosition, BatteryStatus
from std_msgs.msg import Bool
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

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
        self.POSITION_THRESHOLD = 0.5  # meters
        self.VELOCITY_THRESHOLD = 0.1  # m/s
        
        # Initialize state variables
        self.arming_state = 1
        self.nav_state = 0
        self.takeoff_alt = 1.5 # PX4 Down coordinate
        self.current_status = None
        self.current_position = None
        self.battery_status = None
        self.returning = False
        self.mission_complete = False
        self.system_id = None
        self.component_id = None
        self.last_command_time = None

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

    def _local_position_callback(self, msg):
        """Callback for local position updates"""
        self.current_position = msg
        
        

    def _has_reached_position(self, target_pos, current_pos):
        """Check if we've reached a target position"""
        if not self.current_position or not self.current_position.xy_valid:
            return False
            
        # Calculate distance to target
        dx = current_pos['x'] - target_pos['x']
        dy = current_pos['y'] - target_pos['y']
        distance = (dx * dx + dy * dy) ** 0.5
        
        # Check if we're close enough and moving slowly
        return (distance < self.POSITION_THRESHOLD and 
                abs(self.current_position.vx) < self.VELOCITY_THRESHOLD and
                abs(self.current_position.vy) < self.VELOCITY_THRESHOLD)

    def _battery_status_callback(self, msg):
        """Callback for battery status updates"""
        self.battery_status = msg

    def update(self):
        """Periodic update function"""
        if self.current_status is None:
            self.get_logger().warn('No status updates received yet')
            return



        # Safety checks
        if self.current_status.failsafe:
            self.get_logger().error('Failsafe triggered!')
            return
        if self.battery_status is not None and self.battery_status.remaining < self.MIN_BATTERY_LEVEL:
            self.get_logger().warn('Low battery!')
            self._handle_low_battery()

        # State machine updates with error checking
        try:
            if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self.follow_leader()
            elif self.is_leader_drone() and self.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_MISSION:
                self.publish_leader_pose()
            
            
        except Exception as e:
            self.get_logger().error(f'Error in state machine update: {str(e)}')
    def follow_leader(self):
        """Follow the leader drone"""
        if self.current_status is None:
            self.get_logger().error('Cannot follow leader: No status available')
            return

    def publish_leader_pose(self):
        """Publish the leader drone's pose"""
        if self.current_status is None:
            self.get_logger().error('Cannot publish leader pose: No status available')
            return

    def _status_callback(self, msg):
        """Callback for vehicle status updates"""
        self.current_status = msg
        
        # Update system and component IDs if not set
        if self.system_id is None or self.component_id is None:
            self.system_id = msg.system_id
            self.component_id = msg.component_id
            self.get_logger().info(f'Updated IDs - System: {self.system_id}, Component: {self.component_id}')
        
        self.arming_state = msg.arming_state
        self.nav_state = msg.nav_state

    def _arm_command_callback(self, msg):
        """Handle arm/disarm commands"""
        if self.current_status is None:
            self.get_logger().error('Cannot process arm command: No status available')
            return

        command_type = "arm" if msg.data else "disarm"
        self.get_logger().info(f'Received {command_type} command')
        
        # Safety check for arming
        if msg.data and not self._pre_arm_check():
            self.get_logger().error('Pre-arm check failed')
            return
            
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=1.0 if msg.data else 0.0)

    def _launch_command_callback(self, msg):
        """Handle launch commands"""
        if self.current_status is None:
            self.get_logger().error('Cannot process launch command: No status available')
            return

        if msg.data and self.arming_state == VehicleStatus.ARMING_STATE_ARMED:
            self.get_logger().info('Received launch command')
            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF,
                param7=self.takeoff_alt
            )

    def _full_launch_command_callback(self, msg):
        """Handle launch commands"""
        if self.current_status is None:
            self.get_logger().error('Cannot process full launch command: No status available')
            return

        if not msg.data:
            self.get_logger().error('No valid data received')
            return 

        self.get_logger().info('Received full launch command')

        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=1.0 )
        sleep(2)
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF,
            param7=self.takeoff_alt
        )
        sleep(2)
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=0.0 )
        sleep(2)
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=1.0 )

    def _return_command_callback(self, msg):
        """Handle return commands"""
        if self.current_status is None:
            self.get_logger().error('Cannot process return command: No status available')
            return

        if msg.data:
            self.get_logger().info('Received return command')
            self.returning = True
            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH,
            )

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0,param3=0.0, param4=0.0,param5=0.0, param6=0.0,param7=0.0):
        """Helper method to publish vehicle commands"""
        

        if self.current_status is None:
            self.get_logger().error('Cannot send command: No status available')
            return

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
        self.last_command_time = self.get_clock().now()
        
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

    def _handle_failsafe(self):
        """Handle failsafe conditions"""
        self.get_logger().error('Failsafe triggered - Initiating emergency landing')
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_NAV_LAND,
            param1=0.0
        )

    def _handle_low_battery(self):
        """Handle low battery conditions"""
        if not self.returning:
            self.get_logger().warn('Low battery - Initiating return to launch')
            self.returning = True
            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH,
                param1=self.takeoff_alt
            )

    def is_leader_drone(self):
        """Check if this drone is the leader"""
        return self.drone_number == 1

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
