from functools import wraps
import socket
import time
import threading
from pymavlink import mavutil
import rclpy
from rclpy.node import Node
from drones_msgs.srv import UnsetLeader
from drones_msgs.msg import Leaderpos,SwarmStatus
from px4_msgs.msg import VehicleLocalPosition  ,VehicleStatus
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from std_msgs.msg import Empty

class ElectionNode(Node):
    def __init__(self):
        super().__init__('drone_node')
        # Parameters
        self.declare_parameter('drone_id', 0)
        self.declare_parameter('priority', 0)
        self.declare_parameter('cluster_drones_number',0)
        self.drone_id = self.get_parameter('drone_id').value
        self.priority = self.get_parameter('priority').value
        self.cluster_drones_number = self.get_parameter('cluster_drones_number').value

        self.namespace = f'/px4_{self.drone_id}'
        # Configure QoS profile
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        
        self.current_status = None
        self.boot_time = time.time()
        self.leader_heartbeat_time = None
        self.system_id = None
        self.component_id = None
        self.udp_out = None
        self.udp_in = None
        self.current_position = None
        self.leader_id = None
        self.leader_potential_showed_up = False
        self.is_swarm_participant = True
        self.started_election = False
        # Publishers
        self.leader_pose_publisher = self.create_publisher(
            Leaderpos, '/global/leader_pose', 10)

        # Services
        self.unset_leader_service = self.create_service(
            UnsetLeader, 'unset_leader', self.unset_leader_callback)

        # Subscribers
        self.position_sub = self.create_subscription(
            VehicleLocalPosition,
            f'{self.namespace}/fmu/out/vehicle_local_position',
            self._local_position_callback,
            qos_profile
        )

        self.status_sub = self.create_subscription(
            VehicleStatus, f'{self.namespace}/fmu/out/vehicle_status_v1', 
            self._status_callback, qos_profile)
        
        self.start_election_sub = self.create_subscription(
            Empty, f'/global/start_election', 
            self._start_election_callback,10)

        self.start_election_sub = self.create_subscription(
            SwarmStatus, f'/global/swarm_status', 
            self._check_election_need,10)

    def check_status(func):
        """Wrapper that checks if self.current_status exists before executing the function."""
        @wraps(func)
        def wrapper(self, *args, **kwargs):
            if self.current_status is None:
                self.get_logger().warn('No status updates received yet')
                return None  # or raise an exception if preferred
            return func(self, *args, **kwargs)
        return wrapper
    
    @check_status
    def update(self):
        """Periodic update function"""
        if not self.is_swarm_participant:
            return
        
        # receive and handle messages
        msg = self.udp_in.recv_match(blocking=False)
        self.handle_msg(msg)
        
        ## manage leader
        if self.leader_id is None:
            return
        
        if self.is_leader():
            self.leader_work()
        else:
            if self.leader_heartbeat_time is not None and time.time() - self.leader_heartbeat_time > 1:
                # time out, no leader
                self.leader_id = None
                self.start_election()
        
    def is_leader(self):
        return self.leader_id is not None and self.leader_id == self.system_id
    
    def leader_work(self):
        if self.current_position is not None:
            self.leader_pose_publisher.publish(Leaderpos(timestamp=self.get_clock().now().nanoseconds // 1000, 
                                                                  drone_num=self.drone_id,
                                                                  system_id=self.system_id,
                                                                  component_id=self.component_id,
                                                                    pose=self.current_position))
    @check_status
    def _check_election_need(self,msg):
        if self.leader_id == None and msg.swarm_active_drones_num == self.cluster_drones_number:
            self.start_election()

    def _status_callback(self, msg):
        """Callback for vehicle status updates"""
        self.current_status = msg
        
        # Update system and component IDs if not set
        if self.system_id is None or self.component_id is None:
            self.system_id = msg.system_id
            self.component_id = msg.component_id
            self.get_logger().info(f'Updated IDs - System: {self.system_id}, Component: {self.component_id}')
            self.initialize_comms()
            self.create_timer(0.1, self.update)  # 10Hz update rate
            

    def initialize_comms(self):# MAVLink connection
        self.udp_out = mavutil.mavlink_connection('udpout:239.255.0.7:14551',source_system=self.system_id)
        self.udp_in = mavutil.mavlink_connection('udpin:0.0.0.0:14551')
        self.udp_in.port.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.udp_in.port.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, 
                        socket.inet_aton("239.255.0.7") + socket.inet_aton("0.0.0.0"))
    
    def _local_position_callback(self, msg):
        """Callback for local position updates"""
        self.current_position = msg
    
    def unset_leader_callback(self, request, response):
        """Service callback to unset the current leader"""
        if self.is_leader() and request.leader_id == self.system_id:
            self.leader_id = None
            self.is_swarm_participant = False
            self.get_logger().info('Leader status removed')
            response.success = True
        else:
            self.get_logger().warn('Not currently the leader')
            response.success = False
        return response
        
    @check_status
    def send_mavlink_message(self, message_type, **kwargs):
        """Helper function to send MAVLink messages"""
        if message_type == "ELECTION":
            self.udp_out.mav.named_value_int_send(
                int(time.time()-self.boot_time)*1000,  # Timestamp
                b"ELECTION",
                (self.priority << 16) | self.system_id  # Encode priority & ID
            )
        elif message_type == "LEADER":
            self.udp_out.mav.statustext_send(
                mavutil.mavlink.MAV_SEVERITY_NOTICE,
                f"LEADER {self.system_id}".encode()
            )
    def _start_election_callback(self,msg):
        self.start_election()

    def start_election(self):
        if not self.leader_potential_showed_up and not self.started_election:
            self.get_logger().info(f"Drone {self.system_id} initiating election")
            self.send_election()
            self.started_election = True


    def send_election(self):
        """Broadcast election message"""
        self.send_mavlink_message("ELECTION")
        self.get_logger().info(f"Drone {self.system_id} sent ELECTION message")
        # Wait for responses in a separate thread
        threading.Thread(target=self.wait_for_responses).start()

    def wait_for_responses(self):
        """Wait for a certain time to see if a higher-priority drone responds"""
        time.sleep(5)
        if self.is_swarm_participant and not self.leader_potential_showed_up:
            self.declare_leader()
        time.sleep(5)
        self.started_election = False

    def declare_leader(self):
        """Declare itself as the leader"""
        self.leader_id = self.system_id
        self.send_mavlink_message("LEADER")
        self.get_logger().info(f"Drone {self.system_id} declared itself as leader")
            
    def handle_msg(self,msg):
        if not msg:
            return
        t = msg.get_type()
        match t:
            case "NAMED_VALUE_INT":
                if msg.name == "ELECTION":
                    self.handle_election_msg(msg)
            case "STATUSTEXT":
                if "LEADER" in msg.text:
                    self.handle_leader_msg(msg)
            case "HEARTBEAT":
                self.handle_heart_beat_msg(msg)
    
    def handle_heart_beat_msg(self,msg):
        if self.leader_id is None or msg.get_srcSystem() != self.leader_id:
            return
        if msg.system_status == mavutil.mavlink.MAV_STATE_STANDBY or msg.system_status == mavutil.mavlink.MAV_STATE_ACTIVE:
            self.get_logger().info(f"leader {self.leader_id} active",throttle_duration_sec=1)
            self.leader_heartbeat_time = time.time()

    def handle_leader_msg(self,msg):
        leader_id = int(msg.text.split()[1])
        self.leader_id = leader_id
        self.leader_potential_showed_up = False
        self.get_logger().info(f"Drone {self.system_id} acknowledges Drone {leader_id} as leader")

    def handle_election_msg(self,msg):
        sender_priority = (msg.value >> 16) & 0xFFFF
        sender_id = msg.value & 0xFFFF
        self.get_logger().info(f"Drone {self.system_id} received election msg from {sender_id}")
        if sender_priority > self.priority or (sender_priority == self.priority and sender_id > self.system_id):
            self.leader_potential_showed_up = True
            self.get_logger().info(f"Drone {self.system_id} received ELECTION from higher priority Drone {sender_id}")
    
        

def main(args=None):
    rclpy.init(args=args)
    node = ElectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()




















