import socket
import time
import threading
from pymavlink import mavutil
import rclpy
from rclpy.node import Node
class ElectionNode(Node):
    def __init__(self):
        super().__init__('drone_node')
        # Parameters
        
        self.declare_parameter('drone_id', 0)
        self.declare_parameter('priority', 0)
        self.drone_id = self.get_parameter('drone_id').value
        self.priority = self.get_parameter('priority').value
      
        # MAVLink connection
        
        self.udp_out = mavutil.mavlink_connection('udpout:239.255.1.1:14550',source_system=self.drone_id)
        self.udp_in = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
        self.udp_in.port.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.udp_in.port.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, 
                        socket.inet_aton("239.255.1.1") + socket.inet_aton("0.0.0.0"))
        self.boot_time = time.time()
        self.leader_heartbeat_time = time.time()
        self.system_id = self.udp_out.source_system
        self.component_id = self.udp_out.source_component
        self.get_logger().info(f'{self.system_id} , {self.component_id}')
        # State
        self.leader_id = None
        self.is_leader = False

        # Start listening thread
        self.listen_thread = threading.Thread(target=self.listen_for_messages)
        self.listen_thread.daemon = True
        self.listen_thread.start()

        self.leader_potential_showed_up = False

        time.sleep(1)
        self.start_election()

    def send_mavlink_message(self, message_type, **kwargs):
        """Helper function to send MAVLink messages"""
        if message_type == "ELECTION":
            self.udp_out.mav.named_value_int_send(
                int(time.time()-self.boot_time)*1000,  # Timestamp
                b"ELECTION",
                (self.priority << 16) | self.drone_id  # Encode priority & ID
            )
        elif message_type == "LEADER":
            self.udp_out.mav.statustext_send(
                mavutil.mavlink.MAV_SEVERITY_NOTICE,
                f"LEADER {self.drone_id}".encode()
            )

    def start_election(self):
        if not self.leader_potential_showed_up:
            self.get_logger().info(f"Drone {self.drone_id} initiating election")
            self.send_election()

    def send_election(self):
        """Broadcast election message"""
        self.send_mavlink_message("ELECTION")
        self.get_logger().info(f"Drone {self.drone_id} sent ELECTION message")

        # Wait for responses in a separate thread
        threading.Thread(target=self.wait_for_responses).start()

    def wait_for_responses(self):
        """Wait for a certain time to see if a higher-priority drone responds"""
        time.sleep(3)
        if not self.leader_potential_showed_up:
            self.declare_leader()

    def declare_leader(self):
        """Declare itself as the leader"""
        self.is_leader = True
        self.leader_id = self.drone_id
        self.send_mavlink_message("LEADER")
        self.get_logger().info(f"Drone {self.drone_id} declared itself as leader")

    def listen_for_messages(self):
        """Continuously listen for incoming MAVLink messages"""
        while True:
            if self.leader_id:
                self.udp_out.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_QUADROTOR,
                                                mavutil.mavlink.MAV_AUTOPILOT_PX4, mavutil.mavlink.MAV_MODE_FLAG_AUTO_ENABLED,
                                                  0, mavutil.mavlink.MAV_STATE_ACTIVE)
            else:
                # if leader is absent start election
                if (self.leader_id and not self.leader_potential_showed_up) and time.time() - self.leader_heartbeat_time > 1:
                    self.start_election()
            
            msg = self.udp_in.recv_match(blocking=False)
            self.handle_msg(msg)
            
            
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
        if msg.system_status == mavutil.mavlink.MAV_STATE_STANDBY or msg.system_status == mavutil.mavlink.MAV_STATE_ACTIVE:
            self.get_logger().info(f"leader active")
            self.leader_heartbeat_time = time.time()

    def handle_leader_msg(self,msg):
        leader_id = int(msg.text.split()[1])
        self.leader_id = leader_id
        self.leader_potential_showed_up = False
        self.get_logger().info(f"Drone {self.drone_id} acknowledges Drone {leader_id} as leader")

    def handle_election_msg(self,msg):
        sender_priority = (msg.value >> 16) & 0xFFFF
        sender_id = msg.value & 0xFFFF
        self.get_logger().info(f"Drone {self.drone_id} received election msg from {sender_id}")
        if sender_priority > self.priority or (sender_priority == self.priority and sender_id > self.drone_id):
            self.leader_potential_showed_up = True
            self.get_logger().info(f"Drone {self.drone_id} received ELECTION from higher priority Drone {sender_id}")
    
        

def main(args=None):
    rclpy.init(args=args)
    node = ElectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()




















