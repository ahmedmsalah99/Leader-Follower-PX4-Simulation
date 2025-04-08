from pymavlink import mavutil
import time
import math

class MissionTransfer:
    def __init__(self):
        # Connections
        self.server_conn = None
        self.client_conn = None
        self.timeout = 5  # seconds
        
    def connect_drones(self):
        """Connect to both drones"""
        print("Connecting to server drone...")
        self.server_conn = mavutil.mavlink_connection('udp:127.0.0.1:14541',source_system=1,source_component=1)
        self.server_conn.wait_heartbeat()
        print("server target system",self.server_conn.target_system)
        while self.server_conn.target_system == 0:
            self.server_conn = mavutil.mavlink_connection('udp:127.0.0.1:14541',source_system=1,source_component=1)
            self.server_conn.wait_heartbeat()
            print("server target system",self.server_conn.target_system)
        

        print("Connecting to client drone...")
        self.client_conn = mavutil.mavlink_connection('udp:127.0.0.1:14542',source_system=1,source_component=1)
        self.client_conn.wait_heartbeat()
        print("server client system",self.client_conn.target_system)
        while self.client_conn.target_system == 0:
            self.client_conn = mavutil.mavlink_connection('udp:127.0.0.1:14542',source_system=1,source_component=1)
            self.client_conn.wait_heartbeat()
            print("server client system",self.client_conn.target_system)
        
    def get_current_mission_item(self):
        """Get server's current mission sequence number"""
        self.server_conn.mav.mission_request_int_send(
            self.server_conn.target_system,
            self.server_conn.target_component,
            -1  # Special value for current item
        )
        
        msg = self.server_conn.recv_match(type='MISSION_CURRENT', 
                                        blocking=True,
                                        timeout=self.timeout)
        return msg.seq if msg else None
    
    def download_full_mission(self):
        """Download complete mission from server"""
        # self.client_conn.mav.mission_clear_all_send(
        #     self.client_conn.target_system,
        #     self.client_conn.target_component
        # )
        # print('sent clear')
        # Request mission count
        self.server_conn.mav.mission_request_list_send(
            self.server_conn.target_system,
            0
        )
        print(self.server_conn.target_component)
        
        count_msg = self.server_conn.recv_match(type='MISSION_COUNT', 
                                              blocking=True,
                                              timeout=self.timeout)
        if not count_msg:
            raise Exception("Failed to get mission count")
            
        mission_items = []
        
        # Download each item
        for seq in range(count_msg.count):
            self.server_conn.mav.mission_request_int_send(
                self.server_conn.target_system,
                self.server_conn.target_component,
                seq
            )
            
            item = self.server_conn.recv_match(type='MISSION_ITEM_INT',
                                            blocking=True,
                                            timeout=self.timeout)
            if not item:
                self.server_conn.mav.mission_ack_send(
                    self.server_conn.target_system,
                    self.server_conn.target_component,
                    1 # acceptedf
                )
                raise Exception(f"Failed to get mission item {seq}")
                
            mission_items.append(item)
            if item.current == 1:
                item_idx = seq
        self.server_conn.mav.mission_ack_send(
            self.server_conn.target_system,
            self.server_conn.target_component,
            0 # accepted
        )
        return mission_items, item_idx
    
    def upload_to_client(self, mission_items, start_seq):
        """Upload mission to client drone and start from specific sequence"""
        # Clear client mission first
        self.client_conn.mav.mission_clear_all_send(
            self.client_conn.target_system,
            self.client_conn.target_component
        )
        print('sent clear')
        
        # Send mission count
        self.client_conn.mav.mission_count_send(
            self.client_conn.target_system,
            1,
            len(mission_items),
            0  # mission type
        )
        print('sent mission count')
        
        # Upload each item
        for seq, item in enumerate(mission_items):
            # Wait for request
            while True:
                req = self.client_conn.recv_match(type='MISSION_REQUEST_INT',
                                                blocking=True,
                                                timeout=self.timeout)
                print('uploading item')
                print(req)
                if req and req.seq == seq:
                    break
                    
            # Send item
            self.client_conn.mav.mission_item_int_send(
                self.client_conn.target_system,
                self.client_conn.target_component,
                seq,
                item.frame,
                item.command,
                1 if seq == start_seq else 0,  # current
                item.autocontinue,
                item.param1,
                item.param2,
                item.param3,
                item.param4,
                item.x,
                item.y,
                item.z,
                item.mission_type
            )
        
        # Verify upload complete
        ack = self.client_conn.recv_match(type='MISSION_ACK',
                                        blocking=True,
                                        timeout=self.timeout)
        if ack.type != 0:  # MAV_MISSION_ACCEPTED
            raise Exception(f"Mission upload failed with error: {ack.type}")
            
    def position_client(self, mission_item):
        """Position client drone at the takeover point"""
        lat = mission_item.x * 1e-7
        lon = mission_item.y * 1e-7
        alt = mission_item.z
        
        print(f"Positioning client to lat:{lat:.6f}, lon:{lon:.6f}, alt:{alt}m")
        
        self.client_conn.mav.command_long_send(
            self.client_conn.target_system,
            self.client_conn.target_component,
            mavutil.mavlink.MAV_CMD_DO_REPOSITION,
            0,
            -1,  # default ground speed
            1,   # change position
            0,   # yaw
            lat, lon, alt,
            0, 0, 0
        )
        
        # Wait for positioning to complete
        while True:
            pos = self.client_conn.recv_match(type='GLOBAL_POSITION_INT',
                                            blocking=True,
                                            timeout=self.timeout)
            current_lat = pos.lat * 1e-7
            current_lon = pos.lon * 1e-7
            current_alt = pos.alt / 1000  # mm to m
            
            error = math.sqrt(
                (current_lat - lat)**2 +
                (current_lon - lon)**2 +
                (current_alt - alt)**2
            )
            
            if error < 1.0:  # Within 1 meter
                break
                
    def start_mission(self, start_seq):
        """Arm client and start mission from specified sequence"""
        # Set current mission item
        self.client_conn.mav.command_long_send(
            self.client_conn.target_system,
            self.client_conn.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MISSION_CURRENT,
            0,
            start_seq, 0, 0, 0, 0, 0, 0
        )
        
        # Arm
        self.client_conn.mav.command_long_send(
            self.client_conn.target_system,
            self.client_conn.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0
        )
        
        # Start mission
        self.client_conn.mav.command_long_send(
            self.client_conn.target_system,
            self.client_conn.target_component,
            mavutil.mavlink.MAV_CMD_MISSION_START,
            0,
            start_seq, 0, 0, 0, 0, 0, 0
        )
        
    def transfer_mission(self):
        """Complete mission transfer workflow"""
        try:
            self.connect_drones()
            
            # Get current position in mission
            # current_seq = self.get_current_mission_item()
            # if current_seq is None:
            #     raise Exception("Could not get current mission item")
                
            # print(f"Server is on mission item {current_seq}")
            
            # Download full mission
            mission_items, current_seq = self.download_full_mission()
            print(f"Downloaded {len(mission_items)} mission items with current item {current_seq}")
            
            # Upload to client
            self.upload_to_client(mission_items, current_seq)
            
            # Position client at takeover point
            # self.position_client(mission_items[current_seq])
            
            # Start mission
            self.start_mission(current_seq)
            print("Mission transfer and startup complete!")
            
        except Exception as e:
            print(f"Error during mission transfer: {str(e)}")
            # Add cleanup code here

if __name__ == "__main__":
    transfer = MissionTransfer()
    transfer.transfer_mission()