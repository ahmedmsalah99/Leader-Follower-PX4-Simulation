# Definition of DroneStatus
uint8 system_id
bool in_swarm
uint64 last_timestamp  
uint8 system_status    # Based on MAV_STATE enum
bool is_active
bool is_timed_out