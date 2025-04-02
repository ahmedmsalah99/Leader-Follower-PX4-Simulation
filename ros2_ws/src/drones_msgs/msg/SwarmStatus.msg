# Status message for the entire swarm

# Leader information
uint8 leader_id

# Overall swarm metrics
uint8 swarm_size                  # Total number of drones detected
uint8 swarm_active_drones_num     # Number of drones currently active (not timed out)
uint8 swarm_participants_num      # Number of drones participating in swarm
uint8[] swarm_participants_ids    # IDs of drones participating in swarm

# Individual drone status
DroneStatus[] drones_status

