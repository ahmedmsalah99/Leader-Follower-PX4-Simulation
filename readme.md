
---

# Leader-Follower-PX4-Simulation

This project simulates a simple leader-follower drone swarm using **PX4**, **ROS2**, **pymavlink**, and **Gazebo SITL**. It implements a dynamic leader election system using the **bully algorithm**, enabling drones to elect a new leader when the current one times out or at the simulation start.

## 🧠 Overview

- **Swarm simulation** in PX4/Gazebo SITL
- **Leader election** using the bully algorithm
- **ROS2 nodes** handle swarm monitoring and control
- **MAVLink to ROS2 bridging** via Micro XRCE-DDS
- **Mission upload and follow behavior**

## 📊 Example Results

Below is a demonstration of the leader-follower swarm behavior in action. The leader (elected automatically) moves along a mission path, and the follower drones track and follow its movements in offboard mode.

<p align="center">
  <img src="resources/leader_follower2.gif" alt="Leader-Follower Simulation" width="600"/>
</p>

## 🧰 Project Structure

```
Leader-Follower-PX4-Simulation/
├── command_drone.sh
├── automation_scripts/
│   ├── init.sh
│   ├── launch.sh
│   ├── start_system.sh
├── test_mission.py
└── SITL/
    └── QGroundControl.AppImage (must be placed manually)
```

## 🚀 Getting Started

> **Prerequisite:** Make sure `QGroundControl.AppImage` is placed inside the `SITL/` folder.
> cd to the automation_scripts folder

### 1. Initialize GCS and Micro XRCE-DDS

This sets up QGroundControl and MAVLink to ROS2 bridge:

```bash
./init.sh
```

### 2. Launch PX4 Gazebo Simulation

This starts a clean simulation with the desired number of drones:

```bash
./launch.sh -n <drones_num> [-h]
```

- `-n <drones_num>`: Number of drones to simulate
- `-h`: Headless mode (no GUI, uses fewer resources)

### 3. Start the ROS2 System

Starts ROS2 nodes that:
- Monitor the swarm
- Control each drone
- Handle leader election

```bash
./start_system.sh -n <drones_num>
```

> Drones are assigned priorities based on their number (modifiable in the script).

### 4. Command the Swarm

Once drones are hovering and the leader is elected, you can send commands:

```bash
./command_drone.sh
```

## 📋 Typical Usage Flow

1. Run `init.sh`
2. Launch the simulation with `launch.sh -n 3` (or any number)
3. Start the ROS2 control system with `start_system.sh -n 3`
4. Wait for system to initialize and elect the leader
5. Upload a mission to the leader drone using QGroundControl
6. Start the mission – follower drones will follow the leader in real time

## 🧪 Additinal Script

You can also use `test_mission.py` to download a mission from drone 1 and upload it to 2 


## 🛠 Notes

- Tested with **Gazebo Harmonic**
- QGroundControl is launched via AppImage; ensure it exists in `SITL/`
- Leader election is automatic, but the automation script can be modified to use manual priorities

---

### 📩 Contributions

Pull requests and suggestions are welcome!

---

### 📜 License

[MIT License](LICENSE)
```
