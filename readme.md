
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
└── SITL/
    ├── automation_scripts/
    │   ├── init.sh
    │   ├── launch.sh
    │   ├── start_system.sh
    │   ├── command_drone.sh
    │   └── mission_upload.py
    ├── missions/
    │   └── simple_mission.plan
    └── QGroundControl.AppImage (must be placed manually)
```

## 🚀 Getting Started

> **Prerequisite:** Make sure `QGroundControl.AppImage` is placed inside the `SITL/` folder.
> To install the AppImage file refer to: https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html

### Cloning and building

```bash
git clone --recursive https://github.com/ahmedmsalah99/Leader-Follower-PX4-Simulation.git
cd Leader-Follower-PX4-Simulation/ros2_ws
colcon build
```



### 1. Initialize GCS and Micro XRCE-DDS

This sets up QGroundControl and MAVLink to ROS2 bridge:

```bash
./init.sh
```

### 2. Launch PX4 Gazebo Simulation

This starts a clean simulation with the desired number of drones:

```bash
cd SITL/automation_scripts
export PX4_PATH=<your-px4-folder-path>
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
cd SITL/automation_scripts
./start_system.sh -n <drones_num>
```

> Drones are assigned priorities based on their number (modifiable in the script).

### 4. Command the Swarm

Once the leader is elected, control the drones interactively:

```bash
cd SITL/automation_scripts
./command_drone.sh <drones_num>
```

| Command | Effect |
|---|---|
| `takeoff [N]` | Arm and take off (1.5 m above home), retrying until PX4 confirms |
| `land [N]` | Land |
| `return [N]` | Return to launch |
| `mission [plan_file]` | Upload a QGroundControl `.plan` (default `SITL/missions/simple_mission.plan`) to the current leader and start it |
| `exit` | Quit |

Without a drone number, `takeoff`, `land` and `return` go to all drones. Once airborne, the followers
switch to offboard mode and follow the leader, so `mission` moves the whole swarm. Only simple waypoint
items are supported in `.plan` files (no survey/complex items).

## 📋 Typical Usage Flow

1. Run `init.sh`
2. Launch the simulation with `launch.sh -n 3` (or any number)
3. Start the ROS2 control system with `start_system.sh -n 3`
4. Wait for system to initialize and elect the leader
5. Run `command_drone.sh 3`, then `takeoff`
6. Run `mission` – the leader flies the plan and the followers follow it in real time


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
