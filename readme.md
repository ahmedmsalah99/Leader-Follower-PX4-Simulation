# Leader-Follower-PX4-Simulation

A leader-follower drone swarm in **PX4 SITL + Gazebo**, controlled from **ROS 2**. Drones elect a leader with the **bully algorithm** (re-elected if the leader times out); the followers track the leader in offboard mode while it flies a mission.

<p align="center">
  <img src="resources/leader_follower2.gif" alt="Leader-Follower Simulation" width="600"/>
</p>

## Version compatibility

| Component | Version |
|---|---|
| PX4-Autopilot | **v1.17.x** (tested with v1.17.0) |
| `px4_msgs` (submodule `ros2_ws/src/px4_msgs`) | branch **release/1.17** |
| Gazebo | Harmonic |

`px4_msgs` must come from the same PX4 release as your PX4 build. If they don't match, the ROS 2 nodes
get no data from PX4 and keep printing `No status updates received yet`.

## Installation

**1. Clone this repo** and fetch only the submodules it needs:

```bash
git clone https://github.com/ahmedmsalah99/Leader-Follower-PX4-Simulation.git
cd Leader-Follower-PX4-Simulation
git submodule update --init ros2_ws/src/px4_msgs SITL/Micro-XRCE-DDS-Agent
```

**2. Install PX4 v1.17.0** anywhere outside this repo:

```bash
git clone -b v1.17.0 --recursive https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot
bash ./Tools/setup/ubuntu.sh   # installs dependencies and Gazebo, reboot afterwards
make px4_sitl
```

Already have PX4? Switch it to the release and rebuild:

```bash
cd PX4-Autopilot
git fetch --tags && git checkout v1.17.0
git submodule update --init --recursive
make distclean && make px4_sitl
```

**3. Build the Micro XRCE-DDS Agent** (the PX4 ↔ ROS 2 bridge):

```bash
cd SITL/Micro-XRCE-DDS-Agent
mkdir build && cd build
cmake .. && make
sudo make install && sudo ldconfig /usr/local/lib/
```

**4. Build the ROS 2 workspace** (with ROS 2 sourced; `pymavlink` comes with PX4's `ubuntu.sh`):

```bash
cd ros2_ws
colcon build
```

**5. QGroundControl:** download the [AppImage](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html) and place it at `SITL/QGroundControl.AppImage`.

## Usage

Run every script from `SITL/automation_scripts`, each in its own terminal:

```bash
cd SITL/automation_scripts
./init.sh                                   # 1. QGroundControl + Micro XRCE-DDS Agent
PX4_PATH=~/PX4-Autopilot ./launch.sh -n 3   # 2. Gazebo with 3 drones
./start_system.sh -n 3                      # 3. ROS 2 nodes: swarm manager, election, controllers
./command_drone.sh 3                        # 4. interactive commands
```

### `launch.sh` options

| Option | Meaning |
|---|---|
| `-n N` | number of drones (default 1) |
| `-w WORLD` | Gazebo world, any file name in `$PX4_PATH/Tools/simulation/gz/worlds` without `.sdf`, e.g. `default`, `baylands`, `walls`, `windy` (default `default`) |
| `-h` | headless (no Gazebo GUI) |

`PX4_PATH` must point to your PX4-Autopilot folder (`export PX4_PATH=...` once per terminal also works).

### `command_drone.sh` commands

Leave out the drone number `N` to send the command to **all** drones.

| Command | Effect |
|---|---|
| `takeoff [N]` | arm and take off (1.5 m above home) |
| `land [N]` | land |
| `return [N]` | return to launch |
| `mission [plan_file]` | the current leader flies a QGroundControl `.plan` (default `SITL/missions/simple_mission.plan`) |
| `help` / `exit` | show the commands / quit |

Once airborne, followers switch to offboard mode and follow the leader, so `mission` moves the whole swarm.
By default the drone with the highest number becomes the leader.

## Examples

Fly the bundled mission with 3 drones:

```bash
PX4_PATH=~/PX4-Autopilot ./launch.sh -n 3
./start_system.sh -n 3
./command_drone.sh 3
> takeoff
> mission
> land
```

5 drones in the `walls` world without the Gazebo GUI:

```bash
PX4_PATH=~/PX4-Autopilot ./launch.sh -n 5 -w walls -h
./start_system.sh -n 5
```

Command a single drone, or fly your own plan:

```bash
> takeoff 2
> return 2
> mission ~/my_route.plan
```

## Notes

- `simple_mission.plan` uses coordinates near the `default` world origin. Worlds with a different
  location (e.g. `baylands`) need their own plan: draw it in QGroundControl and save it as a `.plan`.
  Only simple waypoint items are supported (no survey/complex items).
- After changing code or `px4_msgs`, run `colcon build` again and re-source `ros2_ws/install/setup.bash`.
- `No status updates received yet` means PX4 data isn't arriving: check that the agent is running
  (`init.sh`) and that `ros2 topic list | grep vehicle_status` shows `/px4_1/fmu/out/vehicle_status_v1`;
  a different name means PX4 and `px4_msgs` versions don't match.

## License

MIT
