# F1TENTH Follow the Gap

Follow the Gap and wall following for the research group F1TENTH car (Jetson AGX Xavier, Ubuntu 20.04, ROS 2 Foxy). The same code also runs in the F1TENTH Gym simulator.

Repo layout:

- `follow_the_gap/`: main autonomous node
- `wall_follow/`: PID wall follower (backup)
- `params/gap_params_circle.yaml`: parameters used for the circle test

---

## Part 1: Run it on the car

### 1.1 One-time setup

```bash
conda deactivate
cd ~
rm -rf f1tenth_share
git clone https://github.com/sree-and-the-bobots/Running-the-F1TENTH-car.git f1tenth_share
ls f1tenth_share
```

`ls` must list `follow_the_gap`, `wall_follow`, `params` and `README.md`. Then:

```bash
cp -r ~/f1tenth_share/follow_the_gap ~/f1tenth_share/wall_follow ~/f1tenth_ws/src/
cp ~/f1tenth_share/params/gap_params_circle.yaml ~/gap_params_circle.yaml
source /opt/ros/foxy/setup.bash
cd ~/f1tenth_ws
colcon build --packages-select follow_the_gap wall_follow
```

The build must end with `Summary: 2 packages finished`.

After every reboot, give the Xavier its LIDAR address and check the LIDAR answers:

```bash
sudo ip addr add 192.168.0.15/24 dev eth0
ping -c 3 192.168.0.10
```

Controller: Logitech F710 in **D mode**. Deadman buttons: **LB** = manual drive, **RB** = autonomous. Release the button and the car stops.

### 1.2 Start the car

Put the car on a stand with the wheels off the ground. Stop programs with Ctrl+C, never Ctrl+Z.

Step 1. Clear leftovers:

```bash
pkill -f reactive_node; pkill -f wall_follow_node; pkill -f bringup_launch; pkill -9 -f f1tenth_ws/install
```

Step 2. Window 1, the car's base stack:

```bash
conda deactivate
source /opt/ros/foxy/setup.bash
cd ~/f1tenth_ws
source install/setup.bash
ros2 launch f1tenth_stack bringup_launch.py
```

Wait until it shows `Opened joystick` and the LIDAR `Streaming data`, with no `process has died`.

Step 3. Window 2, Follow the Gap (main):

```bash
conda deactivate
source /opt/ros/foxy/setup.bash
cd ~/f1tenth_ws
source install/setup.bash
ros2 run follow_the_gap reactive_node --ros-args --params-file ~/gap_params_circle.yaml
```

Or the wall follower instead (never run both at once):

```bash
ros2 run wall_follow wall_follow_node --ros-args -p wall_side:=right -p speed:=0.3 -p front_stop_distance:=1.0
```

Step 4. Hold **RB** to drive autonomously. Release RB to stop. To shut down, Ctrl+C in each window.

### 1.3 Params used on the car

`params/gap_params_circle.yaml`:

```yaml
reactive_node:
  ros__parameters:
    steer_sign: -1.0
    reverse_enable: false
    max_range: 3.0
    slow_zone: 2.0
    half_width: 0.2
    steer_gain: 1.0
    steer_alpha: 0.7
    speed_min: 0.5
    speed_max: 0.7
    hard_stop_distance: 0.35
    stop_distance: 0.45
    ignore_below: 0.35
    center_gain: 0.3
```

`steer_sign: -1.0` is required on this car. Delete the `center_gain` line if the code has no such parameter.

Change a value while it runs, without restarting:

```bash
ros2 param set /reactive_node speed_max 0.6
```

### 1.4 Troubleshooting (car)

| Symptom | Fix |
| --- | --- |
| `Could not open network Hokuyo` | Re-add the eth0 address in 1.1, then `pkill -9 -f f1tenth_ws/install` |
| `Error opening YAML file` | The params file path does not exist |
| `joy_teleop` died | Use whole numbers for `axis` and `device_id` in `joy_teleop.yaml` |
| Car turns into walls | `steer_sign` must be `-1.0` on the real car |
| Config edit has no effect | Rebuild with `colcon build --packages-select f1tenth_stack` and relaunch |

---

## Part 2: Run the same code in the simulator

The sim publishes `/scan` and listens on `/drive`, the same topics the car uses, so the same node runs unchanged. This assumes the F1TENTH Gym sim is already installed in `~/sim_ws` (Ubuntu 22.04, ROS 2 Humble). If your workspace has another name, use that name wherever you see `sim_ws`.

### 2.1 Get the code

```bash
cd $HOME
rm -rf f1tenth_share
git clone https://github.com/sree-and-the-bobots/Running-the-F1TENTH-car.git f1tenth_share
ls f1tenth_share
```

`ls` must list `follow_the_gap`, `wall_follow`, `params` and `README.md`. If it does not, stop and tell Sree the repo is missing files.

### 2.2 Copy and build

```bash
cp -r ~/f1tenth_share/follow_the_gap ~/f1tenth_share/wall_follow ~/sim_ws/src/
ls ~/sim_ws/src
cd ~/sim_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select follow_the_gap wall_follow
```

The `ls` must show `follow_the_gap` and `wall_follow` next to `f1tenth_gym_ros`. The build must end with `Summary: 2 packages finished`.

Check ROS can see the package:

```bash
source ~/sim_ws/install/setup.bash
ros2 pkg list | grep follow
```

It must print `follow_the_gap`. If it does not, fix this before moving on.

### 2.3 Run (two terminals)

Every new terminal needs both `source` lines first.

Terminal 1, the sim:

```bash
source /opt/ros/humble/setup.bash
source ~/sim_ws/install/setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

Wait a few seconds for the sim to load.

Terminal 2, Follow the Gap:

```bash
source /opt/ros/humble/setup.bash
source ~/sim_ws/install/setup.bash
ros2 run follow_the_gap reactive_node --ros-args --params-file ~/f1tenth_share/params/gap_params_circle.yaml -p steer_sign:=1.0
```

Or the wall follower:

```bash
ros2 run wall_follow wall_follow_node --ros-args -p wall_side:=right -p speed:=1.0
```

### 2.4 Troubleshooting (sim)

| Symptom | Fix |
| --- | --- |
| `package 'follow_the_gap' not found` | Run both `source` lines in this terminal. If it still fails, redo 2.2 and check `ls ~/sim_ws/src` and the build summary |
| `git clone` says folder already exists | Run `rm -rf ~/f1tenth_share` and clone again |
| Nothing moves | In a third terminal run `ros2 node list` (expect `/reactive_node`), `ros2 topic echo /scan --once` (expect numbers) and `ros2 topic echo /drive` (expect speed and steering values) |
| No car or laser points in RViz | Add a LaserScan display on `/scan`. If the sim shows no car, run `python3 -m pip install --user -U "coverage>=7.6.1"` |
| Car steers away from the gap | Change `steer_sign:=1.0` to `steer_sign:=-1.0` |

### 2.5 Differences from the real car

- **Steering sign:** the car needs `steer_sign: -1.0`. The sim probably needs `1.0`. If the sim car steers the wrong way, flip it.
- **No joystick or deadman** in the sim: `/drive` goes straight to the sim car.
- **ROS version:** the car runs Foxy and the sim runs Humble. The code is plain `rclpy`, but it has not been tested on Humble.
- **Speed:** the sim uses m/s, and the real car's VESC speed scale differs, so tune each separately.
- **Map:** the default Levine map is a hallway. Use a map with a loop to test the circle behavior.
