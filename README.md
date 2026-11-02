F1TENTH Follow the Gap
Part 1: Run it on the car

Every command is meant to be pasted as-is.

1.1 One-time setup

Copy the packages into the car's workspace and build:
conda deactivate
cd ~
git clone https://github.com/sree-and-the-bobots/Running-the-F1TENTH-car.git f1tenth_share
cp -r ~/f1tenth_share/follow_the_gap ~/f1tenth_share/wall_follow ~/f1tenth_ws/src/
cp ~/f1tenth_share/params/gap_params_circle.yaml ~/gap_params_circle.yaml
source /opt/ros/foxy/setup.bash
cd ~/f1tenth_ws
colcon build --packages-select follow_the_gap wall_follow

Controller: Logitech F710 in D mode. Deadman buttons: LB = manual, RB = autonomous.

2. Clear Leftovers:
pkill -f reactive_node; pkill -f wall_follow_node; pkill -f bringup_launch; pkill -9 -f f1tenth_ws/install

Have three terminal windows: 
Window 1: base stack
conda deactivate
source /opt/ros/foxy/setup.bash
cd ~/f1tenth_ws
source install/setup.bash
ros2 launch f1tenth_stack bringup_launch.py

Window 2: Follow the Gap (main)
conda deactivate
source /opt/ros/foxy/setup.bash
cd ~/f1tenth_ws
source install/setup.bash
ros2 run follow_the_gap reactive_node --ros-args --params-file ~/gap_params_circle.yaml


Params used on the car
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



Part 2: Run the same code in the simulator (For Gabe)

The sim publishes /scan and listens on /drive, the same topics the car uses, so the same node runs unchanged.

2.1 Install the sim (Ubuntu 22.04 + ROS 2 Humble)
bash
sudo apt update && sudo apt install -y python3-pip
cd $HOME
git clone https://github.com/f1tenth/f1tenth_gym
cd f1tenth_gym && pip3 install -e .
mkdir -p $HOME/sim_ws/src && cd $HOME/sim_ws/src
git clone https://github.com/f1tenth/f1tenth_gym_ros

Edit ~/sim_ws/src/f1tenth_gym_ros/config/sim.yaml and set map_path to your full home path, for example /home/<you>/sim_ws/src/f1tenth_gym_ros/maps/levine.

bash
cd $HOME/sim_ws
source /opt/ros/humble/setup.bash
sudo rosdep init; rosdep update --include-eol-distros
rosdep install -i --from-path src --rosdistro humble -y
colcon build

If the car or laser points do not appear: python3 -m pip install --user -U "coverage>=7.6.1".

2.2 Add this repo's packages and build
bash
cd $HOME
git clone https://github.com/sree-and-the-bobots/Running-the-F1TENTH-car.git f1tenth_share
cp -r ~/f1tenth_share/follow_the_gap ~/f1tenth_share/wall_follow ~/sim_ws/src/
cd ~/sim_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select follow_the_gap wall_follow
2.3 Run

Terminal 1: the sim

bash
source /opt/ros/humble/setup.bash
source ~/sim_ws/install/local_setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py

Terminal 2: Follow the Gap

bash
source /opt/ros/humble/setup.bash
source ~/sim_ws/install/local_setup.bash
ros2 run follow_the_gap reactive_node --ros-args --params-file ~/f1tenth_share/params/gap_params_circle.yaml -p steer_sign:=1.0

Or the wall follower:

bash
ros2 run wall_follow wall_follow_node --ros-args -p wall_side:=right -p speed:=1.0
2.4 Differences from the real car
Steering sign: the car needs steer_sign: -1.0. The sim probably needs 1.0 (set above on the command line). If the sim car steers the wrong way, flip it.
No joystick or deadman in the sim: /drive goes straight to the sim car.
ROS version: car is Foxy, sim is Humble. The code is plain rclpy, but it has not been tested on Humble.
Speed: the sim uses m/s, the real car's VESC speed scale differs, so tune each separately.
Map: the default Levine map is a hallway. Use a map with a loop to test the circle behavior.
