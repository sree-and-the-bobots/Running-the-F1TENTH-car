# Running-the-F1TENTH-car
Everything needed to run the research group car (Xavier, ROS 2 Foxy): start-up commands, the edits made, the params file to load,.

Start-up order:
Run these every time, in this order. Stop programs with Ctrl+C, never Ctrl+Z (suspended processes keep holding the LIDAR, VESC and joystick).
1. Put the car on a stand with the wheels off the ground. SSH into the Xavier.
2. Clear leftovers:
pkill -f reactive_node; pkill -f wall_follow_node; pkill -f bringup_launch; pkill -9 -f f1tenth_ws/install
