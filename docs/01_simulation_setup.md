# 01 Simulation Setup

## Goal

Run the BlueROV2 simulation and understand the data flow between Gazebo and ROS 2.

## Important Commands

```bash
source /home/ct03/underwater_ws/install/setup.bash
ros2 launch bluerov2_description world_launch.py
ros2 topic list
ros2 topic echo /bluerov2/odom
```

## Key Topics

```text
/bluerov2/odom
/bluerov2/pose_gt
/bluerov2/image
/bluerov2/sonar
/bluerov2/cloud
/bluerov2/cmd_thruster1..6
```

## What To Understand

- `world_launch.py` starts the Gazebo ocean world.
- `upload_bluerov2_launch.py` spawns the robot and creates ROS-Gazebo bridges.
- The robot is controlled through six thruster command topics.

## Interview Notes

- Gazebo provides the physics simulation.
- ROS 2 provides communication, control nodes and experiment tooling.
- ROS-Gazebo bridge converts Gazebo transport messages into ROS 2 topics.
