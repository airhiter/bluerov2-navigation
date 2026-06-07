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

## Safe Visible Gazebo Workflow

Use this workflow when the default Gazebo GUI is blank, cannot focus `bluerov2`, or has rendering issues in a virtual machine.

The key difference is that Gazebo is started with the OGRE renderer instead of the default OGRE2 renderer.

### Step 1: Clean old processes

Run this before starting a new simulation if old Gazebo / ROS bridge processes may still be alive:

```bash
pkill -f "ign gazebo"
pkill -f "ros_gz_bridge/parameter_bridge"
pkill -f "robot_state_publisher.*bluerov2"
pkill -f "pose_to_tf"
pkill -f "image_bridge"
pkill -f "slider_publisher"
```

It is fine if some commands print nothing.

### Step 2: Terminal 1 - start Gazebo with OGRE

Open Terminal 1 and run:

```bash
cd /home/ct03/underwater_ws
source install/setup.bash
export ROS_LOG_DIR=/tmp/ros_logs

ign gazebo /home/ct03/underwater_ws/install/bluerov2_description/share/bluerov2_description/urdf/demo_world.sdf \
  -r --render-engine-gui ogre --render-engine-server ogre --force-version 6
```

Do not close this terminal.

### Step 3: Terminal 2 - spawn BlueROV2

Open Terminal 2 and run:

```bash
cd /home/ct03/underwater_ws
source install/setup.bash
export ROS_LOG_DIR=/tmp/ros_logs

ros2 launch bluerov2_description upload_bluerov2_launch.py gazebo_world_name:=ocean sliders:=true
```

Wait until the terminal prints:

```text
OK creation of entity
```

### Step 4: Terminal 3 - verify odometry

Open Terminal 3 and run:

```bash
cd /home/ct03/underwater_ws
source install/setup.bash
export ROS_LOG_DIR=/tmp/ros_logs

ros2 topic echo --once /bluerov2/odom
```

If odometry prints position data, the robot is running even if the Gazebo camera is not focused correctly.

### Step 5: Find the robot in Gazebo

In Gazebo:

```text
Entity Tree -> bluerov2
```

The initial pose is near:

```text
x ~= 1
y ~= 0
z ~= 0
```

The robot is small and close to the water plane, so zoom in and rotate the camera from a side view.

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
