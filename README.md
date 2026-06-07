# BlueROV2 3D Navigation and Control

基于 ROS 2 的 BlueROV2 水下机器人三维运动控制与避障仿真项目。

本项目目标是构建一条可复现实验链路：

```text
Gazebo BlueROV2 simulation
  -> sensors / odometry
  -> waypoint tracking
  -> 6-DOF motion control
  -> thruster allocation
  -> /bluerov2/cmd_thruster1..6
```

## Current Status

- 已搭建 `bluerov2_navigation` ROS 2 包。
- 已实现 `thruster_allocator` C++ 节点。
- 已完成 `cmd_wrench -> cmd_thruster1..6` 的节点级验证。

## Package Layout

```text
bluerov2_navigation/
  config/       Runtime parameters
  docs/         Learning notes and design documents
  experiments/  Evaluation scripts, results and plots
  launch/       ROS 2 launch files
  src/          C++ node implementations
```

## Quick Start

Build:

```bash
cd /home/ct03/underwater_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select bluerov2_navigation --symlink-install
```

Run the thruster allocator:

```bash
source /home/ct03/underwater_ws/install/setup.bash
ros2 launch bluerov2_navigation thruster_allocator.launch.py
```

Publish a test wrench:

```bash
ros2 topic pub --rate 10 /bluerov2/cmd_wrench geometry_msgs/msg/WrenchStamped \
"{header: {frame_id: bluerov2/base_link}, wrench: {force: {x: 8.0, y: 0.0, z: 0.0}, torque: {x: 0.0, y: 0.0, z: 0.0}}}"
```

Observe thruster output:

```bash
ros2 topic echo /bluerov2/cmd_thruster1
```

## Visible Gazebo Launch

If the default Gazebo GUI is blank or cannot focus the robot, use the safer OGRE renderer workflow documented here:

```text
docs/01_simulation_setup.md
```

Short version:

```bash
cd /home/ct03/underwater_ws
source install/setup.bash
export ROS_LOG_DIR=/tmp/ros_logs

ign gazebo /home/ct03/underwater_ws/install/bluerov2_description/share/bluerov2_description/urdf/demo_world.sdf \
  -r --render-engine-gui ogre --render-engine-server ogre --force-version 6
```

Then open a second terminal and spawn BlueROV2:

```bash
cd /home/ct03/underwater_ws
source install/setup.bash
export ROS_LOG_DIR=/tmp/ros_logs

ros2 launch bluerov2_description upload_bluerov2_launch.py gazebo_world_name:=ocean sliders:=true
```

## Roadmap

- [x] Thruster allocation with damped pseudo-inverse.
- [ ] 6-DOF PID controller.
- [ ] 3D waypoint tracker with LOS / Pure Pursuit.
- [ ] Local voxel map.
- [ ] 3D A* obstacle avoidance.
- [ ] Experiment logging and plotting.

## Key Technical Point

The thruster allocator builds a thrust allocation matrix:

```text
wrench = A * thrust
```

Because the BlueROV2 thruster layout can be singular or ill-conditioned, the allocator uses damped least-squares pseudo-inverse:

```text
A+ = A^T (A A^T + lambda^2 I)^-1
```

It also applies thrust saturation, deadzone compensation and first-order low-pass filtering.
