# 02 Thruster Allocator

## Goal

Convert a desired 6-DOF wrench into six BlueROV2 thruster commands.

## Input

```text
/bluerov2/cmd_wrench
geometry_msgs/msg/WrenchStamped
```

The wrench is:

```text
[Fx, Fy, Fz, Tx, Ty, Tz]
```

## Output

```text
/bluerov2/cmd_thruster1
/bluerov2/cmd_thruster2
/bluerov2/cmd_thruster3
/bluerov2/cmd_thruster4
/bluerov2/cmd_thruster5
/bluerov2/cmd_thruster6
```

## Core Idea

Each thruster produces:

```text
force = thrust * direction
torque = position x force
```

Combining all thrusters gives:

```text
wrench = A * thrust
```

To compute thruster commands:

```text
thrust = A+ * wrench
```

## Why Damped Pseudo-Inverse

The BlueROV2 thruster layout can be singular or ill-conditioned. Direct matrix inverse may fail or produce unstable commands.

The implemented allocator uses:

```text
A+ = A^T (A A^T + lambda^2 I)^-1
```

This improves numerical stability.

## Safety Shaping

- Saturation: clamps thrust into valid limits.
- Deadzone: removes very small commands.
- Low-pass filter: smooths command changes.

## Run

```bash
source /home/ct03/underwater_ws/install/setup.bash
ros2 launch bluerov2_navigation thruster_allocator.launch.py
```

## Test

```bash
ros2 topic pub --rate 10 /bluerov2/cmd_wrench geometry_msgs/msg/WrenchStamped \
"{header: {frame_id: bluerov2/base_link}, wrench: {force: {x: 8.0, y: 0.0, z: 0.0}, torque: {x: 0.0, y: 0.0, z: 0.0}}}"
```

```bash
ros2 topic echo /bluerov2/cmd_thruster1
```

## Interview Answer

I construct the thrust allocation matrix from each thruster's position and direction. Since the BlueROV2 allocation matrix can be singular, I use damped least-squares pseudo-inverse instead of direct inverse. I also add saturation, deadzone compensation and a first-order low-pass filter for stable actuator commands.
