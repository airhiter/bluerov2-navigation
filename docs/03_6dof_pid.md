# 03 6-DOF PID Controller

## Goal

Track a target pose by generating a desired wrench.

## Planned Data Flow

```text
/bluerov2/odom
target pose
  -> six_dof_pid_controller
  -> /bluerov2/cmd_wrench
  -> thruster_allocator
  -> /bluerov2/cmd_thruster1..6
```

## Control Axes

```text
surge: x
sway: y
heave: z
roll
pitch
yaw
```

## To Implement

- Position error.
- Velocity damping.
- Yaw error normalization.
- Output wrench saturation.
- Basic anti-windup if integral control is added.

## Interview Notes

The controller does not directly command thrusters. It commands a desired wrench. The allocator then maps that wrench to individual thruster commands.
