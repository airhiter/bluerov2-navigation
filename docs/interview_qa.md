# Interview Q&A

## Why ROS 2?

ROS 2 provides modular communication, launch management, visualization and logging tools for robotics systems.

## Why Gazebo?

Gazebo provides physics simulation, sensor simulation and plugin support. It lets us test control and planning before deploying on hardware.

## What is a wrench?

A wrench is a 6-DOF force and torque vector:

```text
[Fx, Fy, Fz, Tx, Ty, Tz]
```

## Why not command thrusters directly?

High-level controllers should reason in body motion or generalized forces. Thruster allocation is a lower-level actuator mapping problem.

## Why use damped pseudo-inverse?

The BlueROV2 thruster allocation matrix can be singular or ill-conditioned. Damped pseudo-inverse gives a stable approximate solution and avoids excessive actuator commands.

## What do deadzone and low-pass filter solve?

Deadzone removes tiny commands that may only cause jitter. Low-pass filtering smooths command changes and reduces attitude oscillation.

## What should I show in a demo?

- Manual thruster control.
- Wrench-to-thruster allocation.
- Depth hold.
- 3D waypoint tracking.
- Obstacle avoidance.
