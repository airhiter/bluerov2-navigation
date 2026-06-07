# 04 3D Waypoint Tracking

## Goal

Track a sequence of 3D waypoints using LOS / Pure Pursuit logic.

## Planned Data Flow

```text
waypoint list + odometry
  -> waypoint_tracker_3d
  -> target pose / velocity
  -> six_dof_pid_controller
```

## Key Ideas

- Select current waypoint.
- Switch waypoint when distance is below a threshold.
- Compute desired yaw from horizontal direction.
- Compute desired depth or pitch from vertical error.
- Use lookahead distance to smooth path tracking.

## Interview Notes

Compared with 2D Pure Pursuit, underwater 3D tracking must handle depth and pitch in addition to yaw and planar position.
