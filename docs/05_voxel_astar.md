# 05 Voxel Map and 3D A*

## Goal

Build a simple local 3D obstacle avoidance module.

## Planned Data Flow

```text
point cloud / sonar
  -> voxel map
  -> 3D A*
  -> local waypoints
  -> waypoint_tracker_3d
```

## First Version Scope

- Static obstacles.
- Fixed-resolution voxel grid.
- 26-connected 3D A*.
- Collision checking by occupied voxel lookup.

## Interview Notes

Start with a simple voxel grid before introducing Octomap. It is easier to implement, debug and explain.
