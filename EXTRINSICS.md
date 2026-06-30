# AutoRacer Extrinsic Parameters

This document records the extrinsic (geometric) parameters of the AutoRacer vehicle, extracted from the URDF model (`urdf/autoracer.urdf.xacro`).

Measured/updated: 2026-06-22

## Coordinate System Convention

Following ROS REP-103:
- **X** = Forward
- **Y** = Left
- **Z** = Up
- **base_link** = Rear axle center, at axle height (Z=0.115m above ground)

## Chassis Parameters

| Parameter | Value | Unit | Description |
|-----------|-------|------|-------------|
| `wheelbase` | 0.600 | m | Front-to-rear axle distance |
| `track_width` | 0.470 | m | Left-to-right wheel distance |
| `wheel_radius` | 0.115 | m | Wheel radius |
| `wheel_diameter` | 0.230 | m | Wheel diameter |
| `wheel_width` | 0.08 | m | Wheel width |
| `max_steering_angle` | 0.262 | rad | Max steering angle (~15.0°) |
| `chassis_length` | 0.83 | m | Vehicle total length |
| `chassis_width` | 0.58 | m | Vehicle total width |
| `chassis_height` | 0.50 | m | Vehicle total height |
| `front_axle_to_front` | 0.115 | m | Front axle to front bumper |
| `rear_axle_to_rear` | 0.115 | m | Rear axle to rear bumper |
| `axle_height` | 0.115 | m | Axle center height above ground |

## Sensor Extrinsics

All sensor positions are relative to `base_link` (rear axle center, at axle height).

### LiDAR (Leishen C32)

| Axis | Value | Unit | Notes |
|------|-------|------|-------|
| X | +0.24 | m | Forward from rear axle |
| Y | 0.00 | m | Centered on vehicle |
| Z | +0.39 | m | Above axle height |
| Roll | 0 | rad | - |
| Pitch | 0 | rad | - |
| Yaw | -1.5708 | rad | -90° rotation to align coordinate frame |

**LiDAR Coordinate Notes:**
- LiDAR native coordinate (with `coordinate_opt=false`): +Y=front(0°), +X=right(90°), +Z=up
- Yaw=-90° transforms LiDAR frame to ROS convention (+X=front)
- Cable exit at rear, LiDAR config `cable_position=180°`, so 0°=front

### Camera / Fixposition

The current RC validation profile has no ZED camera and no Fixposition device.

## Wheel Positions

Relative to `base_link`:

| Wheel | X (m) | Y (m) | Z (m) | Joint Type |
|-------|-------|-------|-------|------------|
| Rear Left | 0.00 | +0.235 | 0.00 | continuous |
| Rear Right | 0.00 | -0.235 | 0.00 | continuous |
| Front Left Steering | +0.600 | +0.235 | 0.00 | revolute (±15.0°) |
| Front Right Steering | +0.600 | -0.235 | 0.00 | revolute (±15.0°) |

## TF Frame Hierarchy

```
odom
└── base_footprint (Z=0, ground level)
    └── base_link (Z=+0.115m, rear axle center)
        ├── rear_left_wheel_link
        ├── rear_right_wheel_link
        ├── front_left_steering_link
        │   └── front_left_wheel_link
        ├── front_right_steering_link
        │   └── front_right_wheel_link
        └── lidar_top (LiDAR C32)
```

## Diagram (Top View)

```
                    Front
                      ↑ X
            +---------+----------+
            |    ○         ○     |  ← Front wheels (X=+0.600m)
            |                    |
            |      [LiDAR]       |  ← LiDAR C32 (X=+0.24m)
            |                    |
            |    ○         ○     |  ← Rear wheels (X=0)
            +---------+----------+
                      |
                 base_link (rear axle center)
        Y ←
```

## Usage

To verify TF transforms at runtime:

```bash
# View TF tree
ros2 run tf2_tools view_frames

# Echo specific transform
ros2 run tf2_ros tf2_echo base_link lidar_top

# Monitor TF
ros2 run tf2_ros tf2_monitor
```
