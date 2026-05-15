# AutoRacer Extrinsic Parameters

This document records the extrinsic (geometric) parameters of the AutoRacer vehicle, extracted from the URDF model (`urdf/autoracer.urdf.xacro`).

Measured: 2026-01-29

Review status: the chassis and sensor extrinsic values below still need manual
verification. The confirmed wheel-size value in the STM32 firmware is the hall
speed conversion diameter in `WHEELTEC_APP/hall_speed.c`:
`HALL_WHEEL_DIAMETER_M = 0.235 m` (radius `0.1175 m`, circumference
`0.738274 m`). Do not treat the URDF `wheel_radius = 0.11 m` entry below as
manually verified until the vehicle geometry check is complete.

## Coordinate System Convention

Following ROS REP-103:
- **X** = Forward
- **Y** = Left
- **Z** = Up
- **base_link** = Rear axle center, at axle height (Z=0.11m above ground)

## Chassis Parameters

| Parameter | Value | Unit | Description |
|-----------|-------|------|-------------|
| `wheelbase` | 0.60 | m | Front-to-rear axle distance |
| `track_width` | 0.48 | m | Left-to-right wheel distance |
| `wheel_radius` | 0.11 | m | Wheel radius |
| `wheel_width` | 0.08 | m | Wheel width |
| `max_steering_angle` | 0.393 | rad | Max steering angle (~22.5°) |
| `chassis_length` | 0.85 | m | Vehicle total length |
| `chassis_width` | 0.50 | m | Vehicle total width |
| `chassis_height` | 0.20 | m | Chassis box height (URDF visual) |
| `front_axle_to_front` | 0.15 | m | Front axle to front bumper |
| `rear_axle_to_rear` | 0.16 | m | Rear axle to rear bumper |
| `axle_height` | 0.11 | m | Axle center height above ground |

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

### ZED X Depth Camera (optional / not enabled)

Current workspace hardware facts do not treat ZED X as an installed or enabled
sensor. Keep the values below as an optional URDF/historical reference until a
vehicle inspection confirms the camera hardware and launch/TF usage.

| Axis | Value | Unit | Notes |
|------|-------|------|-------|
| X | +0.34 | m | Forward from rear axle |
| Y | 0.00 | m | Centered on vehicle |
| Z | +0.29 | m | Above axle height |
| Roll | 0 | rad | - |
| Pitch | 0 | rad | - |
| Yaw | 0 | rad | - |

**ZED X Notes:**
- ZED driver publishes internal TF tree from `zed_camera_link`
- Child frames include: `camera_center`, `left_camera_frame`, `right_camera_frame`, `imu_link`, etc.

## Wheel Positions

Relative to `base_link`:

| Wheel | X (m) | Y (m) | Z (m) | Joint Type |
|-------|-------|-------|-------|------------|
| Rear Left | 0.00 | +0.24 | 0.00 | continuous |
| Rear Right | 0.00 | -0.24 | 0.00 | continuous |
| Front Left Steering | +0.60 | +0.24 | 0.00 | revolute (±22.5°) |
| Front Right Steering | +0.60 | -0.24 | 0.00 | revolute (±22.5°) |

## TF Frame Hierarchy

```
odom
└── base_footprint (Z=0, ground level)
    └── base_link (Z=+0.11m, rear axle center)
        ├── rear_left_wheel_link
        ├── rear_right_wheel_link
        ├── front_left_steering_link
        │   └── front_left_wheel_link
        ├── front_right_steering_link
        │   └── front_right_wheel_link
        ├── laser (LiDAR C32)
        └── zed_camera_link (optional; not current enabled path)
            └── [ZED internal TF tree]
```

## Diagram (Top View)

```
                    Front
                      ↑ X
            +---------+----------+
            |    ○         ○     |  ← Front wheels (X=+0.60m)
            |                    |
            |   [optional ZED X] |  ← ZED camera (X=+0.34m, not enabled)
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
ros2 run tf2_ros tf2_echo base_link laser
# Optional ZED reference only; not part of the current enabled sensor set.
ros2 run tf2_ros tf2_echo base_link zed_camera_link

# Monitor TF
ros2 run tf2_ros tf2_monitor
```
