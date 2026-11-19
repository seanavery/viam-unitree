# Model erh:viam-unitree:g1-imu

Movement sensor for the Unitree G1 **torso IMU**. It subscribes to the `unitree_hg::msg::dds_::IMUState_` DDS topic `rt/secondary_imu`, which Unitree's locomotion computer publishes.

The G1 has two IMUs:

- **Pelvis IMU.** Used by the walking controller. It's carried in `rt/lowstate` and is the orientation that [`g1-movementsensor`](erh_viam-unitree_g1-movementsensor.md) reports.
- **Torso IMU (this model).** The waist joints sit between the two. The head sensors (MID-360 lidar, D435 camera) are rigidly attached to `torso_link`, so this IMU gives their attitude and the pelvis IMU doesn't.

## Configuration

```json
{
  "network_interface": "eth0",
  "topic": "rt/secondary_imu"
}
```

### Attributes

| Name                | Type   | Inclusion | Description |
|---------------------|--------|-----------|-------------|
| `network_interface` | string | Optional  | Network interface for DDS (default: eth0) |
| `topic`             | string | Optional  | DDS topic (default: `rt/secondary_imu`; `rt/lf/secondary_imu` is the same IMU at 20Hz) |

### Frame

From the URDF, `imu_in_torso` sits at (-39.59, -2.24, 137.92) mm from `torso_link`, with no rotation. With a [`g1-body`](erh_viam-unitree_g1-body.md) component named `body`, whose end frame is `torso_link`:

```json
"frame": { "parent": "body", "translation": { "x": -39.59, "y": -2.24, "z": 137.92 } }
```

Measured on a G1 (2026-11-19): `rt/secondary_imu` publishes at ~1000 Hz, and `rt/lf/secondary_imu` at 20 Hz.

## Data

| Method / Reading             | Source                         | Notes |
|------------------------------|--------------------------------|-------|
| Orientation                  | `quaternion` (w,x,y,z)         | |
| AngularVelocity              | `gyroscope`                    | Reported in deg/s |
| LinearAcceleration           | `accelerometer`                | m/s², gravity included |
| Readings.rpy_rad             | `rpy`                          | roll/pitch/yaw in radians |
| Readings.temperature_c       | `temperature`                  | |
| Readings.age_ms              | —                              | Time since the newest sample was taken |
| Readings.source_time         | DDS source timestamp           | Publisher's clock; the message has no time field |
| Readings.receive_minus_source_ms | receive time − source time  | Includes the clock offset between the two computers (~711 s on one test robot), so use it for jitter, not absolute latency |
| Readings.samples_taken       | —                              | Samples taken since startup |
| Position / LinearVelocity / CompassHeading | —                | Unimplemented |
