# Burgerbot sensor firmware — MCU implementation guide

What to implement on the IMU/mag board (LSM6DS3 + HMC5883L) for the burgerbot architecture.

---

## Division of labour

| Layer | Runs where | What it does |
|-------|-----------|--------------|
| Raw sensor reading + unit conversion | MCU | Read registers, convert to SI, publish |
| Gyro bias calibration | MCU (startup) | Remove constant gyro drift |
| Hard-iron mag correction | MCU (runtime) | Remove permanent magnetic offset |
| `imu_filter_madgwick` | ROS host | Fuse gyro + accel + mag → orientation quaternion |
| `robot_localization` EKF | ROS host | Fuse `/imu/data` + `/wheel_odom` → pose |

> **Do NOT run Madgwick on the MCU.** Your architecture already runs `imu_filter_madgwick` as a ROS node. Running it twice would double-filter and waste flash/RAM.

---

## Published topics from this board

| Topic | Message type | Rate | Content |
|-------|-------------|------|---------|
| `/imu/raw` | `sensor_msgs/Imu` | 100 Hz | Raw gyro + accel in SI units, no orientation |
| `/imu/mag` | `sensor_msgs/MagneticField` | 50 Hz | Calibrated mag in Tesla |

---

## LSM6DS3 (IMU — accel + gyro)

### 1. Chip configuration at startup

Set these registers before publishing anything:

| Parameter | Recommended value | Reason |
|-----------|------------------|--------|
| Accel ODR | 104 Hz | Matches 100 Hz publish rate |
| Gyro ODR | 104 Hz | Matches 100 Hz publish rate |
| Gyro full scale | ±500 dps | Diff drive robots don't spin fast |
| Accel full scale | ±4 g | More than enough for a ground robot |
| Low-pass filter | Enabled (built-in) | Reduces high-frequency noise before you even read the data |

### 2. Gyro bias calibration (run once at every power-on)

MEMS gyroscopes have a constant offset — the LSM6DS3 can read 2–5 deg/sec of apparent rotation when the robot is completely still. If uncorrected, the EKF will think the robot is spinning.

**Procedure:**
1. At startup, before moving, collect **500 gyro samples** (takes ~5 s at 100 Hz)
2. Average them: `bias_x = sum(gx) / 500`, same for y and z
3. Store the three bias values in RAM
4. Every subsequent sample: subtract the bias before publishing

```c
// Pseudocode
float bias_x = 0, bias_y = 0, bias_z = 0;
for (int i = 0; i < 500; i++) {
    read_gyro(&gx, &gy, &gz);
    bias_x += gx; bias_y += gy; bias_z += gz;
    delay_ms(10);
}
bias_x /= 500; bias_y /= 500; bias_z /= 500;

// Then every publish loop:
imu_msg.angular_velocity.x = (gx_raw - bias_x) * DEG_TO_RAD;
```

### 3. Unit conversion (mandatory for ROS)

`imu_filter_madgwick` and `robot_localization` expect strict SI units.

| Field | Unit | Conversion |
|-------|------|-----------|
| `linear_acceleration` | m/s² | raw_accel × (full_scale_g / 32768) × 9.81 |
| `angular_velocity` | rad/s | (raw_gyro × (full_scale_dps / 32768)) × π/180 |

### 4. Filling the Imu message correctly

```
sensor_msgs/Imu:
  header.frame_id: "imu_link"       # must match your URDF
  orientation_covariance[0]: -1.0   # tells ROS "I have no orientation estimate"
  angular_velocity: (bias-corrected, in rad/s)
  angular_velocity_covariance: diagonal [0.01, 0.01, 0.01]
  linear_acceleration: (in m/s²)
  linear_acceleration_covariance: diagonal [0.1, 0.1, 0.1]
```

Setting `orientation_covariance[0] = -1.0` is important — it tells `imu_filter_madgwick` not to trust a pre-computed orientation from the chip (the LSM6DS3 doesn't provide one anyway).

---

## HMC5883L (magnetometer)

### 1. Why calibration is mandatory

The motors, motor driver wires, and other PCB components create permanent magnetic fields around the sensor. This is called **hard-iron distortion** and causes the sensor to report a fixed offset in all directions. Without correcting it, the mag output is useless — `imu_filter_madgwick` will compute a wrong yaw and fight the gyro.

### 2. Hard-iron calibration (one-time, after assembly)

**Do this once after building the robot. Store results in firmware.**

**Procedure:**
1. Rotate the robot slowly through a full 360° on a flat surface (the wheels board and all electronics powered on, since the distortion sources must be present)
2. Log raw X, Y, Z readings throughout the rotation
3. Find the min and max for each axis
4. Compute offsets:

```
x_bias = (x_max + x_min) / 2
y_bias = (y_max + y_min) / 2
z_bias = (z_max + z_min) / 2
```

5. Hardcode these into firmware (or store in flash/NVS)

**At runtime, subtract before publishing:**
```c
mag_msg.magnetic_field.x = (raw_x - x_bias) * sensitivity_T;
mag_msg.magnetic_field.y = (raw_y - y_bias) * sensitivity_T;
mag_msg.magnetic_field.z = (raw_z - z_bias) * sensitivity_T;
```

> **Soft-iron distortion** (ferromagnetic materials warping the field into an ellipse) is harder to correct and needs ellipsoid fitting. For a differential drive ground robot, hard-iron correction alone is usually sufficient to get usable yaw from the mag.

### 3. Unit conversion

The HMC5883L default gain (register B = 0x20) gives **0.92 mG/LSB**. ROS expects Tesla:

```
value_in_Tesla = raw_counts × 0.92e-3 × 1e-4
               = raw_counts × 9.2e-8
```

### 4. Chip configuration

| Parameter | Value |
|-----------|-------|
| Data output rate | 75 Hz max; use 50 Hz |
| Gain | Default (±1.3 Ga range) — fine for indoor use |
| Operating mode | Continuous measurement mode |

### 5. Filling the MagneticField message

```
sensor_msgs/MagneticField:
  header.frame_id: "imu_link"    # same frame as the IMU
  magnetic_field: (calibrated, in Tesla)
  magnetic_field_covariance: diagonal [1e-7, 1e-7, 1e-7]
```

---

## Publish rates summary

| Topic | Rate | Notes |
|-------|------|-------|
| `/imu/raw` | 100 Hz | LSM6DS3 at 104 Hz ODR, publish every sample |
| `/imu/mag` | 50 Hz | HMC5883L at 75 Hz ODR, publish every other sample |
| `/wheel_odom` (other board) | 30–50 Hz | Not this board, but affects EKF tuning |

---

## What the ROS side handles (nothing to implement on MCU)

```
/imu/raw + /imu/mag
        │
        ▼
imu_filter_madgwick      ← removes gyro drift, fuses mag for yaw
        │
        ▼ /imu/data (with orientation quaternion)
        │
robot_localization EKF   ← fuses /imu/data + /wheel_odom
        │
        ▼ /odometry/filtered + odom→base_link TF
        │
Nav2 + SLAM toolbox
```

---

## Firmware loop structure (pseudocode)

```c
void setup() {
    lsm6ds3_init(ODR_104HZ, GYRO_500DPS, ACCEL_4G);
    hmc5883l_init(ODR_75HZ, CONTINUOUS_MODE);
    calibrate_gyro_bias();   // robot must be still for ~5 s
    load_mag_hard_iron_offsets();  // from flash/NVS
    microros_init();
}

void loop_100hz() {
    read_lsm6ds3(&ax, &ay, &az, &gx, &gy, &gz);
    publish_imu_raw(
        (ax * ACCEL_SCALE),             // m/s²
        (ay * ACCEL_SCALE),
        (az * ACCEL_SCALE),
        (gx - bias_x) * DEG_TO_RAD,    // rad/s
        (gy - bias_y) * DEG_TO_RAD,
        (gz - bias_z) * DEG_TO_RAD
    );
}

void loop_50hz() {
    read_hmc5883l(&mx, &my, &mz);
    publish_mag(
        (mx - mag_bias_x) * MAG_SCALE,  // Tesla
        (my - mag_bias_y) * MAG_SCALE,
        (mz - mag_bias_z) * MAG_SCALE
    );
}
```

---

## Covariance values — starting point

These are not magic numbers — tune them based on your actual sensor noise, but they are reasonable starting values:

| Field | Diagonal value | Notes |
|-------|---------------|-------|
| `angular_velocity_covariance` | `0.01` | Gyro noise, in (rad/s)² |
| `linear_acceleration_covariance` | `0.1` | Accel noise, in (m/s²)² |
| `magnetic_field_covariance` | `1e-7` | Mag noise, in Tesla² |

All covariance values **must be floats** (`0.01` not `0`) — the RCL YAML parser rejects mixed integer/float sequences (noted in your ARCHITECTURE.md).
