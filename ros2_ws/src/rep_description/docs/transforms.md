# Transformation Matrices - rep

Homogeneous transformation matrices between consecutive frames.
Convention: URDF RPY (XYZ extrinsic / ZYX intrinsic).

## Notation

### Frames

| Index | Link |
|-------|------|
| $L_{0}$ | base_link |
| $L_{1}$ | caster_wheel1 |
| $L_{2}$ | caster_wheel2 |
| $L_{3}$ | lidar |
| $L_{4}$ | imu |
| $L_{5}$ | mag |
| $L_{6}$ | left_wheel |
| $L_{7}$ | right_wheel |

### Joint Variables

| Variable | Joint | Type | From | To |
|----------|-------|------|------|----|
| $q_{1}$ | left_wheel_joint | continuous (rad) | $L_{0}$ | $L_{6}$ |
| $q_{2}$ | right_wheel_joint | continuous (rad) | $L_{0}$ | $L_{7}$ |

Shorthand: $c_i = \cos(q_i)$, $s_i = \sin(q_i)$

### Kinematic Tree

```
L0: base_link
  |-- [fixed] caster_wheel1_joint
  |   L1: caster_wheel1
  |-- [fixed] caster_wheel2_joint
  |   L2: caster_wheel2
  |-- [fixed] lidar_joint
  |   L3: lidar
  |-- [fixed] imu_joint
  |   L4: imu
  |-- [fixed] mag_joint
  |   L5: mag
  |-- [continuous] left_wheel_joint (q1)
  |   L6: left_wheel
  +-- [continuous] right_wheel_joint (q2)
      L7: right_wheel
```

## Transforms

## caster_wheel1_joint

$L_{0}$ **base_link** -> $L_{1}$ **caster_wheel1** (fixed)

- **origin xyz**: (0.1225, 0, -0.023) m
- **origin rpy**: (0, 0, 0) rad

### Local Transform

$$
T^{0}_{1} = \begin{bmatrix}
1 & 0 & 0 & 0.1225 \\
0 & 1 & 0 & 0 \\
0 & 0 & 1 & -0.023 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## caster_wheel2_joint

$L_{0}$ **base_link** -> $L_{2}$ **caster_wheel2** (fixed)

- **origin xyz**: (-0.125, 0, -0.0355) m
- **origin rpy**: (0, 0, 0) rad

### Local Transform

$$
T^{0}_{2} = \begin{bmatrix}
1 & 0 & 0 & -0.125 \\
0 & 1 & 0 & 0 \\
0 & 0 & 1 & -0.0355 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## lidar_joint

$L_{0}$ **base_link** -> $L_{3}$ **lidar** (fixed)

- **origin xyz**: (0.055, 0.002, 0.1232) m
- **origin rpy**: (0, 0, 0) rad

### Local Transform

$$
T^{0}_{3} = \begin{bmatrix}
1 & 0 & 0 & 0.055 \\
0 & 1 & 0 & 0.002 \\
0 & 0 & 1 & 0.1232 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## imu_joint

$L_{0}$ **base_link** -> $L_{4}$ **imu** (fixed)

- **origin xyz**: (0, 0, 0.0755) m
- **origin rpy**: (0, 0, 0) rad

### Local Transform

$$
T^{0}_{4} = \begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & 1 & 0 & 0 \\
0 & 0 & 1 & 0.0755 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## mag_joint

$L_{0}$ **base_link** -> $L_{5}$ **mag** (fixed)

- **origin xyz**: (0.142, -0.003, 0.0055) m
- **origin rpy**: (0, 0, 0) rad

### Local Transform

$$
T^{0}_{5} = \begin{bmatrix}
1 & 0 & 0 & 0.142 \\
0 & 1 & 0 & -0.003 \\
0 & 0 & 1 & 0.0055 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## left_wheel_joint

$L_{0}$ **base_link** -> $L_{6}$ **left_wheel** (continuous)
  Variable: $q_{1}$

- **origin xyz**: (0, 0.1195, -0.027) m
- **origin rpy**: (0, 0, 0) rad
- **axis**: (0, 1, 0)

### Local Transform

$$
T^{0}_{6}(q_{1}) = \begin{bmatrix}
c_{1} & 0 & s_{1} & 0 \\
0 & 1 & 0 & 0.1195 \\
-s_{1} & 0 & c_{1} & -0.027 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## right_wheel_joint

$L_{0}$ **base_link** -> $L_{7}$ **right_wheel** (continuous)
  Variable: $q_{2}$

- **origin xyz**: (0, -0.1195, -0.027) m
- **origin rpy**: (0, 0, 0) rad
- **axis**: (0, -1, 0)

### Local Transform

$$
T^{0}_{7}(q_{2}) = \begin{bmatrix}
c_{2} & 0 & -s_{2} & 0 \\
0 & 1 & 0 & -0.1195 \\
s_{2} & 0 & c_{2} & -0.027 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Global Transform Chains

Transform from root $L_0$ to any link, as product of local transforms along the kinematic chain.

