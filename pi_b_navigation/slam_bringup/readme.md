---

# slam_bringup

## 1. Purpose

The `slam_bringup` package is an **operational bringup package** for **B-Pi (Navigation)**.
Its purpose is to **launch and manage SLAM, Localization, and Nav2** in a stable and reproducible way.

This package **does not implement new algorithms**.
It only **orchestrates existing nodes** so that they satisfy **Nav2 and SLAM timing / TF requirements**.

---

## 2. Design Scope

### What this package does

* Provides launch configurations for SLAM, Localization, and Nav2
* Ensures `/odom` and TF meet **Nav2-safe requirements**
* Simplifies experiments and operation using launch-level abstraction

### What this package does NOT do

* Odometry computation ❌
* Sensor driver control ❌
* Path planning logic ❌
* Hardware control ❌

---

## 3. Package Structure

```text
slam_bringup/
├── README.md
├── launch/
│   ├── slam.launch.py          # Map building (SLAM)
│   ├── localization.launch.py  # Map-based localization
│   └── nav2.launch.py          # Nav2 navigation
├── config/
│   ├── slam_toolbox.yaml
│   └── nav2_params.yaml
└── nodes/
    ├── odom_bridge.py
    ├── odom_stamp_aligner.py
    └── scan_relay.py
```

---

## 4. Internal Nodes (Actual Usage)

### ① `odom_bridge.py`  **(Always Used)**

* Receives `odom_raw` from C-Pi
* Does **not modify odometry values**
* Acts as the **entry point** of the B-Pi navigation pipeline

> Bridges STM32-computed odometry into ROS 2.

---

### ② `odom_stamp_aligner.py`  **(Core Node / Always Used)**

* Converts `/odom_raw` → `/odom`
* Enforces **ROS time–based timestamp alignment**
* Guarantees time consistency between `/odom` and TF

> This node is the **key stability component** for SLAM and Nav2.

---

### ③ `scan_relay.py`  **(Rarely Used)**

* Relays `/scan_raw` → `/scan`
* Overrides LiDAR timestamps using ROS time

#### Usage policy

* ❌ **Not used in normal operation**
* ⭕ Used only when:

  * LiDAR driver timestamps are **ahead of TF**
  * Repeated TF extrapolation errors occur in SLAM/Nav2

> A **fallback stabilization node**, enabled only when necessary.

---

## 5. Launch Modes

### ① SLAM – Map Building

```bash
ros2 launch slam_bringup slam.launch.py
```

* Launched nodes:

  * `odom_bridge`
  * `odom_stamp_aligner`
  * `slam_toolbox` (sync mode)
* Inputs:

  * `/scan`
  * `/odom`
* Output:

  * `/map`

---

### ② Localization – Map-Based Localization

```bash
ros2 launch slam_bringup localization.launch.py
```

* Launched nodes:

  * `odom_bridge`
  * `odom_stamp_aligner`
  * `slam_toolbox` (localization mode) or `amcl`
* Purpose:

  * Estimate robot pose on a pre-built map

---

### ③ Navigation – Nav2

```bash
ros2 launch slam_bringup nav2.launch.py
```

* Launched nodes:

  * `odom_bridge`
  * `odom_stamp_aligner`
  * Nav2 stack (planner, controller, costmaps)

---

## 6. Implicit Execution Rules (Important)

For Nav2 to run correctly, the following conditions **must always hold**:

1. `/odom` must exist
2. TF `odom → base_link` must exist
3. TF timestamp ≥ `/scan` timestamp

The `slam_bringup` package is designed to **structurally enforce** these conditions.

---

## 7. Verify-First Checklist

Always verify after launch:

* [ ] `/odom` publishing stable (≥ 20 Hz)
* [ ] `odom → base_link` TF available
* [ ] No TF warnings when launching Nav2
* [ ] No odometry drift while stationary
* [ ] ~1 m movement matches map scale

---

## 8. Design Philosophy

* Computation is handled by STM32 (C-Pi)
* B-Pi is responsible for **time and TF stabilization**
* `slam_bringup` focuses purely on **execution and operation**
* Additional stabilization nodes (`scan_relay`) are enabled **only when required**

---

## 9. Summary (3 Lines)

* `slam_bringup` is a **navigation bringup package**
* The core stability node is `odom_stamp_aligner`
* `scan_relay` is an **optional fallback**, not part of normal operation

---

### Final Note

> This bringup package exists not to add features,
> but to **prevent known SLAM and Nav2 failure modes**.
