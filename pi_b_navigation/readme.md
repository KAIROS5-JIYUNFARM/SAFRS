---

# B-Pi : Navigation & SLAM Node

## 1. Role

**B-Pi** is responsible for **navigation perception and localization** in the robot system.
It receives **pre-computed odometry (`odom_raw`) from STM32 via C-Pi** and converts it into **ROS 2–compatible `/odom` and TF** for SLAM and Nav2.

Main responsibilities:

* Receive STM-computed `odom_raw` from C-Pi
* Publish ROS 2–compliant `nav_msgs/Odometry`
* Broadcast stable TF (`odom → base_link`)
* Ensure time and TF consistency for SLAM / Nav2
* Act as the **dedicated Navigation node** in the multi-Pi architecture

---

## 2. Position in System Architecture

```text
[ C-Pi (STM32 Motor / Encoder) ]
   - Encoder integration
   - Odometry computation (STM)
              │  (UDP / DDS)
              ▼
        [ B-Pi ]
   - odom_raw reception
   - Odometry topic publishing
   - TF (odom → base_link)
              │
              ▼
        [ D-Pi ]
   - Mission / Mode Manager
```

* **C-Pi**: Handles STM32, motors, encoders, and computes odometry
* **B-Pi**: Converts STM odometry into ROS-safe `/odom` + TF
* **D-Pi**: Manages mission logic and system modes

---

## 3. Goal

> **Provide Nav2-safe odometry and TF that fully satisfy ROS 2 timing and frame requirements**

Design priorities:

* Timestamp consistency (ROS time)
* TF availability before or at sensor timestamps
* Robustness against network latency
* Prevent Nav2 and SLAM runtime failures

---

## 4. Odometry Handling (Updated)

### Input

* `odom_raw` computed on **STM32 (C-Pi side)**
* Position `(x, y)` and orientation `(θ)`
* Linear and angular velocity (if available)

### Processing Flow

```text
STM32 odom_raw
 (via C-Pi, UDP/DDS)
        ↓
 B-Pi receives odom_raw
        ↓
 Timestamp override (ROS time)
        ↓
 Odometry.msg (/odom)
        ↓
 TF broadcast (odom → base_link)
```

### Key Characteristics

* **No encoder-level calculation on B-Pi**
* B-Pi acts as a **conversion and stabilization layer**
* Ensures:

  * Valid ROS timestamps
  * TF–Odometry synchronization
  * Nav2-compatible data flow

### Output

* `/odom` : `nav_msgs/Odometry`
* `/tf` : `odom → base_link`

---

## 5. TF & Time Design Principles

Based on real issues encountered during Nav2 and SLAM integration.

### Applied Rules

* Odometry and TF share the **same timestamp**
* TF is broadcast using a **timer-based publisher**
* Sensor timestamps (`scan`) are **never later than TF**

### Rationale

* Nav2 fails hard on TF lookup errors
* Distributed systems introduce unavoidable latency
* Explicit timestamp alignment improves system safety

---

## 6. Packages & Nodes

```text
slam_bringup/
 ├── launch/
 │   ├── slam.launch.py
 │   ├── localization.launch.py
 │   └── nav2.launch.py
 ├── config/
 │   ├── nav2_params.yaml
 │   └── slam_toolbox.yaml
 └── nodes/
     ├── odom_bridge.py
     ├── odom_stamp_aligner.py
     └── scan_relay.py
```

* `odom_bridge` : Receives STM `odom_raw`
* `odom_stamp_aligner` : Enforces timestamp alignment
* `scan_relay` : LiDAR scan timestamp stabilization

---

## 7. SLAM / Navigation Operation

### ① SLAM (Map Building)

```bash
ros2 launch slam_bringup slam.launch.py
```

* SLAM Toolbox (sync mode)
* Input: `/scan`, `/odom`
* Output: `/map`

### ② Localization + Nav2

```bash
ros2 launch slam_bringup nav2.launch.py
```

* Map-based localization
* AMCL or slam_toolbox localization mode
* Global / Local costmaps enabled

---

## 8. Verification Checklist (Verify-first)

Always verify B-Pi independently:

* [ ] `/odom` publishing stable (≥20 Hz)
* [ ] `odom → base_link` TF exists
* [ ] TF timestamp ≥ scan timestamp
* [ ] No odom drift while stationary
* [ ] ~1 m movement matches map scale
* [ ] Nav2 launches without TF warnings

---

## 9. Troubleshooting Summary

| Issue                               | Root Cause                | Fix                      |
| ----------------------------------- | ------------------------- | ------------------------ |
| Robot rotates while moving straight | Duplicate sign correction | Single sign convention   |
| Nav2 TF errors                      | Timestamp mismatch        | Timestamp override relay |
| SLAM map jumps                      | TF delay                  | Timer-based TF           |

---

## 10. Summary (3 Lines)

* **B-Pi is the stability layer for Navigation**
* It converts STM-computed odometry into **Nav2-safe ROS data**
* Time and TF consistency are its primary responsibilities

---
