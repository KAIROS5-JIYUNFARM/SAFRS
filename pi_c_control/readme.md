---

# C-Pi : Control & STM32 Bridge Node

## 1. Role

**C-Pi** is responsible for **low-level motion control and STM32 interfacing** in the SAFRS system.

It acts as the **control and hardware bridge layer** between ROS 2 and the STM32-based motor controller.

Main responsibilities:

* Receive velocity commands (`/cmd_vel`) from higher-level nodes
* Convert velocity commands into motor PWM signals
* Communicate with STM32 via **serial**
* Receive STM32-computed odometry data
* Publish raw odometry (`/odom_raw`) to B-Pi

---

## 2. Position in System Architecture

```text
[ D-Pi (System Manager) ]
          │
          ▼
      /cmd_vel
          │
          ▼
[ C-Pi (Control & STM Bridge) ]
  - Teleoperation
  - Motor PWM command
  - STM32 serial I/O
  - odom_raw publishing
          │
          ▼
[ B-Pi (Navigation & SLAM) ]
  - /odom, TF
  - SLAM / Nav2
```

* **C-Pi** directly interfaces with **hardware (STM32, motors, encoders)**
* Higher-level logic and navigation are handled by B-Pi and D-Pi

---

## 3. Node Overview

### ① `manual_teleop.py` – Keyboard Teleoperation Node

A simple **keyboard-based teleoperation node** for manual driving and testing.

#### Features

* Publishes `geometry_msgs/Twist` to `/cmd_vel`
* Adjustable speed control
* Terminal-based (no GUI)

#### Key Mapping

| Key        | Action         |
| ---------- | -------------- |
| `w`        | Forward        |
| `s`        | Backward       |
| `a`        | Turn right     |
| `d`        | Turn left      |
| `u`        | Increase speed |
| `p`        | Decrease speed |
| `x`        | Stop           |
| `Ctrl + C` | Exit           |

#### Notes

* Intended for **testing, debugging, and manual driving**
* Not used during autonomous Nav2 operation

---

### ② `stm32_bridge_addTF5.py` – STM32 Bridge Node

This node is the **core of C-Pi**.
It bridges ROS 2 commands and STM32 motor/encoder data.

---

## 4. Command Flow (ROS → STM32)

```text
/cmd_vel (Twist)
        ↓
Differential drive kinematics
        ↓
Wheel RPM calculation
        ↓
RPM → PWM conversion
        ↓
Serial command to STM32
```

### Velocity Handling

* Linear velocity: `msg.linear.x`
* Angular velocity: `msg.angular.z`
* Angular gain adjustment:

  * Moving: ×1.2
  * In-place rotation: ×3.0

### Differential Drive Model

* Wheel radius `R = 0.040 m`
* Wheel base `L = 0.290 m`

---

## 5. Serial Communication (STM32)

### Serial Settings

* Port: `/dev/ttyUSB0`
* Baudrate: `115200`

### Motor Command Format

```text
M <pwm_l> <pwm_l> <pwm_r> <pwm_r>
```

* PWM is signed (direction encoded by sign)
* Minimum RPM threshold prevents low-speed jitter

---

## 6. Odometry Handling (`/odom_raw`)

### Input (from STM32)

STM32 sends pre-integrated odometry:

```text
ODOM,x,y,theta,vx,wz
```

* Position: `x`, `y`
* Orientation: `theta` (yaw)
* Velocities: `vx`, `wz`

### Processing

* No encoder integration on C-Pi
* Values are **trusted as-is**
* Timestamp is assigned using **ROS time**

### Output

* Topic: `/odom_raw`
* Type: `nav_msgs/Odometry`
* Frame:

  * `frame_id`: `odom`
  * `child_frame_id`: `base_footprint`

Covariance values are explicitly set for:

* Pose: `(x, y, yaw)`
* Twist: `(vx, wz)`

---

## 7. Design Philosophy

* **STM32 handles real-time computation**
* **C-Pi focuses on I/O and conversion**
* Avoids duplicated odometry logic across nodes
* Keeps hardware-dependent logic isolated

This separation allows:

* Stable SLAM / Nav2 integration
* Easier debugging of control vs navigation issues
* Clear responsibility boundaries

---

## 8. Usage

### Run Manual Teleoperation

```bash
ros2 run pi_c_control manual_teleop.py
```

### Run STM32 Bridge

```bash
ros2 run pi_c_control stm32_bridge_addTF5.py
```

---

## 9. Verification Checklist

* [ ] `/cmd_vel` is received correctly
* [ ] Motors respond to teleop commands
* [ ] STM32 serial connection stable
* [ ] `/odom_raw` publishing continuously
* [ ] Odometry values reasonable when stationary
* [ ] No duplicated odometry computation elsewhere

---

## 10. Summary (3 Lines)

* **C-Pi is the hardware control layer**
* It converts `/cmd_vel` into STM32 motor commands
* It publishes **raw odometry** for B-Pi to stabilize and use

---