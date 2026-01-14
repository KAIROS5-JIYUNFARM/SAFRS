# 🧠 Main Pi (D-Pi) – System Manager

Main Pi (D-Pi) acts as the **central decision node** of the SAFRS / Halpong_Lunar system.  
It aggregates information from Navigation, Detection, and Action-related nodes and  
defines the **current operational state of the entire system**.

This node does **not** rely on an explicit Finite State Machine (FSM).  
Instead, system behavior is controlled through **ROS2 topic events and timer-based logic**.

---

## 🧭 Overview

- Role: Central system decision and mode management
- Location: Main Raspberry Pi (D-Pi)
- Main responsibilities:
  - Monitor `/cmd_vel`, `/detect`, and `/end` topics
  - Publish the current system state via `/mode`
  - Perform timeout-based safety fallback logic
- Runtime environment:
  - Ubuntu 22.04
  - ROS2 Humble
  - Python (rclpy)

---

## 🏗 System Role

```text
Navigation (/cmd_vel)
        │
        ▼
Detection (/detect) ──▶ Main Pi (D-Pi) ──▶ /mode
        │                      │
        └──────── (/end) ◀─────┘
```
---

## 📁 Project Structure

```
Main_pi/
├── bringup
│   ├── launch
│   │   ├── system.launch.py
│   │   └── system_bringup.launch.py
│   ├── scripts
│   │   └── remote_bringup.sh
│   └── bringup
│
└── system_manager
    ├── system_manager
    │   ├── mode_manager.py        # Main system manager node
    │   └── mode_manager.py.backup # Legacy / reference implementation
    ├── resource
    └── test
```

The Main Pi does not directly control hardware.
It defines and broadcasts the current system state based on incoming events.

---

## 🧩 Key Features

- 📡 ROS2 topic-based centralized state management

- ⏱ Timer-driven watchdog logic

- 🔁 Consecutive detection filtering to reduce false positives

- 🔒 Automatic fallback to standby after action completion

- 🧠 Lightweight interface using simple String messages

---

## 🔄 System Mode Definition

| Mode        | Description                |
| ----------- | -------------------------- |
| BOOT        | System initialization      |
| NAVI        | Autonomous navigation      |
| STANDBY     | Idle monitoring state      |
| TRACK_ALLY  | Ally tracking state        |
| TRACK_ENEMY | Enemy tracking state       |
| PASS        | Ally pass-through handling |
| TRIGGER_ON  | Enemy response trigger     |

---

## 📡 ROS2 Interfaces

---

### 📤 Publishers

| Topic           | Type     | Description         |
| --------------- | -------- | ------------------- |
| `/system_start` | `Bool`   | System start signal |
| `/mode`         | `String` | Current system mode |

---

### 📥 Subscribers

| Topic      | Type                  | Description                           |
| ---------- | --------------------- | ------------------------------------- |
| `/cmd_vel` | `geometry_msgs/Twist` | Navigation velocity command           |
| `/detect`  | `std_msgs/String`     | Detection result (`ALLY`, `ENEMY`)    |
| `/end`     | `std_msgs/String`     | Tracking or action termination signal |

---

🧠 Core Logic Summary

- Publish system start signal during BOOT

- Transition to standby if /cmd_vel is not received within a timeout

- Enter tracking state after consecutive detection events

- Terminate action state upon receiving /end

- Automatically return to standby after a fixed action duration

State transitions are implemented using
event-driven conditions combined with timer-based checks,
rather than an explicit FSM.

---

## 🚀 How to Run

### **1️⃣ Launch the full system**

```
ros2 launch bringup system_bringup.launch.py
```

### **2️⃣ Launch only the Main Pi system manager**
```
ros2 launch bringup system.launch.py
```

---

## 📌 Current Status

 - Central system manager node implemented

 - Watchdog and timeout logic applied

 - Lightweight String-based interface finalized

 - Log visualization and analysis

 - Documentation of state transition flow

 ---
 ## 🛠 Notes

- `mode_manager.py.backup` is kept for reference to earlier message-based designs

- Main Pi performs decision-making only

- All physical control and actuation are handled by downstream nodes

---

## 📜 License

SAFRS Robotics Platform

License: MIT (pending finalization)

---

## 👤 Maintainers

**지윤목장**

SAFRS Robotics Team