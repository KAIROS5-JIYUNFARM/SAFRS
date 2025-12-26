
---

# system_manager (D-Pi)

## 1. Purpose

`system_manager` is the **central decision-making package** of the SAFRS system.

This package does **not** control hardware or sensors directly.
Instead, it observes events from multiple subsystems and determines
the **current global operating mode (`/mode`)** of the robot.

> D-Pi does not act.
> It decides how the system should act.

---

## 2. Role of D-Pi

### What `system_manager` DOES

* Publishes the system start signal
* Monitors system-level events
* Determines the current operating mode
* Broadcasts `/mode` to all subsystems
* Applies timeout-based safety logic

### What `system_manager` DOES NOT DO

* Motor or gimbal control ❌
* Navigation or SLAM ❌
* Vision processing ❌
* Hardware access ❌

---

## 3. Position in System Architecture

```text
A-Pi (Vision) ── /detect ─┐
C-Pi (Control) ─ /cmd_vel ├─▶ system_manager (D-Pi) ─▶ /mode
B-Pi (Navigation) ─ /end ─┘
```

* `system_manager` observes all major subsystems
* `/mode` is the **single source of truth** for system behavior

---

## 4. Core Node

### `mode_manager.py`

This is the **only executable node** in the `system_manager` package.

#### Subscribed Topics

* `/cmd_vel` (`geometry_msgs/Twist`)
  → Indicates active motion commands
* `/detect` (`std_msgs/String`)
  → Detection result from A-Pi (ALLY / ENEMY)
* `/end` (`std_msgs/String`)
  → Action or tracking completion signal

#### Published Topics

* `/mode` (`std_msgs/String`)
  → Current system operating mode
* `/system_start` (`std_msgs/Bool`)
  → System activation signal

---

## 5. System Mode Definition

| Mode        | Description                |
| ----------- | -------------------------- |
| BOOT        | System initialization      |
| NAVI        | Autonomous navigation      |
| STANDBY     | Idle / monitoring state    |
| TRACK_ALLY  | Ally tracking              |
| TRACK_ENEMY | Enemy tracking             |
| PASS        | Ally pass-through handling |
| TRIGGER_ON  | Enemy response trigger     |

All downstream nodes **react only to `/mode`**.

---

## 6. State Transition Logic

`system_manager` does **not** use an explicit Finite State Machine (FSM).

Instead, state transitions are implemented using:

* ROS topic events
* Timer-based conditions
* Consecutive event filtering

### Why this design

* Avoids FSM synchronization issues in distributed systems
* Tolerates network latency
* Improves robustness and maintainability

---

## 7. High-Level Logic Summary

* Publish `/system_start` during system initialization
* Transition to `STANDBY` if `/cmd_vel` is absent for a defined timeout
* Enter tracking states after consecutive `/detect` events
* Terminate action states upon receiving `/end`
* Automatically return to `STANDBY` after action completion

---

## 8. Execution

### Standalone (Debug)

```bash
ros2 run system_manager mode_manager.py
```

### Final Operation (Recommended)

```bash
ros2 launch bringup system_bringup.launch.py
```

> In final operation, `system_manager` is launched via `system_bringup`.

---

## 9. Design Philosophy

* Clear separation between decision and control
* Single centralized decision point
* Event-driven logic instead of rigid FSMs
* Loose coupling between subsystems

`system_manager` is designed to be the **most stable and least frequently changed** component.

---

## 10. Summary (3 Lines)

* `system_manager` is the **central decision layer**
* It defines system behavior via `/mode`
* No direct control or hardware interaction

---

### Final Statement

> **The robot moves elsewhere.
> The system decides here.**

---

