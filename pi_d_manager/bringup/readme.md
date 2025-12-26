
---

# D-Pi : System Manager & Bringup

## 1. Purpose

**D-Pi** is the **central system manager** of the SAFRS platform.

It defines **when the system starts**, **which mode the system is in**, and
**how the entire multi-Pi system is safely reset and recovered**.

In final operation, D-Pi relies on **two core components only**:

* `system_bringup.launch.py` — system startup & mode control
* `remote_cleanup.sh` — distributed system cleanup & recovery

---

## 2. Role of D-Pi in the System

D-Pi does **not** perform perception, navigation, or hardware control.

Its responsibilities are strictly limited to:

* Global system start
* Mode arbitration
* System-level safety and recovery

```text
A-Pi (Vision) ── /detect ─┐
C-Pi (Control) ─ /cmd_vel ├─▶ D-Pi ─▶ /mode
B-Pi (Nav)     ── /end ───┘
```

D-Pi is the **single source of truth** for the system mode.

---

## 3. system_bringup.launch.py (Final Entry Point)

### Purpose

`system_bringup.launch.py` is the **final and only launch file** used on D-Pi
during normal system operation.

Launching this file means:

> “The SAFRS system is now active at the system level.”

---

### What this launch file does

When executed, `system_bringup.launch.py`:

* Starts the **system manager node** (`mode_manager.py`)
* Publishes `/system_start`
* Begins monitoring:

  * `/cmd_vel`
  * `/detect`
  * `/end`
* Starts global mode arbitration logic
* Transitions system modes using **event + timer–based logic**

---

### What it intentionally does NOT do

This launch file does **not** start:

* SLAM / Nav2
* Vision nodes
* STM32 or motor drivers
* Hardware interfaces

Those components are assumed to be:

* Already running on their respective Pis, or
* Launched independently as needed

This design keeps D-Pi **decoupled and robust**.

---

### How to run

```bash
ros2 launch bringup system_bringup.launch.py
```

This command should be executed **once** per system run.

---

## 4. remote_cleanup.sh (Distributed System Reset)

### Purpose

`remote_cleanup.sh` is a **system-wide cleanup and recovery script**.

It exists to handle a common real-world problem:

> “Multiple Pis are running ROS nodes,
> and the system must be stopped or reset cleanly.”

---

### What the script does

When executed, `remote_cleanup.sh`:

* Connects to remote Pis via SSH
* Terminates running ROS 2 processes
* Cleans up orphaned nodes and stale sessions
* Resets the system to a **known idle state**

This prevents:

* Duplicate node launches
* Zombie ROS processes
* Inconsistent system states after crashes

---

### When to use it

* Before a **full system restart**
* After an **abnormal termination**
* When ROS nodes remain alive unexpectedly
* During repeated test iterations

---

### Execution

```bash
./remote_cleanup.sh
```

The script is typically run **before** `system_bringup.launch.py`.

---

## 5. Typical Operation Flow

```text
1. remote_cleanup.sh
   └─ ensure all Pis are clean

2. Start subsystem nodes
   ├─ A-Pi (Vision)
   ├─ B-Pi (Navigation)
   └─ C-Pi (Control)

3. system_bringup.launch.py
   └─ activate global system logic
```

This sequence ensures:

* No leftover processes
* Predictable startup behavior
* Clear system ownership by D-Pi

---

## 6. Design Philosophy

* One launch file for **system-level control**
* One script for **system-level cleanup**
* No hidden dependencies between Pis
* Event-driven logic instead of rigid FSMs

D-Pi is designed to be:

* Simple
* Deterministic
* Easy to recover

---

## 7. Verification Checklist

After running `system_bringup.launch.py`:

* [ ] `/system_start` is published
* [ ] `/mode` topic is active
* [ ] Mode transitions occur correctly
* [ ] System returns to STANDBY when idle
* [ ] No stale ROS processes remain on other Pis

---

## 8. Summary (3 Lines)

* **D-Pi is the system-level decision node**
* `system_bringup.launch.py` starts global mode control
* `remote_cleanup.sh` guarantees safe reset and recovery

---

### Final Note

> **D-Pi does not make the robot move.
> It makes the system behave like one coherent robot.**

---