
---

# system_interfaces

## 1. Purpose

`system_interfaces` is a **shared ROS 2 interface package** for the SAFRS system.

This package defines **common message and interface types** used across
multiple Raspberry Pis (A-Pi, B-Pi, C-Pi, D-Pi).

It contains **no runtime logic** and **no executable nodes**.

---

## 2. Why This Package Exists

In a **multi-Pi distributed ROS 2 system**, using ad-hoc message definitions
inside each package causes:

* Tight coupling between nodes
* Version mismatch across Pis
* Difficult maintenance and refactoring

`system_interfaces` solves this by:

* Centralizing message definitions
* Providing a **single source of truth** for system-level interfaces
* Allowing all nodes to communicate using **consistent message types**

---

## 3. What This Package Does

### system_interfaces DOES

* Define custom ROS 2 message types (`.msg`)
* Define shared system-level data structures
* Provide reusable interfaces across packages

### system_interfaces DOES NOT

* Run ROS nodes ❌
* Contain control logic ❌
* Publish or subscribe topics ❌
* Interact with hardware ❌

---

## 4. Typical Contents

```text
system_interfaces/
├── msg/
│   ├── SystemMode.msg
│   ├── DetectionResult.msg
│   └── TargetError.msg
├── CMakeLists.txt
├── package.xml
└── README.md
```

> Actual message names may vary depending on implementation.

---

## 5. Role in SAFRS Architecture

```text
A-Pi (Vision)
   └─ publishes Detection / Target data
        ▲
        │ (system_interfaces)
        ▼
D-Pi (System Manager)
   └─ consumes system-level messages
```

* A-Pi, B-Pi, C-Pi, D-Pi **all depend on this package**
* No Pi “owns” the logic here
* It is purely a **communication contract**

---

## 6. Design Philosophy

* Interfaces are **stable**
* Logic is **replaceable**
* Communication rules are explicit

This allows:

* Changing perception algorithms without touching system logic
* Modifying navigation internals without breaking D-Pi
* Independent development of each Pi

---

## 7. Build & Usage

This package must be built **before** any package that depends on it.

```bash
colcon build --packages-select system_interfaces
source install/setup.bash
```

Dependent packages must declare:

```xml
<depend>system_interfaces</depend>
```

---

## 8. Common Confusion (Important)

> “I don’t see where this is executed.”

That is **by design**.

* `system_interfaces` is **never executed**
* It is **imported**, not run
* If this package disappears, **the entire system fails to build**

---

## 9. Summary (3 Lines)

* `system_interfaces` defines **shared ROS 2 messages**
* It contains **no executable logic**
* It keeps the SAFRS system **loosely coupled and maintainable**

---

### 한 문장 결론

> **`system_interfaces`는 코드가 아니라
> “약속(Contract)”이다.**

---

