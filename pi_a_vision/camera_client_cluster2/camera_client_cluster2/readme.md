---

# A-Pi : Vision & Target Detection Node

## 1. Purpose

**A-Pi** is responsible for **visual perception and target detection** in the SAFRS system.

It acts as the **eyes of the robot**, detecting targets using a camera and determining
whether the detected object is an **ALLY or ENEMY**.
A-Pi does **not** control motors or system modes directly.

Its role is limited to:

* Perception
* Classification
* Providing targeting information to downstream nodes

---

## 2. Role in System Architecture

```text
[ Camera (USB) ]
        │
        ▼
[ A-Pi (Vision Node) ]
  - Object detection
  - Ally / Enemy classification
  - Target error calculation
        │
        ├── /detect
        ├── /error_xy
        └── /end
        │
        ▼
[ D-Pi (System Manager) ]
```

* **A-Pi** performs perception only
* **D-Pi** makes system-level decisions based on detection results
* **C-Pi** and **B-Pi** do not depend directly on camera data

---

## 3. Responsibilities

### A-Pi DOES

* Capture frames from a USB camera
* Detect objects in the image
* Classify targets as ALLY or ENEMY
* Compute pixel-space targeting error
* Publish detection and tracking status

### A-Pi DOES NOT

* Control motors or gimbal ❌
* Decide system mode ❌
* Perform navigation or SLAM ❌
* Send direct actuation commands ❌

---

## 4. AI Models and Algorithms

### 4.1 Object Detection

* **Model**: EfficientDet-Lite1 (TFLite)
* **Purpose**: Detect objects and estimate bounding boxes
* **Output**:

  * Target center `(cx, cy)`
  * Detection confidence

EfficientDet-Lite1 is selected for **real-time performance on Raspberry Pi**.

---

### 4.2 Target Classification (Hybrid Approach)

A-Pi uses a **hybrid classification strategy** for robustness.

#### (1) Color-Based Heuristics (Primary Override)

* **ALLY detection**

  * Blue pixel ratio > 5%
  * Blue channel stronger than red
* **ENEMY detection**

  * Red pixel ratio > 8%

If these conditions are met, the result **overrides AI output**.

---

#### (2) AI-Based Classification (Secondary)

* **Model**: MobileNet (TFLite, Int8 quantized)
* **Used only when color heuristics are inconclusive**
* Confidence threshold:

  * `< 0.6` → `unknown`

This prevents unreliable decisions under poor lighting or partial occlusion.

---

## 5. Camera Control Logic

### Mode-Based Camera Activation

A-Pi subscribes to `/mode` and adjusts camera usage accordingly.

| Mode        | Camera State      |
| ----------- | ----------------- |
| BOOT        | OFF               |
| NAVI        | OFF (after delay) |
| STANDBY     | ON                |
| TRACK_ALLY  | ON                |
| TRACK_ENEMY | ON                |

* Camera is **disabled during navigation** to save CPU resources
* Camera is **enabled immediately** when tracking is required
* Exposure is set to **-5** to suppress background noise and emphasize LED markers

---

## 6. ROS 2 Interfaces

### Subscribed Topics

| Topic   | Type              | Description         |
| ------- | ----------------- | ------------------- |
| `/mode` | `std_msgs/String` | System mode command |

---

### Published Topics

| Topic       | Type                    | Description                        |
| ----------- | ----------------------- | ---------------------------------- |
| `/error_xy` | `geometry_msgs/Vector3` | Target error vector (pixel space)  |
| `/detect`   | `std_msgs/String`       | Detection result (`ALLY`, `ENEMY`) |
| `/end`      | `std_msgs/String`       | Tracking completion signal         |

#### `/error_xy` Definition

* `x`: Horizontal pixel offset from image center
* `y`: Vertical pixel offset from image center
* `z`:

  * `0.0` → Target detected
  * `-1.0` → No valid target

---

## 7. Completion Condition

A-Pi publishes `/end` when:

* Target horizontal error < **CENTER_THRESHOLD**
* Classification confidence > **0.8**

This indicates that the target is **sufficiently centered**.

---

## 8. Package Structure

```text
camera_client_cluster2/
├── model/
│   ├── EfficientDet-Lite1.tflite
│   └── monkey_classifier_quant_int8.tflite
├── src/
│   └── camera_client_node3.py
├── package.xml
└── setup.py
```

⚠️ Model paths are currently **absolute paths**.
They must match the filesystem or be updated in the code.

---

## 9. Usage

### Build

```bash
cd ~/ros2_ws
colcon build --packages-select camera_client_cluster2
source install/setup.bash
```

### Run

```bash
ros2 run camera_client_cluster2 camera_client_node3
```

---

## 10. Design Philosophy

* Perception and decision-making are **strictly separated**
* AI is **not trusted blindly**
* Rule-based heuristics take priority over ML output
* The vision system provides **information only**, not control

---

## 11. Summary (3 Lines)

* **A-Pi handles vision and target detection**
* Uses **EfficientDet + MobileNet + color heuristics**
* Publishes perception results, never control commands

---

### Final Note

> A-Pi is designed to be **replaceable and isolated**.
> As long as `/detect` and `/error_xy` are preserved,
> the rest of the system remains unaffected.
