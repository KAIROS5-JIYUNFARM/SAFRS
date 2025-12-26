#!/usr/bin/env python3
# A-Pi Camera Client (MobileNetV2 Color Detection + FSM Fixed)
#
# ÃÖÁ¾ ¾÷µ¥ÀÌÆ®: 2025-12-12 PM 12:30 (³í¸® ¿À·ù ¹æÁö ·ÎÁ÷ Ãß°¡)
# + 2025-12-XX detect ½ÇÆÐ 5ÃÊ ¡æ NAV ·ÎÁ÷ Ãß°¡
# 251215 PM 14:56 LAST UPDATE

import platform
import time

import cv2
import numpy as np
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Vector3


# ============================================================
# 0) Interpreter selection and Model Paths (UNCHANGED)
# ============================================================
if platform.system() == "Windows":
    import tensorflow as tf
    Interpreter = tf.lite.Interpreter
else:
    import tflite_runtime.interpreter as tflite
    Interpreter = tflite.Interpreter


POSE_MODEL_PATH = "/home/ubuntu/ros_ws/src/camera_client_cluster/camera_client_cluster/model/pose_landmarker_lite.task"
CLASS_MODEL_PATH = "/home/ubuntu/ros_ws/src/camera_client_cluster/camera_client_cluster/model/mobilenetv2_int8_v2.tflite"
IMG_SIZE = (160, 160)

# ============================================================
# 1) Load classifier details (UNCHANGED)
# ============================================================
interpreter = Interpreter(model_path=CLASS_MODEL_PATH)
interpreter.allocate_tensors()
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()
in_idx = input_details[0]["index"]
in_scale, in_zero = input_details[0]["quantization"]
out_idx = output_details[0]["index"]

# ============================================================
# 2) Utility functions (UNCHANGED)
# ============================================================

def softmax(x):
    x = x.astype(np.float32)
    e = np.exp(x - np.max(x))
    return e / np.sum(e)

def classify_color(roi_bgr):
    if roi_bgr is None or roi_bgr.size == 0:
        return "unknown", 0.0

    input_data = cv2.resize(roi_bgr, IMG_SIZE).astype(np.float32)
    if in_scale != 0:
        input_data = input_data / in_scale + in_zero
    input_data = np.expand_dims(input_data, axis=0).astype(input_details[0]["dtype"])

    interpreter.set_tensor(in_idx, input_data)
    interpreter.invoke()
    output = interpreter.get_tensor(out_idx)[0]

    if output_details[0]["dtype"] == np.uint8:
        out_scale, out_zero = output_details[0]["quantization"]
        output = (output.astype(np.float32) - out_zero) * out_scale

    scores = softmax(output)

    if scores[0] > scores[1] and scores[0] > 0.5:
        return "ally", scores[0]
    elif scores[1] > scores[0] and scores[1] > 0.5:
        return "enemy", scores[1]
    else:
        return "unknown", np.max(scores)


# ============================================================
# Mediapipe Pose (UNCHANGED)
# ============================================================
BaseOptions = python.BaseOptions
PoseLandmarkerOptions = vision.PoseLandmarkerOptions
RunningMode = vision.RunningMode

pose_landmarker = vision.PoseLandmarker.create_from_options(
    PoseLandmarkerOptions(
        base_options=BaseOptions(model_asset_path=POSE_MODEL_PATH),
        running_mode=RunningMode.IMAGE,
        num_poses=1
    )
)

def crop_upper_body(frame, keypoints, margin=30):
    try:
        Ls, Rs, Lh, Rh = keypoints[11], keypoints[12], keypoints[23], keypoints[24]
        xs = np.array([Ls[0], Rs[0], Lh[0], Rh[0]])
        ys = np.array([Ls[1], Rs[1], Lh[1], Rh[1]])
        x1 = int(max(xs.min() - margin, 0))
        y1 = int(max(ys.min() - margin, 0))
        x2 = int(min(xs.max() + margin, frame.shape[1]))
        y2 = int(min(ys.max() + margin, frame.shape[0]))
        if x2 <= x1 or y2 <= y1:
            return None, None
        return frame[y1:y2, x1:x2], (x1, y1, x2, y2)
    except:
        return None, None


def _auto_open_camera():
    for idx in range(10):
        cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
        if cap.isOpened():
            ret, frame = cap.read()
            if ret:
                return cap
            cap.release()
    return None


# ============================================================
# A-Pi Camera Node
# ============================================================
class CameraClientNodeColor(Node):
    def __init__(self):
        super().__init__('camera_client_node_color')

        self.mode = "nav"
        self.cap = None

        self.detect_count = 0
        self.last_label = None
        self.last_detect_time = 0.0

        self.sent_ally = False
        self.sent_enemy = False

        self.track_target_class = None
        self.track_start_time = 0.0
        self.TRACK_TIMEOUT_SEC = 10.0

        self.error_send_count = 0
        self.MAX_ERROR_SEND = 5

        self.stby_enter_time = 0.0

        # ====================================================
        # [ADD] detect ½ÇÆÐ °¨½Ã¿ë º¯¼ö
        # ¸¶Áö¸·À¸·Î "»ç¶÷ÀÌ º¸¿´´ø ½Ã°£"À» ÀúÀå
        # ====================================================
        self.last_detect_success_time = time.time()
        self.DETECT_FAIL_TIMEOUT = 5.0  # 5ÃÊ µ¿¾È ¾Æ¹«µµ ¾È º¸ÀÌ¸é NAV

        # ROS
        self.sub_mode = self.create_subscription(String, '/mode', self.cb_mode, 10)
        self.sub_trigger_done = self.create_subscription(String, '/c_trigger_done', self.cb_trigger_done, 10)
        self.sub_servo_stabilized = self.create_subscription(String, '/servo_stabilized', self.cb_servo_stabilized, 10)

        self.pub_status = self.create_publisher(String, '/a_target_status', 10)
        self.pub_error = self.create_publisher(Vector3, '/c_error_angle', 10)
        self.pub_mode_cmd = self.create_publisher(String, '/mode', 10)

        self.timer = self.create_timer(0.066, self.loop)

        self.get_logger().info("[INIT] Start mode = nav")


    def cb_mode(self, msg):
        prev = self.mode
        self.mode = msg.data.strip().lower()

        if prev == "nav" and self.mode == "stby":
            self.sent_ally = False
            self.sent_enemy = False
            self.track_target_class = None
            self.detect_count = 0
            self.stby_enter_time = time.time()

            # [ADD] STBY ÁøÀÔ ½Ã detect Å¸ÀÌ¸Ó ÃÊ±âÈ­
            self.last_detect_success_time = time.time()

        if self.mode == "nav":
            if self.cap:
                self.cap.release()
                self.cap = None
            cv2.destroyAllWindows()

        self.get_logger().info(f"[MODE] {prev} ¡æ {self.mode}")


    def cb_trigger_done(self, msg):
        if self.mode == "track" and self.track_target_class == "enemy":
            self.sent_enemy = True
            self.track_target_class = None
            self.pub_mode_cmd.publish(String(data="stby"))


    def cb_servo_stabilized(self, msg):
        if self.mode == "track" and self.track_target_class == "ally":
            self.sent_ally = True
            self.track_target_class = None
            self.pub_mode_cmd.publish(String(data="nav"))


    # ============================================================
    # MAIN LOOP
    # ============================================================
    def loop(self):

        if self.mode == "nav":
            return

        # ====================================================
        # [ADD] detect ½ÇÆÐ 5ÃÊ ¡æ NAV
        # STBY / TRACK °øÅë ¾ÈÀü Å»Ãâ ·ÎÁ÷
        # ====================================================
        if self.mode in ("stby", "track"):
            if time.time() - self.last_detect_success_time > self.DETECT_FAIL_TIMEOUT:
                self.get_logger().error("[FSM] Detect failed for 5s ¡æ NAV")
                self.pub_mode_cmd.publish(String(data="nav"))
                return

        if self.cap is None:
            self.cap = _auto_open_camera()
            if self.cap is None:
                return

        ok, frame = self.cap.read()
        if not ok:
            return

        H, W = frame.shape[:2]
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=frame)
        result = pose_landmarker.detect(mp_image)

        if result.pose_landmarks:
            keypoints = np.array([[lm.x * W, lm.y * H, lm.z]
                                  for lm in result.pose_landmarks[0]])

            roi, box = crop_upper_body(frame, keypoints)
            if roi is not None:

                # ====================================================
                # [ADD] »ç¶÷ ÇüÃ¼°¡ º¸ÀÌ¸é detect ¼º°øÀ¸·Î ÆÇ´Ü
                # Å¬·¡½º(ally/enemy/unknown)¿Í ¹«°ü
                # ====================================================
                self.last_detect_success_time = time.time()

                color, conf = classify_color(roi)
                self.get_logger().info(f"[DETECT] {color} ({conf:.2f})")

                if self.mode == "track" and color == self.track_target_class:
                    if self.error_send_count < self.MAX_ERROR_SEND:
                        x1, y1, x2, y2 = box
                        cx = (x1 + x2) // 2
                        cy = (y1 + y2) // 2
                        self.pub_error.publish(Vector3(
                            x=float(cx - W // 2),
                            y=float(cy - H // 2),
                            z=0.0
                        ))
                        self.error_send_count += 1

                if self.mode == "stby" and color in ("ally", "enemy"):
                    now = time.time()
                    if now - self.last_detect_time >= 1.0:
                        self.last_detect_time = now
                        if color == self.last_label:
                            self.detect_count += 1
                        else:
                            self.last_label = color
                            self.detect_count = 1

                        if self.detect_count >= 3:
                            self.pub_status.publish(String(data=color))
                            self.pub_mode_cmd.publish(String(data="track"))
                            self.track_target_class = color
                            self.detect_count = 0

                x1, y1, x2, y2 = box
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0,255,255), 2)

        if self.mode in ("stby", "track"):
            cv2.imshow("A Node View", frame)
            cv2.waitKey(1)


    def destroy_node(self):
        if self.cap:
            self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


# ============================================================
# MAIN
# ============================================================
def main(args=None):
    rclpy.init(args=args)
    node = CameraClientNodeColor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

