#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import platform
import time
import threading
import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Vector3

# ----------------------------
# TFLite ½ÇÇà µµ±¸ ¼³Á¤
# ----------------------------
if platform.system() == "Windows":
    import tensorflow as tf
    Interpreter = tf.lite.Interpreter
else:
    import tflite_runtime.interpreter as tflite
    Interpreter = tflite.Interpreter

# ----------------------------
# ¸ðµ¨ °æ·Î
# ----------------------------
DET_MODEL_PATH = "/home/ubuntu/ros2_ws/src/camera_client_cluster2/camera_client_cluster2/model/EfficientDet-Lite1.tflite"
CLASS_MODEL_PATH = "/home/ubuntu/ros2_ws/src/camera_client_cluster2/camera_client_cluster2/model/monkey_classifier_quant_int8.tflite"
CLASS_IMG_SIZE = (224, 224)

# ----------------------------
# »ö»ó + AI ºÐ·ù±â
# ----------------------------
class ColorClassifier:
    def __init__(self, model_path: str):
        self.interpreter = Interpreter(model_path=model_path)
        self.interpreter.allocate_tensors()
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()

    def _get_red_pixel_ratio(self, roi_bgr):
        hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)
        lower_red1 = np.array([0, 70, 50])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 70, 50])
        upper_red2 = np.array([180, 255, 255])
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        red_mask = cv2.bitwise_or(mask1, mask2)
        return cv2.countNonZero(red_mask) / (roi_bgr.shape[0] * roi_bgr.shape[1]) if roi_bgr.size > 0 else 0.0

    def _get_blue_pixel_ratio(self, roi_bgr):
        """ÇÏ´Ã»ö(Sky Blue) ¹üÀ§¸¦ Æ÷ÇÔÇÏµµ·Ï ÃÖÀûÈ­"""
        hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)
        lower_sky_blue = np.array([80, 30, 50])
        upper_sky_blue = np.array([135, 255, 255])
        blue_mask = cv2.inRange(hsv, lower_sky_blue, upper_sky_blue)
        return cv2.countNonZero(blue_mask) / (roi_bgr.shape[0] * roi_bgr.shape[1]) if roi_bgr.size > 0 else 0.0

    def classify(self, roi_bgr):
        if roi_bgr is None or roi_bgr.size == 0:
            return "unknown", 0.0

        red_ratio = self._get_red_pixel_ratio(roi_bgr)
        blue_ratio = self._get_blue_pixel_ratio(roi_bgr)

        # AI ºÐ·ù ¸ðµ¨ Ãß·Ð (224x224)
        input_data = cv2.resize(roi_bgr, CLASS_IMG_SIZE).astype(np.float32)
        in_scale, in_zero = self.input_details[0]["quantization"]
        if in_scale != 0:
            input_data = (input_data / in_scale) + in_zero
        input_data = np.expand_dims(input_data, axis=0).astype(self.input_details[0]["dtype"])

        self.interpreter.set_tensor(self.input_details[0]["index"], input_data)
        self.interpreter.invoke()
        output = self.interpreter.get_tensor(self.output_details[0]["index"])[0]

        # È®·ü °è»ê
        scores = np.exp(output - np.max(output))
        scores /= scores.sum()
        idx = int(np.argmax(scores))
        dl_label = "ENEMY" if idx == 0 else "ALLY"
        dl_conf = float(scores[idx])

        # »ö»ó ºñÁß ±â¹Ý ÃÖÁ¾ º¸Á¤
        if blue_ratio > 0.05 and blue_ratio > red_ratio:
            return "ALLY", max(dl_conf, 0.93)
        if red_ratio > 0.08:
            return "ENEMY", max(dl_conf, 0.93)

        if dl_conf < 0.6:
            return "unknown", dl_conf
        return dl_label, dl_conf

# ----------------------------
# ¸ÞÀÎ Camera FSM ³ëµå (Ä¿½ºÅÒ msg Á¦°Å ¹öÀü)
# ----------------------------
class CameraNode(Node):
    """
    Sub:
      /mode (std_msgs/String) : "NAVI", "STANDBY", "TRACK_ALLY", "TRACK_ENEMY"
    Pub:
      /error_xy (geometry_msgs/Vector3): x=err_x, y=err_y, z=0.0, (no-detect => z=-1.0)
      /detect (std_msgs/String): "LABEL,cx,cy,conf"
      /end (std_msgs/String): "end"  (¿ä±¸»çÇ×: µ¥ÀÌÅÍ ¼Ò¹®ÀÚ end)
    """
    def __init__(self):
        super().__init__("camera_node")

        self.mode = "BOOT"
        self.running = True

        self.CENTER_THRESHOLD = 25
        self.CAMERA_OFF_DELAY_SEC = 5.0
        self.camera_off_timer = None

        # Å½Áö ¸ðµ¨ ÃÊ±âÈ­ ¹× ÀÔ·Â ±Ô°Ý ÀÚµ¿ È®ÀÎ
        self.det_interpreter = Interpreter(model_path=DET_MODEL_PATH)
        self.det_interpreter.allocate_tensors()
        self.det_in_details = self.det_interpreter.get_input_details()
        self.det_out_details = self.det_interpreter.get_output_details()

        # ¸ðµ¨ÀÌ ¿ä±¸ÇÏ´Â ÀÔ·Â Å©±â (¿¹: EfficientDet-Lite1Àº 384)
        self.det_input_h = int(self.det_in_details[0]["shape"][1])
        self.det_input_w = int(self.det_in_details[0]["shape"][2])

        self.classifier = ColorClassifier(CLASS_MODEL_PATH)

        self.cap = None
        self.latest_frame = None
        self.frame_lock = threading.Lock()
        self.last_ai_result = {"box": None, "label": "Wait...", "conf": 0.0, "b_ratio": 0.0}

        # ---- ÅäÇÈ ¼³Á¤ (Ç¥ÁØ ¸Þ½ÃÁö·Î¸¸ Åë½Å) ----
        self.sub_mode = self.create_subscription(String, "/mode", self.cb_mode, 10)
        self.pub_error = self.create_publisher(Vector3, "/error_xy", 10)
        self.pub_detect = self.create_publisher(String, "/detect", 10)
        self.pub_end = self.create_publisher(String, "/end", 10)  # ¼Ò¹®ÀÚ /end

        threading.Thread(target=self.capture_loop, daemon=True).start()
        threading.Thread(target=self.inference_loop, daemon=True).start()
        threading.Thread(target=self.display_loop, daemon=True).start()

        self.get_logger().info(f"CameraNode started. Det Input Size: {self.det_input_w}x{self.det_input_h}")

    def cb_mode(self, msg: String):
        new_mode = (msg.data or "").strip().upper()
        if not new_mode:
            return
        if self.mode == new_mode:
            return

        self.get_logger().info(f"[MODE] {self.mode} -> {new_mode}")
        self.mode = new_mode

        # ±âÁ¸ Å¸ÀÌ¸Ó Á¤¸®
        if self.camera_off_timer is not None:
            try:
                self.camera_off_timer.cancel()
            except Exception:
                pass
            self.destroy_timer(self.camera_off_timer)
            self.camera_off_timer = None

        if self.mode in ("STANDBY", "TRACK_ALLY", "TRACK_ENEMY"):
            self._open_camera()
        elif self.mode == "NAVI":
            self.camera_off_timer = self.create_timer(self.CAMERA_OFF_DELAY_SEC, self._delayed_camera_off)

    def _open_camera(self):
        if self.cap is not None:
            return
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)  # ÀÚµ¿ ³ëÃâ È°¼ºÈ­
        self.cap.set(cv2.CAP_PROP_EXPOSURE, -5)      # ¾à°£ ¾îµÓ°Ô ¼³Á¤(»ö»óÀÎ½Ä À¯¸®)
        self.get_logger().info("Camera ON")

    def _delayed_camera_off(self):
        # 1È¸ ½ÇÇà ÈÄ Å¸ÀÌ¸Ó Á¦°Å
        if self.camera_off_timer is not None:
            try:
                self.camera_off_timer.cancel()
            except Exception:
                pass
            self.destroy_timer(self.camera_off_timer)
            self.camera_off_timer = None

        if self.mode == "NAVI":
            self._release_camera()

    def _release_camera(self):
        if self.cap:
            self.cap.release()
            self.cap = None
            self.latest_frame = None
            self.get_logger().info("Camera OFF")

    def capture_loop(self):
        while self.running:
            if self.cap:
                ret, frame = self.cap.read()
                if ret:
                    with self.frame_lock:
                        self.latest_frame = frame
            time.sleep(0.01)

    def inference_loop(self):
        while self.running:
            if self.latest_frame is None or self.mode == "BOOT":
                time.sleep(0.1)
                continue

            with self.frame_lock:
                img = self.latest_frame.copy()

            H, W = img.shape[:2]

            # ¸ðµ¨ÀÌ ¿øÇÏ´Â Å©±â·Î ¸®»çÀÌÁî
            det_input = cv2.resize(img, (self.det_input_w, self.det_input_h))
            det_input = np.expand_dims(det_input, axis=0).astype(self.det_in_details[0]["dtype"])

            self.det_interpreter.set_tensor(self.det_in_details[0]["index"], det_input)
            self.det_interpreter.invoke()

            boxes = self.det_interpreter.get_tensor(self.det_out_details[0]["index"])[0]
            scores = self.det_interpreter.get_tensor(self.det_out_details[2]["index"])[0]

            if scores[0] > 0.4:
                b = boxes[0]
                y1, x1, y2, x2 = int(b[0] * H), int(b[1] * W), int(b[2] * H), int(b[3] * W)
                x1 = max(0, x1); y1 = max(0, y1); x2 = min(W, x2); y2 = min(H, y2)

                roi = img[y1:y2, x1:x2]

                # ºÐ·ù ¹× »ö»ó ºñÀ² È®ÀÎ
                b_ratio = self.classifier._get_blue_pixel_ratio(roi)
                label, conf = self.classifier.classify(roi)

                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                err_x, err_y = float(cx - W // 2), float(cy - H // 2)

                # /error_xy
                self.pub_error.publish(Vector3(x=err_x, y=err_y, z=0.0))

                # /detect : "LABEL,cx,cy,conf"
                detect_msg = String()
                detect_msg.data = f"{label},{float(cx):.1f},{float(cy):.1f},{conf:.3f}"
                self.pub_detect.publish(detect_msg)

                # µð½ºÇÃ·¹ÀÌ¿ë °á°ú ÀúÀå
                self.last_ai_result = {"box": (x1, y1, x2, y2), "label": label, "conf": conf, "b_ratio": b_ratio}

                # /end : ¼Ò¹®ÀÚ "end" ¹ßÇà
                if abs(err_x) < self.CENTER_THRESHOLD and conf > 0.8:
                    if self.mode == "STANDBY" or \
                       (self.mode == "TRACK_ALLY" and label == "ALLY") or \
                       (self.mode == "TRACK_ENEMY" and label == "ENEMY"):
                        end_msg = String()
                        end_msg.data = "end"  # ¿ä±¸»çÇ×: ¼Ò¹®ÀÚ end
                        self.pub_end.publish(end_msg)
            else:
                self.last_ai_result["box"] = None
                self.pub_error.publish(Vector3(x=0.0, y=0.0, z=-1.0))

    def display_loop(self):
        while self.running:
            if self.latest_frame is None:
                time.sleep(0.1)
                continue
            with self.frame_lock:
                frame = self.latest_frame.copy()

            H, W = frame.shape[:2]
            res = self.last_ai_result

            # ¸ðµå Ç¥½Ã ¹× ½ÊÀÚ¼±
            cv2.putText(frame, f"MODE: {self.mode}", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.line(frame, (W // 2, 0), (W // 2, H), (100, 100, 100), 1)
            cv2.line(frame, (0, H // 2), (W, H // 2), (100, 100, 100), 1)

            if res.get("box"):
                x1, y1, x2, y2 = res["box"]
                color = (0, 0, 255) if res["label"] == "ENEMY" else (255, 255, 0)
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                cv2.putText(frame, f"{res['label']} (B:{res['b_ratio']:.2f})", (x1, max(0, y1 - 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

            cv2.imshow("Camera View", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    def destroy_node(self):
        self.running = False
        self._release_camera()
        cv2.destroyAllWindows()
        super().destroy_node()

def main():
    rclpy.init()
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

