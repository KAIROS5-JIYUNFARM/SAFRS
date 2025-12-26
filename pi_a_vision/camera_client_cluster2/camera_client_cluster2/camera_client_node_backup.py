#!/usr/bin/env python3
# A-Pi Camera Client (MobileNetV2 Color Detection + FSM Fixed)
#
# ÃÖÁ¾ ¾÷µ¥ÀÌÆ®: 2025-12-12 PM 12:30 (³í¸® ¿À·ù ¹æÁö ·ÎÁ÷ Ãß°¡)

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
# 2) Utility functions for classification and Pose Estimation (UNCHANGED)
# ============================================================

def softmax(x):
    # Softmax function for output scores
    x = x.astype(np.float32)
    e = np.exp(x - np.max(x))
    return e / np.sum(e)

def get_adaptive_thresholds(hsv_roi):
    # This function is retained from the original color logic structure
    # but might not be fully used by the MobileNet model directly.
    V_mean = np.mean(hsv_roi[:, :, 2])
    S_base = 50
    V_base = 50
    if V_mean < 80:
        S_base -= 15
        V_base -= 15
    if V_mean < 50:
        S_base -= 25
        V_base -= 25
    return max(20, S_base), max(20, V_base)

def classify_color(roi_bgr):
    # ------------------------------------------------------------------
    # ¡Ú¡Ú¡Ú ¸¶Áö¸· ¾÷µ¥ÀÌÆ®: ½ÇÁ¦ MobileNet V2 ±â¹Ý »ö»ó ºÐ·ù ·ÎÁ÷ º¹±¸ ¡Ú¡Ú¡Ú
    # ------------------------------------------------------------------
    
    # 1. Input Check
    if roi_bgr is None or roi_bgr.size == 0:
        return "unknown", 0.0

    # 2. Preprocessing
    input_data = cv2.resize(roi_bgr, IMG_SIZE)
    input_data = input_data.astype(np.float32)

    # Quantization (if applicable)
    if in_scale != 0:
        input_data = input_data / in_scale + in_zero
    input_data = np.expand_dims(input_data, axis=0).astype(input_details[0]["dtype"])

    # 3. Inference
    interpreter.set_tensor(in_idx, input_data)
    interpreter.invoke()
    output = interpreter.get_tensor(out_idx)[0]

    # 4. Post-processing (Softmax and Labeling)
    if output_details[0]["dtype"] == np.uint8:
        out_scale, out_zero = output_details[0]["quantization"]
        output = (output.astype(np.float32) - out_zero) * out_scale
    
    scores = softmax(output)
    
    # [0: ally, 1: enemy, 2: other] (MobileNetV2 labels assumed)
    if scores[0] > scores[1] and scores[0] > 0.5:
        return "ally", scores[0]
    elif scores[1] > scores[0] and scores[1] > 0.5:
        return "enemy", scores[1]
    else:
        return "unknown", np.max(scores)

# --- Mediapipe settings (unchanged) ---
BaseOptions = python.BaseOptions
PoseLandmarkerOptions = vision.PoseLandmarkerOptions
RunningMode = vision.RunningMode

base_options = BaseOptions(model_asset_path=POSE_MODEL_PATH)
options = PoseLandmarkerOptions(
    base_options=base_options,
    running_mode=RunningMode.IMAGE,
    num_poses=1
)
pose_landmarker = vision.PoseLandmarker.create_from_options(options)

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
    """»ç¿ë °¡´ÉÇÑ Ä«¸Þ¶ó Æ÷Æ®¸¦ ÀÚµ¿À¸·Î Ã£¾Æ¼­ Ä¸Ã³ °´Ã¼¸¦ ¹ÝÈ¯ÇÕ´Ï´Ù."""
    for idx in range(10):
        cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
        if cap.isOpened():
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            ret, frame = cap.read()
            if ret:
                return cap
            else:
                cap.release()
    return None


# ============================================================
# A-Pi Camera Node (FSM Fixed Version)
# ============================================================
class CameraClientNodeColor(Node):
    def __init__(self):
        super().__init__('camera_client_node_color')

        self.mode = "nav"
        self.cap = None

        self.last_label = None
        self.detect_count = 0
        
        self.error_send_count = 0
        self.MAX_ERROR_SEND = 5

        # Detection completion status: TRACK ¸ðµå ÇÇµå¹é ÈÄ ¼³Á¤µÊ
        self.sent_ally = False
        self.sent_enemy = False

        self.track_target_class = None
        self.last_detect_time = 0.0
        self.track_start_time = 0.0
        self.TRACK_TIMEOUT_SEC = 10.0

        # ¡Ú¡Ú¡Ú ¾ÈÀü¼º °³¼±: STBY ¸ðµå ÁøÀÔ ½Ã°£À» ±â·ÏÇÏ´Â º¯¼ö Ãß°¡
        self.stby_enter_time = 0.0

        # ========= ROS Communication ============= (UNCHANGED)
        self.sub_mode = self.create_subscription(String, '/mode', self.cb_mode, 10)
        self.sub_trigger_done = self.create_subscription(
            String, '/c_trigger_done', self.cb_trigger_done, 10
        )
        # ¡Ú¡Ú¡Ú ¸¶Áö¸· ¾÷µ¥ÀÌÆ®: Fake Target Switch ÅäÇÈ »èÁ¦µÊ ¡Ú¡Ú¡Ú
        self.pub_status = self.create_publisher(String, '/a_target_status', 10)
        self.pub_error = self.create_publisher(Vector3, '/c_error_angle', 10)
        self.pub_mode_cmd = self.create_publisher(String, '/mode', 10)
        
        self.sub_servo_stabilized = self.create_subscription(
            String, '/servo_stabilized', self.cb_servo_stabilized, 10
        )
        
        self.timer = self.create_timer(0.066, self.loop)

        self.get_logger().info(f"[INIT] Start mode = {self.mode}")


    # --------------------------------------------------------
    # [ÄÝ¹é] Ally ÃßÀû ¿Ï·á ½Ã STBY º¹±Í ¿äÃ» (Servo ½ÅÈ£°¡ Ally ÀÓ¹« ¿Ï·á Á¶°Ç)
    # --------------------------------------------------------
    def cb_servo_stabilized(self, msg: String):
        if (self.mode == "track" and 
            self.track_target_class == "ally" and 
            msg.data == "ally_stabilized"):
            
            self.get_logger().info(
                f"[TRACK] Ally Servo stabilized: '{msg.data}', Ally Mission Complete!"
            )
            # ¡Ú¡Ú¡Ú FSM ¼öÁ¤: Ally ÀÓ¹« ¿Ï·á ÇÃ·¡±×¸¦ Servo ½ÅÈ£ ¼ö½Å ½Ã ¼³Á¤ (À¯Áö) ¡Ú¡Ú¡Ú
            self.sent_ally = True
            
            # TRACK º¯¼ö ÃÊ±âÈ­
            self.track_target_class = None
            self.detect_count = 0
            self.error_send_count = 0
            self.track_start_time = 0.0
            
            # ¡Ú¡Ú¡Ú FSM ¼öÁ¤: NAV ¶Ç´Â STBY ºÐ±â (µÎ ÀÓ¹« ¸ðµÎ ¿Ï·á ½Ã NAV) (À¯Áö) ¡Ú¡Ú¡Ú
            if self.sent_ally and self.sent_enemy:
                self.get_logger().info("[FSM] Ally & Enemy both done ¡æ NAV")
                self.pub_mode_cmd.publish(String(data="nav"))
            else:
                self.get_logger().info("[FSM] Remaining mission(s) ¡æ STBY")
                self.pub_mode_cmd.publish(String(data="stby"))


    # --------------------------------------------------------
    # [ÄÝ¹é] Receive mode change
    # --------------------------------------------------------
    def cb_mode(self, msg):
        prev = self.mode
        new_mode = msg.data.strip().lower()

        # NAV ¡æ STBY ÁøÀÔ ½Ã ÇÃ·¡±× ¸®¼Â
        if prev == "nav" and new_mode == "stby":
            self.get_logger().info("[MODE] Starting New Mission: Flags Reset.")
            self.sent_ally = False
            self.sent_enemy = False
            self.track_target_class = None
            self.detect_count = 0
            self.track_start_time = 0.0
            # ¡Ú¡Ú¡Ú ¾ÈÀü¼º °³¼±: STBY ÁøÀÔ ½Ã°£ ±â·Ï (TRACK ¿äÃ» Ãæµ¹ ¹æÁö) ¡Ú¡Ú¡Ú
            self.stby_enter_time = time.time() 

        if self.mode == "track" and new_mode == "stby":
            self.error_send_count = 0

        self.mode = new_mode
        
        if self.mode == "track":
            self.track_start_time = time.time()

        # NAV ¸ðµå ½Ã Ä«¸Þ¶ó Á¾·á ¹× »óÅÂ ÃÊ±âÈ­
        if self.mode == "nav":
            self.sent_ally = False
            self.sent_enemy = False
            self.track_target_class = None
            self.detect_count = 0
            self.last_label = None
            self.last_detect_time = 0.0
            self.error_send_count = 0
            self.track_start_time = 0.0
            self.stby_enter_time = 0.0 # ¡Ú¡Ú¡Ú ¾ÈÀü¼º °³¼±: ÃÊ±âÈ­
            
            if self.cap is not None:
                self.cap.release()
                self.cap = None
            cv2.destroyAllWindows()

        self.get_logger().info(f"[MODE] {prev} ¡æ {self.mode}")


    # --------------------------------------------------------
    # [ÄÝ¹é] C-Pi trigger done (Enemy fire completed) (Trigger ½ÅÈ£°¡ Enemy ÀÓ¹« ¿Ï·á Á¶°Ç)
    # --------------------------------------------------------
    def cb_trigger_done(self, msg: String):
        if self.mode == "track" and self.track_target_class == "enemy":
            self.get_logger().info(
                f"[TRIGGER] C-Pi trigger done: '{msg.data}', Enemy Mission Complete!"
            )
            # ¡Ú¡Ú¡Ú FSM ¼öÁ¤: Enemy ÀÓ¹« ¿Ï·á ÇÃ·¡±×¸¦ Trigger ½ÅÈ£ ¼ö½Å ½Ã ¼³Á¤ (À¯Áö) ¡Ú¡Ú¡Ú
            self.sent_enemy = True
            
            # TRACK º¯¼ö ÃÊ±âÈ­
            self.track_target_class = None
            self.detect_count = 0
            self.error_send_count = 0
            self.track_start_time = 0.0
            
            # ¡Ú¡Ú¡Ú FSM ¼öÁ¤: NAV ¶Ç´Â STBY ºÐ±â (µÎ ÀÓ¹« ¸ðµÎ ¿Ï·á ½Ã NAV) (À¯Áö) ¡Ú¡Ú¡Ú
            if self.sent_ally and self.sent_enemy:
                self.get_logger().info("[FSM] Ally & Enemy both done ¡æ NAV")
                self.pub_mode_cmd.publish(String(data="nav"))
            else:
                self.get_logger().info("[FSM] Remaining mission(s) ¡æ STBY")
                self.pub_mode_cmd.publish(String(data="stby"))


    # ============================================================
    # MAIN LOOP
    # ============================================================
    def loop(self):

        if self.mode == "nav":
            return

        # [TRACK ¸ðµå Å¸ÀÓ¾Æ¿ô Ã¼Å©] (Enemy »Ó¸¸ ¾Æ´Ï¶ó Ally¿¡°Ôµµ Àû¿ëÇÏ¿© ¹«ÇÑ ´ë±â ¹æÁö)
        if self.mode == "track":
            
            # ¡Ú¡Ú¡Ú ¾ÈÀü¼º °³¼±: Ally/Enemy ¸ðµÎ Å¸ÀÓ¾Æ¿ô Ã¼Å© (Ally ¹«ÇÑ ´ë±â ¹æÁö) ¡Ú¡Ú¡Ú
            # ¾ÆÁ÷ ÀÓ¹« ¿Ï·á°¡ ¾ÈµÈ Å¸°Ù¿¡ ´ëÇØ¼­¸¸ Ã¼Å©
            is_target_active = (self.track_target_class == "enemy" and not self.sent_enemy) or \
                               (self.track_target_class == "ally" and not self.sent_ally)
            
            if is_target_active and time.time() - self.track_start_time > self.TRACK_TIMEOUT_SEC:
                
                self.get_logger().error(
                    f"[TRACK] TIMEOUT! {self.track_target_class.upper()} response failed after {self.TRACK_TIMEOUT_SEC}s. Force requesting STBY."
                )
                
                # Å¸ÀÓ¾Æ¿ô ½Ã ÇØ´ç ÀÓ¹« ¿Ï·á Ã³¸® (µ¥µå¶ô ¹æÁö)
                if self.track_target_class == "enemy":
                    self.sent_enemy = True 
                elif self.track_target_class == "ally":
                    self.sent_ally = True # ¡Ú¡Ú¡Ú ¼öÁ¤: Allyµµ Å¸ÀÓ¾Æ¿ô Ã³¸®
                    
                self.track_target_class = None
                self.detect_count = 0
                self.error_send_count = 0
                self.track_start_time = 0.0
                
                # NAV/STBY ºÐ±â ·ÎÁ÷ Àû¿ë
                if self.sent_ally and self.sent_enemy:
                    self.pub_mode_cmd.publish(String(data="nav"))
                else:
                    self.pub_mode_cmd.publish(String(data="stby"))
                return


        if self.cap is None:
            self.cap = _auto_open_camera()
            if self.cap is None:
                self.get_logger().warn("[CAM] No camera detected. Retrying...")
                return
            # Ä«¸Þ¶ó ¼³Á¤
            self.get_logger().info("[CAM] Camera opened (auto-detected).")
            self.cap.set(cv2.CAP_PROP_AUTO_WB, 0)
            self.cap.set(cv2.CAP_PROP_WB_TEMPERATURE, 4600)
            self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75)
            self.cap.set(cv2.CAP_PROP_EXPOSURE, 120)
            self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)


        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warn("[CAM] Frame read failed. Releasing camera.")
            self.cap.release()
            self.cap = None
            return

        H, W = frame.shape[:2]

        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=frame)
        result = pose_landmarker.detect(mp_image)

        if result.pose_landmarks:
            keypoints = np.array([[lm.x * W, lm.y * H, lm.z]
                                  for lm in result.pose_landmarks[0]])

            roi, box = crop_upper_body(frame, keypoints)
            if roi is not None:

                color, conf = classify_color(roi)
                # ¡Ú¡Ú¡Ú ¸¶Áö¸· ¾÷µ¥ÀÌÆ®: Fake Target ·Î±× Á¦°Å ¹× ½ÇÁ¦ °¨Áö Á¤º¸ Ãâ·Â ¡Ú¡Ú¡Ú
                self.get_logger().info(f"[DETECT] {color} ({conf:.2f})")

                # =========================================
                # TRACK Mode
                # =========================================
                if self.mode == "track":
                    
                    if self.track_target_class is None:
                        if color in ("ally", "enemy"):
                            self.track_target_class = color
                            self.get_logger().info(f"[TRACK] Target fixed ¡æ {self.track_target_class}")
                            self.error_send_count = 0
                            self.track_start_time = time.time()
                        else:
                            color = "ignore"

                    if color == self.track_target_class:
                        
                        # ¿¡·¯ °¢µµ Àü¼Û
                        if self.error_send_count < self.MAX_ERROR_SEND:
                            x1, y1, x2, y2 = box
                            cx = (x1 + x2) // 2
                            cy = (y1 + y2) // 2
                            err_yaw = cx - W // 2
                            err_pitch = cy - H // 2

                            self.pub_error.publish(Vector3(
                                x=float(err_yaw),
                                y=float(err_pitch),
                                z=0.0
                            ))
                            self.error_send_count += 1
                            self.get_logger().info(f"[TRACK] error ¡æ yaw={err_yaw}, pitch={err_pitch} ({self.error_send_count}/{self.MAX_ERROR_SEND})")

                # =========================================
                # STBY Mode
                # =========================================
                if self.mode == "stby" and color in ("ally", "enemy"):

                    # Enemy priority (UNCHANGED)
                    if (not self.sent_enemy) and (color == "ally"):
                        self.get_logger().info("[STBY] Enemy not processed yet ¡æ ignore ally detection")
                    else:
                        # 1ÃÊ¿¡ ÇÑ ¹ø¸¸ Ä«¿îÆ® Ã¼Å© (UNCHANGED)
                        now = time.time()
                        if now - self.last_detect_time >= 1.0:
                            self.last_detect_time = now

                            # 3ÃÊ Ä«¿îÆ® ½ÃÀÛ (¾ÆÁ÷ ÀÓ¹« ¿Ï·á ¾ÈµÈ Å¸°Ù¸¸) (UNCHANGED)
                            if ((color == "ally" and not self.sent_ally) or
                                (color == "enemy" and not self.sent_enemy)):
                                
                                # ¡Ú¡Ú¡Ú ¾ÈÀü¼º °³¼±: STBY ÁøÀÔ Á÷ÈÄ 0.5ÃÊ µ¿¾ÈÀº TRACK ¿äÃ» ¾ïÁ¦
                                if time.time() - self.stby_enter_time < 0.5:
                                    self.get_logger().warn("[STBY] Mode stabilizing (0.5s lock). Suppress request.")
                                    return # Ä«¿îÆ® ¹× ¿äÃ» Àü¼Û ¾ïÁ¦

                                if color == self.last_label:
                                    self.detect_count += 1
                                else:
                                    self.last_label = color
                                    self.detect_count = 1

                                self.get_logger().info(f"[COUNT] {color}: {self.detect_count}/3")

                                if self.detect_count >= 3:
                                    # D-Pi¿¡°Ô »óÅÂ º¸°í (UNCHANGED)
                                    self.pub_status.publish(String(data=color))
                                    self.get_logger().info(f"[SEND] Report '{color}' to D (3s hold)")

                                    # ÀÓ¹«°¡ ³²¾ÆÀÖÀ¸¹Ç·Î TRACK ¿äÃ» (UNCHANGED)
                                    self.pub_mode_cmd.publish(String(data="track"))
                                    self.track_target_class = color
                                    self.track_start_time = time.time()
                                    
                                    self.detect_count = 0
                                    self.error_send_count = 0
                                    
                # =========================================
                # Display on screen (UNCHANGED)
                # =========================================
                x1, y1, x2, y2 = box
                if color == "ally":
                    draw_color = (255, 0, 0)
                    display_text = f"Ally {conf:.2f}"
                elif color == "enemy":
                    draw_color = (0, 0, 255)
                    display_text = f"Enemy {conf:.2f}"
                else:
                    draw_color = (0, 255, 255)
                    display_text = f"Unknown {conf:.2f}"

                cv2.rectangle(frame, (x1, y1), (x2, y2), draw_color, 3)
                cv2.putText(frame, display_text, (x1, y1-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, draw_color, 2)

        if self.mode in ("stby", "track"):
            cv2.imshow("A Node View", frame)
            cv2.waitKey(1)

    def destroy_node(self):
        if self.cap:
            self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


# ============================================================
# MAIN ENTRY (UNCHANGED)
# ============================================================
def main(args=None):
    rclpy.init(args=args)
    node = CameraClientNodeColor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
