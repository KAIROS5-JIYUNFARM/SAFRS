#!/usr/bin/env python3
# A-Pi Camera Client (FSM Corrected - TRACK completion required for NAV)
# [Ally/Enemy FSM Logic Fixed]
#
# ÀÌ ÄÚµå´Â ½ÇÁ¦ »ö»ó °¨Áö ¸ðµ¨ ´ë½Å ÀÓ½Ã ·ÎÁ÷À» »ç¿ëÇÏ¿© 
# FSM(Finite State Machine)ÀÇ ÀÛµ¿À» Å×½ºÆ®ÇÏ±â À§ÇØ »ç¿ëµË´Ï´Ù.

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
# 0) Interpreter selection and Model Paths
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
# 1) Load classifier details (±¸Á¶ À¯Áö¸¦ À§ÇØ ³²°ÜµÒ)
# ============================================================
interpreter = Interpreter(model_path=CLASS_MODEL_PATH)
interpreter.allocate_tensors()
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()
in_idx = input_details[0]["index"]
in_scale, in_zero = input_details[0]["quantization"]
out_idx = output_details[0]["index"]

# --- FAKE TARGET CLASS º¯¼ö (¿ÜºÎ ÀüÈ¯ °¡´É) ---
GLOBAL_FAKE_TARGET_CLASS = "enemy" 

# ============================================================
# 2) Utility functions for classification and Pose Estimation
# ============================================================

def softmax(x):
    # FSM Å×½ºÆ®¿ë ÀÓ½Ã ÇÔ¼ö
    return np.ones(2) 
def get_adaptive_thresholds(hsv_roi):
    # FSM Å×½ºÆ®¿ë ÀÓ½Ã ÇÔ¼ö
    return 50, 50

def classify_color(roi_bgr):
    # ------------------------------------------------------------------
    # ¡Ú¡Ú¡Ú ÀÓ½Ã »ö»ó °¨Áö ·ÎÁ÷: GLOBAL_FAKE_TARGET_CLASS °ªÀ¸·Î °íÁ¤ ¹ÝÈ¯ ¡Ú¡Ú¡Ú
    # ------------------------------------------------------------------
    if roi_bgr is not None and roi_bgr.size > 0:
        return GLOBAL_FAKE_TARGET_CLASS, 1.0 
    return "unknown", 0.0


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

        # Fake target class (±âº»°ª: enemy)
        self.fake_target_class = "enemy" 
        global GLOBAL_FAKE_TARGET_CLASS
        GLOBAL_FAKE_TARGET_CLASS = self.fake_target_class

        # ========= ROS Communication =============
        self.sub_mode = self.create_subscription(String, '/mode', self.cb_mode, 10)
        self.sub_trigger_done = self.create_subscription(
             String, '/c_trigger_done', self.cb_trigger_done, 10
        )
        self.sub_fake_switch = self.create_subscription(
             String, '/fake_target_switch', self.cb_fake_target_switch, 10
        )
        self.pub_status = self.create_publisher(String, '/a_target_status', 10)
        self.pub_error = self.create_publisher(Vector3, '/c_error_angle', 10)
        self.pub_mode_cmd = self.create_publisher(String, '/mode', 10)
        
        self.sub_servo_stabilized = self.create_subscription(
             String, '/servo_stabilized', self.cb_servo_stabilized, 10
        )
        
        self.timer = self.create_timer(0.066, self.loop)

        self.get_logger().info(f"[INIT] Start mode = {self.mode}")
        self.get_logger().info(f"[INIT] Fake Target Default: {self.fake_target_class}")


    # --------------------------------------------------------
    # [ÄÝ¹é] Fake Target Switch (Å×½ºÆ® ÆíÀÇ¸¦ À§ÇØ º¹±¸)
    # --------------------------------------------------------
    def cb_fake_target_switch(self, msg: String):
        new_fake_class = msg.data.strip().lower()
        if new_fake_class in ("ally", "enemy"):
            self.fake_target_class = new_fake_class
            global GLOBAL_FAKE_TARGET_CLASS
            GLOBAL_FAKE_TARGET_CLASS = new_fake_class
            self.get_logger().warn(
                f"[SWITCH] Fake Target Switched to: **{new_fake_class.upper()}** (FSM Test Mode)"
            )
            # ¸ðµå ÀüÈ¯ ½Ã STBY »óÅÂ¿¡¼­ Ä«¿îÅÍ ÃÊ±âÈ­
            if self.mode == "stby":
                 self.detect_count = 0
                 self.last_label = None
                 self.last_detect_time = 0.0
        else:
            self.get_logger().warn(
                f"[SWITCH] Invalid command: '{new_fake_class}'. Use 'ally' or 'enemy'."
            )
            
            
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
            # ¡Ú¡Ú¡Ú FSM ¼öÁ¤: Ally ÀÓ¹« ¿Ï·á ÇÃ·¡±×¸¦ Servo ½ÅÈ£ ¼ö½Å ½Ã ¼³Á¤ ¡Ú¡Ú¡Ú
            self.sent_ally = True
            
            # TRACK º¯¼ö ÃÊ±âÈ­
            self.track_target_class = None
            self.detect_count = 0
            self.error_send_count = 0
            self.track_start_time = 0.0
            
            # ¡Ú¡Ú¡Ú FSM ¼öÁ¤: NAV ¶Ç´Â STBY ºÐ±â (µÎ ÀÓ¹« ¸ðµÎ ¿Ï·á ½Ã NAV) ¡Ú¡Ú¡Ú
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
            # ¡Ú¡Ú¡Ú FSM ¼öÁ¤: Enemy ÀÓ¹« ¿Ï·á ÇÃ·¡±×¸¦ Trigger ½ÅÈ£ ¼ö½Å ½Ã ¼³Á¤ ¡Ú¡Ú¡Ú
            self.sent_enemy = True
            
            # TRACK º¯¼ö ÃÊ±âÈ­
            self.track_target_class = None
            self.detect_count = 0
            self.error_send_count = 0
            self.track_start_time = 0.0 
            
            # ¡Ú¡Ú¡Ú FSM ¼öÁ¤: NAV ¶Ç´Â STBY ºÐ±â (µÎ ÀÓ¹« ¸ðµÎ ¿Ï·á ½Ã NAV) ¡Ú¡Ú¡Ú
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

        # [TRACK ¸ðµå Å¸ÀÓ¾Æ¿ô Ã¼Å©] (Enemy¿¡°Ô¸¸ Àû¿ë)
        if self.mode == "track" and self.track_target_class == "enemy":
            if time.time() - self.track_start_time > self.TRACK_TIMEOUT_SEC:
                self.get_logger().error(
                    f"[TRACK] TIMEOUT! C-Pi response failed after {self.TRACK_TIMEOUT_SEC}s. Force requesting STBY."
                )
                self.sent_enemy = True # Å¸ÀÓ¾Æ¿ô ½Ã Enemy ÀÓ¹« ¿Ï·á Ã³¸®
                self.track_target_class = None
                self.detect_count = 0
                self.error_send_count = 0
                self.track_start_time = 0.0
                
                # ¡Ú¡Ú¡Ú FSM ¼öÁ¤: Å¸ÀÓ¾Æ¿ô ½Ã¿¡µµ NAV/STBY ºÐ±â ·ÎÁ÷ Àû¿ë ¡Ú¡Ú¡Ú
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
                self.get_logger().info(f"[DETECT] {color} ({conf:.2f}) - Fake: {self.fake_target_class.upper()}")

                # =========================================
                # TRACK Mode
                # =========================================
                if self.mode == "track":
                    
                    if self.track_target_class is None:
                        # TRACK ¸ðµå¿¡ ÁøÀÔÇßÁö¸¸ targetÀÌ ¼³Á¤µÇÁö ¾ÊÀº °æ¿ì (ÁÖ·Î ¿ÜºÎ /mode ÅäÇÈÀ¸·Î ÁøÀÔ ½Ã ¹ß»ý °¡´É)
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

                    # Enemy priority
                    if (not self.sent_enemy) and (color == "ally"):
                        self.get_logger().info("[STBY] Enemy not processed yet ¡æ ignore ally detection")
                    else:
                        # 1ÃÊ¿¡ ÇÑ ¹ø¸¸ Ä«¿îÆ® Ã¼Å©
                        now = time.time()
                        if now - self.last_detect_time >= 1.0:
                            self.last_detect_time = now

                            # 3ÃÊ Ä«¿îÆ® ½ÃÀÛ (¾ÆÁ÷ ÀÓ¹« ¿Ï·á ¾ÈµÈ Å¸°Ù¸¸)
                            if ((color == "ally" and not self.sent_ally) or
                                (color == "enemy" and not self.sent_enemy)):

                                if color == self.last_label:
                                    self.detect_count += 1
                                else:
                                    self.last_label = color
                                    self.detect_count = 1

                                self.get_logger().info(f"[COUNT] {color}: {self.detect_count}/3")

                                if self.detect_count >= 3:
                                    # D-Pi¿¡°Ô »óÅÂ º¸°í
                                    self.pub_status.publish(String(data=color))
                                    self.get_logger().info(f"[SEND] Report '{color}' to D (3s hold)")

                                    # ¡Ú¡Ú¡Ú FSM ¼öÁ¤: 3ÃÊ Ä«¿îÆ® ¼º°ø ½Ã ÀÓ¹« ¿Ï·á ÇÃ·¡±× ¼³Á¤ ·ÎÁ÷ Á¦°Å ¡Ú¡Ú¡Ú
                                    # ÀÓ¹« ¿Ï·á ÇÃ·¡±× ¼³Á¤(self.sent_ally/enemy)Àº TRACK ¸ðµå ÇÇµå¹é¿¡¼­ Ã³¸®µÊ.

                                    # ÀÓ¹«°¡ ³²¾ÆÀÖÀ¸¹Ç·Î TRACK ¿äÃ»
                                    self.pub_mode_cmd.publish(String(data="track"))
                                    self.track_target_class = color
                                    self.track_start_time = time.time()
                                        
                                    self.detect_count = 0
                                    self.error_send_count = 0
                                    
                # =========================================
                # Display on screen
                # =========================================
                x1, y1, x2, y2 = box
                if color == "ally":
                    draw_color = (255, 0, 0)
                elif color == "enemy":
                    draw_color = (0, 0, 255)
                else:
                    draw_color = (0, 255, 255)

                cv2.rectangle(frame, (x1, y1), (x2, y2), draw_color, 3)
                cv2.putText(frame, f"{color} {conf:.2f} (Fake: {self.fake_target_class[0].upper()})", (x1, y1-10),
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
# MAIN ENTRY
# ============================================================
def main(args=None):
    rclpy.init(args=args)
    node = CameraClientNodeColor() 
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
