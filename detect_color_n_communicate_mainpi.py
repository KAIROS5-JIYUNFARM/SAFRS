
#!/usr/bin/env python3            # 이 파이썬 파일을 직접 실행할 때 사용할 인터프리터(파이썬3)를 지정
#camerapi : 172.30.1.5
# ubuntu@ubuntu:~/ros_ws/src/camera_client_cluster/camera_client_cluster/camera_test_zmq.py
import zmq                        # ZMQ(ZeroMQ) 라이브러리 – 소켓 통신(PUB/SUB 등)을 위해 사용
import cv2                       # OpenCV – 카메라에서 영상 받기, 이미지 처리용
import numpy as np               # NumPy – 수치 계산과 배열 연산용
import mediapipe as mp           # MediaPipe – 포즈 인식(BlazePose) 사용
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
import platform                  # 현재 OS(윈도우/리눅스 등)를 확인하기 위한 모듈

# ============================================================
# 0) 플랫폼 자동 분기 (Windows ↔ Raspberry Pi)
#    - 윈도우: tensorflow의 TFLite 인터프리터 사용
#    - 라즈베리파이: tflite_runtime 인터프리터 사용
# ============================================================
if platform.system() == "Windows":          # 현재 OS 이름이 "Windows"인지 확인
    import tensorflow as tf                 # 윈도우에서는 tensorflow 전체를 import
    Interpreter = tf.lite.Interpreter       # TFLite 인터프리터 클래스를 Interpreter라는 이름으로 사용
    print("[INFO] Windows → TensorFlow Lite 사용")
else:
    import tflite_runtime.interpreter as tflite  # 라즈베리파이는 가벼운 tflite_runtime 사용
    Interpreter = tflite.Interpreter             # tflite_runtime의 Interpreter를 동일한 이름으로 사용
    print("[INFO] Raspberry Pi → tflite_runtime 사용")

# ============================================================
# 1) 모델 경로 설정 (라즈베리파이에 맞게 수정 가능)
# ============================================================
POSE_MODEL_PATH = "/home/ubuntu/ros_ws/src/camera_client_cluster/camera_client_cluster/model/pose_landmarker_lite.task"  # BlazePose Lite 모델 파일 경로
CLASS_MODEL_PATH = "/home/ubuntu/ros_ws/src/camera_client_cluster/camera_client_cluster/model/mobilenetv2_ally_enemy_int8.tflite"  # MobileNetV2 분류 모델 경로 (INT8 양자화)

IMG_SIZE = (128, 128)  # 색상 분류용 MobileNet 입력 이미지 크기 (가로 128, 세로 128)

# ============================================================
# 2) MobileNet INT8 로드 (ally/enemy 분류용)
# ============================================================
interpreter = Interpreter(model_path=CLASS_MODEL_PATH)  # TFLite 분류 모델 파일 로드
interpreter.allocate_tensors()                          # 모델에 필요한 텐서(버퍼) 메모리 할당

input_details = interpreter.get_input_details()         # 입력 텐서 정보(인덱스, 크기, 양자화 정보 등) 가져오기
output_details = interpreter.get_output_details()       # 출력 텐서 정보 가져오기

in_idx = input_details[0]["index"]                     # 첫 번째(유일한) 입력 텐서의 인덱스
in_scale, in_zero = input_details[0]["quantization"]   # 입력 텐서의 양자화 스케일과 zero-point 값
out_idx = output_details[0]["index"]                   # 첫 번째(유일한) 출력 텐서의 인덱스

# ============================================================
# 3) 조도 따라 HSV 임계값(채도/밝기) 자동 조절
#    - 어두운 환경일수록 기준치를 조금 낮춰주는 역할
# ============================================================
def get_adaptive_thresholds(hsv_roi):
    V_mean = np.mean(hsv_roi[:, :, 2])   # HSV 중 V(밝기) 채널의 평균값 계산
    S_base = 50                          # 기본 채도 기준
    V_base = 50                          # 기본 밝기 기준

    if V_mean < 80:                      # 전체가 조금 어두우면
        S_base -= 15                     # 채도 기준 조금 낮추기
        V_base -= 15                     # 밝기 기준 조금 낮추기
    if V_mean < 50:                      # 더 많이 어두우면
        S_base -= 25                     # 채도 기준 더 많이 낮추기
        V_base -= 25                     # 밝기 기준 더 많이 낮추기

    S_base = max(20, S_base)            # 너무 낮아지지 않도록 최소값 20 보장
    V_base = max(20, V_base)            # 마찬가지로 밝기 기준도 최소 20 보장
    return S_base, V_base               # 최종 채도/밝기 기준치 반환

# ============================================================
# 4) MobileNet + 3중 필터 Voting 방식 색상 분류
#    - ① HSV 범위
#    - ② RGB 비교
#    - ③ YCrCb 비교
#    + MobileNet 분류 결과까지 참고
# ============================================================
def classify_color(roi_bgr):
    # 1) ROI 유효성 검사
    if roi_bgr is None or roi_bgr.size == 0:   # ROI가 없거나 픽셀 수가 0이면
        return "unknown", 0.0                  # 판별 불가

    H, W = roi_bgr.shape[:2]                   # ROI 높이(H), 너비(W) 가져오기
    if H < 10 or W < 10:                       # 너무 작은 영역이면 노이즈 취급
        return "unknown", 0.0

    # 2) BGR → HSV 변환
    hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)  # 색 공간을 HSV로 변환
    h, s, v = hsv[:,:,0], hsv[:,:,1], hsv[:,:,2]   # H, S, V 채널 각각 분리

    # --------------------------------------------------
    # (1) 채도 기반 무채색 필터 (회색/하양/검정 제거)
    # --------------------------------------------------
    if np.mean(s) < 45:                       # 전체 채도 평균이 낮으면
        return "unknown", 0.0                 # 색이 뚜렷하지 않다고 판단

    # --------------------------------------------------
    # (2) Hue 기반 빨강/파랑 1차 필터
    # --------------------------------------------------
    blue_mask = ((h >= 95) & (h <= 130))      # 파란색으로 볼 수 있는 Hue 범위
    red_mask  = ((h <= 10) | (h >= 160))      # 빨간색으로 볼 수 있는 Hue 범위

    roi_h, roi_w = hsv.shape[:2]              # ROI의 높이/너비
    total = roi_h * roi_w                     # 전체 픽셀 수

    blue_ratio = np.sum(blue_mask) / total    # 파랑으로 인식된 픽셀 비율
    red_ratio  = np.sum(red_mask) / total     # 빨강으로 인식된 픽셀 비율

    if blue_ratio < 0.10 and red_ratio < 0.10:  # 둘 다 10% 미만이면
        return "unknown", 0.0                   # 색 덩어리가 충분하지 않다고 판단

    # --------------------------------------------------
    # (3) RGB 대비 기반 필터
    # --------------------------------------------------
    b, g, r = cv2.split(roi_bgr)               # B, G, R 채널 분리
    if abs(int(np.mean(r)) - int(np.mean(b))) < 30:  # R 평균과 B 평균 차이가 30 미만이면
        return "unknown", 0.0                        # 빨강/파랑 차이가 작다고 판단

    V_mean = np.mean(v)                         # 밝기 채널 평균
    S_mean = np.mean(s)                         # 채도 채널 평균

    # 너무 어둡거나, 채도가 너무 낮으면 색 판별이 신뢰하기 어려움
    if (V_mean < 80 and S_mean < 60) or S_mean < 40:
        return "unknown", 0.0

    # --------------------------------------------------
    # (4) MobileNet 분류 모델로 ally/enemy 예측
    # --------------------------------------------------
    img = cv2.resize(roi_bgr, IMG_SIZE)        # ROI를 128x128 크기로 리사이즈
    img = (img.astype(np.float32) / 255.0)[None, :, :, :]  # 0~1로 정규화 후 배치 차원 추가(1, H, W, C)

    # 양자화: float32 → int8 (scale, zero-point 사용)
    img_q = (img / in_scale + in_zero).clip(-128, 127).astype(np.int8)

    interpreter.set_tensor(in_idx, img_q)      # 입력 텐서에 데이터 집어넣기
    interpreter.invoke()                       # 모델 추론 실행
    pred = interpreter.get_tensor(out_idx)[0]  # 출력 텐서에서 결과 꺼내기 (예: [p_ally, p_enemy])

    cls_mn = int(np.argmax(pred))              # 가장 확률이 높은 클래스 인덱스(0: ally, 1: enemy)
    conf_mn = float(pred[cls_mn])              # 그 클래스의 확률값
    diff_mn = abs(pred[0] - pred[1])           # 두 클래스 확률 차이(얼마나 확실한지)

    # MobileNet 결과가 애매하면 "unknown"으로 처리
    if conf_mn < 0.80 or diff_mn < 0.25:       # 확률이 0.8 미만이거나 둘의 차이가 0.25 미만이면
        mobile_vote = "unknown"
    else:
        mobile_vote = "ally" if cls_mn == 0 else "enemy"  # 0이면 ally, 1이면 enemy

    # --------------------------------------------------
    # (5) 추가 색상 필터 (HSV + RGB + YCrCb 3중 Voting)
    # --------------------------------------------------
    S_min, V_min = get_adaptive_thresholds(hsv)  # 앞에서 만든 함수로 채도/밝기 기준 구하기

    # HSV 필터: Hue + 적응형 S/V 기준
    blue_hsv = ((h >= 95) & (h <= 130) & (s >= S_min) & (v >= V_min))   # 파란 영역
    red_hsv  = (((h <= 10) | (h >= 160)) & (s >= S_min) & (v >= V_min)) # 빨간 영역

    # RGB 필터: B와 R 채널의 상대 비교
    b, g, r = cv2.split(roi_bgr)
    blue_rgb = b > (r + 20)    # 파랑은 B가 R보다 충분히 큰 경우
    red_rgb  = r > (b + 20)    # 빨강은 R이 B보다 충분히 큰 경우

    # YCrCb 색 공간으로 변환해 추가 정보 사용
    ycrcb = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2YCrCb)
    Y, Cr, Cb = cv2.split(ycrcb)
    blue_ycc = Cb > 140        # 파랑 성분이 강한 경우
    red_ycc  = Cr > 140        # 빨강 성분이 강한 경우

    total = H * W              # ROI 전체 픽셀 수
    blue_votes = 0             # 파란색으로 인정된 필터 개수
    red_votes = 0              # 빨간색으로 인정된 필터 개수

    # 각 필터별로 일정 비율 이상이면 vote +1
    if np.sum(blue_hsv)/total > 0.20: blue_votes += 1
    if np.sum(blue_rgb)/total > 0.20: blue_votes += 1
    if np.sum(blue_ycc)/total > 0.20: blue_votes += 1
    if np.sum(red_hsv)/total > 0.20:  red_votes += 1
    if np.sum(red_rgb)/total > 0.20:  red_votes += 1
    if np.sum(red_ycc)/total > 0.20:  red_votes += 1

    # 3개 필터 중 2개 이상이 같은 색에 투표하면 그 색으로 판정
    if blue_votes >= 2:
        return "ally", conf_mn
    if red_votes >= 2:
        return "enemy", conf_mn

    # 필터가 애매하면 MobileNet의 결과를 최종적으로 참고
    if mobile_vote != "unknown":
        return mobile_vote, conf_mn

    # 그래도 애매하면 unknown
    return "unknown", conf_mn

# ============================================================
# 5) BlazePose Lite (MediaPipe) 로드
# ============================================================
BaseOptions = python.BaseOptions
PoseLandmarkerOptions = vision.PoseLandmarkerOptions
RunningMode = vision.RunningMode

base_options = BaseOptions(model_asset_path=POSE_MODEL_PATH)  # 포즈 모델 파일 경로 설정
options = PoseLandmarkerOptions(
    base_options=base_options,                # 위에서 설정한 모델 옵션
    running_mode=RunningMode.IMAGE,           # 한 장 한 장 이미지 입력 모드
    num_poses=1                               # 최대 1명의 사람만 추적
)
pose_landmarker = vision.PoseLandmarker.create_from_options(options)  # PoseLandmarker 객체 생성

# ============================================================
# 6) 상반신 crop
#    - BlazePose 전체 관절 좌표 중 어깨/엉덩이 4점만 사용
#    - 그 사각형을 약간 margin을 줘서 잘라냄
# ============================================================
def crop_upper_body(frame, keypoints, margin=30):
    try:
        # BlazePose 인덱스 기준:
        # 11: 왼쪽 어깨, 12: 오른쪽 어깨, 23: 왼쪽 엉덩이, 24: 오른쪽 엉덩이
        Ls = keypoints[11]; Rs = keypoints[12]
        Lh = keypoints[23]; Rh = keypoints[24]

        # x좌표, y좌표 각각 배열로 모음
        xs = np.array([Ls[0], Rs[0], Lh[0], Rh[0]])
        ys = np.array([Ls[1], Rs[1], Lh[1], Rh[1]])

        # 최소/최대 좌표에 margin을 더해서 박스 생성
        x1 = int(max(xs.min() - margin, 0))                # 왼쪽 위 x
        y1 = int(max(ys.min() - margin, 0))                # 왼쪽 위 y
        x2 = int(min(xs.max() + margin, frame.shape[1]))   # 오른쪽 아래 x
        y2 = int(min(ys.max() + margin, frame.shape[0]))   # 오른쪽 아래 y

        if x2 <= x1 or y2 <= y1:                           # 박스가 잘못된 경우(폭/높이가 음수 또는 0)
            return None, None
        return frame[y1:y2, x1:x2], (x1, y1, x2, y2)       # 상반신 ROI와 박스 좌표 반환
    except:
        # 포즈가 제대로 안 잡혔거나 인덱스 에러가 나면 여기로 옴
        return None, None

# ============================================================
# 7) ZMQ 초기화 (PUB 소켓 생성)
# ============================================================
def init_zmq():
    context = zmq.Context()                        # ZMQ 컨텍스트(소켓 공장 같은 것) 생성
    publisher = context.socket(zmq.PUB)           # PUB 타입 소켓 생성 (데이터를 방송하는 역할)
    publisher.bind("tcp://172.30.1.5:7000")       # 이 IP:포트로 바인딩 (다른 SUB들이 여기에 접속)
    return publisher                              # 만든 publisher 소켓 반환

# ============================================================
# 8) main() – 전체 프로그램 흐름
# ============================================================
def main():

    # ZMQ 초기화 함수 호출해서 publisher 소켓 준비
    publisher = init_zmq()

    """
    main() 함수는 실제 실행 루프를 담당한다.
    ROS2에서 'entry_points'가 이 main()을 호출할 수도 있고,
    그냥 python3로 직접 실행해도 된다.
    """

    # 카메라 열기 (장치번호 0, V4L2 백엔드 사용)
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)   # 라즈베리파이에서 주로 사용하는 방식
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)    # 가로 해상도 640 설정
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)   # 세로 해상도 480 설정
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)    # 자동 노출 모드 설정 (카메라에 따라 동작 방식 다를 수 있음)

    if not cap.isOpened():                    # 카메라 열기에 실패했는지 확인
        raise RuntimeError("[ERROR] 카메라 열기 실패")

    while True:                               # 무한 루프 (계속해서 프레임 읽고 처리)
        ok, frame = cap.read()                # 한 프레임 읽기
        if not ok:                            # 읽기 실패 시
            print("[ERROR] 프레임 읽기 실패")
            continue                          # 다음 루프로 넘어감

        H, W = frame.shape[:2]                # 프레임의 높이(H), 너비(W) 가져오기

        # MediaPipe용 Image 객체로 변환 (SRGB 포맷, BGR → RGB는 자동 변환됨)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=frame)

        # BlazePose로 포즈 추정 수행
        result = pose_landmarker.detect(mp_image)

        if result.pose_landmarks:             # 포즈가 하나 이상 검출된 경우
            lm_list = result.pose_landmarks[0]  # 첫 번째 사람만 사용
            # 각 랜드마크를 [x픽셀, y픽셀, z] 형태로 변환
            keypoints = np.array([
                [lm.x * W, lm.y * H, lm.z]    # 정규화 좌표(0~1)를 실제 픽셀 좌표로 변환
                for lm in lm_list
            ])

            # 어깨/엉덩이 좌표 (화면에 점 찍기용)
            shoulder_1 = (int(keypoints[11][0]), int(keypoints[11][1]))  # 왼쪽 어깨
            shoulder_2 = (int(keypoints[12][0]), int(keypoints[12][1]))  # 오른쪽 어깨
            hip_1 = (int(keypoints[23][0]), int(keypoints[23][1]))       # 왼쪽 엉덩이
            hip_2 = (int(keypoints[24][0]), int(keypoints[24][1]))       # 오른쪽 엉덩이

            # 어깨/엉덩이 4개 포인트 화면에 초록색 동그라미로 표시
            for pt in [shoulder_1, shoulder_2, hip_1, hip_2]:
                cv2.circle(frame, pt, 5, (0, 255, 0), -1)

            # 상반신 영역 crop
            roi, box = crop_upper_body(frame, keypoints)
            if roi is not None:                       # crop 성공한 경우만 처리
                color, conf = classify_color(roi)     # ROI 색상을 ally/enemy/unknown으로 분류

                # ① ZMQ로 색상 문자열 전송 (ally/enemy/unknown)
                publisher.send_string(color)

                # ② 터미널에 현재 인식 결과 출력
                if color == "ally":
                    print("[ALLY DETECTED] conf:", conf, flush=True)
                elif color == "enemy":
                    print("[ENEMY DETECTED] conf:", conf, flush=True)
                else:
                    print("[UNKNOWN DETECTED] conf:", conf, flush=True)

                # ③ 화면에 상자 그리기 및 텍스트 표시
                x1, y1, x2, y2 = box                  # crop 영역의 좌상단/우하단 좌표

                # 색상에 따라 상자 색 다르게 설정
                if color == "ally":
                    draw_color = (255, 0, 0)          # 파랑(BGR: 파랑)
                elif color == "enemy":
                    draw_color = (0, 0, 255)          # 빨강(BGR: 빨강)
                else:
                    draw_color = (0, 255, 255)        # 노랑(BGR: 노랑, unknown 표시용)

                # 상반신 영역에 사각형 박스 그리기
                cv2.rectangle(frame, (x1, y1), (x2, y2), draw_color, 3)

                # 박스 중심점 계산 및 표시 (레이저 포인터로 쏠 위치와 연동 가능)
                cx = (x1 + x2) // 2
                cy = (y1 + y2) // 2
                cv2.circle(frame, (cx, cy), 5, (255, 255, 255), -1)  # 흰색 점으로 중심 표시

                # 상단에 "ally 0.95" 이런 형식으로 텍스트 표시
                cv2.putText(
                    frame,
                    f"{color} {conf:.2f}",              # 예: "ally 0.93"
                    (x1, y1 - 10),                     # 박스 위쪽에 위치
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    draw_color,
                    2
                )

        # 최종 프레임 화면 출력
        cv2.imshow("BlazePose + MobileNet INT8 (Raspberry Pi)", frame)

        # 키보드에서 'q'를 누르면 루프 종료
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # 루프 종료 후 자원 정리
    cap.release()                 # 카메라 장치 해제
    cv2.destroyAllWindows()       # 모든 OpenCV 창 닫기

# ============================================================
# 9) main() 실행 트리거
#    - 이 파일을 python3로 직접 실행할 때만 동작
#    - 다른 모듈에서 import할 때는 실행되지 않음
# ============================================================
if __name__ == "__main__":
    """
    python3로 직접 실행할 때는 이 블록이 실행된다.
    ROS2에서는 setup.py의 entry_points가 main()을 직접 호출할 수 있다.
    """
    main()                         # 위에서 정의한 main() 함수 실행
