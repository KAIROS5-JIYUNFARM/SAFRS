#!/bin/bash

echo "=============================="
echo " Remote Bringup Start (tmux)"
echo "=============================="

# ==============================
# 공통 환경
# ==============================
ROS_SETUP="source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash"

# ==============================
# A-Pi
# ==============================
A_IP=172.30.1.5
A_SESSION=a_pi
A_CMD="ros2 run camera_client_cluster2 camera_client_node3"

ssh ubuntu@$A_IP "
tmux has-session -t $A_SESSION 2>/dev/null || \
tmux new-session -d -s $A_SESSION \
'bash -lc \"\
$ROS_SETUP && \
$A_CMD\
\"'
"

echo "[OK] A-Pi bringup command sent"

# ==============================
# B-Pi
# ==============================
B_IP=172.30.1.14
B_SESSION=b_pi

ssh ubuntu@$B_IP "
tmux has-session -t $B_SESSION 2>/dev/null && exit 0

tmux new-session -d -s $B_SESSION

# Pane 0 : Nav2
tmux send-keys -t $B_SESSION:0 \
'bash -lc \"\
export DISPLAY=:1.0 && \
source /opt/ros/humble/setup.bash && \
source ~/ros2_ws/install/setup.bash && \
ros2 launch slam_bringup nav2.launch.py\
\"' C-m

# Pane 1 : RViz
tmux split-window -h -t $B_SESSION

tmux send-keys -t $B_SESSION:0.1 \
'bash -lc \"\
export DISPLAY=:1.0 && \
source /opt/ros/humble/setup.bash && \
source ~/ros2_ws/install/setup.bash && \
rviz2\
\"' C-m
"

echo "[OK] B-Pi bringup command sent (nav2 + rviz2)"

# ==============================
# C-Pi (구조 완전 개편)
# ==============================
C_IP=172.30.1.133
C_SESSION=c_pi
C_CMD="python3 /home/ubuntu/new_ws/stm32_bridg_add_TF5.py"
C_SETUP="source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash && source ~/new_ws/install/setup.bash"

ssh ubuntu@$C_IP "
  # 1. 기존 세션이 있다면 종료
  tmux kill-session -t $C_SESSION 2>/dev/null
  
  # 2. 새 세션 생성 (대기 상태)
  tmux new-session -d -s $C_SESSION
  
  # 3. 환경 설정 및 실행 명령 전송 (C-m은 Enter 키를 의미)
  tmux send-keys -t $C_SESSION \"$C_SETUP\" C-m
  tmux send-keys -t $C_SESSION \"$C_CMD\" C-m
"

echo "[OK] C-Pi bringup command sent"

echo "=============================="
echo " Remote Bringup Finished"
echo "=============================="
