#!/bin/bash

echo "=============================="
echo " Remote Cleanup Start (tmux)"
echo "=============================="

A_IP=172.30.1.5
B_IP=172.30.1.14
C_IP=172.30.1.78

ssh ubuntu@$A_IP "tmux kill-session -t a_pi 2>/dev/null || true"
ssh ubuntu@$B_IP "tmux kill-session -t b_pi 2>/dev/null || true"
ssh ubuntu@$C_IP "tmux kill-session -t c_pi 2>/dev/null || true"

echo "=============================="
echo " Remote Cleanup Finished"
echo "=============================="
