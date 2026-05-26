#!/bin/bash
# 测试 vision_recv 通信（上位机 → 下位机）
# 用法: bash tools/test_vision_recv.sh [fire_mode] [target_state] [target_type] [yaw_deg] [pitch_deg] [fire_tem]

FM=${1:-2}   # fire_mode:  0=停火 1=单发 2=连发
TS=${2:-2}   # target_state: 0=无目标 1=锁定 2=跟踪
TT=${3:-3}   # target_type
YW=${4:-157} # yaw centi-degree (1.57°)
PT=${5:--30} # pitch centi-degree (-0.30°)
FT=${6:-1}   # fire_tem: 0=不开火 1=开火

source /opt/ros/humble/setup.bash

echo "发送: f=$FM s=$TS t=$TT y=$YW p=$PT m=$FT"
echo "下位机监视: fire_mode=$FM, target_state=$TS, target_type=$TT, yaw=$(echo "scale=2; $YW/100" | bc), pitch=$(echo "scale=2; $PT/100" | bc), fire_tem=$FT"

ros2 topic pub --once /stm32h723/vision_recv std_msgs/msg/String \
  "{\"data\":\"{\\\"f\\\":$FM,\\\"s\\\":$TS,\\\"t\\\":$TT,\\\"y\\\":$YW,\\\"p\\\":$PT,\\\"m\\\":$FT}\"}"
