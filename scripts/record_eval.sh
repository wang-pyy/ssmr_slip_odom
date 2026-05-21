#!/usr/bin/env bash
# Record evaluation bag for slip odometry analysis.
# Usage: ./record_eval.sh [BAG_NAME]
#   BAG_NAME defaults to "slip_eval_<timestamp>"

set -euo pipefail

BAG_NAME="${1:-slip_eval_$(date +%Y%m%d_%H%M%S)}"

echo "Recording bag: ${BAG_NAME}"
echo "Press Ctrl+C to stop recording."

ros2 bag record \
    /tf \
    /tf_static \
    /scan \
    /odom \
    /imu/data \
    /slip/s_raw \
    /slip/s_bar \
    /slip/lambda \
    /slip/w_enc \
    /slip/w_imu \
    /slip/w_fused \
    /cmd_vel \
    -o "${BAG_NAME}"
