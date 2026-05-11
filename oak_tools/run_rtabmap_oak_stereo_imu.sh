#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"

env -u CONDA_PREFIX -u CONDA_DEFAULT_ENV -u CONDA_EXE -u CONDA_PYTHON_EXE -u PYTHONHOME -u PYTHONPATH -u LD_LIBRARY_PATH \
WORKSPACE_ROOT="${WS_ROOT}" \
bash --noprofile --norc -lc '
  source /opt/ros/humble/setup.bash
  source "${WORKSPACE_ROOT}/install/setup.bash"
  ros2 launch rtabmap_launch rtabmap.launch.py \
    use_sim_time:=true \
    stereo:=true \
    rtabmap_viz:=true \
    rviz:=false \
    approx_sync:=false \
    wait_imu_to_init:=false \
    frame_id:=oak \
    vo_frame_id:=oak_odom \
    imu_topic:=/oak/imu/data \
    left_image_topic:=/oak/left/image_rect \
    right_image_topic:=/oak/right/image_rect \
    left_camera_info_topic:=/oak/left/camera_info \
    right_camera_info_topic:=/oak/right/camera_info \
    database_path:=/tmp/rtabmap_oak_d_pro_w.db \
    cfg:='"${WS_ROOT}"'/src/rtabmap/docs/rtabmap_oak_d_pro_w_stereo_imu.ini \
    args:="--delete_db_on_start" \
    "$@"
' bash "$@"
