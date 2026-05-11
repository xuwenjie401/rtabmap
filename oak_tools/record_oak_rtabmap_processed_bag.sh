#!/usr/bin/env bash
set -eo pipefail

OUT_ROOT="${1:-/home/wjxu22/Datasets/rosbags/processed}"
BAG_NAME="${2:-oak_rtabmap_processed_$(date +%Y%m%d_%H%M%S)}"
OUT_DIR="${OUT_ROOT}/${BAG_NAME}"

mkdir -p "${OUT_ROOT}"

source /opt/ros/humble/setup.bash
if [[ -f /home/wjxu22/SLAMBot/rtab_ws/install/setup.bash ]]; then
  source /home/wjxu22/SLAMBot/rtab_ws/install/setup.bash
fi

set -u

topics=(
  /clock
  /tf
  /tf_static
  /rtabmap/odom
  /rtabmap/odom_info
  /rtabmap/info
  /rtabmap/mapGraph
  /rtabmap/mapData
  /rtabmap/mapOdomCache
  /rtabmap/mapPath
  /rtabmap/local_grid_obstacle
  /rtabmap/local_grid_ground
  /rtabmap/local_grid_empty
  /rtabmap/cloud_map
  /rtabmap/cloud_obstacles
  /rtabmap/cloud_ground
  /rtabmap/octomap_occupied_space
  /rtabmap/octomap_obstacles
  /rtabmap/octomap_ground
  /rtabmap/octomap_empty_space
  /rtabmap/map
  /rtabmap/grid_prob_map
  /oak/left/camera_info_rtabmap
  /oak/right/camera_info_rtabmap
)

if [[ "${RECORD_RGB:-0}" == "1" ]]; then
  topics+=(
    /oak/rgb/image_raw
    /oak/rgb/camera_info
  )
fi

printf 'Recording processed RTAB-Map bag to: %s\n' "${OUT_DIR}"
printf 'Set RECORD_RGB=1 before this script only if you want to duplicate RGB images into the processed bag.\n'

exec ros2 bag record \
  --use-sim-time \
  --max-cache-size 1073741824 \
  --compression-mode file \
  --compression-format zstd \
  -o "${OUT_DIR}" \
  "${topics[@]}"
