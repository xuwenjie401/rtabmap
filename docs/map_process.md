# OAK Stereo+IMU Map Process

终端 1：启动 RTAB-Map

  不要用 record_processed_bag:=true。

  source /home/wjxu22/SLAMBot/rtab_ws/install/setup.bash

  ros2 launch /home/wjxu22/SLAMBot/rtab_ws/src/rtabmap/oak_tools/oak_rtabmap_stereo_imu_gravity_lossless_cloud.launch.py \
    use_sim_time:=true \
    record_processed_bag:=false

  终端 2：持续录轻量 processed bag

  这里保留 /rtabmap/mapData，因为当前 exporter 需要它；但不录 /cloud_map。

  source /home/wjxu22/SLAMBot/rtab_ws/install/setup.bash

  ros2 bag record --use-sim-time \
    --compression-mode file \
    --compression-format zstd \
    --max-cache-size 1073741824 \
    -o /home/wjxu22/Datasets/rosbags/processed/oak_rtabmap_pose_graph \
    /clock \
    /tf \
    /tf_static \
    /rtabmap/odom \
    /rtabmap/odom_info \
    /rtabmap/info \
    /rtabmap/mapGraph \
    /rtabmap/mapData \
    /rtabmap/mapPath \
    /oak/left/camera_info_rtabmap \
    /oak/right/camera_info_rtabmap

  终端 3：播放原始 bag

  必须加 --clock。

  ros2 bag play /path/to/raw_bag --clock

  如果处理慢：

  ros2 bag play /path/to/raw_bag --clock --rate 0.3

  原始 bag 播完后

  不要先关 RTAB-Map，也不要先关终端 2 的 recorder。保持终端 1、终端 2 都运行。

  终端 4：只录最终 cloud

  source /home/wjxu22/SLAMBot/rtab_ws/install/setup.bash

  ros2 bag record --use-sim-time \
    --compression-mode file \
    --compression-format zstd \
    --max-cache-size 1073741824 \
    -o /home/wjxu22/Datasets/rosbags/processed/oak_rtabmap_final_cloud \
    /rtabmap/cloud_map \
    /tf_static

  等它显示已经开始 recording 后，再开终端 5。

  终端 5：触发一次最终全局地图发布

  source /home/wjxu22/SLAMBot/rtab_ws/install/setup.bash

  ros2 service call /rtabmap/publish_map rtabmap_msgs/srv/PublishMap \
    "{global_map: true, optimized: true, graph_only: false}"

  等 10-30 秒，看到 RTAB-Map 终端里有发布/组装 cloud 的日志后：

  1. Ctrl-C 终端 4，停止 final cloud recorder。
  2. Ctrl-C 终端 2，停止 pose/mapData recorder。
  3. Ctrl-C 终端 1，停止 RTAB-Map，让它保存 .db。

  之后导出时：

  /usr/bin/python3 oak_tools/export_oak_rtabmap_processed_outputs.py \
    --processed-bag /home/wjxu22/Datasets/rosbags/processed/oak_rtabmap_pose_graph \
    --cloud-bag /home/wjxu22/Datasets/rosbags/processed/oak_rtabmap_final_cloud \
    --output-dir /home/wjxu22/Datasets/outputs/rtab/oak_stereo_imu_gravity_lossless_export \
    --overwrite

  注意：每次运行时 -o 的 bag 目录要换新名字，已有目录会报错。当前最占空间的 /rtabmap/cloud_map 只在最后录一次。
