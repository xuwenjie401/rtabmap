# OAK RGB Pointcloud Map

分成两个独立入口:

- 相机驱动:
  [oak_rgb_pointcloud_map.launch.py](/home/wjxu22/sensor_base/depthai/ros2_ws/src/depthai-ros/depthai_ros_driver/launch/oak_rgb_pointcloud_map.launch.py)
- RTAB-Map:
  [oak_rtabmap_rgb_pointcloud_map.launch.py](/home/wjxu22/SLAMBot/rtab_ws/src/rtabmap/oak_tools/oak_rtabmap_rgb_pointcloud_map.launch.py)

默认输入链路:

- OAK 发布 `RGB + aligned depth + IMU + VIO odometry`
- RTAB-Map 使用外部 `/oak/vio/odometry` 做建图与回环
- RTAB-Map 默认直接订阅 raw 话题 `/oak/rgb/image_raw` 和 `/oak/stereo/image_raw`
- `rtabmap_util/point_cloud_xyzrgb` 生成每帧 RGB 点云
- `rtabmap_util/point_cloud_assembler` 累计为 `/oak/rgb_map`

## 启动

### 1. depthai workspace: 只起相机驱动

```bash
source /opt/ros/humble/setup.bash
source /home/wjxu22/sensor_base/depthai/ros2_ws/install/setup.bash

ros2 launch /home/wjxu22/sensor_base/depthai/ros2_ws/src/depthai-ros/depthai_ros_driver/launch/oak_rgb_pointcloud_map.launch.py
```

这一步只负责发布:

- `/oak/rgb/image_raw`
- `/oak/stereo/image_raw`
- `/oak/rgb/camera_info`
- `/oak/imu/data`
- `/oak/vio/odometry`

### 2. rtab_ws: 只起 RTAB-Map 和点云累计

```bash
source /opt/ros/humble/setup.bash
source /home/wjxu22/SLAMBot/rtab_ws/install/setup.bash

ros2 launch /home/wjxu22/SLAMBot/rtab_ws/src/rtabmap/oak_tools/oak_rtabmap_rgb_pointcloud_map.launch.py
```

默认使用的 RTAB-Map 配置:

- [rtabmap_oak_rgb_pointcloud_map.ini](/home/wjxu22/SLAMBot/rtab_ws/src/rtabmap/docs/rtabmap_oak_rgb_pointcloud_map.ini)
- [oak_rtabmap_rgb_pointcloud_map.rviz](/home/wjxu22/SLAMBot/rtab_ws/src/rtabmap/docs/oak_rtabmap_rgb_pointcloud_map.rviz)

常用可选参数:

- `use_rviz:=false`
- `rviz_cfg:=/absolute/path/custom.rviz`
- `save_rgb_map_on_exit:=false`
- `rgb_map_snapshot_output:=/absolute/path/live_rgb_map.ply`
- `rtabmap_compressed:=true`
- `rtabmap_viz:=true`
- `delete_db_on_start:=true`
- `database_path:=/absolute/path/my_map.db`

当前默认行为:

- RViz 默认会启动。
- 默认 RViz 配置文件是 [oak_rtabmap_rgb_pointcloud_map.rviz](/home/wjxu22/SLAMBot/rtab_ws/src/rtabmap/docs/oak_rtabmap_rgb_pointcloud_map.rviz)。
- 里面的视角默认用 `ThirdPersonFollower` 跟随 `oak_parent_frame`。
- 如果你想改成跟随别的 frame，直接编辑这个 `.rviz` 文件里的 `Target Frame` 即可。
- RTAB-Map 数据库里的 RGB 图像当前用 `.png` 保存，属于无损压缩；当前 `Mem\ImagePreDecimation = 1`、`Mem\ImagePostDecimation = 1`，不会额外降低分辨率。
- 默认 `rtabmap_compressed:=false`，所以 RTAB-Map 不是在订阅 ROS 的 compressed image topic；DB 里看到的 `imageCompressed` 是 RTAB-Map 内部存储压缩格式，不等于输入 transport 是 compressed。
- `/oak/rgb_map` 是运行时旁路累计云，默认只保留最近 `assembler_max_clouds=400` 帧并在 `oak_odom` 下累计；它不是 DB 里保存的“最终优化地图”的原样镜像。
- 默认会在退出时把最后一帧 live `/oak/rgb_map` 另存成 [oak_rgb_pointcloud_map_live_rgb_map.ply](/home/wjxu22/Datasets/outputs/rtab/oak_rgb_pointcloud_map_live_rgb_map.ply)；同目录还会写一个同名 `.json` 元数据。
- live `/oak/rgb_map` 保存器在 launch 里固定走 `/usr/bin/python3`，这是为了避开当前机器上 Anaconda Python 和 ROS Humble 的 `rclpy` ABI 冲突。

如果不手动覆盖，当前默认数据库路径已经改到:

- `/home/wjxu22/Datasets/outputs/rtab/oak_rgb_pointcloud_map.db`

主要输出:

- `/oak/vio/odometry`
- `/oak/rgb_cloud`
- `/oak/rgb_map`
- `/rtabmap/mapData`

## 停止和保存

可以直接 `Ctrl+C`。

`Ctrl+C` 后会保留:

- RTAB-Map 数据库 `database_path`
- 默认是 `/home/wjxu22/Datasets/outputs/rtab/oak_rgb_pointcloud_map.db`
- live `/oak/rgb_map` 快照
  - 默认是 `/home/wjxu22/Datasets/outputs/rtab/oak_rgb_pointcloud_map_live_rgb_map.ply`
  - 配套元数据默认是 `/home/wjxu22/Datasets/outputs/rtab/oak_rgb_pointcloud_map_live_rgb_map.json`

`Ctrl+C` 不会自动导出:

- `PLY/PCD` 点云文件

## 导出最终点云

停掉 launch 后，再导出数据库里的最终地图:

```bash
source /opt/ros/humble/setup.bash
source /home/wjxu22/SLAMBot/rtab_ws/install/setup.bash

rtabmap-export --cloud \
  --output_dir /home/wjxu22/Datasets/outputs/rtab \
  --output oak_rgb_pointcloud_map \
  /home/wjxu22/Datasets/outputs/rtab/oak_rgb_pointcloud_map.db
```

默认会生成:

- `/home/wjxu22/Datasets/outputs/rtab/oak_rgb_pointcloud_map_cloud.ply`

如果你还想同时导出轨迹:

```bash
rtabmap-export --cloud --poses \
  --output_dir /home/wjxu22/Datasets/outputs/rtab \
  --output oak_rgb_pointcloud_map \
  /home/wjxu22/Datasets/outputs/rtab/oak_rgb_pointcloud_map.db
```

会额外得到:

- `/home/wjxu22/Datasets/outputs/rtab/oak_rgb_pointcloud_map_poses.txt`

## 说明

- `/oak/rgb_map` 是运行时累计出来的 RGB 点云，便于边跑边看。
- 更适合“最终保存”的版本是停机后从 `database_path` 里导出的 `PLY`，因为它基于 RTAB-Map 已保存的图优化结果。
- `/rtabmap/cloud_map` 不是相机真实 RGB 纹理点云，不建议把它当作最终彩色地图。
