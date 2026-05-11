# OAK RGB-D 按时间戳取位姿的 Bag 录制说明

本文目标:

- 录一个 `ros2 bag`
- 之后能对每一帧 `RGB` 图像按 `header.stamp` 取到位姿
- 对当前这条链路，因为 RTAB-Map launch 也使用了深度图，所以也同时支持 `depth` 图像按时间戳取位姿

## 1. 当前 launch 是否使用深度图

是，当前 [oak_rtabmap_rgb_pointcloud_map.launch.py](/home/wjxu22/SLAMBot/rtab_ws/src/rtabmap/oak_tools/oak_rtabmap_rgb_pointcloud_map.launch.py) 明确使用了深度图。

具体有两处:

- RTAB-Map 主建图输入:
  - `depth_topic := /oak/stereo/image_raw`
- RGB 点云生成:
  - `point_cloud_xyzrgb` 订阅 `depth/image := /oak/stereo/image_raw`

所以如果你的目标是后面完整复现当前这条 RGB-D + RTAB-Map 链路，`/oak/stereo/image_raw` 必须录。

## 2. 建议录制时的运行方式

推荐分两个终端先起系统，再录 bag。

### 2.1 depthai workspace: 只起相机驱动

```bash
source /opt/ros/humble/setup.bash
source /home/wjxu22/sensor_base/depthai/ros2_ws/install/setup.bash

ros2 launch depthai_ros_driver_v3 oak_rgb_pointcloud_map.launch.py
```

### 2.2 rtab_ws: 只起 RTAB-Map 和点云累计

```bash
source /opt/ros/humble/setup.bash
source /home/wjxu22/SLAMBot/rtab_ws/install/setup.bash

ros2 launch /home/wjxu22/SLAMBot/rtab_ws/src/rtabmap/oak_tools/oak_rtabmap_rgb_pointcloud_map.launch.py
```

### 2.3 第三个终端开始录 bag

```bash
source /opt/ros/humble/setup.bash

ros2 bag record \
  -o /home/wjxu22/Datasets/outputs/rtab/oak_rgbd_pose_bag \
  /oak/rgb/image_raw \
  /oak/stereo/image_raw \
  /oak/rgb/camera_info \
  /oak/imu/data \
  /oak/vio/odometry \
  /tf \
  /tf_static
```

## 3. 这几个 topic 为什么必须录

### 3.1 图像本体

- `/oak/rgb/image_raw`
  - 之后每帧 RGB 图像的时间戳来源
- `/oak/stereo/image_raw`
  - 当前 launch 使用的深度图
  - 之后每帧 depth 图像的时间戳来源

### 3.2 标定

- `/oak/rgb/camera_info`
  - 当前这条 RGB-D 链路用的是 RGB 相机内参
  - 深度图是对齐到 RGB 的，所以这里录 RGB 的 `camera_info`

### 3.3 位姿/运动

- `/oak/vio/odometry`
  - 最直接的时间戳位姿来源
  - 这是 `oak_parent_frame` 在 `oak_odom` 下的运动

### 3.4 坐标变换

- `/tf`
  - 动态 TF
  - 包含至少两类你关心的变换:
    - `oak_odom -> oak_parent_frame` 来自 OAK VIO
    - `map -> oak_odom` 来自 RTAB-Map
- `/tf_static`
  - 静态 TF
  - 用来从机体 frame 变到相机 frame
  - 没有它，就很难把图像帧精确对应到相机光学坐标系位姿

## 4. 之后“按时间戳取 pose”时，推荐取哪种 pose

推荐区分两种 pose:

### 4.1 VIO 原始 pose

直接来自:

- `/oak/vio/odometry`

特点:

- 好处: 简单，直接按 `header.stamp` 做最近邻或插值即可
- 坏处: 没有 RTAB-Map 回环后的全局修正

适合:

- 只关心短时连续轨迹
- 只需要一个稳定的时间戳到 pose 映射

### 4.2 RTAB-Map 全局修正后的 pose

推荐从 TF 取:

- `map -> <image.header.frame_id>` at `image.header.stamp`

这才是更适合和最终地图对应的 pose。

对当前链路:

- RGB 图像建议查:
  - `map -> oak_rgb_camera_optical_frame`
- 深度图如果也是对齐到 RGB 光学系:
  - 也查 `map -> oak_rgb_camera_optical_frame`

如果你不想硬编码 frame 名:

- 直接使用图像消息自己的:
  - `image.header.frame_id`

## 5. 最实用的匹配规则

对每一帧 `RGB` 或 `depth` 图像:

1. 读 `msg.header.stamp`
2. 记为 `t`
3. 如果要 VIO pose:
   - 找 `/oak/vio/odometry` 中时间最接近 `t` 的消息
4. 如果要全局修正 pose:
   - 用 TF 查询 `map -> msg.header.frame_id` 在时间 `t` 的变换

建议:

- 优先用 `TF` 方案拿最终 pose
- `/oak/vio/odometry` 作为兜底和调试参考

## 6. 最小可用录制集

如果你的唯一目标就是:

- RGB 时间戳 -> pose
- depth 时间戳 -> pose

那最小建议就是:

```bash
ros2 bag record \
  -o /home/wjxu22/Datasets/outputs/rtab/oak_rgbd_pose_bag \
  /oak/rgb/image_raw \
  /oak/stereo/image_raw \
  /oak/rgb/camera_info \
  /oak/vio/odometry \
  /tf \
  /tf_static
```

如果后面还想保留 IMU 相关分析，再加:

- `/oak/imu/data`

## 7. 不建议省掉的内容

下面这些不要省:

- `/tf`
- `/tf_static`

原因:

- 只录图像和 `/oak/vio/odometry`，你最多只能稳妥拿到 base frame 的 VIO pose
- 你拿不到图像自身光学坐标系在 `map` 下的精确 pose
- 也无法可靠恢复 RTAB-Map 的全局修正结果

## 8. 一个简单判断标准

如果你希望将来能做这件事:

- “给我某一帧 RGB 图像时间戳，返回这帧图像在 map 坐标系下的位姿”

那录制时至少要有:

- 图像 topic
- 对应 `camera_info`
- `/tf`
- `/tf_static`

对当前这条链路，再加上:

- `/oak/vio/odometry`

这样后处理最方便。

## 9. 当前建议的输出目录

后续这条链路相关的 bag 和 RTAB-Map 数据库，文档里统一建议输出到:

- `/home/wjxu22/Datasets/outputs/rtab`
