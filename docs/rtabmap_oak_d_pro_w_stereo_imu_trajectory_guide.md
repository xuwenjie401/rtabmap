# OAK-D-PRO-W + RTAB-Map 轨迹生成指南

本文面向当前目标: 用 `OAK-D-PRO-W` 录制或回放的 `stereo + imu` 数据，生成 `时间戳 + pose` 轨迹。

## 1. 结论先行

- 对这台相机，RTAB-Map stereo 输入应使用:
  - `/oak/left/image_rect`
  - `/oak/right/image_rect`
  - `/oak/left/camera_info`
  - `/oak/right/camera_info`
  - `/oak/imu/data`
- 不需要把 `fx / fy / cx / cy / baseline` 手填进 RTAB-Map 的 `cfg`。
- `cfg` 里只放算法参数。相机标定和外参应来自 bag 里的 `camera_info` 和 `TF`。
- 建议 `frame_id:=oak`。
  - 你现有相机文档里 `oak -> oak_rgb_camera_frame` 是 identity，所以 `oak_rgb_camera_frame` 也能用。
  - 如果你最终想要的是“整机机体”轨迹而不是“左相机光学坐标系”轨迹，`oak` 更顺手。
- 如果当前 shell 继承了 `/opt/MVS` 的 `LD_LIBRARY_PATH`，要用干净环境启动 RTAB-Map。
  - 否则可能出现 `libpcl_io.so.1.12: undefined symbol: libusb_set_option`

## 2. 这台 OAK 的标定信息，哪些要写进 RTAB-Map，哪些不用

对当前这条 `stereo + imu` 链路:

- `left/right camera_info` 决定双目内参。
- 左右相机的相对 TF 决定 baseline 的兜底来源。
- `imu frame -> frame_id` 的 TF 决定 IMU 如何投到 RTAB-Map 的 base frame。

因此:

- 你不用把相机内参抄进 `cfg`。
- 你不用手工在 `cfg` 里写 baseline。
- 你要确保 bag 里保留:
  - 左右 `camera_info`
  - `/tf`
  - `/tf_static`

你现有 OAK 文档已经给出两个对 RTAB-Map 很重要的事实:

- 图像是 rectified 输出，`D` 全 0，所以应显式使用 `Rtabmap\ImagesAlreadyRectified=true`。
- 当前 `camera_info` 形式不是最标准的“右相机 `P(0,3)` 带 Tx”写法，RTAB-Map 在这种情况下会尝试用左右相机 TF 来补 baseline。

这意味着:

- 如果 bag 里没有 `/tf_static`，RTAB-Map 可能拿不到 baseline。
- 如果回放时看到一次类似“right camera info doesn't have Tx set, we used TF to get baseline”的 warning，而轨迹仍正常，这通常是可接受的。

### 2.1 OAK 的一个已知坑: baseline 符号可能反

如果你看到这类报错:

```text
The stereo baseline (-0.074568) should be positive (baseline=-Tx/fx)
```

这不是 RTAB-Map launch 参数写错，而是 OAK 驱动发布出来的 stereo `camera_info` 顺序/投影矩阵符号不符合 RTAB-Map 对左右相机的假设。

对你当前这台 OAK，优先修法不是改 RTAB-Map，而是改 DepthAI 驱动参数:

```text
stereo.i_reverse_stereo_socket_order:=true
```

本地 `depthai-ros` 源码里已经有这个参数，官方 `isaac_vslam.yaml` 也在用它。

影响:

- live camera 时，重启 OAK driver 后即可生效
- 如果你已经录了 bag，而 bag 里的 `camera_info` 已经带着错误的 baseline 符号，那么之后光改 RTAB-Map 不够
  - 要么重录 bag
  - 要么在播放链路里修正 `camera_info`

## 3. 录 bag 时最小必带话题

如果你是从 live camera 录 bag，最小建议录这些:

```bash
ros2 bag record \
  /oak/left/image_rect \
  /oak/right/image_rect \
  /oak/left/camera_info \
  /oak/right/camera_info \
  /oak/imu/data \
  /tf \
  /tf_static
```

可选:

- `/oak/rgb/image_raw`
  - 只在你想保留彩色画面或以后做 RGB-D/可视化时才有用。

当前“先生成轨迹”的场景下，不需要把下面这些当作核心输入:

- `/oak/stereo/image_raw`
- `/oak/rgbd/points`

## 4. 推荐先用这份最小 cfg

我已经放了一份最小化配置在 [rtabmap_oak_d_pro_w_stereo_imu.ini](/home/wjxu22/SLAMBot/rtab_ws/src/rtabmap/docs/rtabmap_oak_d_pro_w_stereo_imu.ini)。

这份配置的意图是:

- 明确告诉 RTAB-Map 输入图像已经 rectified。
- 先专注“轨迹”，不生成 occupancy grid，减少不必要负担。
- 先把 `Optimizer\GravitySigma` 设成 `0`，避免在 IMU orientation 还没确认可靠前把重力约束硬打开。

## 5. 第一轮推荐跑法: 先跑稳 stereo 轨迹，IMU 先只接入不强依赖

### 5.0 先避开当前机器上的 `libusb` 冲突

这台机器当前 shell 里有来自海康/MVS SDK 的:

- `/opt/MVS/lib/64`
- `/opt/MVS/lib/32`

它们会通过 `LD_LIBRARY_PATH` 抢在系统 `libusb` 前面，导致:

```text
/lib/x86_64-linux-gnu/libpcl_io.so.1.12: undefined symbol: libusb_set_option
```

所以当前建议不要直接在普通 shell 里执行 `ros2 launch ...`，而是:

- 用下面的干净环境命令启动
- 或直接使用脚本 [run_rtabmap_oak_stereo_imu.sh](/home/wjxu22/SLAMBot/rtab_ws/src/rtabmap/oak_tools/run_rtabmap_oak_stereo_imu.sh)

### 5.1 回放 bag 时启动 RTAB-Map

```bash
env -u CONDA_PREFIX -u CONDA_DEFAULT_ENV -u CONDA_EXE -u CONDA_PYTHON_EXE -u PYTHONHOME -u PYTHONPATH -u LD_LIBRARY_PATH \
bash --noprofile --norc -lc '
  source /opt/ros/humble/setup.bash
  source /home/wjxu22/SLAMBot/rtab_ws/install/setup.bash
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
    cfg:=/home/wjxu22/SLAMBot/rtab_ws/src/rtabmap/docs/rtabmap_oak_d_pro_w_stereo_imu.ini \
    args:=\"--delete_db_on_start\"
'
```

说明:

- 这里不要额外设置 `odom_frame_id`。
  - 当前模式下让前端 odometry 走 `/rtabmap/odom` 话题即可。
- `vo_frame_id:=oak_odom` 只是把 odometry TF 的 frame 名字改清楚一点。
- 这是“回放 bag”的启动方式，所以这里用 `use_sim_time:=true`。
- 如果你录下来的 OAK 左右图和 `camera_info` 在 ROS 时间戳上不是完全相等，回放 bag 时也可能仍然要改成 `approx_sync:=true`。

### 5.2 再播 bag

```bash
ros2 bag play /path/to/your_bag --clock
```

更稳的顺序是:

1. 先起 RTAB-Map。
2. 如果要录输出轨迹，再先起 recorder。
3. 最后再 `ros2 bag play --clock`。

## 6. IMU 现在怎么用更稳

你现有 OAK 文档里，`/oak/imu/data` 的采样消息显示:

- `orientation = (0, 0, 0, 1)`
- orientation / gyro / accel covariance 都是 0

这说明两件事:

- RTAB-Map 不会因为 quaternion 是 identity 就直接丢掉这条 IMU。
- 但这不自动等于“它已经提供了有意义的姿态估计或重力方向约束”。

因此建议分两步:

### 6.1 第一步

先按上一节的命令跑:

- `wait_imu_to_init:=false`
- `Optimizer\GravitySigma=0`

目标是先确认:

- stereo 前端能稳定出 `/rtabmap/odom`
- 后端能稳定出 `/rtabmap/mapPath`
- TF 链、bag 内容、topic 命名都没问题

### 6.2 第二步

如果你确认 IMU 的 orientation 是有效的，或者你后面给了一个滤波后的 IMU topic，再切到:

```bash
env -u CONDA_PREFIX -u CONDA_DEFAULT_ENV -u CONDA_EXE -u CONDA_PYTHON_EXE -u PYTHONHOME -u PYTHONPATH -u LD_LIBRARY_PATH \
bash --noprofile --norc -lc '
  source /opt/ros/humble/setup.bash
  source /home/wjxu22/SLAMBot/rtab_ws/install/setup.bash
  ros2 launch rtabmap_launch rtabmap.launch.py \
    use_sim_time:=true \
    stereo:=true \
    rtabmap_viz:=true \
    rviz:=false \
    approx_sync:=false \
    wait_imu_to_init:=true \
    frame_id:=oak \
    vo_frame_id:=oak_odom \
    imu_topic:=/oak/imu/data \
    left_image_topic:=/oak/left/image_rect \
    right_image_topic:=/oak/right/image_rect \
    left_camera_info_topic:=/oak/left/camera_info \
    right_camera_info_topic:=/oak/right/camera_info \
    database_path:=/tmp/rtabmap_oak_d_pro_w.db \
    cfg:=/home/wjxu22/SLAMBot/rtab_ws/src/rtabmap/docs/rtabmap_oak_d_pro_w_stereo_imu.ini \
    args:=\"--delete_db_on_start --Optimizer/GravitySigma 0.3\"
'
```

如果回放过程中发现 IMU orientation 一直是固定单位四元数，没有随姿态变化，那么先保持 `Optimizer/GravitySigma=0` 更稳。

## 7. 哪些输出能直接拿来做轨迹

最相关的是:

- `/rtabmap/mapPath`
  - 后端优化后的全局轨迹，类型是 `nav_msgs/Path`
  - 更适合作为最终 SLAM 轨迹
- `/rtabmap/localization_pose`
  - 当前时刻的全局位姿，类型是 `geometry_msgs/PoseWithCovarianceStamped`
- `/rtabmap/odom`
  - 前端 odometry 轨迹，频率更连续，但没有后端图优化修正

建议:

- 如果你要“最终优化后的时间戳 + pose”，优先保存 `/rtabmap/mapPath`
- 如果你要“连续 pose stream”，同时保存 `/rtabmap/localization_pose` 和 `/rtabmap/odom`

## 8. 一个容易漏掉的点: `mapPath` 需要有人订阅

`/rtabmap/mapPath` 在当前 ROS2 封装里不是无条件发布的。

如果你想稳定拿到它，最简单的办法是在回放前先开一个 recorder:

```bash
ros2 bag record \
  /rtabmap/mapPath \
  /rtabmap/localization_pose \
  /rtabmap/odom
```

这样既能保证 `mapPath` 有订阅者，也能把 RTAB-Map 输出单独存一份 bag，后续再做 CSV 导出更方便。

## 9. 如果 live camera 直连 RTAB-Map

如果你不是回放 bag，而是 live camera 直连:

- 继续使用你已经整理过的 DepthAI 启动方式。
- RTAB-Map 侧仍然只关心:
  - `/oak/left/image_rect`
  - `/oak/right/image_rect`
  - `/oak/left/camera_info`
  - `/oak/right/camera_info`
  - `/oak/imu/data`
  - `/tf`
  - `/tf_static`

也就是说:

- OAK 的 RGB 和点云不是当前“先出轨迹”的必要条件。
- 你可以先把链路压到最小，再去加 GUI、RGB 或点云。

更重要的是，live camera 和 bag playback 的启动参数不要混用:

- live camera 不要用 `use_sim_time:=true`
- 对 OAK live 输出，优先试:
  - `use_sim_time:=false`
  - `approx_sync:=true`
  - `approx_sync_max_interval:=0.02`
  - `qos:=2`

一个更适合 OAK live 的启动命令:

```bash
ros2 launch rtabmap_launch rtabmap.launch.py \
  use_sim_time:=false \
  stereo:=true \
  rtabmap_viz:=true \
  rviz:=false \
  approx_sync:=true \
  approx_sync_max_interval:=0.02 \
  qos:=2 \
  wait_imu_to_init:=false \
  frame_id:=oak \
  vo_frame_id:=oak_odom \
  imu_topic:=/oak/imu/data \
  left_image_topic:=/oak/left/image_rect \
  right_image_topic:=/oak/right/image_rect \
  left_camera_info_topic:=/oak/left/camera_info \
  right_camera_info_topic:=/oak/right/camera_info \
  database_path:=/tmp/rtabmap_oak_d_pro_w.db \
  cfg:=/home/wjxu22/SLAMBot/rtab_ws/src/rtabmap/docs/rtabmap_oak_d_pro_w_stereo_imu.ini \
  args:="--delete_db_on_start"
```

同时，OAK 驱动那边建议这样起:

```bash
ros2 launch depthai_ros_driver_v3 oak_d_pro_w_rgb_720_stereo_720.launch.py \
  stereo.i_reverse_stereo_socket_order:=true
```

如果你用的是别的 OAK launch，同样把这个参数加到相机驱动 launch 上。

对 `oak_d_pro_w_view_no_dot.launch.py` 这类 wrapper launch，更稳的方式不是直接在命令行后面追加
`stereo.i_reverse_stereo_socket_order:=true`，而是直接换 `params_file`。

我已经放好一份可直接用的文件:

- [oak_d_pro_w_view_no_dot_rtabmap.yaml](/home/wjxu22/sensor_base/depthai/ros2_ws/src/depthai-ros/depthai_ros_driver/config/oak_d_pro_w_view_no_dot_rtabmap.yaml)

建议这样起 OAK:

```bash
ros2 launch depthai_ros_driver_v3 oak_d_pro_w_view_no_dot.launch.py \
  params_file:=/home/wjxu22/sensor_base/depthai/ros2_ws/src/depthai-ros/depthai_ros_driver/config/oak_d_pro_w_view_no_dot_rtabmap.yaml
```

原因:

- `oak_d_pro_w_view_no_dot.launch.py` 本身只显式暴露了少量 launch 参数
- 它最可靠的自定义入口就是 `params_file`
- 把 `i_reverse_stereo_socket_order: true` 写死进 yaml，比在 wrapper launch 后面直接追加 nested 参数更稳

顺序建议:

1. 先关掉当前 RTAB-Map 和 OAK driver
2. 重新启动 OAK driver，并带上 `stereo.i_reverse_stereo_socket_order:=true`
3. 确认 `/oak/left/camera_info` 和 `/oak/right/camera_info` 已更新
4. 再启动 RTAB-Map

这一步如果没做对，RTAB-Map 里的表现通常就是:

- `stereo_odometry` 先报 baseline 为负
- `/rtabmap/odom` 不会正常产出
- `rtabmap` 再报 `Did not receive data since 5 seconds`

为什么 live camera 更推荐这组参数:

- `use_sim_time:=true` 在没有 `/clock` 时是错的。
- 你的 OAK 驱动虽然是同一设备输出，但 RTAB-Map 这里做的是 `left image + right image + left camera_info + right camera_info` 四路同步，live 情况下不一定能做到严格 exact sync。
- DepthAI/图像链路常见 QoS 是 `best_effort`，RTAB-Map 用 `qos:=0` 时可能只是“订阅建立了”，但没有真正匹配到 publisher。

## 10. 遇到问题先查什么

- 没有轨迹:
  - 先看 `/rtabmap/odom` 有没有
- 先看到 `The stereo baseline (...) should be positive`:
  - 优先修 OAK driver 的 `stereo.i_reverse_stereo_socket_order:=true`
  - 不要先去怀疑 RTAB-Map 的 topic remap
- `stereo_odometry` 起不来:
  - 先查左右 `camera_info` 和 `/tf_static`
- 提示 IMU TF 不可用:
  - 先查 `oak -> oak_imu_frame` 这条 TF 是否在 bag 里
- 收不到图像/IMU:
  - 先查 topic 名字是否和 `/oak/...` 完全一致
  - 如果是 live camera 或某些 bag 回放 QoS 不兼容，再试给 launch 追加 `qos:=2`
- 日志里明确写了 `subscribed to /oak/...`，但仍然没有 odom:
  - 这通常不是 topic 名错
  - 更像是 `use_sim_time`、QoS 或 exact sync 的问题
