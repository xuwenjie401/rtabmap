# RTAB-Map 使用说明

本文面向你当前目标: 用 `stereo + imu` 的 `ros2bag` 生成轨迹，并能看到 RTAB-Map 自己的 GUI 可视化。

## 0. 先分清三个东西

- `cfg`: RTAB-Map 核心参数文件，给 odom/slam 节点用。
- `gui_cfg`: `rtabmap_viz` 的 GUI 布局和显示配置。
- `database_path`: 运行产生的 `.db` 地图数据库。

最容易犯的错是把这三者混在一起。

## 1. 运行前准备

### 1.1 环境

```bash
cd /home/wjxu22/SLAMBot/rtab_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
conda deactivate
```

如果你需要重新编译本工作区，额外加:

```bash
export CPATH=/home/wjxu22/SLAMBot/rtab_ws/src/rtabmap/compat_include:${CPATH:-}
```

### 1.2 数据和 TF 前提

对你这条 `stereo + imu` 链路，运行前至少要满足:

- 左右图 topic 正确。
- 左右 `camera_info` 正确，标定里 baseline 正确。
- `imu_topic` 有数据。
- `imu` frame 到 `frame_id` 有可用 TF。
- 播 bag 时打开仿真时钟。

如果 bag 中已经硬时间同步，建议:

- `approx_sync:=false`
- 如果使用 `rgbd_sync:=true`，则 `approx_rgbd_sync:=false`

## 2. 推荐启动模式

## 2.1 最推荐: stereo + imu 建图 + RTAB-Map 自己的 GUI

这是最贴近你当前目标的入口。

先播 bag:

```bash
ros2 bag play /path/to/your_bag --clock
```

再起 RTAB-Map:

```bash
ros2 launch rtabmap_launch rtabmap.launch.py \
  use_sim_time:=true \
  stereo:=true \
  rtabmap_viz:=true \
  rviz:=false \
  approx_sync:=false \
  wait_imu_to_init:=true \
  imu_topic:=/your/imu \
  left_image_topic:=/your/left/image_rect \
  right_image_topic:=/your/right/image_rect \
  left_camera_info_topic:=/your/left/camera_info \
  right_camera_info_topic:=/your/right/camera_info \
  frame_id:=base_link \
  database_path:=/tmp/rtabmap_stereo_imu.db \
  args:="--delete_db_on_start" \
  cfg:=/abs/path/to/rtabmap_stereo_imu.ini
```

说明:

- `rtabmap_viz:=true` 是 RTAB-Map 自己的 ROS GUI，不是 RViz。
- `args:="--delete_db_on_start"` 适合每次从空库重新跑。
- 如果你的 base frame 不是 `base_link`，按实际 TF 改 `frame_id`。
- 如果 bag 是“纯相机机体”没有机器人底盘框架，也可以把 `frame_id` 设成相机主 frame。

## 2.2 建图但不打开 GUI

用于只跑轨迹/数据库，不看界面:

```bash
ros2 launch rtabmap_launch rtabmap.launch.py \
  use_sim_time:=true \
  stereo:=true \
  rtabmap_viz:=false \
  rviz:=false \
  approx_sync:=false \
  wait_imu_to_init:=true \
  imu_topic:=/your/imu \
  left_image_topic:=/your/left/image_rect \
  right_image_topic:=/your/right/image_rect \
  left_camera_info_topic:=/your/left/camera_info \
  right_camera_info_topic:=/your/right/camera_info \
  frame_id:=base_link \
  database_path:=/tmp/rtabmap_stereo_imu.db \
  args:="--delete_db_on_start" \
  cfg:=/abs/path/to/rtabmap_stereo_imu.ini
```

## 2.3 localization 模式

当你已经有一份数据库，只想在老地图上定位时:

```bash
ros2 launch rtabmap_launch rtabmap.launch.py \
  use_sim_time:=true \
  stereo:=true \
  localization:=true \
  rtabmap_viz:=true \
  approx_sync:=false \
  wait_imu_to_init:=true \
  imu_topic:=/your/imu \
  left_image_topic:=/your/left/image_rect \
  right_image_topic:=/your/right/image_rect \
  left_camera_info_topic:=/your/left/camera_info \
  right_camera_info_topic:=/your/right/camera_info \
  frame_id:=base_link \
  database_path:=/abs/path/to/existing_map.db \
  cfg:=/abs/path/to/rtabmap_stereo_imu.ini
```

这个模式下，launch 会自动把:

- `Mem/IncrementalMemory=false`
- `Mem/InitWMWithAllNodes=true`

## 2.4 使用外部 odometry

如果你不想让 RTAB-Map 自己做 visual odometry，而是接外部 odom:

```bash
ros2 launch rtabmap_launch rtabmap.launch.py \
  use_sim_time:=true \
  stereo:=true \
  visual_odometry:=false \
  odom_topic:=/your/external_odom \
  rtabmap_viz:=true \
  approx_sync:=false \
  imu_topic:=/your/imu \
  left_image_topic:=/your/left/image_rect \
  right_image_topic:=/your/right/image_rect \
  left_camera_info_topic:=/your/left/camera_info \
  right_camera_info_topic:=/your/right/camera_info \
  frame_id:=base_link \
  database_path:=/tmp/rtabmap_external_odom.db \
  args:="--delete_db_on_start" \
  cfg:=/abs/path/to/rtabmap_stereo_imu.ini
```

## 2.5 直接起节点，不走 `rtabmap_launch`

这个模式适合调模块，不适合第一次跑全链路。优点是边界最清楚。

1. 起 odom:

```bash
ros2 run rtabmap_odom stereo_odometry --ros-args \
  -p frame_id:=base_link \
  -p odom_frame_id:=odom \
  -p publish_tf:=true \
  -p wait_imu_to_init:=true \
  -p approx_sync:=false \
  -p config_path:=/abs/path/to/rtabmap_stereo_imu.ini \
  -r left/image_rect:=/your/left/image_rect \
  -r right/image_rect:=/your/right/image_rect \
  -r left/camera_info:=/your/left/camera_info \
  -r right/camera_info:=/your/right/camera_info \
  -r imu:=/your/imu \
  -r odom:=/rtabmap/odom
```

2. 起 slam:

```bash
ros2 run rtabmap_slam rtabmap --ros-args \
  -p frame_id:=base_link \
  -p map_frame_id:=map \
  -p subscribe_stereo:=true \
  -p approx_sync:=false \
  -p database_path:=/tmp/rtabmap_stereo_imu.db \
  -p config_path:=/abs/path/to/rtabmap_stereo_imu.ini \
  -r left/image_rect:=/your/left/image_rect \
  -r right/image_rect:=/your/right/image_rect \
  -r left/camera_info:=/your/left/camera_info \
  -r right/camera_info:=/your/right/camera_info \
  -r imu:=/your/imu \
  -r odom:=/rtabmap/odom
```

3. 起 RTAB-Map GUI:

```bash
ros2 run rtabmap_viz rtabmap_viz ~/.ros/rtabmap_gui.ini --ros-args \
  -p frame_id:=base_link \
  -p subscribe_stereo:=true \
  -p approx_sync:=false \
  -r left/image_rect:=/your/left/image_rect \
  -r right/image_rect:=/your/right/image_rect \
  -r left/camera_info:=/your/left/camera_info \
  -r right/camera_info:=/your/right/camera_info \
  -r odom:=/rtabmap/odom
```

## 3. 输出什么可以拿来做轨迹

最相关的是这几个 topic:

- `/rtabmap/mapPath`
- `/rtabmap/localization_pose`
- `/rtabmap/odom`

怎么理解:

- `mapPath`: 优化后的全局轨迹。类型是 `nav_msgs/Path`，其中每个 `PoseStamped` 都有自己的时间戳，适合拿来做最终 SLAM 轨迹。
- `localization_pose`: 当前时刻的全局位姿。类型是 `geometry_msgs/PoseWithCovarianceStamped`。
- `odom`: 里程计前端输出，更接近连续帧率轨迹，但没有后端图优化修正。

如果你的目标是“最终优化后的时间戳+pose 轨迹”，优先看:

- `/rtabmap/mapPath`

如果你的目标是“每一帧都要一个 pose stream”，优先看:

- `/rtabmap/odom`
- `/rtabmap/localization_pose`

## 4. 参数到底怎么配

## 4.1 覆盖顺序

当前这套 ROS2 封装里，参数优先级可以理解成:

`默认值 < cfg 文件 < ROS 参数覆盖 < args / odom_args`

还要注意:

- 同一个 `cfg` 文件可以同时放 odom 参数和 slam 参数。
- odom 节点只会从 `cfg` 中读取 odometry 相关参数。
- slam 节点会忽略 `Odom/*`，主要读取其余 core 参数。
- `args` 传给 `rtabmap` 节点。
- `odom_args` 只传给 odometry 节点，优先级高于 `args` 里的同名 odom 参数。

## 4.2 `cfg` 文件怎么新建

最稳妥的方法:

1. 从已有 preset 或 standalone app 生成的 `.ini` 开始。
2. 保留 `[Core]` 段，手动改需要的参数。
3. `topic`、`frame_id`、`database_path`、`use_sim_time` 这类运行态参数放 launch 命令里，不要混进 `cfg`。

这个仓库自带了可参考的 preset:

- `rtabmap/data/presets/camera_tof_icp.ini`
- `rtabmap/data/presets/lidar3d_icp.ini`

注意写法差异:

- 在 ROS 参数和 launch 覆盖里，参数名写成 `Optimizer/Strategy`
- 在 `.ini` 里，通常写成 `Optimizer\Strategy`

一个最小化的 stereo + imu 例子:

```ini
[Core]
Mem\IncrementalMemory = true
Mem\InitWMWithAllNodes = false
Odom\Strategy = 0
Reg\Strategy = 0
Vis\FeatureType = 8
Vis\MaxFeatures = 1500
RGBD\LinearUpdate = 0.05
RGBD\AngularUpdate = 0.02
Optimizer\Strategy = 1
Optimizer\GravitySigma = 0.3
RGBD\CreateOccupancyGrid = true
Grid\Sensor = 1
Grid\3D = true
Grid\CellSize = 0.05
Grid\RayTracing = true
```

这份配置表达的是:

- `Odom\Strategy = 0`: 用 F2M。
- `Reg\Strategy = 0`: 以前端视觉配准为主。
- `Optimizer\Strategy = 1`: 明确用 g2o。
- `Optimizer\GravitySigma = 0.3`: 打开基于 IMU/重力方向的图优化约束。
- `Grid\3D = true` + `Grid\RayTracing = true`: 允许 3D occupancy / OctoMap 路径。

## 4.3 `args` / `odom_args` 怎么用

`args` 更适合:

- 传 RTAB-Map 风格命令行参数。
- 临时覆盖个别参数。
- 开关类 flag。

例如:

```bash
args:="--delete_db_on_start --udebug --Optimizer/Strategy 1 --RGBD/LinearUpdate 0.05"
```

`odom_args` 更适合只改 odom 节点:

```bash
odom_args:="--Odom/Strategy 1 --Vis/MaxFeatures 2000"
```

## 4.4 GUI 配置文件 `gui_cfg`

`gui_cfg` 只影响 `rtabmap_viz` 的界面布局、窗口状态、显示开关。

它不负责:

- odom 算法
- 后端优化
- 地图生成

也就是说:

- 算法调参看 `cfg`
- GUI 布局看 `gui_cfg`

## 5. 当前这条 stereo + imu 链路最重要的参数

## 5.1 同步和时间

- `use_sim_time:=true`
- `approx_sync:=false`
- `wait_imu_to_init:=true`

如果你额外启用 `rgbd_sync:=true`，再明确:

- `approx_rgbd_sync:=false`

## 5.2 前端 odometry

- `Odom/Strategy`
  - `0=F2M`
  - `1=F2F`
- `Reg/Strategy`
  - `0=Vis`
  - `1=Icp`
  - `2=VisIcp`
- `Vis/FeatureType`
- `Vis/MaxFeatures`
- `Stereo/DenseStrategy`

## 5.3 后端优化

- `Mem/IncrementalMemory`
- `Mem/InitWMWithAllNodes`
- `Optimizer/Strategy`
  - `0=TORO`
  - `1=g2o`
  - `2=GTSAM`
  - `3=Ceres`
- `Optimizer/GravitySigma`

建议:

- 既然你已经装了 `g2o` 和 `gtsam`，把 `Optimizer/Strategy` 写死，不要依赖“编译时默认值”。
- 对当前 stereo + imu 任务，先从 `g2o` 或 `GTSAM` 二选一稳定跑通，再比较效果。

## 5.4 地图生成

- `RGBD/CreateOccupancyGrid`
- `Grid/Sensor`
- `Grid/3D`
- `Grid/CellSize`
- `Grid/RayTracing`
- `Grid/GroundIsObstacle`
- `GridGlobal/OccupancyThr`
- `cloud_output_voxelized`
- `octomap_tree_depth`

如果你只是想让地图更“voxel 化”，优先试:

- 更大的 `Grid/CellSize`
- `cloud_output_voxelized:=true`

而不是一上来改源码。

## 6. 常见运行组合

## 6.1 每次重跑都从空库开始

```bash
args:="--delete_db_on_start"
```

## 6.2 想看 RTAB-Map 自己的 GUI，不想看 RViz

```bash
rtabmap_viz:=true rviz:=false
```

## 6.3 想同时开 RViz

```bash
rtabmap_viz:=true rviz:=true
```

注意:

- `rviz:=true` 依赖 `rtabmap_rviz_plugins`
- `rtabmap_viz:=true` 依赖的是 `rtabmap_viz`

## 6.4 只想做定位，不再扩图

```bash
localization:=true
```

## 7. 一个建议的使用顺序

建议你按这个顺序跑:

1. 先用 `stereo:=true + rtabmap_viz:=true + rviz:=false` 跑通最基本建图。
2. 先固定 `approx_sync:=false`、`wait_imu_to_init:=true`，确认 TF 和 topic 都没问题。
3. 轨迹先看 `/rtabmap/mapPath` 和 `/rtabmap/localization_pose`。
4. 跑稳以后，再去比较 `Optimizer/Strategy=1` 和 `2`，以及 `Odom/Strategy=0` 和 `1`。
5. 地图形式先用参数调，再决定是否需要改源码。
