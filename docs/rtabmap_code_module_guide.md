# RTAB-Map 代码模块说明

本文只面向你当前这条链路: `stereo + imu + ros2bag -> 轨迹/可视化`。重点是回答两件事:

1. 想改某类能力时，应该进哪个模块。
2. 改完以后，`colcon build` 应该最少编哪些包。

## 0. 当前工作区的编译前提

从工作区根目录执行:

```bash
cd /home/wjxu22/SLAMBot/rtab_ws
source /opt/ros/humble/setup.bash
conda deactivate
export CPATH=/home/wjxu22/SLAMBot/rtab_ws/src/rtabmap/compat_include:${CPATH:-}
```

说明:

- `compat_include/message_filters/*.hpp` 是当前 Humble 工作区的本地兼容头，解决 `rtabmap_ros` 里 `.hpp` include 与 Humble 系统包只提供 `.h` 的差异。
- 上面这个 `CPATH` 不是系统安装，只是当前工作区的本地修补。
- 下面所有 `colcon build` 命令默认都在 `/home/wjxu22/SLAMBot/rtab_ws` 执行。

## 1. 总体结构

| 你想改的能力 | 主要模块 | 关键文件/目录 | 最小重编 |
| --- | --- | --- | --- |
| 传感器同步、预处理、topic 入口 | `rtabmap_ros/rtabmap_sync`, `rtabmap_ros/rtabmap_util` | `CommonDataSubscriber*`, `stereo_sync.hpp`, `rgbd_sync.hpp`, `disparity_to_depth.cpp`, `point_cloud_xyzrgb.cpp`, `rgbd_relay.cpp`, `imu_to_tf.cpp` | `rtabmap_sync` 或 `rtabmap_util` |
| 帧间跟踪、视觉/激光里程计前端 | `rtabmap_ros/rtabmap_odom` + `rtabmap/corelib/src/odometry` | `OdometryROS.cpp`, `stereo_odometry.cpp`, `OdometryF2F.cpp`, `OdometryF2M.cpp`, `RegistrationVis.cpp`, `RegistrationIcp.cpp` | `rtabmap_odom`，若动了 core 再加 `rtabmap` |
| 后端图优化、回环、记忆管理 | `rtabmap_ros/rtabmap_slam` + `rtabmap/corelib/src` | `CoreWrapper.cpp`, `Rtabmap.cpp`, `Memory.cpp`, `optimizer/OptimizerG2O.cpp`, `optimizer/OptimizerGTSAM.cpp`, `optimizer/OptimizerCeres.cpp` | `rtabmap_slam`，若动了 core 再加 `rtabmap` |
| 地图表达、局部栅格、全局地图发布 | `rtabmap/corelib/src/global_map`, `LocalGridMaker`, `rtabmap_ros/rtabmap_util/src/MapsManager.cpp` | `LocalGridMaker.cpp`, `OccupancyGrid.cpp`, `OctoMap.cpp`, `CloudMap.cpp`, `GridMap.cpp`, `MapsManager.cpp` | `rtabmap` + `rtabmap_util` |
| RTAB-Map 自己的 ROS GUI | `rtabmap_ros/rtabmap_viz` | `GuiNode.cpp`, `GuiWrapper.cpp`, `PreferencesDialogROS.cpp` | `rtabmap_viz` |
| RTAB-Map 上游 GUI/离线导出逻辑 | `rtabmap/guilib`, `rtabmap/app` | `guilib/src/ExportCloudsDialog.cpp`, `app/src/main.cpp` | `rtabmap` |

## 2. 传感器预处理、同步、入口

### 2.1 改什么属于这一层

- 左右图、RGB-D、Scan、IMU 的同步策略。
- topic 订阅结构、消息拼装方式。
- stereo 转 `rgbd_image` 的逻辑。
- disparity/depth/point cloud 的预处理。
- 进入 odometry/slam 前的 relay、deskew、点云拼接。

### 2.2 主要位置

ROS 入口和同步:

- `rtabmap_ros/rtabmap_sync/include/rtabmap_sync/CommonDataSubscriber.h`
- `rtabmap_ros/rtabmap_sync/src/CommonDataSubscriber.cpp`
- `rtabmap_ros/rtabmap_sync/src/impl/CommonDataSubscriberStereo.cpp`
- `rtabmap_ros/rtabmap_sync/src/impl/CommonDataSubscriberRGBD.cpp`
- `rtabmap_ros/rtabmap_sync/include/rtabmap_sync/stereo_sync.hpp`
- `rtabmap_ros/rtabmap_sync/include/rtabmap_sync/rgbd_sync.hpp`

ROS 预处理节点:

- `rtabmap_ros/rtabmap_util/src/nodelets/disparity_to_depth.cpp`
- `rtabmap_ros/rtabmap_util/src/nodelets/point_cloud_xyz.cpp`
- `rtabmap_ros/rtabmap_util/src/nodelets/point_cloud_xyzrgb.cpp`
- `rtabmap_ros/rtabmap_util/src/nodelets/point_cloud_assembler.cpp`
- `rtabmap_ros/rtabmap_util/src/nodelets/lidar_deskewing.cpp`
- `rtabmap_ros/rtabmap_util/src/nodelets/imu_to_tf.cpp`
- `rtabmap_ros/rtabmap_util/src/nodelets/rgbd_relay.cpp`

### 2.3 对应重编命令

只改同步/订阅层:

```bash
colcon build --packages-select rtabmap_sync
```

只改 util 预处理节点:

```bash
colcon build --packages-select rtabmap_util
```

如果你改了消息流入口，想顺手验证 odom/slam 端是否还兼容，建议保守一点:

```bash
colcon build --packages-select \
  rtabmap_sync \
  rtabmap_util \
  rtabmap_odom \
  rtabmap_slam \
  rtabmap_viz
```

## 3. 帧间 tracking / 里程计前端

### 3.1 改什么属于这一层

- stereo visual odometry 的帧间匹配方式。
- F2F / F2M 的状态更新逻辑。
- 视觉特征、PnP、RANSAC、ICP、Vis+ICP 配准。
- odom 端 IMU 初始化和 odom guess 的接入方式。

### 3.2 主要位置

ROS odom 封装:

- `rtabmap_ros/rtabmap_odom/src/OdometryROS.cpp`
- `rtabmap_ros/rtabmap_odom/src/nodelets/stereo_odometry.cpp`
- `rtabmap_ros/rtabmap_odom/src/nodelets/rgbd_odometry.cpp`
- `rtabmap_ros/rtabmap_odom/src/nodelets/icp_odometry.cpp`

RTAB-Map core 前端算法:

- `rtabmap/corelib/src/odometry/OdometryF2F.cpp`
- `rtabmap/corelib/src/odometry/OdometryF2M.cpp`
- `rtabmap/corelib/src/Registration.cpp`
- `rtabmap/corelib/src/RegistrationVis.cpp`
- `rtabmap/corelib/src/RegistrationIcp.cpp`

可选 odometry 变体:

- `rtabmap/corelib/src/odometry/OdometryMSCKF.cpp`
- `rtabmap/corelib/src/odometry/OdometryVINS.cpp`
- `rtabmap/corelib/src/odometry/OdometryOpenVINS.cpp`
- `rtabmap/corelib/src/odometry/OdometryMono.cpp`

### 3.3 怎么判断改哪边

- 想改 ROS topic、参数声明、TF/IMU 接法: 先看 `rtabmap_odom`。
- 想改真正的 frame-to-frame / frame-to-map 估计逻辑: 进 `rtabmap/corelib/src/odometry`。
- 想改视觉匹配或 ICP 配准: 进 `RegistrationVis.cpp` / `RegistrationIcp.cpp`。

### 3.4 对应重编命令

只改 ROS odom 包装层:

```bash
colcon build --packages-select rtabmap_odom
```

改了 core 前端算法实现:

```bash
colcon build --packages-select \
  rtabmap \
  rtabmap_odom \
  rtabmap_slam \
  rtabmap_viz
```

如果你动的是 `rtabmap/corelib/include/...` 这种公共头文件，建议进一步保守一些:

```bash
colcon build --packages-select \
  rtabmap \
  rtabmap_conversions \
  rtabmap_sync \
  rtabmap_util \
  rtabmap_odom \
  rtabmap_slam \
  rtabmap_viz
```

## 4. 后端优化、回环、图管理

### 4.1 改什么属于这一层

- 回环检测通过后如何建图。
- pose graph 更新、优化触发、约束筛选。
- Working Memory / Long-Term Memory 管理。
- g2o / GTSAM / Ceres 的优化后端接入。
- gravity constraint、landmark/gps/tag 进入图优化的方式。

### 4.2 主要位置

ROS SLAM 封装:

- `rtabmap_ros/rtabmap_slam/src/CoreWrapper.cpp`
- `rtabmap_ros/rtabmap_slam/src/CoreNode.cpp`

RTAB-Map core:

- `rtabmap/corelib/src/Rtabmap.cpp`
- `rtabmap/corelib/src/Memory.cpp`

优化器实现:

- `rtabmap/corelib/src/optimizer/OptimizerG2O.cpp`
- `rtabmap/corelib/src/optimizer/OptimizerGTSAM.cpp`
- `rtabmap/corelib/src/optimizer/OptimizerCeres.cpp`
- `rtabmap/corelib/src/optimizer/OptimizerTORO.cpp`

### 4.3 g2o / GTSAM 相关判断

- 只调参数: 不用进源码，直接改 `Optimizer/Strategy`、`Optimizer/GravitySigma`、`g2o/*`、`GTSAM/*`。
- 想改优化图构造、约束权重、后端求解流程: 才需要进 `Optimizer*.cpp` 和 `Rtabmap.cpp`。

### 4.4 对应重编命令

只改 ROS SLAM 包装层:

```bash
colcon build --packages-select rtabmap_slam
```

改了 core backend 或优化器:

```bash
colcon build --packages-select \
  rtabmap \
  rtabmap_util \
  rtabmap_slam \
  rtabmap_viz
```

## 5. 地图形式: occupancy / voxel / octomap / elevation / TSDF

这一块最容易混淆。当前仓库里，“地图形式”至少分成三层:

1. 局部栅格如何从深度/点云生成。
2. 全局地图在 core 里用什么结构存。
3. ROS 侧把哪种地图发成 topic。

### 5.1 局部地图与分割

入口:

- `rtabmap/corelib/include/rtabmap/core/LocalGridMaker.h`
- `rtabmap/corelib/src/LocalGridMaker.cpp`

这里控制的主要是:

- 深度/scan 生成局部 occupancy grid 的方式。
- 地面/障碍物分割。
- 预 voxel filtering。
- `Grid/*` 参数对应的局部地图生成逻辑。

如果你只是想把点云更粗一些、变成“更 voxel 风格”的局部地图，第一优先通常不是改类结构，而是先调:

- `Grid/CellSize`
- `Grid/PreVoxelFiltering`
- `cloud_output_voxelized`

### 5.2 当前已经存在的全局地图类型

core 全局地图实现:

- `rtabmap/corelib/include/rtabmap/core/global_map/CloudMap.h`
- `rtabmap/corelib/include/rtabmap/core/global_map/OccupancyGrid.h`
- `rtabmap/corelib/include/rtabmap/core/global_map/OctoMap.h`
- `rtabmap/corelib/include/rtabmap/core/global_map/GridMap.h`

对应实现:

- `rtabmap/corelib/src/global_map/CloudMap.cpp`
- `rtabmap/corelib/src/global_map/OccupancyGrid.cpp`
- `rtabmap/corelib/src/global_map/OctoMap.cpp`
- `rtabmap/corelib/src/global_map/GridMap.cpp`

这些大致对应:

- `CloudMap`: 组装后的彩色点云、ground、obstacles、empty cells。
- `OccupancyGrid`: 2D occupancy/probability map。
- `OctoMap`: 3D octree occupancy map。
- `GridMap`: elevation/terrain map。

ROS 发布总控在:

- `rtabmap_ros/rtabmap_util/include/rtabmap_util/MapsManager.h`
- `rtabmap_ros/rtabmap_util/src/MapsManager.cpp`

`MapsManager` 会管理并发布:

- `map`
- `grid_prob_map`
- `cloud_map`
- `cloud_obstacles`
- `cloud_ground`
- `octomap_*`

### 5.3 “voxel map” 如果只是想用现有能力

先不要急着改源码。RTAB-Map 当前在线 ROS 流程里，最接近“voxel map / voxelized cloud”的现成入口是:

- `Grid/CellSize`
- `Grid/PreVoxelFiltering`
- `cloud_output_voxelized`
- `Grid/3D=true` + `Grid/RayTracing=true` + OctoMap 路径

换句话说:

- 想要 2D/3D occupancy: 走 `OccupancyGrid` / `OctoMap`。
- 想要更粗的稠密点云: 先调 voxel 相关参数。
- 想要 elevation/terrain: 看 `GridMap`。

### 5.4 TSDF 目前在这个仓库里的实际状态

当前这份代码树里，TSDF 不是现成的在线 ROS 地图后端。

已经存在的 TSDF 痕迹主要在上游 GUI 的离线导出流程:

- `rtabmap/guilib/src/ExportCloudsDialog.cpp`
- `rtabmap/CMakeLists.txt` 里的 `WITH_CPUTSDF`

也就是说:

- `CPU-TSDF` / `OpenChisel` 相关代码主要用于 GUI 中的导出/重建流程。
- 当前 `rtabmap_ros/rtabmap_util/src/MapsManager.cpp` 并没有像 `OccupancyGrid` / `OctoMap` 那样直接维护一个在线 TSDF map 对象。

因此，如果你想做“在线 ROS runtime 的 TSDF 地图”而不是“离线导出 TSDF mesh”，改动入口应该是:

1. 在 `rtabmap/corelib/include/rtabmap/core/global_map/` 和 `src/global_map/` 下增加一个新的 `GlobalMap` 子类。
2. 在 `Parameters.h` 里加参数，并让新 map backend 能读到它们。
3. 在 `rtabmap_ros/rtabmap_util/src/MapsManager.cpp` 中实例化、更新、发布这个新地图。
4. 视需要在 `rtabmap_slam/CoreWrapper.cpp` 和消息层补 topic / service。

这是一个实质性的源码改造，不建议直接闷头改，最好先单独讨论设计。

### 5.5 对应重编命令

只调 ROS 发布层:

```bash
colcon build --packages-select rtabmap_util
```

改局部/全局地图 core 实现:

```bash
colcon build --packages-select \
  rtabmap \
  rtabmap_util \
  rtabmap_slam \
  rtabmap_viz
```

## 6. 可视化相关模块

### 6.1 RTAB-Map 自己的 ROS GUI

这个是你当前更关心的 GUI:

- `rtabmap_ros/rtabmap_viz/src/GuiNode.cpp`
- `rtabmap_ros/rtabmap_viz/src/GuiWrapper.cpp`

改它只需要:

```bash
colcon build --packages-select rtabmap_viz
```

### 6.2 上游 RTAB-Map GUI 库

如果你改的是 RTAB-Map 上游 GUI 库，比如导出、mesh、CPU-TSDF、OpenChisel 这些:

- `rtabmap/guilib/...`
- `rtabmap/app/...`

对应重编:

```bash
colcon build --packages-select rtabmap
```

如果 `rtabmap_viz` 也要联动验证，再补:

```bash
colcon build --packages-select rtabmap rtabmap_viz
```

## 7. 一个实用的判断顺序

你下次准备改代码时，可以先按这个顺序判断:

1. 这是 topic/sync/预处理问题，还是 core 算法问题。
2. 如果只是参数和消息流问题，优先改 `rtabmap_ros/*`，不要先动 `rtabmap/corelib`。
3. 如果只是想把地图变粗、变 sparse、变 voxelized，优先改参数，不要先创造新地图类型。
4. 只有当 `OccupancyGrid` / `OctoMap` / `CloudMap` / `GridMap` 都不满足目标时，再考虑新增 backend。

## 8. 当前最常用的重编组合

前端 odom 改动:

```bash
colcon build --packages-select \
  rtabmap \
  rtabmap_odom \
  rtabmap_slam \
  rtabmap_viz
```

地图/点云改动:

```bash
colcon build --packages-select \
  rtabmap \
  rtabmap_util \
  rtabmap_slam \
  rtabmap_viz
```

只想重新跑当前完整链路:

```bash
colcon build --packages-select \
  rtabmap \
  rtabmap_msgs \
  rtabmap_conversions \
  rtabmap_sync \
  rtabmap_util \
  rtabmap_odom \
  rtabmap_slam \
  rtabmap_viz \
  rtabmap_rviz_plugins \
  rtabmap_launch
```
