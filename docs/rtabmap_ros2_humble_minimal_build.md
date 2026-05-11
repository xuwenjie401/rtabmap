# RTAB-Map ROS2 Humble 最小源码构建说明

## 目标

当前目标只针对离线 `ros2 bag` 回放，输入为：

- stereo 黑白双目
- IMU
- 已硬时间同步

输出目标是轨迹：时间戳 + pose。

因此这里优先追求：

- 先把最小 SLAM / odom 链路编通
- 不把 `nav2` / `costmap` / demo / 可视化插件当作前置条件
- 不把 `g2o` / `gtsam` 当成第一阻塞项

## 推荐的源码对齐方式

### 结论

如果当前机器使用的是 `ROS 2 Humble`，推荐把 `rtabmap` 和 `rtabmap_ros` 都对齐到 `humble-devel`。

原因：

- 官方 ROS Index 中，`rtabmap` 在 `humble` 下的 `VCS Version` 是 `humble-devel`，版本是 `0.22.1`
- 官方 ROS Index 中，`rtabmap_ros` 在 `humble` 下的 `VCS Version` 也是 `humble-devel`，版本是 `0.22.1`
- 本地验证显示：
  - 当前 `rtabmap` 分支是 `humble-devel`，版本 `0.22.1`
  - 当前 `rtabmap_ros` 分支是 `ros2`
  - 当前 `ros2` 分支里的 `rtabmap_conversions` 要求 `find_package(RTABMap 0.23.4 REQUIRED)`
  - 这和本地 `rtabmap 0.22.1` 不兼容
- 本地还验证了 `origin/humble-devel` 分支里的 `rtabmap_conversions` 要求的是 `find_package(RTABMap 0.22.0 REQUIRED)`，和 `rtabmap 0.22.1` 是兼容的

### 推荐操作

不要先装 `g2o` / `gtsam` 试图绕过当前问题。

先解决版本对齐：

```bash
cd /home/wjxu22/SLAMBot/rtab_ws/src/rtabmap_ros
git checkout humble-devel
```

如果本地分支不存在：

```bash
cd /home/wjxu22/SLAMBot/rtab_ws/src/rtabmap_ros
git checkout -b humble-devel origin/humble-devel
```

## 不推荐的当前方案

保留 `rtabmap_ros` 的 `ros2` 分支，同时继续使用 `rtabmap 0.22.1`。

这会直接撞到版本约束：

```cmake
find_package(RTABMap 0.23.4 REQUIRED)
```

如果坚持保留 `ros2` 分支，那么就不是补装 `g2o` / `gtsam` 能解决的问题，而是需要把 `rtabmap` 也升级到满足 `>=0.23.4` 的版本。

对当前 Humble 目标，这条路不如直接切回 `humble-devel` 稳。

## `g2o` / `gtsam` 的建议

### 当前建议

先不装，先做以下两步：

1. 把 `rtabmap_ros` 切回 `humble-devel`
2. 用下面的最小 `packages-select` 先验证最小链路能否编过

原因：

- 我已经验证过，当前机器上 `rtabmap` 在缺少 `g2o` / `gtsam` / `libpointmatcher` 时，仍然可以完成 CMake 配置并进入实际编译
- 当前真正的硬阻塞是源码版本不匹配，不是 solver 缺失

### 如果后续要装

如果后面你希望尽量接近官方 Humble 依赖集合，再装也可以。优先顺序建议是：

1. `ros-humble-libg2o`
2. `ros-humble-gtsam`
3. `ros-humble-libpointmatcher`

建议按 ROS 包安装，而不是自己手编上游库，这样和 `rosdep` / `ament` 的查找方式更一致。

## 最小 `colcon build` 选择哪些包

### 先说当前机器上的实测结论

- 已确认 `humble-devel + ros-humble-libg2o + ros-humble-gtsam + ros-humble-libpointmatcher + conda deactivate` 可以把下面这条主链编过：
  - `rtabmap`
  - `rtabmap_msgs`
  - `rtabmap_conversions`
  - `rtabmap_sync`
  - `rtabmap_util`
  - `rtabmap_odom`
  - `rtabmap_slam`
  - `rtabmap_viz`
- 已确认 `rtabmap_launch` 不能单独编。
  - 它还要求 `rtabmap_rviz_plugins` 已经先被构建。
- 因此，当前机器上有两套推荐包集：
  - 只要 SLAM + RTAB-Map 自带 GUI：不编 `rtabmap_launch`
  - 想保留官方 launch 入口：把 `rtabmap_rviz_plugins` 和 `rtabmap_launch` 一起加上

### 这台机器当前还需要一个 Humble 兼容头前置

当前 `rtabmap_ros humble-devel` 这份源码里，很多文件包含的是：

```cpp
#include <message_filters/subscriber.hpp>
```

但 Humble 实际安装的是 `.h` 头文件。

当前工作区里已经放了一组本地兼容头：

- `/home/wjxu22/SLAMBot/rtab_ws/src/rtabmap/compat_include`

所以编译前要确保：

```bash
conda deactivate
export CPATH=/home/wjxu22/SLAMBot/rtab_ws/src/rtabmap/compat_include:${CPATH:-}
```

### 包集 A: 只保留当前目标最小运行链路 + RTAB-Map 自带 GUI

```bash
cd /home/wjxu22/SLAMBot/rtab_ws

source /opt/ros/humble/setup.bash
conda deactivate
export CPATH=/home/wjxu22/SLAMBot/rtab_ws/src/rtabmap/compat_include:${CPATH:-}

colcon build \
  --symlink-install \
  --packages-select \
    rtabmap \
    rtabmap_msgs \
    rtabmap_conversions \
    rtabmap_sync \
    rtabmap_util \
    rtabmap_odom \
    rtabmap_slam \
    rtabmap_viz \
  --cmake-clean-cache \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_APP=OFF \
    -DBUILD_TOOLS=OFF \
    -DBUILD_EXAMPLES=OFF
```

### 包集 B: 还要保留官方 `rtabmap_launch`

```bash
cd /home/wjxu22/SLAMBot/rtab_ws

source /opt/ros/humble/setup.bash
conda deactivate
export CPATH=/home/wjxu22/SLAMBot/rtab_ws/src/rtabmap/compat_include:${CPATH:-}

colcon build \
  --symlink-install \
  --packages-select \
    rtabmap \
    rtabmap_msgs \
    rtabmap_conversions \
    rtabmap_sync \
    rtabmap_util \
    rtabmap_odom \
    rtabmap_slam \
    rtabmap_viz \
    rtabmap_rviz_plugins \
    rtabmap_launch \
  --cmake-clean-cache \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_APP=OFF \
    -DBUILD_TOOLS=OFF \
    -DBUILD_EXAMPLES=OFF
```

### 这几个包为什么要保留

- `rtabmap`
  - RTAB-Map 核心库
- `rtabmap_msgs`
  - 自定义消息定义
- `rtabmap_conversions`
  - ROS 消息和 RTAB-Map 核心数据结构之间的转换
- `rtabmap_sync`
  - `stereo_sync` / `rgbd_sync` 等同步组件
- `rtabmap_util`
  - `rtabmap_launch` 里会用到一些工具节点
- `rtabmap_odom`
  - `stereo_odometry` 在这里
- `rtabmap_slam`
  - `rtabmap` ROS2 节点在这里
- `rtabmap_viz`
  - RTAB-Map 自带 GUI，可直接看轨迹和点云
- `rtabmap_rviz_plugins`
  - 不是当前可视化刚需，但 `rtabmap_launch` 依赖它
- `rtabmap_launch`
  - 官方 launch 入口

## 当前不建议一起编的包

下面这些包不是当前目标的前置条件，建议先不要一起编：

```bash
rtabmap_costmap_plugins
rtabmap_demos
rtabmap_examples
rtabmap_ros
```

原因：

- `rtabmap_costmap_plugins` 现在会因为缺 `nav2_costmap_2d` 直接失败
- `demos` / `examples` 对当前“离线 bag 生成轨迹”不是必须
- `rtabmap_ros` 是元包，不是最小运行链路的关键

## 如果想看轨迹和三维点云可视化

### 先说结论

这部分依赖在当前机器上不算麻烦。

我已经验证过：

- 机器上已有 `rviz2` / `rviz_common` / `rviz_default_plugins`
- 机器上已有 Qt5 相关开发包
- 本机 `rtabmap` 在 `WITH_QT=ON` 时可以完成 GUI 相关的 CMake 配置

### 你有两种选择

#### 方案 A: 只用标准 RViz2

如果你只是想看：

- 轨迹
- TF
- 3D 点云

那通常不一定要编 `rtabmap_rviz_plugins`，标准 `rviz2` 就够用了。

这种情况下：

- 可以不编 `rtabmap_rviz_plugins`
- 可以不编 `rtabmap_launch`
- 直接使用 `stereo_odometry` / `rtabmap` / `rtabmap_viz` 这条链
- 不需要 `nav2`
- 不需要 `costmap`

#### 方案 B: 还想用 RTAB-Map 自带 GUI / RTAB-Map 的 RViz 插件

如果你想要：

- `rtabmap_viz`
- 或者 `rtabmap_rviz_plugins`

那建议这样改构建策略。

### 编 `rtabmap_viz` 时

需要：

- 把 `rtabmap_viz` 加进 `packages-select`
- 不要再传 `-DWITH_QT=OFF`
- 编译前先 `conda deactivate`
- 编译前导出 `CPATH=/home/wjxu22/SLAMBot/rtab_ws/src/rtabmap/compat_include:${CPATH:-}`

示例：

```bash
cd /home/wjxu22/SLAMBot/rtab_ws

colcon build \
  --symlink-install \
  --packages-select \
    rtabmap \
    rtabmap_msgs \
    rtabmap_conversions \
    rtabmap_sync \
    rtabmap_util \
    rtabmap_odom \
    rtabmap_slam \
    rtabmap_viz \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_APP=OFF \
    -DBUILD_TOOLS=OFF \
    -DBUILD_EXAMPLES=OFF
```

### 编 `rtabmap_rviz_plugins` 时

如果你还想要 RTAB-Map 的 RViz 自定义显示插件，或者你想保留 `rtabmap_launch`，再把它加上：

```bash
cd /home/wjxu22/SLAMBot/rtab_ws

colcon build \
  --symlink-install \
  --packages-select \
    rtabmap \
    rtabmap_msgs \
    rtabmap_conversions \
    rtabmap_sync \
    rtabmap_util \
    rtabmap_odom \
    rtabmap_slam \
    rtabmap_viz \
    rtabmap_rviz_plugins \
    rtabmap_launch \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_APP=OFF \
    -DBUILD_TOOLS=OFF \
    -DBUILD_EXAMPLES=OFF
```

### 额外提醒

当前机器在配置 Qt/VTK GUI 时有一个小风险：

- CMake 提示 `/home/wjxu22/anaconda3/lib` 里的库可能会遮住系统 Qt/SQLite/zlib 等库

这不是当前的硬阻塞，但如果后面出现 GUI 启动异常、插件加载异常或者奇怪的运行时链接问题，优先怀疑 `conda` 环境污染。

最简单的规避方式是：

- 用没有激活 conda 的 shell 编译和运行
- 或者先 `conda deactivate`

## 如果最小构建通过，再考虑的下一步

构建通过后，再进入运行阶段：

- `source install/setup.bash`
- 如果你编了 `rtabmap_launch`，可直接用 `rtabmap_launch/rtabmap.launch.py`
- 如果你只编了包集 A，就直接手动起：
  - `rtabmap_sync/stereo_sync`
  - `rtabmap_odom/stereo_odometry`
  - `rtabmap_slam/rtabmap`
  - `rtabmap_viz/rtabmap_viz`
- 参数重点关注：
  - `stereo:=true`
  - `use_sim_time:=true`
  - `approx_sync:=false`
  - `wait_imu_to_init:=true`
  - `rtabmap_viz:=false`
  - `rviz:=false`
  - `left_image_topic`
  - `right_image_topic`
  - `left_camera_info_topic`
  - `right_camera_info_topic`
  - `imu_topic`
  - `frame_id`

## 参考

- ROS Index `rtabmap`:
  - https://index.ros.org/p/rtabmap/
- ROS Index `rtabmap_ros`:
  - https://index.ros.org/r/rtabmap_ros/
- 官方仓库：
  - https://github.com/introlab/rtabmap
  - https://github.com/introlab/rtabmap_ros
