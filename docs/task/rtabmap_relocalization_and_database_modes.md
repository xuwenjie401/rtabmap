# RTAB-Map 重定位与历史数据库模式

这份文档记录当前工作区里 RTAB-Map 重定位模式的 launch 方法、对应源码流程，以及“加载已有 `.db` 并继续修改旧地图/Memory”的正确模式。

## 重定位模式怎么 Launch

RTAB-Map 的重定位模式就是 localization mode：加载已有数据库，在旧地图上估计当前位姿，不按普通建图逻辑继续扩图。

通用上游 launch：

```bash
source /opt/ros/humble/setup.bash
source /home/wjxu22/SLAMBot/rtab_ws/install/setup.bash

ros2 launch rtabmap_launch rtabmap.launch.py \
  use_sim_time:=true \
  stereo:=true \
  localization:=true \
  rtabmap_viz:=true \
  rviz:=false \
  approx_sync:=true \
  approx_sync_max_interval:=0.005 \
  wait_imu_to_init:=true \
  frame_id:=oak \
  vo_frame_id:=oak_odom \
  imu_topic:=/oak/imu/data \
  left_image_topic:=/oak/left/image_rect \
  right_image_topic:=/oak/right/image_rect \
  left_camera_info_topic:=/oak/left/camera_info \
  right_camera_info_topic:=/oak/right/camera_info \
  database_path:=/abs/path/to/existing_map.db
```

重定位时不要传 `--delete_db_on_start`，否则要加载的旧数据库会被删除。

`rtabmap_launch/launch/rtabmap.launch.py` 里有 `localization` 参数。它为 `rtabmap_slam/rtabmap` 节点设置：

```text
Mem/IncrementalMemory=false
Mem/InitWMWithAllNodes=true
```

含义：

- `Mem/IncrementalMemory=false`：进入 localization mode，不做普通 SLAM 增量建图。
- `Mem/InitWMWithAllNodes=true`：把数据库里的所有节点加载进 Working Memory，便于在整张旧地图里重定位。

## 当前 OAK Wrapper 怎么跑

当前 `oak_tools/` 下的 stereo+IMU wrapper 会 include 上游 `rtabmap.launch.py`，但还没有把 `localization` 参数透传出来，并且这些 wrapper 面向建图默认 `delete_db_on_start:=true`。

不改 wrapper 时，可以用 `rtabmap_args` 注入两个 RTAB-Map 参数：

```bash
source /opt/ros/humble/setup.bash
source /home/wjxu22/SLAMBot/rtab_ws/install/setup.bash

ros2 launch /home/wjxu22/SLAMBot/rtab_ws/src/rtabmap/oak_tools/oak_rtabmap_stereo_imu_gravity_lossless_cloud.launch.py \
  database_path:=/home/wjxu22/Datasets/outputs/rtab/oak_stereo_imu_gravity_lossless_cloud.db \
  delete_db_on_start:=false \
  rtabmap_args:="--Mem/IncrementalMemory false --Mem/InitWMWithAllNodes true"
```

`oak_tools/oak_rtabmap_stereo_imu_cloud.launch.py` 同理。

后续更干净的做法是在这些 wrapper 里增加：

```python
DeclareLaunchArgument("localization", default_value="false")
```

并把它传给 include 的 `rtabmap.launch.py`。

## 对应代码流程

ROS launch 到核心库的主路径：

```text
rtabmap_launch/launch/rtabmap.launch.py
  -> Node(package='rtabmap_slam', executable='rtabmap')
  -> rtabmap_ros/rtabmap_slam/src/CoreNode.cpp
  -> rtabmap_slam::CoreWrapper
  -> rtabmap_.init(parameters_, databasePath_)
  -> Memory::init()
  -> Memory::loadDataFromDb()
```

关键文件：

- `rtabmap_ros/rtabmap_launch/launch/rtabmap.launch.py`
  - 声明 `localization`
  - 设置 `Mem/IncrementalMemory` 和 `Mem/InitWMWithAllNodes`
- `rtabmap_ros/rtabmap_slam/src/CoreNode.cpp`
  - 创建 `CoreWrapper`
- `rtabmap_ros/rtabmap_slam/src/CoreWrapper.cpp`
  - 读取 `database_path`、`config_path`、ROS 参数和 CLI `args`
  - 调用 `rtabmap_.init(parameters_, databasePath_)`
  - 处理同步后的 sensor data，并发布 `localization_pose`
- `rtabmap/corelib/src/Rtabmap.cpp`
  - RTAB-Map 后端 SLAM/localization 主状态机
- `rtabmap/corelib/src/Memory.cpp`
  - 从数据库加载节点，管理 STM/WM 和数据库保存

每帧处理路径：

```text
stereo/RGB-D/odom/IMU callbacks
  -> CoreWrapper::common*Callback()
  -> CoreWrapper::processAsync()
  -> CoreWrapper::process()
  -> rtabmap_.process(data, odom, covariance, ...)
  -> Memory::update()
  -> loop closure / proximity / landmark localization checks
  -> update map correction
  -> publish /rtabmap/localization_pose
```

主要输出：

```text
/rtabmap/localization_pose
/rtabmap/mapPath
/rtabmap/odom
TF: map -> oak_odom
```

验证命令：

```bash
ros2 param get /rtabmap/rtabmap Mem/IncrementalMemory
ros2 param get /rtabmap/rtabmap Mem/InitWMWithAllNodes
ros2 topic echo /rtabmap/localization_pose
ros2 run tf2_ros tf2_echo map oak_odom
```

## 是否有加载历史 .db 并修改旧 Memory 的模式

有。这个模式不是 localization mode，而是“用已有数据库继续建图”的普通 SLAM/mapping mode：

```text
database_path=/abs/path/to/existing_map.db
delete_db_on_start=false
Mem/IncrementalMemory=true
```

`Mem/IncrementalMemory=true` 是默认 SLAM 模式。只要 `database_path` 指向已有数据库，并且启动时不删除数据库，RTAB-Map 会打开这个 `.db`，加载之前的 Memory，然后继续添加当前运行的新数据。

通用启动：

```bash
ros2 launch rtabmap_launch rtabmap.launch.py \
  use_sim_time:=true \
  stereo:=true \
  localization:=false \
  database_path:=/abs/path/to/existing_map.db \
  frame_id:=oak \
  vo_frame_id:=oak_odom \
  imu_topic:=/oak/imu/data \
  left_image_topic:=/oak/left/image_rect \
  right_image_topic:=/oak/right/image_rect \
  left_camera_info_topic:=/oak/left/camera_info \
  right_camera_info_topic:=/oak/right/camera_info
```

当前 OAK wrapper：

```bash
ros2 launch /home/wjxu22/SLAMBot/rtab_ws/src/rtabmap/oak_tools/oak_rtabmap_stereo_imu_gravity_lossless_cloud.launch.py \
  database_path:=/abs/path/to/existing_map.db \
  delete_db_on_start:=false
```

如果旧库较大，并且希望启动时直接把所有旧节点放进 Working Memory，可额外加：

```bash
rtabmap_args:="--Mem/InitWMWithAllNodes true"
```

这会提升“马上和整张旧地图匹配”的确定性，但会增加内存和启动耗时。不设置时，默认加载上一轮保存的 working set，再按需要从数据库取节点。

## 这个模式会修改什么

继续建图模式下，RTAB-Map 可能会回写已有 `.db`：

- 添加当前运行产生的新 signatures/nodes；
- 添加新的 neighbor、loop closure、proximity、landmark、gravity links；
- 更新数据库里保存的 optimized graph poses；
- 保存 statistics 和当前 WM state；
- 关闭数据库时保存 changed links，并清理 trash。

所以，如果目标是“旧地图继续扩展、纠正或补充”，应该用 mapping mode 加已有 `database_path`，而不是 `localization:=true`。

localization mode 默认行为不同：

```text
Mem/IncrementalMemory=false
Mem/LocalizationDataSaved=false
```

它会用当前帧和旧地图做匹配，但不会按普通建图方式扩展地图。关闭数据库时，RTAB-Map 仍可能保存最新 optimized poses 和 last localization pose；但默认不会追加一段新的建图 session。

如果显式启用：

```text
Mem/LocalizationDataSaved=true
```

localization 数据也可以保存，数据库可能增长。源码参数说明把这个用途标为调试模式，不建议把它当成常规“继续建图”方式。

## 运行时切换

`CoreWrapper` 还暴露这些服务：

```text
/rtabmap/rtabmap/set_mode_localization
/rtabmap/rtabmap/set_mode_mapping
/rtabmap/rtabmap/load_database
```

`set_mode_localization` 会把 `Mem/IncrementalMemory` 设置为 `false`。

`set_mode_mapping` 会把 `Mem/IncrementalMemory` 设置为 `true`。

`load_database` 会关闭当前数据库，再打开另一个数据库，并沿用当前节点参数。如果新数据库是用不同 RTAB-Map 参数建出来的，源码会警告更推荐重启节点。

## 实用结论

按目标选择模式：

```text
只在旧地图上定位:
  localization:=true
  delete_db_on_start:=false

加载旧地图并继续修改/扩展它:
  localization:=false
  delete_db_on_start:=false
  Mem/IncrementalMemory=true

启动时把旧库所有节点放进 WM:
  Mem/InitWMWithAllNodes=true
```

