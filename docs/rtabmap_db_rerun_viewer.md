# RTAB-Map `.db` 读取与 Rerun 可视化

本文对应脚本:

- [rtabmap_db_rerun_viewer.py](/home/wjxu22/SLAMBot/rtab_ws/src/rtabmap/oak_tools/rtabmap_db_rerun_viewer.py)

目标:

- 直接读取 RTAB-Map 的 `.db`
- 恢复优化后的位姿和 RGB-D 数据
- 重建全局 RGB 点云地图
- 记录相机 RGB 图像和对应相机位姿时间轴
- 用 `rerun` 可视化

## 0. 图像质量说明

当前 [rtabmap_db_rerun_viewer.py](/home/wjxu22/SLAMBot/rtab_ws/src/rtabmap/oak_tools/rtabmap_db_rerun_viewer.py) 对 `world/camera/rgb` 不做额外降采样、缩放或重编码。

也就是说:

- viewer 显示的是 `.db` 里保存的原图解码结果
- 如果这份 `.db` 里 RGB 是 JPEG，损失来自 `.db` 本身
- 如果这份 `.db` 里 RGB 是 PNG，就会按 PNG 的质量显示

对你之前检查过的旧库:

- `/home/wjxu22/Datasets/outputs/rtab/pcd_map_tydk_room.db`

其 `Data.image` 头部是 JPEG，所以旧库的图像质量瓶颈不在 viewer。

## 1. 这种 `.db` 文件本质上是什么

RTAB-Map 的数据库本质上是一个 `sqlite3` 文件，但里面很多关键字段不是“人类可直接读”的文本，而是 RTAB-Map 自己定义的二进制 blob。

当前这类 RGB 点云数据库，最关键的是这几张表:

- `Node`
  - 节点 id
  - 时间戳 `stamp`
  - 节点 pose
- `Data`
  - `image`: 压缩图像
  - `depth`: 压缩深度图
  - `calibration`: 序列化后的相机模型
- `Admin`
  - `opt_ids`
  - `opt_poses`
  - 这里保存了 RTAB-Map 优化后的位姿

对你现在这份库，已经确认:

- `Data.image` 是 JPEG
- `Data.depth` 是 `DEPTHRVL`
- `Admin.opt_poses` 非空

所以最靠谱的读法不是只查 `Node.pose`，而是:

1. 先从 `Admin.opt_poses` 取优化后的最终位姿
2. 再从 `Data.image/depth/calibration` 恢复每帧 RGB-D
3. 用最终位姿把每帧点云拼成全局地图

## 2. 环境

这个脚本按你的要求，默认使用 conda 环境:

- `jarvis`

当前已确认这个环境里有:

- `rerun`
- `numpy`
- `opencv-python`

没有安装或卸载任何新的第三方库。

## 3. 默认输入输出目录

当前文档统一建议使用:

- 输出目录: `/home/wjxu22/Datasets/outputs/rtab`

如果不显式传 `--db-path`，脚本会按下面规则找数据库:

1. 优先找 `/home/wjxu22/Datasets/outputs/rtab/pointcloud.db`
2. 如果没有，但目录下只有一个 `.db`，就直接用它
3. 如果有多个 `.db`，要求你显式传 `--db-path`

## 4. 直接启动可视化

默认行为:

- 不做 image decimation
- 不做 voxel downsampling
- 不做 `max_points` 截断
- 默认会同时记录:
  - 全局 RGB 点云地图
  - 相机轨迹
  - `world/camera` 下的逐帧相机位姿和内参
  - `world/camera/rgb` 下的逐帧 RGB 图像

```bash
conda run -n jarvis python /home/wjxu22/SLAMBot/rtab_ws/src/rtabmap/oak_tools/rtabmap_db_rerun_viewer.py \
  --db-path /home/wjxu22/Datasets/outputs/rtab/pointcloud.db \
  --spawn
```

如果你现在目录里实际文件名不是 `pointcloud.db`，就把路径换成真实文件名，比如:

```bash
conda run -n jarvis python /home/wjxu22/SLAMBot/rtab_ws/src/rtabmap/oak_tools/rtabmap_db_rerun_viewer.py \
  --db-path /home/wjxu22/Datasets/outputs/rtab/pcd_map_tydk_room.db \
  --spawn
```

## 5. 不开窗口，只导出 `.rrd`

如果你想先生成 rerun recording，再手动打开:

```bash
conda run -n jarvis python /home/wjxu22/SLAMBot/rtab_ws/src/rtabmap/oak_tools/rtabmap_db_rerun_viewer.py \
  --db-path /home/wjxu22/Datasets/outputs/rtab/pointcloud.db \
  --save-rrd /home/wjxu22/Datasets/outputs/rtab/pointcloud.rrd
```

如果你既不传 `--spawn`，也不传 `--save-rrd`，脚本会默认输出到:

- `/home/wjxu22/Datasets/outputs/rtab/<db_name>.rrd`

## 6. 常用参数

- `--image-decimation 2`
  - 默认是 `1`
  - 大于 `1` 时才做像素降采样
- `--min-depth 0.2`
  - 过滤太近的点
- `--max-depth 5.0`
  - 过滤太远的点
- `--voxel-size 0.03`
  - 默认是 `0`
  - 大于 `0` 时才做全局体素降采样
- `--max-points 800000`
  - 默认是 `0`
  - 大于 `0` 时才做最终点数截断
- `--max-nodes 50`
  - 只处理前 50 个节点，适合快速试跑
- `--node-step 2`
  - 每 2 帧取 1 帧

一个显式降采样的试跑例子:

```bash
conda run -n jarvis python /home/wjxu22/SLAMBot/rtab_ws/src/rtabmap/oak_tools/rtabmap_db_rerun_viewer.py \
  --db-path /home/wjxu22/Datasets/outputs/rtab/pointcloud.db \
  --spawn \
  --image-decimation 2 \
  --voxel-size 0.05 \
  --max-points 800000 \
  --max-nodes 50
```

## 7. 当前脚本支持范围

当前脚本面向:

- RGB-D 类型的 RTAB-Map 数据库

具体来说，它依赖:

- `Data.image`
- `Data.depth`
- `Data.calibration`
- `Admin.opt_poses`

它目前不打算支持:

- 只有双目左右图、没有现成 depth 的纯 stereo 数据库

因为那类库里 `Data.depth` 实际可能是右目图像，不是深度图，想恢复点云就得重新做双目匹配，这已经不是“读库可视化”而是“重新跑一次深度恢复”。

## 8. 你如果只想自己 inspect `.db`

最简单的几个命令:

```bash
sqlite3 /home/wjxu22/Datasets/outputs/rtab/pointcloud.db '.tables'
```

```bash
sqlite3 /home/wjxu22/Datasets/outputs/rtab/pointcloud.db '.schema Node'
```

```bash
sqlite3 /home/wjxu22/Datasets/outputs/rtab/pointcloud.db '.schema Data'
```

```bash
sqlite3 /home/wjxu22/Datasets/outputs/rtab/pointcloud.db '.schema Admin'
```

但只靠 `sqlite3` 查看 schema 不够恢复彩色点云，真正恢复地图还是建议直接用上面的 Python 脚本。

## 9. Rerun 里默认会看到什么

默认会有这几类实体:

- `world/map`
  - 全局 RGB 点云地图
- `world/trajectory`
  - 相机轨迹折线
- `world/cameras`
  - 所有相机位姿采样点
- `world/camera`
  - 按时间变化的当前相机位姿和相机内参
- `world/camera/rgb`
  - 按时间变化的 RGB 图像

所以你拖动时间轴时，可以同时看:

- 当前时刻相机在哪
- 当前时刻对应的 RGB 图像是什么
- 它相对于整张地图处在什么位置
