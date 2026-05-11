# rosbag2 `.db3` 提取与快速 Rerun 加载

本文对应两个新工具:

- [rosbag_db3_extract_dataset.py](/home/wjxu22/SLAMBot/rtab_ws/src/rtabmap/oak_tools/rosbag_db3_extract_dataset.py)
- [rerun_extracted_dataset_viewer.py](/home/wjxu22/SLAMBot/rtab_ws/src/rtabmap/oak_tools/rerun_extracted_dataset_viewer.py)

目标:

- 从 rosbag2 sqlite3 bag 里把图像和点云单独提取出来
- 给图像和点云分别生成 manifest JSON
- manifest 里记录 `index`、时间戳、路径、frame、对应位姿
- 后续直接从提取结果加载到 `Rerun`，不再碰 `.db3`

## 1. 当前 RTAB-Map DB viewer 的图像质量结论

对 [rtabmap_db_rerun_viewer.py](/home/wjxu22/SLAMBot/rtab_ws/src/rtabmap/oak_tools/rtabmap_db_rerun_viewer.py)：

- `world/camera/rgb` 当前没有额外缩放、降采样或重编码
- 它直接使用数据库里的原始图像 blob 解码后显示
- 所以显示质量瓶颈不在 viewer，而在 `.db` 写入时用的图像压缩格式

对你现在检查过的这份库:

- `/home/wjxu22/Datasets/outputs/rtab/pcd_map_tydk_room.db`

库里 `Data.image` 头部是 JPEG，所以当前可见的损失主要来自这份旧 `.db` 本身，而不是 `Rerun` viewer。

如果要让后续新建的 RTAB-Map `.db` 保存无损 RGB：

- 当前 ini 已经改成 `Mem\\ImageCompressionFormat = .png`

这只影响新录的 `.db`，不会改善已经写成 JPEG 的旧库。

## 2. 提取器用哪个 Python

提取器依赖:

- `rosbag2_py`
- `rclpy.serialization`

所以请用系统 Python 3.10 运行，不要用 conda 的 Python 3.12：

```bash
python3.10 /home/wjxu22/SLAMBot/rtab_ws/src/rtabmap/oak_tools/rosbag_db3_extract_dataset.py --help
```

## 3. 默认输出目录

如果不显式传 `--output-dir`，提取结果默认写到:

- `/home/wjxu22/Datasets/outputs/rtab/extracted/<bag_name>`

目录结构大致是:

```text
<dataset_dir>/
  dataset_info.json
  images_manifest.json
  pointclouds_manifest.json
  images/
  pointclouds/
```

## 4. 先看 bag 里有哪些 topic

```bash
python3.10 /home/wjxu22/SLAMBot/rtab_ws/src/rtabmap/oak_tools/rosbag_db3_extract_dataset.py \
  --bag-path /path/to/bag_dir_or_db3 \
  --list-topics
```

## 5. 提取图像和点云

一个典型例子:

```bash
python3.10 /home/wjxu22/SLAMBot/rtab_ws/src/rtabmap/oak_tools/rosbag_db3_extract_dataset.py \
  --bag-path /path/to/bag_dir_or_db3 \
  --image-topic /oak/rgb/image_raw \
  --pointcloud-topic /oak/rgb_map \
  --fixed-frame map \
  --overwrite
```

含义:

- 图像从 `/oak/rgb/image_raw` 提取
- 点云从 `/oak/rgb_map` 提取
- 位姿优先通过 `/tf` 链在 `map` 下查
- 输出目录已有内容时允许覆盖

如果你更想直接用某个 pose topic 做最近邻关联:

```bash
python3.10 /home/wjxu22/SLAMBot/rtab_ws/src/rtabmap/oak_tools/rosbag_db3_extract_dataset.py \
  --bag-path /path/to/bag_dir_or_db3 \
  --image-topic /oak/rgb/image_raw \
  --pointcloud-topic /oak/rgb_map \
  --pose-topic /oak/vio/odometry \
  --pose-source pose \
  --overwrite
```

## 6. manifest 里有什么

图像 manifest:

- `index`
- `timestamp_ns`
- `path`
- `frame_id`
- `width`
- `height`
- `encoding`
- `pose`

点云 manifest:

- `index`
- `timestamp_ns`
- `path`
- `frame_id`
- `point_count`
- `fields`
- `pose`

`pose` 里包含:

- `reference_frame`
- `target_frame`
- `source`
- `translation_xyz`
- `quaternion_xyzw`

所以后处理时，直接按 `index` 找图像文件，再从同一条 manifest 取 pose 即可。

## 7. 图像如何保存

- `sensor_msgs/msg/Image`
  - 默认写成 `.png`
  - 不再额外有损压缩
- `sensor_msgs/msg/CompressedImage`
  - 直接按原始压缩格式落盘
  - 如果 bag 里原来就是 JPEG，就保持 JPEG
  - 如果原来是 PNG，就保持 PNG

## 8. 点云如何保存

- 每帧点云写成一个 `.npz`
- 当前支持保存:
  - `xyz`
  - `rgb`
  - 或 `intensity`

相比直接回读 `.db3`，后续用 `numpy` 直接加载 `.npz` 会快很多。

## 9. 用新 viewer 直接加载提取结果

这个 viewer 不依赖 ROS bag 库，直接用 `jarvis` 环境即可:

```bash
conda run -n jarvis python /home/wjxu22/SLAMBot/rtab_ws/src/rtabmap/oak_tools/rerun_extracted_dataset_viewer.py \
  --dataset-dir /home/wjxu22/Datasets/outputs/rtab/extracted/my_bag \
  --spawn
```

如果只想生成 `.rrd`:

```bash
conda run -n jarvis python /home/wjxu22/SLAMBot/rtab_ws/src/rtabmap/oak_tools/rerun_extracted_dataset_viewer.py \
  --dataset-dir /home/wjxu22/Datasets/outputs/rtab/extracted/my_bag \
  --save-rrd /home/wjxu22/Datasets/outputs/rtab/extracted/my_bag.rrd
```

## 10. 这个新 viewer 默认会显示什么

- 图像:
  - `world/camera/rgb`
- 图像对应位姿:
  - `world/camera`
- 点云:
  - 有 pose 时在 `world/cloud_sensor/points`
  - 没有 pose 时在 `world/clouds`
- 图像轨迹:
  - `world/trajectory`
  - `world/camera_positions`

## 11. 当前验证情况

我已经验证过:

- 三个脚本都能通过语法检查
- 新 viewer 能加载一份合成的小数据集并成功生成 `.rrd`

我没有在你的数据目录里找到实际存在的 `.db3` 文件，所以还没法对真实 bag 做一轮完整实跑。当前 `/home/wjxu22/Datasets/galbot/test_replay/gal0` 这类目录只有 `metadata.yaml`，缺少 metadata 里引用的 `.db3` 文件本体。
