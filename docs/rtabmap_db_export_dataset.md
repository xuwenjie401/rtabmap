# RTAB-Map `.db` 导出图像与点云

这个工具用于直接读取 RTAB-Map 保存的 `.db`，导出：

- 全局彩色点云
- RGB 图像
- 每张图像对应的优化后位姿
- 可直接被 `rerun_extracted_dataset_viewer.py` 加载的 manifest

工具路径：

- [export_rtabmap_db_dataset.py](/home/wjxu22/SLAMBot/rtab_ws/src/rtabmap/oak_tools/export_rtabmap_db_dataset.py)

## 1. 基本用法

当前脚本依赖 `cv2`，建议直接用你的 `jarvis` 环境：

```bash
conda run -n jarvis python /home/wjxu22/SLAMBot/rtab_ws/src/rtabmap/oak_tools/export_rtabmap_db_dataset.py \
  --db-path /home/wjxu22/Datasets/outputs/rtab/xwj_room.db \
  --output-dir /home/wjxu22/Datasets/outputs/rtab/xwj_room_export
```

默认行为：

- 图像导出为 `png`
- 点云重建不降采样
- 使用 RTAB-Map 优化后的位姿 `Admin.opt_poses`
- 如果数据库里没有优化位姿，则回退到 `Node.pose`

## 2. 仅在需要时显式降采样

```bash
conda run -n jarvis python /home/wjxu22/SLAMBot/rtab_ws/src/rtabmap/oak_tools/export_rtabmap_db_dataset.py \
  --db-path /home/wjxu22/Datasets/outputs/rtab/xwj_room.db \
  --output-dir /home/wjxu22/Datasets/outputs/rtab/xwj_room_export_ds \
  --image-decimation 2 \
  --voxel-size 0.03 \
  --max-points 1000000
```

含义：

- `--image-decimation 2`: 每 2 个像素采一个点
- `--voxel-size 0.03`: 最终点云做 3cm 体素下采样
- `--max-points 1000000`: 最终最多保留 100 万点

## 3. 输出目录结构

示例：

```text
xwj_room_export/
  dataset_info.json
  export_info.json
  images_manifest.json
  pointclouds_manifest.json
  rgb/
    000000.png
    000001.png
    ...
  pointcloud/
    map_cloud.npz
    map_cloud.ply
```

其中：

- `rgb/*.png`: 导出的 RGB 图像
- `images_manifest.json`: 图像索引、时间戳、文件路径、位姿、相机内参
- `pointcloud/map_cloud.npz`: `xyz + rgb` 的 numpy 点云
- `pointcloud/map_cloud.ply`: 通用点云文件
- `pointclouds_manifest.json`: 点云文件描述

## 4. 图像位姿清单格式

`images_manifest.json` 中每项包含：

- `index`
- `node_id`
- `timestamp_ns`
- `timestamp_seconds`
- `path`
- `pose`
- `node_pose`
- `camera_model`

其中：

- `pose`: `optimized_map -> camera_optical_frame`
- `node_pose`: `optimized_map -> base_frame`

如果你只是想按图像时间戳查相机位姿，直接用 `pose` 即可。

## 5. 导出的点云如何读取

两种方式：

```bash
python - <<'PY'
import numpy as np
cloud = np.load('/home/wjxu22/Datasets/outputs/rtab/xwj_room_export/pointcloud/map_cloud.npz')
print(cloud['xyz'].shape, cloud['rgb'].shape)
PY
```

或者直接读：

- `pointcloud/map_cloud.ply`

## 6. 用 Rerun 直接加载导出结果

导出目录可直接给现有 viewer：

```bash
conda run -n jarvis python /home/wjxu22/SLAMBot/rtab_ws/src/rtabmap/oak_tools/rerun_extracted_dataset_viewer.py \
  --dataset-dir /home/wjxu22/Datasets/outputs/rtab/xwj_room_export \
  --spawn
```

## 7. 关于图像质量

这个导出工具不会主动压低图像画质：

- 默认导出为 `png`
- 不会额外缩放或重压缩成更低质量

但如果原始 `.db` 中保存的 `Data.image` 本身已经是 JPEG，那么导出为 PNG 也只能保留当时库里的画质，无法恢复丢失细节。
