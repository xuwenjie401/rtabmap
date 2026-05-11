# RTAB-Map / OAK 文档入口

本文是 `docs/` 的入口目录。除非单篇文档另有说明，命令示例默认从 RTAB-Map 仓库根目录运行：

```bash
cd /home/wjxu22/SLAMBot/rtab_ws/src/rtabmap
```

本地 OAK/SLAMBot 工具位于 `oak_tools/`，配置和说明文档位于 `docs/`，ROS Humble 兼容头位于 `compat_include/`。

## 快速运行

- [RTAB-Map 使用说明](rtabmap_usage_guide.md)：通用启动、常见参数和基础工作流。
- [OAK-D-PRO-W + RTAB-Map 轨迹生成指南](rtabmap_oak_d_pro_w_stereo_imu_trajectory_guide.md)：Stereo+IMU 轨迹、OAK 驱动参数、RTAB-Map 配置。
- [OAK RGB Pointcloud Map](oak_rgb_pointcloud_map_guide.md)：RGB-D/VIO 模式、实时 RGB 点云累计、RViz 和最终导出。
- [OAK RGB-D 按时间戳取位姿的 Bag 录制说明](oak_rgbd_timestamp_pose_bag_guide.md)：录 bag 时如何保留图像、深度、位姿时间关系。
- [OAK Stereo+IMU Map Process](map_process.md)：分终端运行、录轻量 processed bag、最后触发全局 cloud 发布和导出。

## 导出与可视化

- [RTAB-Map `.db` 导出图像与点云](rtabmap_db_export_dataset.md)：从 RTAB-Map 数据库导出图像、点云、位姿 manifest。
- [RTAB-Map `.db` 读取与 Rerun 可视化](rtabmap_db_rerun_viewer.md)：直接查看 `.db` 中的 RGB、深度、点云和轨迹。
- [rosbag2 `.db3` 提取与快速 Rerun 加载](rosbag_db3_extract_and_view.md)：从 rosbag2 提取数据集，再用 Rerun 快速加载。
- [RTAB-Map Map Notes](map_notes.md)：点云地图发布、保存、重建质量和相关源码位置笔记。
- [OAK Stereo+IMU Processed Export Handoff](task/oak_stereo_imu_processed_export_handoff.md)：processed bag 到导出数据集的操作交接记录。

## 构建与代码

- [RTAB-Map ROS2 Humble 最小源码构建说明](rtabmap_ros2_humble_minimal_build.md)：Humble 下源码构建、依赖、兼容头和常见坑。
- [RTAB-Map 代码模块说明](rtabmap_code_module_guide.md)：核心模块、ROS wrapper、参数和代码阅读入口。
- [RTAB-Map Stereo Pipeline Framework](rtabmap_stereo_pipeline_framework.md)：Stereo pipeline 的总体框架说明。
- [RTAB-Map 重定位与历史数据库模式](task/rtabmap_relocalization_and_database_modes.md)：localization、继续建图、旧数据库加载和相关服务。

## 流程图

- [rtabmap_stereo_pipeline_overall.mmd](rtabmap_stereo_pipeline_overall.mmd)：整体 pipeline。
- [rtabmap_stereo_pipeline_frontend.mmd](rtabmap_stereo_pipeline_frontend.mmd)：前端数据流。
- [rtabmap_stereo_pipeline_backend.mmd](rtabmap_stereo_pipeline_backend.mmd)：后端图优化和地图维护。

## 配置资产

- [rtabmap_oak_d_pro_w_stereo_imu.ini](rtabmap_oak_d_pro_w_stereo_imu.ini)：OAK-D-PRO-W Stereo+IMU RTAB-Map 配置。
- [rtabmap_oak_rgb_pointcloud_map.ini](rtabmap_oak_rgb_pointcloud_map.ini)：OAK RGB pointcloud map 配置。
- [oak_rtabmap_rgb_pointcloud_map.rviz](oak_rtabmap_rgb_pointcloud_map.rviz)：RGB pointcloud map 的 RViz 默认视角。

## 学习资料

- [study/README.md](study/README.md)：RTAB-Map 论文和参数快照说明。
- [study/rtabmap_parameters.h](study/rtabmap_parameters.h)：参数头文件快照，便于离线查参数。
- [2011 Memory Management paper](study/2011_labbe_memory_management_for_real_time_appearance_based_loop_closure_detection_iros_arxiv.pdf)
- [2013 Appearance-Based Loop Closure paper](study/2013_labbe_appearance_based_loop_closure_detection_for_online_large_scale_and_long_term_operation_tro_arxiv.pdf)
- [2014 Online Global Loop Closure paper](study/2014_labbe_online_global_loop_closure_detection_for_large_scale_multi_session_graph_based_slam_iros_arxiv.pdf)
- [2018 RTAB-Map JFR paper](study/2018_labbe_rtab_map_as_an_open_source_lidar_and_visual_slam_library_for_large_scale_and_long_term_online_operation_jfr_arxiv.pdf)

## 历史记录

- `agent_notes/`：开发过程中的按日期工作记录，默认被 `.gitignore` 忽略。
- `logs/`：运行日志和历史终端输出，默认被 `.gitignore` 忽略。
