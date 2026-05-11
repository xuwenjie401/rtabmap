# RTAB-Map Map Notes

  ## 可视化点云 / Metric Map

  - mapData、mapGraph、local_grid_* 发布：rtabmap_ros/rtabmap_slam/src/CoreWrapper.cpp:288
  - /rtabmap/map、/rtabmap/cloud_map、/rtabmap/cloud_obstacles 发布器：rtabmap_ros/rtabmap_util/src/MapsManager.cpp:135
  - cloud_map = obstacles + ground：rtabmap_ros/rtabmap_util/src/MapsManager.cpp:1009
  - rtabmap_viz 订阅 info + mapData：rtabmap_ros/rtabmap_viz/src/GuiWrapper.cpp:150
  - RViz MapCloudDisplay 从 MapData 生成点云：rtabmap_ros/rtabmap_rviz_plugins/src/MapCloudDisplay.cpp:266

  ## Cloud Map 是否随图优化调整

  - 每次处理后更新 map caches 并发布：rtabmap_ros/rtabmap_slam/src/CoreWrapper.cpp:2389
  - MapsManager::updateMapCaches()：rtabmap_ros/rtabmap_util/src/MapsManager.cpp:382
  - 检测 graph pose 是否变化：rtabmap_ros/rtabmap_util/src/MapsManager.cpp:737
  - graph 变化后清空/重组 assembled cloud：rtabmap_ros/rtabmap_util/src/MapsManager.cpp:766
  - 按优化后 node pose 变换 local cloud：rtabmap_ros/rtabmap_util/src/MapsManager.cpp:877
  - 输出前体素化：rtabmap_ros/rtabmap_util/src/MapsManager.cpp:965

  ## Stereo Dense Depth / SGBM / BM

  - 当前配置 Stereo/DenseStrategy=0：docs/rtabmap_oak_d_pro_w_stereo_imu.ini:309
  - 参数含义：0=StereoBM, 1=StereoSGBM：rtabmap/corelib/include/rtabmap/core/Parameters.h:809
  - dense stereo 工厂选择 BM/SGBM：rtabmap/corelib/src/StereoDense.cpp:33
  - 视差入口：rtabmap/corelib/src/util2d.cpp:738
  - stereo 图像转 disparity 再转 cloud：rtabmap/corelib/src/util3d.cpp:1102
  - disparity 投影成 3D 点：rtabmap/corelib/src/util3d.cpp:670
  - OpenCV StereoBM 实现：rtabmap/corelib/src/stereo/StereoBM.cpp:79
  - OpenCV StereoSGBM 实现：rtabmap/corelib/src/stereo/StereoSGBM.cpp:68

  - 没有看到 RTAB-Map 内置类似 DSO/LSD-SLAM 的半稠密深度模块，也没有 ELAS、PatchMatch、深度网络、跨帧持续深度优化这类 stereo dense backend。标准路径是“单帧 stereo pair -> BM/SGBM disparity -> depth/point cloud -> local grid”。
  “调 BM/SGBM 的置信度阈值让结果接近半稠密”是合理的，但只是效果上接近。比如：
  
  - StereoBM/TextureThreshold 会让低纹理区域失效，点更多集中在有纹理/梯度的地方。
  - StereoBM|SGBM/UniquenessRatio
  - StereoBM|SGBM/Disp12MaxDiff
  - StereoBM|SGBM/SpeckleWindowSize, SpeckleRange
  - Grid/RangeMax, Grid/DepthDecimation, Grid/MinClusterSize  


  ## 地图保存

  .db 里默认不保存一个完整的全局 /rtabmap/cloud_map 点云 blob。RTAB-Map 保存的是每个节点的 memory 数据：图像/右图、标定、位姿图、特征、以及可选的局部 occupancy grid，也就是 ground_cells / obstacle_cells / empty_cells。cloud_map
  是之后按优化后的 pose graph 把这些每节点局部 cloud/grid 重新拼出来的。

  你当前配置不是无损 cloud map：

  - docs/rtabmap_oak_d_pro_w_stereo_imu.ini:13: RGBD\CreateOccupancyGrid = false，不保存每节点局部 grid。
  - docs/rtabmap_oak_d_pro_w_stereo_imu.ini:177: Mem\ImageCompressionFormat = .jpg，stereo 左图和右图都会有损保存。
  - docs/rtabmap_oak_d_pro_w_stereo_imu.ini:57: Grid\DepthDecimation = 4，生成局部 cloud/grid 前已经降采样。
  - docs/rtabmap_oak_d_pro_w_stereo_imu.ini:74: Grid\PreVoxelFiltering = true，生成局部 grid 前还会按 Grid/CellSize 体素化。

  建议配置

  要让 .db 尽量“无损地”保存可重建 cloud map，至少改成：

  RGBD\CreateOccupancyGrid = true
  Mem\BinDataKept = true
  Mem\ImageCompressionFormat = .png

  Grid\Sensor = 1
  Grid\3D = true
  Grid\DepthDecimation = 1
  Grid\PreVoxelFiltering = false
  Grid\RangeMin = 0.0
  Grid\RangeMax = 0.0
  Grid\RayTracing = false
  Grid\NoiseFilteringRadius = 0.0

  如果你想让所有有效双目点都进入 cloud map，而不是经过地面/障碍分割后可能丢小簇，可再加：

  Grid\GroundIsObstacle = true

  这样会牺牲 ground/obstacle 语义，把点基本都作为 obstacle 保存。若仍想保留地面/障碍分类，则不要开这个，但把 Grid\MinClusterSize 调小，比如 1。

  另外，发布出来的 /rtabmap/cloud_map 默认还会二次体素化。要让可视化/导出的 ROS 点云也不再被二次降采样，需要在 rtabmap 节点 ROS 参数里设：

  cloud_output_voxelized: false
  cloud_subtract_filtering: false

  这两个不是 RTAB-Map core 的 --Grid/... 参数，代码在 rtabmap_ros/rtabmap_util/src/MapsManager.cpp:99。当前 rtabmap_ros/rtabmap_launch/launch/rtabmap.launch.py:294 的 node parameters 里没有显式暴露它们，需要在 launch 的
  parameters=[{...}] 里加，或写一个 wrapper launch。

###  关键代码位置

  - 启动入口：你的脚本实际调用 rtabmap_launch/rtabmap.launch.py，见 oak_tools/run_rtabmap_oak_stereo_imu.sh:12，DB 路径和 cfg 在 oak_tools/run_rtabmap_oak_stereo_imu.sh:26。
  - 参数定义：Mem/BinDataKept、Mem/ImageCompressionFormat 在 rtabmap/corelib/include/rtabmap/core/Parameters.h:204；RGBD/CreateOccupancyGrid 在 rtabmap/corelib/include/rtabmap/core/Parameters.h:387；Grid/DepthDecimation、Grid/
    CellSize、Grid/PreVoxelFiltering 在 rtabmap/corelib/include/rtabmap/core/Parameters.h:839。
  - memory 参数读取：Mem/BinDataKept、压缩格式在 rtabmap/corelib/src/Memory.cpp:566；RGBD/CreateOccupancyGrid 在 rtabmap/corelib/src/Memory.cpp:610。
  - 图像/右图压缩进 DB 前的逻辑：见 rtabmap/corelib/src/Memory.cpp:5885。关键是 rtabmap/corelib/src/Memory.cpp:5942：stereo 右图不是 16UC1/32FC1 depth 时，也使用 _rgbCompressionFormat，所以当前 .jpg 会让右图也有损。
  - local occupancy grid 生成并挂到节点数据：见 rtabmap/corelib/src/Memory.cpp:6175。
  - local grid/cell 压缩保存：见 rtabmap/corelib/src/SensorData.cpp:544。这里用 compressData2 保存 ground/obstacles/empty，不是 JPEG。
  - SQLite 表结构：Data.image、Data.depth、ground_cells、obstacle_cells、empty_cells 在 rtabmap/corelib/src/resources/DatabaseSchema.sql.in:31。
  - 写入 DB：saveQuery() 在 rtabmap/corelib/src/DBDriverSqlite3.cpp:4376，保存 SensorData 在 rtabmap/corelib/src/DBDriverSqlite3.cpp:4510，具体 image/depth/right blob 绑定在 rtabmap/corelib/src/DBDriverSqlite3.cpp:6260，grid blob
    绑定在 rtabmap/corelib/src/DBDriverSqlite3.cpp:6485。
  - 从 DB 读回 local grid：查询字段在 rtabmap/corelib/src/DBDriverSqlite3.cpp:1297，读 ground/obstacle/empty 在 rtabmap/corelib/src/DBDriverSqlite3.cpp:1787。
  - cloud_map 重建：MapsManager::updateMapCaches() 会优先用 DB 中已有 grid，否则从图像/scan 重新生成，见 rtabmap_ros/rtabmap_util/src/MapsManager.cpp:503。
  - cloud_map 发布和二次体素化：默认 cloudOutputVoxelized_(true) 在 rtabmap_ros/rtabmap_util/src/MapsManager.cpp:63，体素化发生在 rtabmap_ros/rtabmap_util/src/MapsManager.cpp:965，最终 /cloud_map 发布在 rtabmap_ros/rtabmap_util/
    src/MapsManager.cpp:1009。



## 地图保存 appendix
按这个“无损”的定义，答案是：

  oak_tools/export_oak_rtabmap_processed_outputs.py 对 bag 里的 /rtabmap/cloud_map 消息本身基本是无损导出，但 /rtabmap/cloud_map 本身通常已经比 rtabmap_viz 里看到的实时 3D 点云粗很多。所以你看到“运行中轮廓清晰，导出后模糊粗糙”，大概
  率不是 Python 导出损失，而是比较的不是同一类点云。

  关键区别

  rtabmap_viz 里显示的 3D cloud 通常是按每个节点的 SensorData 直接生成的显示点云，代码在 rtabmap/guilib/src/MainWindow.cpp:3715。它调用：

  - rtabmap/guilib/src/MainWindow.cpp:3716: util3d::cloudRGBFromSensorData(...)
  - decimation 用 GUI 的 getCloudDecimation(0)，voxel 用 GUI 的 getVoxel()，见 rtabmap/guilib/src/MainWindow.cpp:3741

  而你导出的 /rtabmap/cloud_map 是 ROS map publisher 的全局 cloud map，来自 local grid/cache 拼接：

  - cache/重建入口：rtabmap_ros/rtabmap_util/src/MapsManager.cpp:382
  - 从 DB/local grid 或图像生成 local map：rtabmap_ros/rtabmap_util/src/MapsManager.cpp:503
  - 发布 /cloud_map：rtabmap_ros/rtabmap_util/src/MapsManager.cpp:1009

  它会受这些参数影响：

  - Grid/DepthDecimation
  - Grid/PreVoxelFiltering
  - Grid/CellSize
  - Grid/RangeMin/RangeMax
  - Grid/MinClusterSize
  - cloud_output_voxelized

  你当前 cloud launch / DB 里有明显会变粗的配置：Grid/DepthDecimation=2、Grid/CellSize=0.04、Grid/PreVoxelFiltering=true、Grid/MinClusterSize=20、Mem/ImageCompressionFormat=.jpg。这会让最终 cloud map 比实时 GUI cloud 粗。

  导出脚本是否损失

  oak_tools/export_oak_rtabmap_processed_outputs.py 不重新生成点云，也不 voxel/downsample。它只是读 processed bag 里的最后一帧 PointCloud2：

  - 订阅/读取 topic：oak_tools/export_oak_rtabmap_processed_outputs.py:578
  - 只保留 latest cloud：oak_tools/export_oak_rtabmap_processed_outputs.py:591
  - 转数组：oak_tools/export_oak_rtabmap_processed_outputs.py:459
  - 写 PLY/NPZ：oak_tools/export_oak_rtabmap_processed_outputs.py:550

  唯一的小差异是它会丢弃 NaN/Inf 点，并且只保存 xyz/rgb/intensity，不会保存 normals 等额外字段。坐标仍是 float32，颜色是 uint8，和 PointCloud2 常见格式一致。

  怎么让导出接近运行中清晰点云

  核心是让 /rtabmap/cloud_map 本身高分辨率：

  RGBD/CreateOccupancyGrid = true
  Grid/DepthDecimation = 1
  Grid/PreVoxelFiltering = false
  Grid/RangeMax = 0
  Grid/MinClusterSize = 1
  Mem/ImageCompressionFormat = .png

  并在 ROS 参数里关掉发布前二次体素化：

  cloud_output_voxelized: false
  cloud_subtract_filtering: false

  这两个参数位置在 rtabmap_ros/rtabmap_util/src/MapsManager.cpp:99，默认 cloudOutputVoxelized_(true) 在 rtabmap_ros/rtabmap_util/src/MapsManager.cpp:63，真正二次 voxel 在 rtabmap_ros/rtabmap_util/src/MapsManager.cpp:965。

  一句话：export_oak... 相对 /rtabmap/cloud_map 基本无损；但 /rtabmap/cloud_map 相对 rtabmap_viz 直接显示的 dense/per-node cloud 不是无损，粗糙主要来自 RTAB-Map map cloud 生成/发布链路里的 decimation 和 voxel。
