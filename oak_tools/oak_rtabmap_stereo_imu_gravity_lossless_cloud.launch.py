import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParameter


DEFAULT_DB_PATH = "/home/wjxu22/Datasets/outputs/rtab/oak_stereo_imu_gravity_lossless_cloud.db"
DEFAULT_RECORD_BAG = (
    "/home/wjxu22/Datasets/rosbags/processed/oak_rtabmap_gravity_lossless_cloud"
)


def _is_true(value):
    return value.lower() in ("1", "true", "yes", "on")


def _topic(namespace, name):
    namespace = namespace.strip("/")
    if namespace:
        return f"/{namespace}/{name}"
    return f"/{name}"


def launch_setup(context, *args, **kwargs):
    rtabmap_prefix = get_package_share_directory("rtabmap_launch")

    delete_db_on_start = _is_true(
        LaunchConfiguration("delete_db_on_start").perform(context)
    )
    fix_camera_info = _is_true(LaunchConfiguration("fix_camera_info").perform(context))
    swap_stereo = _is_true(LaunchConfiguration("swap_stereo").perform(context))
    record_processed_bag = _is_true(
        LaunchConfiguration("record_processed_bag").perform(context)
    )
    extra_args = LaunchConfiguration("rtabmap_args").perform(context).strip()
    extra_odom_args = LaunchConfiguration("rtabmap_odom_args").perform(context).strip()

    namespace = LaunchConfiguration("namespace").perform(context)
    left_image_topic = LaunchConfiguration("left_image_topic").perform(context)
    right_image_topic = LaunchConfiguration("right_image_topic").perform(context)
    left_camera_info_topic = LaunchConfiguration("left_camera_info_topic").perform(
        context
    )
    right_camera_info_topic = LaunchConfiguration("right_camera_info_topic").perform(
        context
    )
    actions = []

    if fix_camera_info:
        raw_left_camera_info_topic = left_camera_info_topic
        raw_right_camera_info_topic = right_camera_info_topic
        left_camera_info_topic = LaunchConfiguration(
            "fixed_left_camera_info_topic"
        ).perform(context)
        right_camera_info_topic = LaunchConfiguration(
            "fixed_right_camera_info_topic"
        ).perform(context)
        actions.append(
            ExecuteProcess(
                cmd=[
                    LaunchConfiguration("python_executable").perform(context),
                    os.path.join(
                        os.path.dirname(__file__), "oak_stereo_camera_info_fix.py"
                    ),
                    "--left-in",
                    raw_left_camera_info_topic,
                    "--right-in",
                    raw_right_camera_info_topic,
                    "--left-out",
                    left_camera_info_topic,
                    "--right-out",
                    right_camera_info_topic,
                    "--baseline",
                    LaunchConfiguration("stereo_baseline").perform(context),
                ],
                output="screen",
            )
        )

    if swap_stereo:
        left_image_topic, right_image_topic = right_image_topic, left_image_topic
        left_camera_info_topic, right_camera_info_topic = (
            right_camera_info_topic,
            left_camera_info_topic,
        )

    rtabmap_args = [
        "--Rtabmap/ImagesAlreadyRectified",
        "true",
        "--Reg/Strategy",
        "0",
        "--Reg/Force3DoF",
        "false",
        "--RGBD/ForceOdom3DoF",
        "false",
        "--Optimizer/Strategy",
        LaunchConfiguration("optimizer_strategy").perform(context),
        "--Optimizer/GravitySigma",
        LaunchConfiguration("gravity_sigma").perform(context),
        "--Mem/UseOdomGravity",
        LaunchConfiguration("use_odom_gravity").perform(context),
        "--Mem/BinDataKept",
        "true",
        "--Mem/ImageCompressionFormat",
        ".png",
        "--Mem/DepthCompressionFormat",
        ".rvl",
        "--Mem/ImagePreDecimation",
        "1",
        "--Mem/ImagePostDecimation",
        "1",
        "--RGBD/CreateOccupancyGrid",
        "true",
        "--Grid/Sensor",
        "1",
        "--Grid/3D",
        "true",
        "--Grid/DepthDecimation",
        LaunchConfiguration("grid_depth_decimation").perform(context),
        "--Grid/CellSize",
        LaunchConfiguration("grid_cell_size").perform(context),
        "--Grid/RangeMin",
        LaunchConfiguration("grid_range_min").perform(context),
        "--Grid/RangeMax",
        LaunchConfiguration("grid_range_max").perform(context),
        "--Grid/PreVoxelFiltering",
        "false",
        "--Grid/NormalsSegmentation",
        "false",
        "--Grid/GroundIsObstacle",
        "true",
        "--Grid/MinClusterSize",
        "1",
        "--Grid/NoiseFilteringRadius",
        "0.0",
        "--Grid/FootprintLength",
        "0.0",
        "--Grid/FootprintWidth",
        "0.0",
        "--Grid/FootprintHeight",
        "0.0",
        "--Grid/MaxObstacleHeight",
        "0.0",
        "--Grid/MinGroundHeight",
        "0.0",
        "--Grid/MaxGroundHeight",
        LaunchConfiguration("grid_cell_size").perform(context),
        "--Grid/RayTracing",
        "false",
        "--Stereo/DenseStrategy",
        "1",
        "--StereoSGBM/Mode",
        "2",
    ]
    odom_args = [
        "--Odom/Strategy",
        "0",
        "--Reg/Strategy",
        "0",
        "--Reg/Force3DoF",
        "false",
        "--RGBD/ForceOdom3DoF",
        "false",
        "--Odom/AlignWithGround",
        "false",
        "--Rtabmap/ImagesAlreadyRectified",
        "true",
        "--Stereo/DenseStrategy",
        "1",
        "--StereoSGBM/Mode",
        "2",
    ]
    if delete_db_on_start:
        rtabmap_args.insert(0, "--delete_db_on_start")
    if extra_args:
        rtabmap_args.append(extra_args)
    if extra_odom_args:
        odom_args.append(extra_odom_args)

    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rtabmap_prefix, "launch", "rtabmap.launch.py")
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "namespace": LaunchConfiguration("namespace"),
            "stereo": "true",
            "visual_odometry": "true",
            "rtabmap_viz": LaunchConfiguration("rtabmap_viz"),
            "rviz": LaunchConfiguration("rviz"),
            "frame_id": LaunchConfiguration("frame_id"),
            "vo_frame_id": LaunchConfiguration("vo_frame_id"),
            "map_frame_id": LaunchConfiguration("map_frame_id"),
            "database_path": LaunchConfiguration("database_path"),
            "approx_sync": LaunchConfiguration("approx_sync"),
            "approx_sync_max_interval": LaunchConfiguration(
                "approx_sync_max_interval"
            ),
            "wait_imu_to_init": LaunchConfiguration("wait_imu_to_init"),
            "always_check_imu_tf": LaunchConfiguration("always_check_imu_tf"),
            "wait_for_transform": LaunchConfiguration("wait_for_transform"),
            "topic_queue_size": LaunchConfiguration("topic_queue_size"),
            "sync_queue_size": LaunchConfiguration("sync_queue_size"),
            "qos": LaunchConfiguration("qos"),
            "qos_image": LaunchConfiguration("qos_image"),
            "qos_camera_info": LaunchConfiguration("qos_camera_info"),
            "qos_imu": LaunchConfiguration("qos_imu"),
            "left_image_topic": left_image_topic,
            "right_image_topic": right_image_topic,
            "left_camera_info_topic": left_camera_info_topic,
            "right_camera_info_topic": right_camera_info_topic,
            "imu_topic": LaunchConfiguration("imu_topic"),
            "publish_tf_map": "true",
            "publish_tf_odom": "true",
            "args": " ".join(rtabmap_args),
            "odom_args": " ".join(odom_args),
        }.items(),
    )

    actions.append(
        GroupAction(
            [
                SetParameter(name="cloud_output_voxelized", value=False),
                SetParameter(name="cloud_subtract_filtering", value=False),
                SetParameter(name="map_filter_radius", value=0.0),
                SetParameter(name="map_always_update", value=True),
                SetParameter(name="Grid/DepthRoiRatios", value="0.0 0.0 0.0 0.0"),
                rtabmap_launch,
            ]
        )
    )

    if record_processed_bag:
        prefix = namespace.strip("/")
        record_topics = [
            "/clock",
            "/tf",
            "/tf_static",
            _topic(prefix, "odom"),
            _topic(prefix, "odom_info"),
            _topic(prefix, "info"),
            _topic(prefix, "mapGraph"),
            _topic(prefix, "mapData"),
            _topic(prefix, "mapPath"),
            _topic(prefix, "local_grid_obstacle"),
            _topic(prefix, "local_grid_ground"),
            _topic(prefix, "local_grid_empty"),
            _topic(prefix, "cloud_map"),
            _topic(prefix, "cloud_obstacles"),
            _topic(prefix, "cloud_ground"),
            _topic(prefix, "map"),
            _topic(prefix, "grid_prob_map"),
            left_camera_info_topic,
            right_camera_info_topic,
        ]
        actions.append(
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "bag",
                    "record",
                    "--use-sim-time",
                    "--max-cache-size",
                    "1073741824",
                    "--compression-mode",
                    "file",
                    "--compression-format",
                    "zstd",
                    "-o",
                    LaunchConfiguration("record_bag_path").perform(context),
                    *record_topics,
                ],
                output="screen",
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("namespace", default_value="rtabmap"),
            DeclareLaunchArgument("rtabmap_viz", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="false"),
            DeclareLaunchArgument("frame_id", default_value="oak"),
            DeclareLaunchArgument("vo_frame_id", default_value="oak_odom"),
            DeclareLaunchArgument("map_frame_id", default_value="map"),
            DeclareLaunchArgument("database_path", default_value=DEFAULT_DB_PATH),
            DeclareLaunchArgument("delete_db_on_start", default_value="true"),
            DeclareLaunchArgument("approx_sync", default_value="true"),
            DeclareLaunchArgument("approx_sync_max_interval", default_value="0.005"),
            DeclareLaunchArgument("wait_imu_to_init", default_value="true"),
            DeclareLaunchArgument("always_check_imu_tf", default_value="true"),
            DeclareLaunchArgument("wait_for_transform", default_value="0.2"),
            DeclareLaunchArgument("topic_queue_size", default_value="30"),
            DeclareLaunchArgument("sync_queue_size", default_value="30"),
            DeclareLaunchArgument("qos", default_value="0"),
            DeclareLaunchArgument("qos_image", default_value="0"),
            DeclareLaunchArgument("qos_camera_info", default_value="0"),
            DeclareLaunchArgument("qos_imu", default_value="0"),
            DeclareLaunchArgument(
                "left_image_topic", default_value="/oak/left/image_rect"
            ),
            DeclareLaunchArgument(
                "right_image_topic", default_value="/oak/right/image_rect"
            ),
            DeclareLaunchArgument(
                "left_camera_info_topic", default_value="/oak/left/camera_info"
            ),
            DeclareLaunchArgument(
                "right_camera_info_topic", default_value="/oak/right/camera_info"
            ),
            DeclareLaunchArgument(
                "fix_camera_info",
                default_value="true",
                description=(
                    "Republish camera_info with left Tx=0 and right Tx=-fx*baseline "
                    "for bags where OAK puts Tx on the left camera_info."
                ),
            ),
            DeclareLaunchArgument(
                "fixed_left_camera_info_topic",
                default_value="/oak/left/camera_info_rtabmap",
            ),
            DeclareLaunchArgument(
                "fixed_right_camera_info_topic",
                default_value="/oak/right/camera_info_rtabmap",
            ),
            DeclareLaunchArgument("stereo_baseline", default_value="0.074568"),
            DeclareLaunchArgument("python_executable", default_value="/usr/bin/python3"),
            DeclareLaunchArgument(
                "swap_stereo",
                default_value="false",
                description=(
                    "Diagnostic only. Swapping images fixes the sign check but "
                    "usually breaks positive-disparity stereo tracking."
                ),
            ),
            DeclareLaunchArgument("imu_topic", default_value="/oak/imu/data"),
            DeclareLaunchArgument(
                "optimizer_strategy",
                default_value="1",
                description="1=g2o, 2=GTSAM. Gravity constraints require g2o or GTSAM.",
            ),
            DeclareLaunchArgument(
                "gravity_sigma",
                default_value="0.2",
                description=(
                    "Gravity constraint sigma. Smaller is stronger; set 0 to disable."
                ),
            ),
            DeclareLaunchArgument(
                "use_odom_gravity",
                default_value="false",
                description=(
                    "false uses IMU orientation for gravity links; true trusts odometry "
                    "orientation as already gravity-aligned."
                ),
            ),
            DeclareLaunchArgument("grid_depth_decimation", default_value="1"),
            DeclareLaunchArgument("grid_cell_size", default_value="0.01"),
            DeclareLaunchArgument("grid_range_min", default_value="0.0"),
            DeclareLaunchArgument(
                "grid_range_max",
                default_value="0.0",
                description="0 disables max range clipping for local cloud/grid creation.",
            ),
            DeclareLaunchArgument("record_processed_bag", default_value="false"),
            DeclareLaunchArgument("record_bag_path", default_value=DEFAULT_RECORD_BAG),
            DeclareLaunchArgument(
                "rtabmap_args",
                default_value="",
                description="Extra RTAB-Map CLI parameters appended after defaults.",
            ),
            DeclareLaunchArgument(
                "rtabmap_odom_args",
                default_value="",
                description="Extra stereo_odometry CLI parameters appended after defaults.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
