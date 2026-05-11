import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


TOOLS_DIR = os.path.dirname(os.path.realpath(__file__))
RTABMAP_ROOT = os.path.dirname(TOOLS_DIR)
DEFAULT_RTABMAP_CFG = os.path.join(
    RTABMAP_ROOT, "docs", "rtabmap_oak_rgb_pointcloud_map.ini"
)
DEFAULT_DB_PATH = "/home/wjxu22/Datasets/outputs/rtab/oak_rgb_pointcloud_map.db"
DEFAULT_RVIZ_CFG = os.path.join(
    RTABMAP_ROOT, "docs", "oak_rtabmap_rgb_pointcloud_map.rviz"
)
DEFAULT_RGB_MAP_SNAPSHOT_PATH = (
    "/home/wjxu22/Datasets/outputs/rtab/oak_rgb_pointcloud_map_live_rgb_map.ply"
)


def _is_true(value):
    return value.lower() in ("1", "true", "yes", "on")


def _join_ns(namespace, name):
    namespace = namespace.strip("/")
    if namespace:
        return f"/{namespace}/{name}"
    return f"/{name}"


def launch_setup(context, *args, **kwargs):
    name = LaunchConfiguration("camera_name").perform(context)
    namespace = LaunchConfiguration("camera_namespace").perform(context)
    camera_prefix = _join_ns(namespace, name)
    rtabmap_args = LaunchConfiguration("rtabmap_args").perform(context).strip()
    delete_db_on_start = _is_true(
        LaunchConfiguration("delete_db_on_start").perform(context)
    )

    merged_args = []
    if delete_db_on_start:
        merged_args.append("--delete_db_on_start")
    if rtabmap_args:
        merged_args.append(rtabmap_args)
    merged_args = " ".join(merged_args)

    rtabmap_prefix = get_package_share_directory("rtabmap_launch")

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(rtabmap_prefix, "launch", "rtabmap.launch.py")
            ),
            launch_arguments={
                "namespace": LaunchConfiguration("rtabmap_namespace"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "stereo": "false",
                "visual_odometry": "false",
                "rtabmap_viz": LaunchConfiguration("rtabmap_viz"),
                "rviz": LaunchConfiguration("use_rviz"),
                "rviz_cfg": LaunchConfiguration("rviz_cfg"),
                "compressed": LaunchConfiguration("rtabmap_compressed"),
                "rgb_image_transport": LaunchConfiguration(
                    "rtabmap_rgb_image_transport"
                ),
                "depth_image_transport": LaunchConfiguration(
                    "rtabmap_depth_image_transport"
                ),
                "approx_sync": LaunchConfiguration("approx_sync"),
                "approx_sync_max_interval": LaunchConfiguration(
                    "approx_sync_max_interval"
                ),
                "frame_id": LaunchConfiguration("frame_id"),
                "map_frame_id": LaunchConfiguration("map_frame_id"),
                "odom_topic": f"{camera_prefix}/vio/odometry",
                "rgb_topic": f"{camera_prefix}/rgb/image_raw",
                "depth_topic": f"{camera_prefix}/stereo/image_raw",
                "camera_info_topic": f"{camera_prefix}/rgb/camera_info",
                "imu_topic": f"{camera_prefix}/imu/data",
                "database_path": LaunchConfiguration("database_path"),
                "cfg": LaunchConfiguration("cfg"),
                "qos": LaunchConfiguration("qos"),
                "wait_for_transform": LaunchConfiguration("wait_for_transform"),
                "publish_tf_map": "true",
                "publish_tf_odom": "false",
                "args": merged_args,
            }.items(),
        ),
        Node(
            package="rtabmap_util",
            executable="point_cloud_xyzrgb",
            name=f"{name}_rgb_cloud",
            output="screen",
            parameters=[
                {
                    "approx_sync": LaunchConfiguration("approx_sync"),
                    "approx_sync_max_interval": LaunchConfiguration(
                        "approx_sync_max_interval"
                    ),
                    "decimation": LaunchConfiguration("cloud_decimation"),
                    "voxel_size": LaunchConfiguration("cloud_voxel_size"),
                    "min_depth": LaunchConfiguration("cloud_min_depth"),
                    "max_depth": LaunchConfiguration("cloud_max_depth"),
                    "qos": LaunchConfiguration("qos"),
                    "qos_camera_info": LaunchConfiguration("qos"),
                }
            ],
            remappings=[
                ("rgb/image", f"{camera_prefix}/rgb/image_raw"),
                ("depth/image", f"{camera_prefix}/stereo/image_raw"),
                ("rgb/camera_info", f"{camera_prefix}/rgb/camera_info"),
                ("cloud", f"{camera_prefix}/rgb_cloud"),
            ],
        ),
        Node(
            package="rtabmap_util",
            executable="point_cloud_assembler",
            name=f"{name}_rgb_map_assembler",
            output="screen",
            parameters=[
                {
                    "fixed_frame_id": LaunchConfiguration("odom_frame_id"),
                    "frame_id": LaunchConfiguration("odom_frame_id"),
                    "max_clouds": LaunchConfiguration("assembler_max_clouds"),
                    "circular_buffer": True,
                    "linear_update": LaunchConfiguration("assembler_linear_update"),
                    "angular_update": LaunchConfiguration("assembler_angular_update"),
                    "voxel_size": LaunchConfiguration("assembler_voxel_size"),
                    "noise_radius": LaunchConfiguration("assembler_noise_radius"),
                    "noise_min_neighbors": LaunchConfiguration(
                        "assembler_noise_min_neighbors"
                    ),
                    "wait_for_transform": LaunchConfiguration("wait_for_transform"),
                    "qos": LaunchConfiguration("qos"),
                }
            ],
            remappings=[
                ("cloud", f"{camera_prefix}/rgb_cloud"),
                ("assembled_cloud", f"{camera_prefix}/rgb_map"),
            ],
        ),
        ExecuteProcess(
            cmd=[
                "/usr/bin/python3",
                os.path.join(TOOLS_DIR, "save_pointcloud2_snapshot.py"),
                "--topic",
                f"{camera_prefix}/rgb_map",
                "--output",
                LaunchConfiguration("rgb_map_snapshot_output"),
            ],
            output="screen",
            condition=IfCondition(LaunchConfiguration("save_rgb_map_on_exit")),
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("camera_name", default_value="oak"),
            DeclareLaunchArgument("camera_namespace", default_value=""),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("rtabmap_namespace", default_value="rtabmap"),
            DeclareLaunchArgument("rtabmap_viz", default_value="false"),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Launch the RTAB-Map RViz window.",
            ),
            DeclareLaunchArgument(
                "cfg",
                default_value=DEFAULT_RTABMAP_CFG,
                description="RTAB-Map ini for the OAK RGB pointcloud mode.",
            ),
            DeclareLaunchArgument(
                "database_path",
                default_value=DEFAULT_DB_PATH,
                description="RTAB-Map database path.",
            ),
            DeclareLaunchArgument(
                "delete_db_on_start",
                default_value="false",
                description="Delete the previous RTAB-Map database before starting.",
            ),
            DeclareLaunchArgument(
                "rviz_cfg",
                default_value=DEFAULT_RVIZ_CFG,
                description="RViz config path.",
            ),
            DeclareLaunchArgument(
                "save_rgb_map_on_exit",
                default_value="true",
                description="Save the latest live /<camera>/rgb_map cloud to PLY when the launch stops.",
            ),
            DeclareLaunchArgument(
                "rgb_map_snapshot_output",
                default_value=DEFAULT_RGB_MAP_SNAPSHOT_PATH,
                description="PLY path used by the live /<camera>/rgb_map snapshot saver.",
            ),
            DeclareLaunchArgument(
                "rtabmap_compressed",
                default_value="false",
                description="If true, RTAB-Map subscribes through compressed image_transport relays.",
            ),
            DeclareLaunchArgument(
                "rtabmap_rgb_image_transport",
                default_value="compressed",
                description="RGB image transport used when rtabmap_compressed=true.",
            ),
            DeclareLaunchArgument(
                "rtabmap_depth_image_transport",
                default_value="compressedDepth",
                description="Depth image transport used when rtabmap_compressed=true.",
            ),
            DeclareLaunchArgument(
                "frame_id",
                default_value="oak_parent_frame",
                description="Base frame used by RTAB-Map with external OAK VIO.",
            ),
            DeclareLaunchArgument(
                "odom_frame_id",
                default_value="oak_odom",
                description="OAK VIO odom frame.",
            ),
            DeclareLaunchArgument("map_frame_id", default_value="map"),
            DeclareLaunchArgument("approx_sync", default_value="true"),
            DeclareLaunchArgument("approx_sync_max_interval", default_value="0.02"),
            DeclareLaunchArgument("qos", default_value="0"),
            DeclareLaunchArgument("wait_for_transform", default_value="0.2"),
            DeclareLaunchArgument(
                "rtabmap_args",
                default_value="",
                description="Extra RTAB-Map args.",
            ),
            DeclareLaunchArgument("cloud_decimation", default_value="2"),
            DeclareLaunchArgument("cloud_voxel_size", default_value="0.03"),
            DeclareLaunchArgument("cloud_min_depth", default_value="0.2"),
            DeclareLaunchArgument("cloud_max_depth", default_value="5.0"),
            DeclareLaunchArgument("assembler_max_clouds", default_value="400"),
            DeclareLaunchArgument("assembler_linear_update", default_value="0.10"),
            DeclareLaunchArgument("assembler_angular_update", default_value="0.05"),
            DeclareLaunchArgument("assembler_voxel_size", default_value="0.03"),
            DeclareLaunchArgument("assembler_noise_radius", default_value="0.0"),
            DeclareLaunchArgument(
                "assembler_noise_min_neighbors", default_value="5"
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
