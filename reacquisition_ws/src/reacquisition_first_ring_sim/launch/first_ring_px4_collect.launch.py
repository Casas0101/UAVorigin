from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sim_share = FindPackageShare("reacquisition_first_ring_sim")
    world_path = PathJoinSubstitution(
        [sim_share, "worlds", "first_ring_px4_target_world.sdf"]
    )
    gz_sim_launch = PathJoinSubstitution(
        [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
    )

    args = [
        DeclareLaunchArgument("start_gazebo", default_value="true"),
        DeclareLaunchArgument("gz_args", default_value=[world_path, " -r"]),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("image_topic", default_value="/camera"),
        DeclareLaunchArgument("camera_info_topic", default_value="/camera/camera_info"),
        DeclareLaunchArgument("uav_state_topic", default_value="/fmu/out/vehicle_odometry"),
        DeclareLaunchArgument("target_odom_topic", default_value="/target/odom_gt"),
        DeclareLaunchArgument("raw_frame_topic", default_value="/reacquisition/first_ring/raw_frame"),
        DeclareLaunchArgument("output_path", default_value="outputs/raw_frames/px4_rectangle_raw_frames.jsonl"),
        DeclareLaunchArgument("overwrite", default_value="true"),
        DeclareLaunchArgument("target_motion", default_value="static"),
        DeclareLaunchArgument("target_line_vx", default_value="0.2"),
        DeclareLaunchArgument("target_line_vy", default_value="0.0"),
    ]

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        condition=IfCondition(LaunchConfiguration("start_gazebo")),
        launch_arguments={"gz_args": LaunchConfiguration("gz_args")}.items(),
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="first_ring_px4_gz_bridge",
        output="screen",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock",
            [LaunchConfiguration("image_topic"), "@sensor_msgs/msg/Image@gz.msgs.Image"],
            [
                LaunchConfiguration("camera_info_topic"),
                "@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
            ],
        ],
    )

    target_state_publisher = Node(
        package="reacquisition_first_ring_sim",
        executable="first_ring_sim_state_node",
        name="first_ring_px4_target_state_node",
        output="screen",
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "uav_odom_topic": "/unused/uav_odom_for_px4_collect",
                "target_odom_topic": LaunchConfiguration("target_odom_topic"),
                "target_motion": LaunchConfiguration("target_motion"),
                "target_line_vx": LaunchConfiguration("target_line_vx"),
                "target_line_vy": LaunchConfiguration("target_line_vy"),
            }
        ],
    )

    raw_frame_node = Node(
        package="reacquisition_first_ring_ros",
        executable="first_ring_raw_frame_node",
        name="first_ring_px4_raw_frame_node",
        output="screen",
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "image_topic": LaunchConfiguration("image_topic"),
                "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                "uav_state_topic": LaunchConfiguration("uav_state_topic"),
                "uav_state_msg_type": "px4_msgs/VehicleOdometry",
                "target_state_topic": LaunchConfiguration("target_odom_topic"),
                "target_state_enabled": True,
                "output_topic": LaunchConfiguration("raw_frame_topic"),
                "raw_frame_convention": "NED_FRD",
                "uav_frame_id": "px4_ned",
                "uav_source": "px4_vehicle_odometry",
                "target_source": "gazebo_ground_truth",
                "primary_time_source": "image.header.stamp",
                "producer": "ros2_px4_gazebo_raw_frame_publisher",
                "environment": "wsl2_px4_gazebo_rectangle_mission",
            }
        ],
    )

    recorder = Node(
        package="reacquisition_first_ring_sim",
        executable="raw_frame_jsonl_recorder",
        name="px4_raw_frame_jsonl_recorder",
        output="screen",
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "input_topic": LaunchConfiguration("raw_frame_topic"),
                "output_path": LaunchConfiguration("output_path"),
                "overwrite": LaunchConfiguration("overwrite"),
                "validate": True,
                "flush_every": 1,
            }
        ],
    )

    return LaunchDescription(
        args + [gazebo, bridge, target_state_publisher, raw_frame_node, recorder]
    )
