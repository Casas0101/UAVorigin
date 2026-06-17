from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sim_share = FindPackageShare("reacquisition_first_ring_sim")
    world_path = PathJoinSubstitution(
        [sim_share, "worlds", "first_ring_flat_world.sdf"]
    )
    gz_sim_launch = PathJoinSubstitution(
        [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
    )

    args = [
        DeclareLaunchArgument("world", default_value=world_path),
        DeclareLaunchArgument("gz_args", default_value=[world_path, " -r"]),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("image_topic", default_value="/camera/image_raw"),
        DeclareLaunchArgument("camera_info_topic", default_value="/camera/image_raw/camera_info"),
        DeclareLaunchArgument("uav_odom_topic", default_value="/uav/odom"),
        DeclareLaunchArgument("target_odom_topic", default_value="/target/odom_gt"),
        DeclareLaunchArgument("raw_frame_topic", default_value="/reacquisition/first_ring/raw_frame"),
        DeclareLaunchArgument("output_path", default_value="outputs/raw_frames/gazebo_raw_frames.jsonl"),
        DeclareLaunchArgument("overwrite", default_value="true"),
        DeclareLaunchArgument("target_motion", default_value="static"),
    ]

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        launch_arguments={"gz_args": LaunchConfiguration("gz_args")}.items(),
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="first_ring_gz_bridge",
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

    state_publisher = Node(
        package="reacquisition_first_ring_sim",
        executable="first_ring_sim_state_node",
        name="first_ring_sim_state_node",
        output="screen",
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "uav_odom_topic": LaunchConfiguration("uav_odom_topic"),
                "target_odom_topic": LaunchConfiguration("target_odom_topic"),
                "target_motion": LaunchConfiguration("target_motion"),
            }
        ],
    )

    raw_frame_node = Node(
        package="reacquisition_first_ring_ros",
        executable="first_ring_raw_frame_node",
        name="first_ring_raw_frame_node",
        output="screen",
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "image_topic": LaunchConfiguration("image_topic"),
                "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                "uav_state_topic": LaunchConfiguration("uav_odom_topic"),
                "uav_state_msg_type": "nav_msgs/Odometry",
                "target_state_topic": LaunchConfiguration("target_odom_topic"),
                "target_state_enabled": True,
                "output_topic": LaunchConfiguration("raw_frame_topic"),
                "raw_frame_convention": "ENU_FLU",
                "uav_source": "gazebo_model_state",
                "target_source": "gazebo_ground_truth",
                "primary_time_source": "image.header.stamp",
                "producer": "ros2_gazebo_raw_frame_publisher",
                "environment": "wsl2_gazebo_first_ring_flat_world",
            }
        ],
    )

    recorder = Node(
        package="reacquisition_first_ring_sim",
        executable="raw_frame_jsonl_recorder",
        name="raw_frame_jsonl_recorder",
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
        args + [gazebo, bridge, state_publisher, raw_frame_node, recorder]
    )
