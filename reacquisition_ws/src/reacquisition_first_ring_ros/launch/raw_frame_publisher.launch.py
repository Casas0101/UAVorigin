from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument("image_topic", default_value="/camera/image_raw"),
        DeclareLaunchArgument("camera_info_topic", default_value="/camera/camera_info"),
        DeclareLaunchArgument("uav_state_topic", default_value="/uav/odom"),
        DeclareLaunchArgument("uav_state_msg_type", default_value="nav_msgs/Odometry"),
        DeclareLaunchArgument("target_state_topic", default_value="/target/odom_gt"),
        DeclareLaunchArgument("target_state_enabled", default_value="true"),
        DeclareLaunchArgument("output_topic", default_value="/reacquisition/first_ring/raw_frame"),
        DeclareLaunchArgument("raw_frame_convention", default_value="ENU_FLU"),
        DeclareLaunchArgument("uav_source", default_value="gazebo_model_state"),
        DeclareLaunchArgument("target_source", default_value="gazebo_ground_truth"),
        DeclareLaunchArgument("primary_time_source", default_value="image.header.stamp"),
        DeclareLaunchArgument("config_revision", default_value="0"),
    ]

    node = Node(
        package="reacquisition_first_ring_ros",
        executable="first_ring_raw_frame_node",
        name="first_ring_raw_frame_node",
        output="screen",
        parameters=[
            {
                "image_topic": LaunchConfiguration("image_topic"),
                "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                "uav_state_topic": LaunchConfiguration("uav_state_topic"),
                "uav_state_msg_type": LaunchConfiguration("uav_state_msg_type"),
                "target_state_topic": LaunchConfiguration("target_state_topic"),
                "target_state_enabled": LaunchConfiguration("target_state_enabled"),
                "output_topic": LaunchConfiguration("output_topic"),
                "raw_frame_convention": LaunchConfiguration("raw_frame_convention"),
                "uav_source": LaunchConfiguration("uav_source"),
                "target_source": LaunchConfiguration("target_source"),
                "primary_time_source": LaunchConfiguration("primary_time_source"),
                "config_revision": LaunchConfiguration("config_revision"),
            }
        ],
    )

    return LaunchDescription(args + [node])
