# reacquisition_first_ring_ros

ROS 2 adapter package for first-ring runtime integration.

## Raw Frame Publisher

`first_ring_raw_frame_node` publishes schema-0.1 JSON on:

```text
/reacquisition/first_ring/raw_frame  std_msgs/msg/String
```

Default subscriptions:

```text
/camera/image_raw    sensor_msgs/msg/Image
/camera/camera_info  sensor_msgs/msg/CameraInfo
/uav/odom            nav_msgs/msg/Odometry
/target/odom_gt      nav_msgs/msg/Odometry
```

The node only packages raw inputs. It does not run detection, projection,
filtering, prediction, or control.

## WSL2 Commands

From the workspace root:

```bash
colcon build --symlink-install --packages-select reacquisition_first_ring_raw reacquisition_first_ring_ros
source install/setup.bash
ros2 launch reacquisition_first_ring_ros raw_frame_publisher.launch.py
```

For direct PX4 `VehicleOdometry` input, make sure `px4_msgs` is available and
launch with:

```bash
ros2 launch reacquisition_first_ring_ros raw_frame_publisher.launch.py \
  uav_state_topic:=/fmu/out/vehicle_odometry \
  uav_state_msg_type:=px4_msgs/VehicleOdometry \
  raw_frame_convention:=NED_FRD \
  uav_source:=px4_vehicle_odometry
```

If target ground truth is unavailable, disable it:

```bash
ros2 launch reacquisition_first_ring_ros raw_frame_publisher.launch.py \
  target_state_enabled:=false
```
