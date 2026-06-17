# reacquisition_first_ring_sim

Minimal Gazebo scene and ROS 2 launch files for collecting first-ring raw frames.

The scene contains:

- a flat ground plane;
- a static UAV body at 10 m height with a downward camera;
- a red ground target visible from the downward camera;
- a ROS 2 helper node publishing `/uav/odom` and `/target/odom_gt`;
- the first-ring raw frame publisher;
- a JSONL recorder for `/reacquisition/first_ring/raw_frame`.

Run from the workspace root in WSL2:

```bash
colcon build --symlink-install --packages-select \
  reacquisition_first_ring_raw \
  reacquisition_first_ring_ros \
  reacquisition_first_ring_sim
source install/setup.bash
ros2 launch reacquisition_first_ring_sim first_ring_gz_collect.launch.py
```

The default output file is:

```text
outputs/raw_frames/gazebo_raw_frames.jsonl
```

If no camera messages arrive, inspect Gazebo topics:

```bash
gz topic -l | grep camera
```

Then relaunch with the detected camera info topic, for example:

```bash
ros2 launch reacquisition_first_ring_sim first_ring_gz_collect.launch.py \
  camera_info_topic:=/camera/image_raw/camera_info
```
