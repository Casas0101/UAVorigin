# Gazebo 场景中运行第一环并收集 raw frame

本文档对应包：

- `reacquisition_first_ring_raw`
- `reacquisition_first_ring_ros`
- `reacquisition_first_ring_sim`

## 1. WSL2 依赖

在 WSL2 的 ROS 2 环境中需要具备：

- ROS 2
- Gazebo Sim / `gz sim`
- `ros_gz_sim`
- `ros_gz_bridge`

Ubuntu/ROS 2 环境中通常需要：

```bash
sudo apt update
sudo apt install \
  ros-$ROS_DISTRO-ros-gz-sim \
  ros-$ROS_DISTRO-ros-gz-bridge
```

如果 `$ROS_DISTRO` 未设置，先 source ROS 2：

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
```

## 2. 编译

从工作区根目录执行：

```bash
cd ~/reacquisition_ws
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install --packages-select \
  reacquisition_first_ring_raw \
  reacquisition_first_ring_ros \
  reacquisition_first_ring_sim
source install/setup.bash
```

## 3. 启动 Gazebo 场景并收集数据

一条命令启动：

```bash
ros2 launch reacquisition_first_ring_sim first_ring_gz_collect.launch.py
```

该 launch 会启动：

1. Gazebo 世界 `first_ring_flat_world.sdf`;
2. `ros_gz_bridge`，桥接相机图像、相机内参和 `/clock`;
3. `first_ring_sim_state_node`，发布 `/uav/odom` 和 `/target/odom_gt`;
4. `first_ring_raw_frame_node`，发布 `/reacquisition/first_ring/raw_frame`;
5. `raw_frame_jsonl_recorder`，写入 JSONL。

默认输出：

```text
outputs/raw_frames/gazebo_raw_frames.jsonl
```

## 4. 验证数据流

另开终端：

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash
ros2 topic list
ros2 topic hz /camera/image_raw
ros2 topic hz /reacquisition/first_ring/raw_frame
ros2 topic echo /reacquisition/first_ring/raw_frame --once
```

检查输出文件：

```bash
tail -n 3 outputs/raw_frames/gazebo_raw_frames.jsonl
python -m pytest -q
```

## 5. 常用参数

改输出文件：

```bash
ros2 launch reacquisition_first_ring_sim first_ring_gz_collect.launch.py \
  output_path:=outputs/raw_frames/run_001.jsonl
```

不覆盖旧文件：

```bash
ros2 launch reacquisition_first_ring_sim first_ring_gz_collect.launch.py \
  overwrite:=false \
  output_path:=outputs/raw_frames/run_001.jsonl
```

目标状态改为圆周运动真值：

```bash
ros2 launch reacquisition_first_ring_sim first_ring_gz_collect.launch.py \
  target_motion:=circle
```

注意：当前 Gazebo 视觉目标是静态红色圆柱；`target_motion:=circle` 只改变发布给第一环的目标真值，用于测试接口时序，不代表 Gazebo 目标视觉模型也运动。

## 6. CameraInfo topic 排查

如果 `/camera/image_raw` 有数据，但第一环提示 `camera_info is unavailable or stale`：

```bash
gz topic -l | grep camera
ros2 topic list | grep camera
```

如果 Gazebo 发布的 CameraInfo topic 与默认值不同，重启 launch 并指定：

```bash
ros2 launch reacquisition_first_ring_sim first_ring_gz_collect.launch.py \
  camera_info_topic:=/camera/camera_info
```

## 7. 无 GUI / 服务器模式

如果 WSL2 没有 WSLg 或图形界面，可尝试服务器模式：

```bash
ros2 launch reacquisition_first_ring_sim first_ring_gz_collect.launch.py \
  gz_args:="-r -s $(ros2 pkg prefix reacquisition_first_ring_sim)/share/reacquisition_first_ring_sim/worlds/first_ring_flat_world.sdf"
```

不同 Gazebo 版本的 `gz_args` 解析略有差异。如果服务器模式无法产生相机图像，先用默认 GUI 模式确认场景和相机 topic。
