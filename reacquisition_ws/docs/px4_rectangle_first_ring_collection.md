# PX4 SITL + Gazebo 矩形任务飞行下的第一环数据采集

目标场景：

- PX4 SITL + Gazebo；
- UAV 起飞到 10 m；
- PX4 执行预设 4 航点矩形 mission；
- UAV 带下视相机；
- 地面红色目标默认静止；
- 第一环只订阅数据并记录 JSONL，不发布控制信号。

## 1. 当前文件

新增 PX4 采集入口：

```text
src/reacquisition_first_ring_sim/launch/first_ring_px4_collect.launch.py
```

PX4 目标世界：

```text
src/reacquisition_first_ring_sim/worlds/first_ring_px4_target_world.sdf
```

QGroundControl mission 模板：

```text
src/reacquisition_first_ring_sim/missions/px4_rectangle_10m_qgc.plan
```

默认输出：

```text
outputs/raw_frames/px4_rectangle_raw_frames.jsonl
```

## 2. 依赖准备

工作区需要能编译：

- `reacquisition_first_ring_raw`
- `reacquisition_first_ring_ros`
- `reacquisition_first_ring_sim`
- `px4_msgs`

PX4 ROS 2 通信需要 Micro XRCE-DDS Agent。PX4 SITL 会自动启动 uXRCE-DDS client，默认连接本机 UDP 8888。

## 3. 编译 ROS 2 工作区

```bash
cd ~/reacquisition_ws
source /opt/ros/$ROS_DISTRO/setup.bash

colcon build --symlink-install --packages-select \
  px4_msgs \
  reacquisition_first_ring_raw \
  reacquisition_first_ring_ros \
  reacquisition_first_ring_sim

source install/setup.bash
```

如果你的工作区没有 `px4_msgs`：

```bash
cd ~/reacquisition_ws/src
git clone https://github.com/PX4/px4_msgs.git
```

`px4_msgs` 分支应与 PX4 版本对应。PX4 v1.16 以后存在消息版本兼容问题，不匹配时需要按 PX4 官方说明使用 message translation node。

## 4. 启动顺序

### 终端 1：启动 Micro XRCE-DDS Agent

```bash
MicroXRCEAgent udp4 -p 8888
```

### 终端 2：启动 Gazebo 目标世界、第一环和记录器

```bash
cd ~/reacquisition_ws
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash

ros2 launch reacquisition_first_ring_sim first_ring_px4_collect.launch.py
```

默认使用 `gz sim -s -r` server/headless 模式启动 Gazebo world，避免 WSLg/OpenGL GUI 渲染崩溃。这个模式仍然会启动 Gazebo server、传感器和 transport，PX4 可以连接世界。

这个 launch 会启动：

- Gazebo 世界 `first_ring_px4_target_world.sdf`;
- `ros_gz_bridge`，桥接相机图像、CameraInfo、`/clock`;
- `/target/odom_gt` 真值发布节点；
- 第一环 `first_ring_raw_frame_node`;
- JSONL 记录器。

### 终端 3：启动 PX4 SITL

推荐先使用 PX4 standalone 模式，让 PX4 连接已经启动的 Gazebo：

```bash
cd ~/PX4-Autopilot
PX4_GZ_STANDALONE=1 PX4_GZ_WORLD=first_ring_px4_target_world make px4_sitl gz_x500_mono_cam_down
```

如果你的 PX4 版本没有 `gz_x500_mono_cam_down`，先列出可用目标：

```bash
make list_config_targets | grep gz_x500
```

可退回到带相机的可用模型，例如 `gz_x500_mono_cam`，但需要确认相机朝向和 topic。

如果 PX4 持续输出 `Waiting for Gazebo world...`，先确认 Gazebo world 是否存在：

```bash
gz topic -l | grep first_ring_px4_target_world
```

若没有输出，说明 Gazebo server 没有成功启动。若看到 `Ogre::RenderingAPIException`、`Out of GPU memory or driver refused`、`Ogre::UnimplementedException`，这是 WSLg/OpenGL 渲染层崩溃，不是第一环节点错误。保持默认 headless launch，或显式运行：

```bash
ros2 launch reacquisition_first_ring_sim first_ring_px4_collect.launch.py \
  gz_args:="$(ros2 pkg prefix reacquisition_first_ring_sim)/share/reacquisition_first_ring_sim/worlds/first_ring_px4_target_world.sdf -r -s"
```

### 终端 4：打开 QGroundControl，导入 mission

导入：

```text
~/reacquisition_ws/install/reacquisition_first_ring_sim/share/reacquisition_first_ring_sim/missions/px4_rectangle_10m_qgc.plan
```

Mission 内容：

- takeoff 到 10 m；
- 4 个矩形航点；
- 回到起点附近；
- 近似围绕 PX4 默认 home `47.397742, 8.545594`。

导入后上传到 vehicle，然后在 QGroundControl 中 Start Mission。

## 5. 关键 topic

PX4 odometry：

```bash
ros2 topic hz /fmu/out/vehicle_odometry
```

Gazebo 相机：

```bash
ros2 topic list | grep camera
ros2 topic hz /camera
ros2 topic hz /camera/camera_info
```

第一环输出：

```bash
ros2 topic hz /reacquisition/first_ring/raw_frame
ros2 topic echo /reacquisition/first_ring/raw_frame --once
```

落盘：

```bash
tail -n 3 outputs/raw_frames/px4_rectangle_raw_frames.jsonl
```

## 6. 常见 topic 修正

PX4/Gazebo 相机 topic 可能随模型版本不同。如果 `/camera` 没有数据：

```bash
gz topic -l | grep camera
ros2 topic list | grep camera
```

按实际 topic 重启：

```bash
ros2 launch reacquisition_first_ring_sim first_ring_px4_collect.launch.py \
  image_topic:=/实际图像topic \
  camera_info_topic:=/实际camera_info_topic
```

## 7. 静止目标和慢速直线目标

默认：

```bash
target_motion:=static
```

慢速直线真值：

```bash
ros2 launch reacquisition_first_ring_sim first_ring_px4_collect.launch.py \
  target_motion:=line \
  target_line_vx:=0.2 \
  target_line_vy:=0.0
```

当前限制：`target_motion:=line` 只改变 `/target/odom_gt` 真值，不会移动 Gazebo 里的红色目标视觉模型。若要让相机画面中的红色目标同步移动，需要后续增加 Gazebo model pose 控制。

## 8. 当前边界

第一环程序只做：

- 订阅图像；
- 订阅 CameraInfo；
- 订阅 PX4 `/fmu/out/vehicle_odometry`;
- 订阅 `/target/odom_gt`;
- 封装 `/reacquisition/first_ring/raw_frame`;
- 记录 JSONL。

第一环程序不做：

- arm；
- takeoff；
- mission upload；
- waypoint 控制；
- Offboard setpoint；
- 轨迹控制；
- 视觉检测；
- 反投影；
- 滤波预测；
- MPC。
