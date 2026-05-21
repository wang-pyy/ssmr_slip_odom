# ssmr_slip_odom — RPLIDAR A1 + slam_toolbox 集成说明

本文档说明如何在树莓派 5 + ROS 2 Humble (Docker) 环境下，把已有的滑移感知里程计 `slip_odom_node.py` 与 RPLIDAR A1、`slam_toolbox` 集成，完成长走廊 2D SLAM 建图，并提供 baseline / proposed 两种对照实验。

---

## 1. 文件树

```
ssmr_slip_odom/
├── Dockerfile
├── package.xml                              # 修改
├── setup.py                                 # 修改
├── setup.cfg
├── resource/
│   └── ssmr_slip_odom                       # 新增 (ament 资源索引标记)
├── config/
│   ├── params.yaml                          # 既有
│   ├── slam_toolbox_params.yaml             # 新增
│   └── slip_odom_params.yaml                # 新增
├── launch/
│   ├── slip_odom_launch.py                  # 既有
│   ├── slam_bringup.launch.py               # 新增 (proposed)
│   └── slam_bringup_baseline.launch.py      # 新增 (baseline)
├── rviz/
│   └── slam.rviz                            # 新增
├── scripts/
│   └── record_eval.sh
└── ssmr_slip_odom/
    ├── slip_odom_node.py                    # 既有，未改
    ├── simple_encoder_odom_node.py          # 新增 (baseline 节点)
    ├── bno055_node.py
    ├── yahboom_base_node.py
    ├── wasd_teleop_node.py
    └── experiment_runner.py
```

TF 树（两种模式都必须满足）：

```
map -> odom            (slam_toolbox 发布)
odom -> base_link      (proposed: slip_odom_node / baseline: simple_encoder_odom_node)
base_link -> laser_frame  (static_transform_publisher)
```

---

## 2. 新增 / 修改的文件清单

| 路径 | 说明 |
|---|---|
| `launch/slam_bringup.launch.py` | proposed 启动文件：RPLIDAR + 静态 TF + slip_odom_node + slam_toolbox + 可选 rviz2 |
| `launch/slam_bringup_baseline.launch.py` | baseline 启动文件：用 `simple_encoder_odom_node` 替代 slip_odom_node |
| `config/slam_toolbox_params.yaml` | 适配长走廊的 slam_toolbox 参数 |
| `config/slip_odom_params.yaml` | slip_odom_node 的保守初值 |
| `rviz/slam.rviz` | TF / LaserScan / Map / Odometry / RobotModel 基础视图 |
| `ssmr_slip_odom/simple_encoder_odom_node.py` | baseline 编码器里程计节点 |
| `package.xml` | 补齐依赖（slam_toolbox、rplidar_ros、rviz2、launch、launch_ros） |
| `setup.py` | 补 `rviz/*.rviz` 到 data_files；注册 `simple_encoder_odom_node` 入口 |
| `resource/ssmr_slip_odom` | ament 资源索引空文件 |

---

## 3. 编译命令（在 Pi 5 Docker 容器内）

```bash
# 1) 进入容器（按你的容器名替换，常见名字如 ros2_humble）
docker exec -it ros2_humble bash

# 2) source ROS 2 环境
source /opt/ros/humble/setup.bash

# 3) 安装系统依赖（仅首次）
apt-get update && apt-get install -y \
  ros-humble-slam-toolbox \
  ros-humble-rplidar-ros \
  ros-humble-tf2-ros \
  ros-humble-rviz2

# 4) 编译工作区
cd /ros2_ws
colcon build --symlink-install --packages-select ssmr_slip_odom

# 5) source install
source /ros2_ws/install/setup.bash
```

---

## 4. 启动命令

### 4.1 检查雷达串口 & 授权

```bash
ls -l /dev/ttyUSB*
# 若设备存在但没权限：
sudo chmod 666 /dev/ttyUSB0
# 容器外要把 /dev/ttyUSB0 透传进容器：
#   docker run ... --device=/dev/ttyUSB0
```

### 4.2 Proposed（slip-aware odom + slam_toolbox）

```bash
ros2 launch ssmr_slip_odom slam_bringup.launch.py \
  serial_port:=/dev/ttyUSB0 \
  serial_baudrate:=256000 \
  use_slip_odom:=true \
  use_rviz:=false
```

`slip_odom_node` 由本 launch 起。注意 `yahboom_base_node` 和 `bno055_node` 仍要单独起（它们提供 `/wheel_speeds` 和 `/imu/data`）：

```bash
# 另一个终端
ros2 launch ssmr_slip_odom slip_odom_launch.py \
  run_base:=true run_imu:=true \
  params_file:=/ros2_ws/install/ssmr_slip_odom/share/ssmr_slip_odom/config/params.yaml
# 注意：此处不要再起 slip_odom_node，避免重复发布 /odom
```

如果想用一条命令同时起底盘 + IMU + slip_odom + SLAM，最简单的方法是把 `slip_odom_launch.py` 里 `slip_odom_node` 那一段删掉，保留 `run_base/run_imu`，然后 `slam_bringup.launch.py` 里用 `use_slip_odom:=true`。

### 4.3 Baseline（编码器里程计 + slam_toolbox）

```bash
ros2 launch ssmr_slip_odom slam_bringup_baseline.launch.py \
  serial_port:=/dev/ttyUSB0 \
  serial_baudrate:=256000 \
  use_rviz:=false
```

Baseline 模式下：

- `simple_encoder_odom_node` 发布 `/odom` 与 `odom -> base_link`
- **不要**再启动 `slip_odom_node`，否则 TF 冲突

---

## 5. 调试命令

```bash
# Topic 列表
ros2 topic list

# 雷达
ros2 topic hz /scan
ros2 topic echo /scan --once

# 里程计
ros2 topic hz /odom
ros2 topic echo /odom --once

# slip 子话题
ros2 topic hz /slip/lambda
ros2 topic echo /slip/b_eff --once

# IMU & 轮速
ros2 topic hz /imu/data
ros2 topic hz /wheel_speeds

# TF 树
ros2 run tf2_tools view_frames
# 生成 frames.pdf，期望: map -> odom -> base_link -> laser_frame
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link laser_frame

# slam_toolbox 是否在发 map
ros2 topic hz /map
ros2 service list | grep slam_toolbox
```

---

## 6. rosbag 录制命令

```bash
mkdir -p /ros2_ws/bags

ros2 bag record \
  -o /ros2_ws/bags/test_run_30_slam_corridor_proposed \
  /cmd_vel \
  /imu/data \
  /wheel_speeds \
  /odom \
  /slip/w_imu \
  /slip/w_enc \
  /slip/w_fused \
  /slip/lambda \
  /slip/s_raw \
  /slip/s_bar \
  /slip/b_eff \
  /slip/v_eff \
  /scan \
  /tf \
  /tf_static \
  /map \
  /pose
```

baseline 跑同一段路时把命名改成 `test_run_30_slam_corridor_baseline`。

---

## 7. 常见错误排查

| 现象 | 排查 |
|---|---|
| **`/scan` 没有数据** | `ros2 topic hz /scan` 无输出 → 看 `rplidar_node` log；确认 `serial_port`、`serial_baudrate`；电源是否够（A1 高速版要 5V/1A） |
| **雷达串口打不开** | `ls -l /dev/ttyUSB0` 检查存在性 + 权限；容器要 `--device=/dev/ttyUSB0` 透传；冲突进程：`fuser /dev/ttyUSB0` |
| **A1 256000 不工作** | 改为 `serial_baudrate:=115200`。老固件或某些克隆模块只支持 115200 |
| **`Failed to compute odom pose`** | slam_toolbox 没拿到 `odom -> base_link`。检查 `ros2 run tf2_ros tf2_echo odom base_link`；多半是 slip_odom_node 没起，或 `publish_tf=false`；也可能是 `/wheel_speeds` / `/imu/data` 没有，slip_odom 一直没积分 |
| **RViz 中 LaserScan 看不到** | 1) Fixed Frame 设成 `map`；2) LaserScan 的 Reliability 改 `Best Effort`（雷达驱动默认 best-effort）；3) `/scan` 是否真的在发 |
| **TF 树缺 `base_link -> laser_frame`** | static_transform_publisher 没启动，或 `frame_id` 参数对不上（launch 里默认 `laser_frame`，确认雷达节点没改成 `laser`） |
| **TF 冲突：多个节点同时发 `odom -> base_link`** | proposed 和 baseline 不能同时跑；`yahboom_base_node` 若内部也发布 odom TF，要在 params 里关掉；只能有一个 `publish_tf=true` |
| **`/map` 不更新** | scan_topic 没接好（确认 slam_toolbox 收到 `/scan`：`ros2 node info /slam_toolbox`）；机器人静止 → 调小 `minimum_travel_distance/heading`；时间戳错乱 → 检查 `use_sim_time` 一致 |
| **长走廊地图歪斜** | 1) `loop_search_maximum_distance` 调大到 5.0–8.0 让回环触发；2) 缩短 `map_update_interval`；3) 检查 `b_eff` 是否漂大（看 `/slip/b_eff`），必要时收紧 `beff_max`；4) 走廊纹理太少时让机器人偶尔做小角度回头让 scan-match 拿到更多约束 |
| **`slam_toolbox` 一启动就崩** | 多半是参数 YAML 顶层 key 写错。必须是 `slam_toolbox:` 而不是节点名；本仓库的 yaml 已经按 slam_toolbox 默认节点名给好 |
| **slip_odom_node 一直输出原点** | 没有收到 `/imu/data` 或 `/wheel_speeds`；用 `ros2 topic hz` 一一确认 |

---

## 8. Proposed vs Baseline 总结

| | Proposed | Baseline |
|---|---|---|
| Launch | `slam_bringup.launch.py` | `slam_bringup_baseline.launch.py` |
| Odom 节点 | `slip_odom_node` | `simple_encoder_odom_node` |
| Odom 话题 | `/odom` | `/odom` |
| `odom→base_link` TF | slip_odom_node 发 | simple_encoder_odom_node 发 |
| 用 IMU 融合 | ✅ | ❌（仅编码器） |
| Slip-aware | ✅ | ❌ |
| 给 slam_toolbox 的输入 | 同一份 `/scan` + 不同 `/odom` | 同一份 `/scan` + 不同 `/odom` |

录制时只需更换 launch + bag 名后缀（`_proposed` / `_baseline`），其它命令完全一致，可直接做横向对比。

---

## 9. 参数文件速查

### `config/slam_toolbox_params.yaml` 关键项

- `mode: mapping`
- `map_frame / odom_frame / base_frame`：`map / odom / base_link`
- `transform_publish_period: 0.02`（50 Hz）
- `resolution: 0.05`，`max_laser_range: 12.0`
- `minimum_travel_distance: 0.05`，`minimum_travel_heading: 0.05`
- `scan_buffer_size: 10`，`scan_buffer_maximum_scan_distance: 12.0`
- 回环：`do_loop_closing: true`，`loop_search_maximum_distance: 3.0`

### `config/slip_odom_params.yaml` 关键项

- 几何：`wheel_base: 0.13`，`wheel_radius: 0.0225`
- TF：`publish_tf: true`
- Sigmoid：`sigmoid_s0: 0.25`，`sigmoid_k: 12.0`
- 死区：`slip_deadband: 0.04`，`w_deadband: 0.015`
- λ 限幅：`lambda_min: 0.05`，`lambda_max: 0.90`
- 非线性速度修正：`use_nonlinear_velocity_correction: true`，`slip_sigma_s: 0.35`，`turn_gamma: 0.20`
- ICR：`use_icr_beff: true`，`beff_min/max/init: 0.10/0.80/0.13`
