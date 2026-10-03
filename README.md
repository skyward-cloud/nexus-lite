# nexus-lite

轻量 ROS 2 ↔ PX4 控制桥：用统一任务接口驱动解锁、切模式、指点、速度控制，并适配 Gazebo / AirSim 仿真。

## 仓库内容


| 包                    | 作用                                                 |
| -------------------- | -------------------------------------------------- |
| `px4_bridge`         | 控制桥：任务队列、Offboard、失效保护                             |
| `px4_bridge_msgs`    | `TaskCommand` / `RealtimeControl` / `BridgeStatus` |
| `px4_sitl_gz_launch` | PX4 SITL + Gazebo，以及 IMU / 里程计 / 点云桥               |
| `airsim_bridge`      | AirSim NED → ENU 里程计与点云转换                          |


相关目录：`simulation/`（Gazebo 机型与世界）、`examples/`（示例脚本）、`web_view/`（本地控制面板）、`docs/`（接口说明）。

## 依赖

- ROS 2 Humble
- [PX4-Autopilot](https://github.com/PX4/PX4-Autopilot)（已编译 POSIX SITL）
- [Micro XRCE-DDS Agent](https://github.com/eProsima/Micro-XRCE-DDS-Agent)
- `px4_msgs`（与当前 PX4 版本匹配）
- Gazebo 仿真：Gazebo Harmonic / Fortress（与本机 `gz-transport` 一致）
- AirSim 仿真（可选）：AirSim + `none_iris` SITL



### PX4 自定义机架 `drone260`

Gazebo 默认机型为 `drone260`。在 PX4 源码中增加 POSIX 机架：

1. 复制 `ROMFS/px4fmu_common/init.d-posix/airframes/4001_gz_x500` 为 `5001_drone260`（或按 PX4 约定命名为 `5001_gz_drone260`）。
2. 在同目录 `CMakeLists.txt` 中登记该文件。
3. 重新编译 PX4 SITL。



## 编译

```bash
cd ~/app/nexus-lite
source /opt/ros/humble/setup.bash
source /path/to/px4_msgs/install/setup.bash   # 按实际路径
colcon build
source install/setup.bash
```



## 环境变量

按本机路径修改后写入 `~/.bashrc`，或每次开终端执行：

```bash
source /opt/ros/humble/setup.bash
source ~/app/px4-lib/install/setup.bash      # px4_msgs 等工作空间
source ~/app/nexus-lite/install/setup.bash
source ~/app/super/install/setup.bash        # 仅跑 SUPER 时需要

export PX4_SOURCE_DIR=~/app/PX4-Autopilot
export PX4_SIM_HOST_ADDR=192.168.138.1       # AirSim / 远程仿真主机；本机可省略
export GZ_SIM_RESOURCE_PATH="$HOME/app/nexus-lite/simulation/models:$GZ_SIM_RESOURCE_PATH"
```

`sitl_gz_with_bridge.launch.py` 也会把包内 `share/px4_sitl_gz_launch/{models,worlds}` 加入资源路径。若已 `colcon build`，`GZ_SIM_RESOURCE_PATH` 可只作备用。

## Gazebo 仿真

三个终端，均先 `source` 上述环境：

```bash
# 1) PX4 SITL + Gazebo + 传感器桥
ros2 launch px4_sitl_gz_launch sitl_gz_with_bridge.launch.py

# 2) MicroXRCEAgent
MicroXRCEAgent udp4 -p 8888

# 3) 控制桥 + 视觉里程计桥
ros2 launch px4_bridge bridge.launch.py
```

常用 launch 参数：`px4_src`（默认读 `PX4_SOURCE_DIR`）、`px4_sim_model`（默认 `drone260`）、`px4_gz_world`（默认 `custom`）、`headless:=true`。

默认 ROS 话题：`/drone260/odom`、`/imu/data`、`/lidar/points_body`、`/lidar/points_world`。

## AirSim 仿真

```bash
cd "$PX4_SOURCE_DIR"
make px4_sitl_default none_iris

# 另开终端：雷达 / 位姿（按本机 AirSim 配置）
python3 ~/app/nexus-lite/src/airsim_bridge/src/getLidarData.py

# 另开终端：NED → ENU
ros2 launch airsim_bridge airsim_converter.launch.py

MicroXRCEAgent udp4 -p 8888
ros2 launch px4_bridge bridge.launch.py
```



## 控制接口

外部只发一次性任务，桥接**只保留最新一条**。非法任务会打日志并丢弃。


| 话题                                | 类型                                | 方向                 |
| --------------------------------- | --------------------------------- | ------------------ |
| `/px4_bridge/in/task_cmd`         | `px4_bridge_msgs/TaskCommand`     | 任务：解锁、切模式、指点、速度、杀机 |
| `/px4_bridge/in/realtime_control` | `px4_bridge_msgs/RealtimeControl` | 高频速度 / 姿态          |
| `/px4_bridge/out/status`          | `px4_bridge_msgs/BridgeStatus`    | 状态回读               |


`task_type`：`ARMING` · `MODE_SWITCH` · `POSITION_CONTROL` · `VELOCITY_CONTROL` · `KILL_SWITCH`。

解锁示例：

```bash
ros2 topic pub --once /px4_bridge/in/task_cmd px4_bridge_msgs/msg/TaskCommand "{
  task_id: 'arm-1',
  task_type: 'ARMING',
  arm: true
}"
```

字段、坐标系与各类型样例见 [docs/task_cmd.md](docs/task_cmd.md)。可运行脚本见 [examples/README.md](examples/README.md)。

## 规划算法（可选）

本仓库适配了 [SUPER](https://github.com/ZJU-FAST-Lab/SUPER) 与 [FUEL](https://github.com/HKUST-Aerial-Robotics/FUEL)。需先编译对应工作空间并 `source` 其 `install/setup.bash`。后续若有其他实用的算法也会继续接入。

[super](https://github.com/skyward-cloud/super) 
[fuel](https://github.com/skyward-cloud/fuel) 

```bash
# SUPER：发布 /goal_pose 后开始导航
ros2 launch super_planner nexus.launch.py

# FUEL：发布 /goal_pose 后开始搜索
ros2 launch exploration_manager exploration.launch.py
```



## 可视化与工具

**Web 面板**（pywebview + HTML，订阅 status / 点云，按钮发 `task_cmd`）：

```bash
source install/setup.bash
pip install -r web_view/requirements.txt
python3 web_view/main.py
# python3 web_view/main.py --cloud-topic /lidar/points_world
```

**Foxglove：**

```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

在 Windows 或其他设备打开 Foxglove 桌面端连接即可。