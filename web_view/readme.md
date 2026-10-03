### web 操作服务

基于 **Python + pywebview + Three.js** 的桌面操作面板。

- 订阅 `/px4_bridge/out/status`，在侧栏显示连接/解锁/模式/位姿等
- 订阅点云话题（默认 `/lidar/points_world`），用 Three.js 渲染
- 页面按钮经后端发布 `/px4_bridge/in/task_cmd` 下发控制

## 依赖

```bash
# 需已 source ROS 工作空间（含 px4_bridge_msgs、sensor_msgs）
source /home/lemon/nexus-lite/install/setup.bash

pip install -r web_view/requirements.txt
# Linux 还需 WebView 后端，例如：
# sudo apt install python3-gi python3-gi-cairo gir1.2-webkit2-4.1
```

## 运行

```bash
source install/setup.bash
python3 web_view/main.py

# 指定点云话题
python3 web_view/main.py --cloud-topic /lidar/points_body

# 打开开发者工具
python3 web_view/main.py --debug
```

## 控制按钮 → task_cmd

前端仍传 dict，后端写入扁平 `TaskCommand` 字段（无 JSON）。详见 [`docs/task_cmd.md`](../docs/task_cmd.md)。

| 按钮 | task_type | 主要字段 |
|------|-----------|----------|
| 解锁 / 上锁 | `ARMING` | `arm: true/false` |
| 起飞 | `POSITION_CONTROL` | `x,y,z,...` |
| 降落 | `MODE_SWITCH` | `mode: LAND` |
| 悬停 / 定点 / OFFBOARD / 返航 | `MODE_SWITCH` | 对应 `mode` |
| 停桨 | `KILL_SWITCH` | （无额外字段） |

## 目录

```
web_view/
  main.py           # pywebview 入口 + JS API
  ros_bridge.py     # ROS 订阅/发布
  requirements.txt
  frontend/
    index.html
    style.css
    app.js
    vendor/         # 本地 three.js（避免 file:// 拉 CDN）
```
