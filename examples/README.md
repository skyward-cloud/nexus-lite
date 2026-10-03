# 示例脚本

在已启动 `px4_bridge`（及 PX4 / MicroXRCE-DDS-Agent）的前提下运行。

```bash
cd nexus-lite
source /opt/ros/humble/setup.bash
source /path/to/px4_msgs/install/setup.bash   # 若需要
source install/setup.bash

ros2 launch px4_bridge bridge.launch.py      # 另开终端
```

| 脚本 | 说明 |
|------|------|
| `01_arm_takeoff_land.py` | 解锁 → 起飞 1m → 悬停 → 降落 → 上锁 |
| `02_position_point.py` | 起飞后飞到指定 ENU 点（POSITION_CONTROL） |
| `03_realtime_velocity.py` | 起飞后实时速度画圆 |
| `04_mode_switch.py` | 只切模式并打印 status（默认不起飞） |
| `05_keyboard_teleop.py` | 键盘：解锁/起飞/位置积分遥操/降落/上锁 |

```bash
python3 examples/04_mode_switch.py
python3 examples/01_arm_takeoff_land.py
python3 examples/02_position_point.py --x 2 --y 0 --z 1.5
python3 examples/03_realtime_velocity.py --duration 8
python3 examples/05_keyboard_teleop.py
```

坐标系为局部 **ENU**：`x` 东、`y` 北、`z` 向上为正。真机请先在安全场地确认。
