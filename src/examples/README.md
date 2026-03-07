# Examples - 示例节点

本目录包含两个示例节点，用于演示如何使用 PX4 Offboard 控制框架。

## 目录

- [Offboard Demo](#offboard-demo) - 基础航点飞行示例
- [Animal Testing](#animal-testing) - 动物检测路径规划系统

---

## Offboard Demo

### 简介

基础的航点飞行示例，展示如何通过发布轨迹指令控制无人机完成方形航线飞行。

### 功能特性

- 自动起飞至指定高度
- 按预设航点顺序飞行
- 自动返回起点
- 自动降落

### 启动方式

```bash
# 启动 offboard_node
ros2 launch offboard_cpp offboard_control.launch.py

# 另开终端，启动 demo 节点
ros2 launch offboard_cpp offboard_demo.launch.py auto_start_demo:=true
```

或者一次性启动：

```bash
ros2 launch offboard_cpp offboard_demo.launch.py auto_start_demo:=true
```

### 航点配置

在 [offboard_demo.cpp](offboard_demo.cpp) 中修改航点列表：

```cpp
waypoints_ = {
    {2.0, 0.0, -2.0, 0.0},      // 航点1: 向北2米
    {2.0, 2.0, -2.0, M_PI/2},   // 航点2: 向东2米
    {0.0, 2.0, -2.0, M_PI},     // 航点3: 向南2米
    {0.0, 0.0, -2.0, -M_PI/2}   // 航点4: 向西2米，回到起点
};
```

**注意**: 坐标使用 NED 坐标系（北-东-地），z 为负值表示向上飞行。

### 参数说明

- `position_threshold_`: 位置容差 (默认: 0.1m)
- `velocity_threshold_`: 速度稳定判断阈值 (默认: 0.12m/s)
- `stable_time_`: 稳定停留时间 (默认: 1.0s)
- `takeoff_height_`: 起飞高度 (默认: -2.0m，NED坐标)

---

## Animal Testing

### 简介

动物检测路径规划与控制系统，实现 9×7 网格地图的完整遍历，支持 3 个禁飞区避障。

### 功能特性

- **智能路径规划**: BFS + 贪心算法生成最优遍历路径
- **禁飞区避障**: 支持 3 个动态禁飞区配置
- **路径压缩**: 自动去除共线点，插入中间点保证平滑
- **实时反馈**: 发布当前地图 ID 和完整飞行路径
- **地面站控制**: 等待地面站启动信号后执行任务
- **自动起降**: 完整的起飞-飞行-返航-降落流程

### 地图说明

地图为 **9 行 × 7 列**网格，共 63 个格点：

```
地图编号规则: (行号+1)*10 + (列号+1)
例如: 第0行第0列 = 11, 第8行第6列 = 97

起点固定为: 91 (第8行第0列)

坐标系: NED (北-东-地)
- x轴: 北方向，从 4.0m 到 0.0m (每格0.5m)
- y轴: 东方向，从 0.0m 到 -3.0m (每格0.5m)
- z轴: 地，负值向上，飞行高度 -1.2m

地图布局:
    A1   A2   A3   A4   A5   A6   A7
B1  11   12   13   14   15   16   17
B2  21   22   23   24   25   26   27
B3  31   32   33   34   35   36   37
B4  41   42   43   44   45   46   47
B5  51   52   53   54   55   56   57
B6  61   62   63   64   65   66   67
B7  71   72   73   74   75   76   77
B8  81   82   83   84   85   86   87
B9  91*  92   93   94   95   96   97
     |
   起点
```

### 启动方式

```bash
ros2 launch offboard_cpp animal_testing.launch.py
```

`animal_testing` 作为上层规划节点运行，依赖 `offboard_node` 作为底层控制。

### 使用流程

#### 配置禁飞区

正常流程：

通过地面站发布 3 个禁飞区的地图 ID，再通过蓝牙串口进行接收

测试流程：

自己直接往话题里发布 3 个禁飞区的地图 ID

```bash
# 发布禁飞区 (示例: B3A3=33, B5A5=55, B7A7=77)
ros2 topic pub /no_fly_zone std_msgs/msg/Int32MultiArray "{data: [33, 55, 77]}" --once
```

#### 发送启动信号

正常流程：

通过地面站的启动按钮，发送启动信号，再通过蓝牙串口进行接收

测试流程：

自己直接往话题里发送启动信号

```bash
ros2 topic pub /start_flight std_msgs/msg/Bool "{data: true}" --once
```

需要在遥控器上依次进入 `悬停模式` -> `命令模式` 否则 offboard_node 底层不会接收外部命令进行控制

#### 3. 监控飞行状态

实时查看当前所在地图格点：

```bash
ros2 topic echo /current_map_id
```

查看完整飞行路径（只发布一次）：

```bash
ros2 topic echo /final_path
```

### 话题接口

#### 订阅话题

| 话题名称 | 消息类型 | 说明 |
|---------|---------|------|
| `/fmu/out/vehicle_odometry` | `px4_msgs/msg/VehicleOdometry` | 无人机位置和速度 |
| `/no_fly_zone` | `std_msgs/msg/Int32MultiArray` | 禁飞区列表（3个ID） |
| `/start_flight` | `std_msgs/msg/Bool` | 地面站启动信号 |

#### 发布话题

| 话题名称 | 消息类型 | 说明 | 频率 |
|---------|---------|------|------|
| `/offboard/cmd` | `px4_msgs/msg/TrajectorySetpoint` | 位置控制指令 | 10Hz |
| `/offboard/cmd_mode` | `px4_msgs/msg/OffboardControlMode` | 控制模式 | 飞行时 |
| `/offboard/takeoff_land` | `std_msgs/msg/UInt8` | 起飞/降落命令 | 状态切换时 |
| `/current_map_id` | `std_msgs/msg/Int32` | 当前所在地图格点ID | 10Hz |
| `/final_path` | `std_msgs/msg/Int32MultiArray` | 完整飞行路径（压缩后） | 规划完成时一次 |

### 状态机流程

```
系统启动
   |
   v
路径规划 (PLANNING)
   |-- 等待禁飞区数据
   |-- BFS + 贪心算法生成路径
   |-- 路径压缩优化 (注：T265 不能压缩，否则会导致速度过快传感器漂移)
   v
等待地面站信号 (WAIT_GCS)
   |-- 等待 /start_flight 话题
   v
起飞 (TAKEOFF)
   |-- 发送起飞命令
   |-- 爬升至 -1.2m
   v
航点飞行 (WAYPOINT)
   |-- 依次飞向路径中的每个格点
   |-- 实时发布当前地图ID
   v
返回原点 (RETURN_HOME)
   |-- 飞回起点 (0, 0, -1.2)
   v
降落 (LAND)
   |-- 发送降落命令
   |-- 判断着地稳定
   v
任务完成 (DONE)
```

### 参数配置

在 [animal_testing.hpp](../../include/examples/animal_testing.hpp) 中修改参数：

```cpp
// 飞行参数
double position_threshold_{0.1};      // 位置到达判断阈值 (m)
double velocity_threshold_{0.12};     // 速度稳定判断阈值 (m/s)
double stable_time_{1.0};             // 稳定时间要求 (s)
double flight_height_{-1.2};          // 飞行高度，NED坐标 (m)

// 地图参数
static constexpr int ROWS = 9;        // 地图行数
static constexpr int COLS = 7;        // 地图列数
```

### 路径规划算法

- BFS 最短路径

   使用广度优先搜索在禁飞区约束下寻找两点间最短路径

- 贪心遍历

   从起点开始，每次选择最近的未访问格点，直到遍历完所有可达格点

- 路径压缩

   **去共线点**: 移除直线上的中间点，减少航点数量

   **插入中点**: 对于过长的直线段（≥2行或≥4列），插入中间点保证平滑

### 调试技巧

#### 查看日志等级

```bash
# 查看详细调试信息
ros2 run offboard_cpp animal_testing --ros-args --log-level debug
```

#### 可视化路径

发布的 `/final_path` 可以用于在 RViz 中可视化路径：

```python
# Python 示例：订阅并打印路径
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

class PathVisualizer(Node):
    def __init__(self):
        super().__init__('path_visualizer')
        self.sub = self.create_subscription(
            Int32MultiArray, '/final_path', self.callback, 10)
    
    def callback(self, msg):
        path = msg.data
        print(f"路径长度: {len(path)}")
        print(f"路径: {path}")
        
        # 转换为 A列B行 格式
        for i, id in enumerate(path):
            row = id // 10 - 1
            col = id % 10 - 1
            print(f"{i+1}. ID={id} -> A{col+1}B{row+1}")

rclpy.init()
node = PathVisualizer()
rclpy.spin(node)
```

### 常见问题

#### Q1: 节点启动后一直在等待禁飞区数据？

**A**: 发布禁飞区信息：

```bash
ros2 topic pub /no_fly_zone std_msgs/msg/Int32MultiArray "{data: [23, 45, 67]}" --once
```

#### Q2: 路径规划失败，提示"没有可用格点"？

**A**: 检查禁飞区配置是否合理，确保不会阻断整个地图。

#### Q3: 无人机不起飞？

**A**: 
1. 确认已发送启动信号：`/start_flight`
2. 检查无人机是否正常连接（`ros2 topic echo /fmu/out/vehicle_odometry`）
3. 查看节点日志判断当前状态

#### Q4: 如何修改飞行高度？

**A**: 修改 `flight_height_` 参数（NED坐标，负值表示向上）：

```cpp
double flight_height_{-1.5};  // 改为1.5m高度
```

#### Q5: 如何添加更多禁飞区？

**A**: 当前硬编码为 3 个禁飞区，如需更多可修改 `noFlyCallback` 函数中的检查：

```cpp
void noFlyCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
    if (msg->data.size() != 5) {  // 改为5个
        RCLCPP_WARN(this->get_logger(), "接收到无效数量的禁飞区");
        return;
    }
    // ...
}
```

### 安全注意

**首次飞行**: 务必在空旷区域或仿真环境中测试
**禁飞区配置**: 确认禁飞区配置正确，避免碰撞
**飞手接管**: 随时准备接管控制
**高度限制**: 确认飞行高度符合安全规范

