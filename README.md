# PX4 Offboard

PX4 无人机 Offboard 模式底层控制(C++)

基于 ubuntu20.04 系统下的 ros2(foxy)

该功能包仅作为 Offboard **底层主控**

消息依赖：[px4_msgs](https://github.com/PX4/px4_msgs/tree/release/1.14)

具体 Offboard 轨迹控制需要外部规划器(或自行写个轨迹节点，并将轨迹发布至对应话题)

## FSM 有限状态机框架图

框架参考了浙大高飞老师的 [PX4CTRL](https://github.com/ZJU-FAST-Lab/Fast-Drone-250/tree/master/src/realflight_modules/px4ctrl) 的部分框架

	         系统启动
	            |
	            |
	            v
	-------> 位置控制 <-------------------
	|         ^   |   \                 |
	|         |   |    \                |
	|         |   |     > 自动起飞       |
	|         |   |      /              |
	|         |   |     /               |
	|         |   |    /                |
	|         |   v   /                 |
	|       自动悬停 <                   |
	|         ^   |  \                  |
	|         |   |   \                 |
	|         |	  |    > 自动降落 -------|
	|         |   |          ^          |
	|         |   v          |          |
	-------- OFFBOARD--->定高模式--------

### 状态机说明

状态机包含以下状态:
- **POSITION**: 位置控制模式 (初始状态)
- **AUTO_HOVER**: 自动悬浮，等待进入Offboard
- **OFFBOARD**: Offboard 模式，执行外部指令
- **AUTO_TAKEOFF**: 自动起飞
- **AUTO_LAND**: 自动降落
- **WARNING**: 低电量警告，自动降落

### 自定义轨迹

参考 `src/offboard_demo.cpp` 修改航点列表:

```cpp
waypoints_ = {
    {x1, y1, z1, yaw1},  // 航点1 (NED坐标系)
    {x2, y2, z2, yaw2},  // 航点2
    // ... 添加更多航点
};
```

### 参数配置

编辑 `config/ctrl_param.yaml` 修改参数:

```yaml
msg_timeout:
  rc: 0.5              # 遥控器超时时间
  odom: 0.5            # 里程计超时时间
  
takeoff_land:
  enable: true         # 启用自动起飞降落
  enable_arm: true     # 自动解锁
  speed: 0.3           # 起飞降落速度 (m/s)
  height: 1.0          # 起飞高度 (m)
  
low_voltage: 13.2      # 低电压报警阈值
odom_pos_jump: 0.3     # 位置跳变容忍度

rc_debug:
  ch_mode: 8           # 遥控器悬浮模式通道
  ch_gear: 9           # 遥控器 Offboard 模式通道
```

## 节点说明

- **订阅话题**:

  - `/fmu/out/vehicle_odometry` - 里程计数据

  - `/fmu/out/vehicle_status` - 飞控状态

  - `/fmu/out/rc_channels` - 遥控器通道

  - `/fmu/out/battery_status` - 电池状态

  - `/fmu/out/vehicle_land_detected` - 着陆检测

  - `/offboard/cmd` - Offboard 控制指令

  - `/offboard/cmd_mode` - Offboard 控制模式

  - `/offboard/takeoff_land` - 起飞/降落命令

- **发布话题**:

  - `/fmu/in/trajectory_setpoint` - 轨迹设定点

  - `/fmu/in/offboard_control_mode` - Offboard 控制模式

  - `/fmu/in/vehicle_command` - 飞行器命令

  - `/offboard/trigger` - 由控制器发布的里程计数据，用于触发外部命令并统一时间戳

## 故障排除

### 问题1: 无法接收飞控数据

- 检查 PX4 是否正常运行

- 检查 DDS 连接: `ros2 topic list | grep fmu`

### 问题2: 无法进入Offboard模式

- 确认遥控器通道映射正确

- 检查参数文件中的 `ch_mode` 和 `ch_gear` 配置

- 确保里程计数据正常

### 问题3: 编译错误

```bash
# 清理构建
rm -rf build/ install/ log/
colcon build --packages-select offboard_cpp --cmake-clean-first
```

## 安全注意事项

1. **首次使用**: 建议在仿真环境中测试
2. **电量监控**: 系统会在低电量时自动降落
3. **失控保护**: 遥控器失联或里程计失效时自动返回位置(高度)控制模式(如果里程计失效且没有定高信息，则会返回手动模式)
4. **急停**: 随时可以通过遥控器切换回手动模式




## 其他部分

[兼容 ros2(foxy) 的 vision_to_mavros 功能包实现(BoomBoomFly, Not open source)](https://github.com/BoomBoomFly/ros2_foxy_vision_to_mavros)

[PX4 无人机 Offboard 模式控制(python)(BoomBoomFly, Not open source)](https://github.com/BoomBoomFly/offboard)

[YOLO and 飞桨推理(python)](https://github.com/AyasOwen/cv_yolo_paddle_pkg)

[OpenCv 等基础功能实现(C++)](https://github.com/AyasOwen/opencv_cpp)

[D435 和 T265 的联合使用(blog, ros1 暂未做 ros2 移植)](https://ayasowen.github.io/2024/11/17/T265%E5%92%8CD435%E8%81%94%E5%90%88%E4%BD%BF%E7%94%A8/)
