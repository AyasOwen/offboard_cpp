#ifndef INPUT_HPP
#define INPUT_HPP

#include <lib/param.hpp>
#include <Eigen/Dense>
#include <cstdint>
#include <iostream>
#include <functional>
#include <memory>
#include <chrono>
#include <memory>
#include <cmath>
#include <array>
#include <vector>
#include <set>
#include <queue>
#include <limits>

#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/bool.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/rc_in.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/msg/extended_state.hpp>

double normalize_angle(double angle);
// 遥控器映射
class RC_Data_t{
public:
    rclcpp::Node::SharedPtr node_;          // Node 指针

    // 这里以 PID 为例
    // 同理也能改成 OpenCv 的 RGB阈值
    // p, i, d(也可以是其他名字)为遥控器映射的动态参数
    double p;
    double i;
    double d;

    // mode 和 gear 分别需要一个遥控器扳机，都为拉到最高时启动
    // mode 控制无人机是否处于悬浮模式
    // gear 控制无人机是否由 offboard代码接管
    // 只有当 mode 处于悬浮模式时 gear 扳机拉动才能激活 offboard代码
    double mode;
    double gear;
    double last_mode;
    double last_gear;
    bool have_init_last_mode{false};
    bool have_init_last_gear{false};
    double ch[4];                           // 摇杆映射

    rclcpp::Time rcv_stamp;                 // 接收时间戳
    mavros_msgs::msg::RCIn msg;

    bool is_hover_mode;                     // 是否悬浮
    bool enter_hover_mode;                  // 是否进入悬浮
    bool is_offboard;                       // 是否是 Offboard
    bool enter_offboard;                    // 是否进入 Offboard

    // 阔值
    static constexpr double API_MODE_THRESHOLD_VALUE = 0.75;    // 模式切换阔值
    static constexpr double GEAR_SHIFT_VALUE = 0.75;            // 档位变换阈值
    static constexpr double DEAD_ZONE = 0.25;                   // 死区阔值

    RC_Data_t(const rclcpp::Node::SharedPtr& node);
    void check_validity();                  // 检查数据有效性
    bool check_centered();                  // 建成摇杆是否回正
    void feed(mavros_msgs::msg::RCIn::SharedPtr pMsg, const Param_t& param);       // 回调
};

// Odom 信息
class Odom_Data_t{
public:
    rclcpp::Node::SharedPtr node_;

    Eigen::Vector3d p;              // 位置: x, y, z
    Eigen::Vector3d before_p;       // 前一个时刻的位置
    Eigen::Vector3d v;              // 速度: vx, vy, yz
    Eigen::Quaterniond q;           // 四元数姿态
    Eigen::Vector3d w;              // 角速度

    nav_msgs::msg::Odometry msg;    // 消息
    rclcpp::Time rcv_stamp;         // 接收时间戳
    bool recv_new_msg;              // 是否接收新信息的标志位
    bool pos_jump;                  // 位置是否发生突变

    Odom_Data_t(const rclcpp::Node::SharedPtr& node);
    void feed(nav_msgs::msg::Odometry::SharedPtr pMsg, const Param_t& param);
};

class State_Data_t{
public:
    rclcpp::Node::SharedPtr node_;
    mavros_msgs::msg::State current_state;             // 当前模式
    mavros_msgs::msg::State state_before_offboard;     // 进入 Offboard 模式前的模式

    State_Data_t(const rclcpp::Node::SharedPtr& node);
    void feed(mavros_msgs::msg::State::SharedPtr pMsg);
};

// 接收目标缓存区
// 如果是单纯的写死路径，可以考虑脱离缓存区，但不建议
class Offboard_Data_t{
public:
    rclcpp::Node::SharedPtr node_;

    Eigen::Vector3d p;          // 位置: x, y, z
    Eigen::Vector3d v;          // 速度: vx, vy, yz
    Eigen::Vector3d a;          // 加速度: ax, ay, az
    Eigen::Vector3d j;          // 加加速度
    double yaw;                 // 航向角
    double yaw_rate;            // 航向角速度

    mavros_msgs::msg::PositionTarget msg;

    rclcpp::Time rcv_stamp;

    Offboard_Data_t(const rclcpp::Node::SharedPtr& node);
    void feed(mavros_msgs::msg::PositionTarget::SharedPtr pMsg);
};

class Offboard_Mode_Data_t{
public:
    rclcpp::Node::SharedPtr node_;

    uint16_t mask{0};

    rclcpp::Time rcv_stamp;

    Offboard_Mode_Data_t(const rclcpp::Node::SharedPtr& node);
    void feed(std_msgs::msg::UInt16::SharedPtr pMsg);
};

// 电池电量信息
class Battery_Data_t{
public:
    rclcpp::Node::SharedPtr node_;

    double volt{0.0};           // 当前电量(V)
    double percentage{0.0};     // 剩余电量百分比
    double flyTime{0.0};        // 剩余飞行时间
    uint8_t warning{0};

    sensor_msgs::msg::BatteryState msg;
    rclcpp::Time rcv_stamp;     //  接收时间戳

    Battery_Data_t(const rclcpp::Node::SharedPtr& node);
    void feed(sensor_msgs::msg::BatteryState::SharedPtr pMsg);
};

// 起飞降落处理
class Takeoff_Land_Data_t{
public:
    rclcpp::Node::SharedPtr node_;

    bool triggered{false};          // 标志位: 是否触发起飞或降落动作
    uint8_t takeoff_land_cmd{0};    // 0: none, 1: takeoff, 2: land
    bool landed{true};              // 是否到达地面

    mavros_msgs::msg::ExtendedState land_msg;
    rclcpp::Time rcv_stamp;         // 时间戳

    Takeoff_Land_Data_t(const rclcpp::Node::SharedPtr& node);

    void feed_takeoff_land(std_msgs::msg::UInt8::SharedPtr pMsg);
    void feed_landed(mavros_msgs::msg::ExtendedState::SharedPtr pMsg);
};

#endif // INPUT_HPP
