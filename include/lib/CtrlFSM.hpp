#ifndef CTRLFSM_HPP
#define CTRLFSM_HPP

#include <lib/input.hpp>

class CtrlFSM{
public:
    rclcpp::Node::SharedPtr node_;          // Node 指针
    std::pair<bool, rclcpp::Time> delay_trigger{false, rclcpp::Time(0, 0, RCL_ROS_TIME)};
    
    // 实例化
    Param_t &param_;

    RC_Data_t rc_data;
    Odom_Data_t odom_data;
    State_Data_t state_data;
    Offboard_Data_t offboard_data;
    Offboard_Mode_Data_t offboard_mode_data;
    Battery_Data_t battery_data;
    Takeoff_Land_Data_t takeoff_land_data;

    px4_msgs::msg::OffboardControlMode des_mode;

    Eigen::Vector4d start_pose;             // 起飞 / 降落前的位置
    Eigen::Vector4d hover_pose;             // 悬浮时的位置

    // 枚举无人机的状态
    enum State_t{
        POSITION = 1,   // 位置控制模式
        AUTO_HOVER,     // 自动盘旋模式，等待接收指令进入 OFFBOARD
        OFFBOARD,       // OFFBOARD 模式
        AUTO_TAKEOFF,   // 自动起飞
        AUTO_LAND,      // 自动降落
        WANRING,        // 电量过低时自动降落
        NONE
    };

    // 枚举 PX4 飞控模式
    // DDS 不像 Mavros 一样，发送和接收到的 Mode 类型及数值不同，需要做映射
    enum class Mode_t {
        MANUAL,
        ALTCTL,
        POSCTL,
        OFFBOARD,
        UNKNOWN
    };
    
    // 发布者
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr offboard_pub;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trigger_pub;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_com_pub;

    CtrlFSM(const Param_t& param, const rclcpp::Node::SharedPtr& node);

    void FSM();         // 状态机控制

    // 判断数据时间戳是否有效
    bool rc_is_received(rclcpp::Time& now_time);
    bool odom_is_received(rclcpp::Time& now_time);
    bool offboard_is_received(rclcpp::Time& now_time);
    bool offboard_mode_is_received(rclcpp::Time& now_time);
    bool battery_is_received(rclcpp::Time& now_time);

private:
    State_t state;      // 只允许在 FSM() 中进行修改

    // 控制相关
    px4_msgs::msg::TrajectorySetpoint get_hover_des(rclcpp::Time& now_time);

    // 起飞降落相关
    bool arm_in_progress{false};            // 标志位，判断是否在上锁 / 解锁进程中
    bool arm_target{false};                 // 如果目标发生变化 → 重新发命令
    bool land_in_progress{false};           // 标志位，判断是否在着陆进程中
    rclcpp::Time arm_start_time;            // 开始 Arm 进程的时间
    rclcpp::Time land_start_time;           // 开始 Land 进程的时间
    rclcpp::Time takeoff_start_time;        // 开始 Takeoff 进程的时间

    void set_start_pose_for_takeoff_land();
    void land(rclcpp::Time& now_time);                              // 降落
    bool arm_to_disarm(rclcpp::Time& now_time, bool arm);           // 无人机解锁 / 上锁
    px4_msgs::msg::TrajectorySetpoint get_takeoff_des();            // 起飞

    // 工具
    bool mode_in_progress{false};           // 标志位，判断是否在切换 Mode 进程中
    bool altctl_in_progress{false};         // 标志位，判断是否在进入 ALTCTL 模式进程中
    bool position_in_progress{false};       // 标志位，判断是否在进入 POSITION 模式进程中
    rclcpp::Time mode_start_time;           // 开始切换 Mode 进程的时间
    rclcpp::Time altctl_start_time;         // 开始进入 ALTCTL 模式进程的时间
    rclcpp::Time position_start_time;       // 开始进入 POSITION 模式进程的时间

    void get_yaw_from_odom();               // 从 Odom 中的四元数中提取出 yaw
    float mode_to_com(Mode_t mode);         // Mode_t 映射到发送的 Param2 值
    Mode_t status_to_mode(const px4_msgs::msg::VehicleStatus& status);      // 接收到的 Mode 值映射到 Mode_t
    std::string mode_to_string(Mode_t mode);                                // 将 Mode_t 转为字符串
    void switch_to_offboard(rclcpp::Time& now_time, bool on_off);           // 进入 / 退出 Offboard 模式
    void switch_to_altctl(rclcpp::Time& now_time);                          // 进入定高模式
    void switch_to_position(rclcpp::Time& now_time);                        // 进入定高模式
    void set_hover_pos();                   // 根据当前 Odom 位置记录悬浮位置
    void set_offboard_mode(rclcpp::Time& now_time, bool position = true, 
        bool velocity = true, bool acceleration = false, bool attitude = false, bool body_rate = false);
    void publish_trajectory_setpoint(rclcpp::Time& now_time);
    void publish_vehicle_command(rclcpp::Time& now_time, uint16_t command, float param1 = 0.0, float param2 = 0.0);
};

#endif // CTRLFSM_HPP
