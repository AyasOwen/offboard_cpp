#ifndef CTRLFSM_HPP
#define CTRLFSM_HPP

#include <lib/input.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/command_tol.hpp>

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

    mavros_msgs::msg::PositionTarget des_mode;

    Eigen::Vector4d start_pose;             // 起飞 / 降落前的位置
    Eigen::Vector4d hover_pose;             // 悬浮时的位置
    bool landed{true};                      // 通过函数判断的是否着陆

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
    
    // 发布者
    rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr offboard_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr trigger_pub;
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr offboard_mode_pub;

    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_client;
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr land_client;

    CtrlFSM(Param_t& param, const rclcpp::Node::SharedPtr& node);

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
    mavros_msgs::msg::PositionTarget get_hover_des(rclcpp::Time& now_time);

    // 起飞降落相关
    bool land_in_progress{false};           // 标志位，判断是否在着陆进程中
    rclcpp::Time land_start_time;           // 开始 Land 进程的时间
    rclcpp::Time takeoff_start_time;        // 开始 Takeoff 进程的时间

    void set_start_pose_for_takeoff_land();
    bool land(rclcpp::Time& now_time);                              // 降落
    bool arm_to_disarm(rclcpp::Time& now_time, bool arm);           // 无人机解锁 / 上锁
    mavros_msgs::msg::PositionTarget get_takeoff_des(rclcpp::Time& now_time);            // 起飞

    // 工具
    double get_yaw_from_odom();             // 从 Odom 中的四元数中提取出 yaw
    bool switch_to_offboard(rclcpp::Time& now_time, bool on_off);           // 进入 / 退出 Offboard 模式
    bool switch_to_altctl(rclcpp::Time& now_time);                          // 进入定高模式
    bool switch_to_position(rclcpp::Time& now_time);                        // 进入定高模式
    void set_hover_pos();                   // 根据当前 Odom 位置记录悬浮位置
    bool request_mode_change(const std::string &mode);
    bool request_arm(bool arm);
    bool request_land();
    void land_detector(const mavros_msgs::msg::PositionTarget& des, const rclcpp::Time& now_time);
};

#endif // CTRLFSM_HPP
