#ifndef DEMO_HPP
#define DEMO_HPP

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <vector>
#include <cmath>

/**
 * @brief Offboard Demo Node - 发布简单的轨迹点给控制节点
 * 
 * 这个节点向 /offboard/cmd 和 /offboard/cmd_mode 话题发布目标位置和控制模式
 * 让无人机执行预设的飞行任务
 */
class OffboardDemoNode : public rclcpp::Node {
public:
    OffboardDemoNode();

private:
    // 发布者
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr cmd_pub_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr cmd_mode_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr takeoff_land_pub_;
    
    // 订阅者
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
    
    // 定时器
    rclcpp::TimerBase::SharedPtr timer_;
    
    // 当前位置
    std::array<double, 3> current_position_{0.0, 0.0, 0.0};  // x, y, z
    bool position_received_{false};
    
    // 任务状态
    enum class MissionState {
        IDLE,           // 空闲
        TAKEOFF,        // 起飞
        WAYPOINT_1,     // 航点1
        WAYPOINT_2,     // 航点2
        WAYPOINT_3,     // 航点3
        WAYPOINT_4,     // 航点4
        RETURN_HOME,    // 返回起点
        LAND,           // 降落
        DONE            // 任务完成
    };
    
    MissionState mission_state_;
    rclcpp::Time state_start_time_;
    
    // 航点列表 (x, y, z, yaw)
    std::vector<std::array<double, 4>> waypoints_;
    
    // 到达阈值
    double position_threshold_{0.3};  // 位置到达阈值 (米)
    double takeoff_height_{-1.5};     // 起飞高度 (NED坐标系)
    
    // 回调函数
    void timerCallback();
    void odomCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
    
    // 辅助函数
    bool reachedTarget(double target_x, double target_y, double target_z);
    
    // 发布控制指令
    void publishCommand(double x, double y, double z, double yaw);
    void publishControlMode(bool position = true, bool velocity = false, 
                           bool acceleration = false);
    void publishTakeoffCommand();
    void publishLandCommand();
};

#endif // DEMO_HPP
