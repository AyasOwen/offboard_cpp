#ifndef OFFBOARD_DEMO_HPP
#define OFFBOARD_DEMO_HPP

#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/bool.hpp>

#include <vector>
#include <array>
#include <cmath>

class OffboardDemoNode : public rclcpp::Node {
public:
    OffboardDemoNode();

private:

    /* -------------------- 任务状态 -------------------- */
    enum class MissionState {
        IDLE,
        WAYPOINT,
        RETURN_HOME,
        LAND,
        DONE
    };

    void changeState(MissionState new_state);

    /* -------------------- ROS 通信 -------------------- */
    rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr cmd_mode_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr takeoff_land_pub_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr trigger_sub_;

    rclcpp::TimerBase::SharedPtr timer_;

    /* -------------------- 回调函数 -------------------- */
    void timerCallback();
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void triggerCallback(const std_msgs::msg::Bool::SharedPtr msg);

    /* -------------------- 发布接口 -------------------- */
    void publishCommand(double x, double y, double z, double yaw);
    void publishControlMode(bool position = true,
                            bool velocity = false,
                            bool acceleration = false);
    void publishTakeoffCommand();
    void publishLandCommand();

    /* -------------------- 任务逻辑 -------------------- */
    void handleIdle();
    void handleWaypoint();
    void handleReturnHome();
    void handleLand();

    bool reachedTarget(double x, double y, double z);
    bool isVelocityNearZero() const;

    /* -------------------- 数据成员 -------------------- */
    MissionState mission_state_{MissionState::IDLE};
    rclcpp::Time state_start_time_;

    std::vector<std::array<double, 4>> waypoints_;
    size_t current_wp_index_{0};

    std::array<double, 3> current_position_{0.0, 0.0, 0.0};
    std::array<double, 3> current_velocity_{0.0, 0.0, 0.0};

    bool position_received_{false};
    bool trigger_{false};

    /* 起飞/降落判定 */
    bool takeoff_sent_{false};
    bool stable_started_{false};
    rclcpp::Time stable_start_time_;

    /* 参数 */
    double position_threshold_{0.1};
    double velocity_threshold_{0.12};
    double stable_time_{1.0};
    double takeoff_height_{2.0};

        /* 起飞逻辑 */
    bool takeoff_command_sent_{false};
    double takeoff_start_z_{0.0};
    bool takeoff_stable_started_{false};

    /* 降落逻辑 */
    bool land_stable_started_{false};
};

#endif