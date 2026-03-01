#ifndef OFFBOARD_DEMO_HPP
#define OFFBOARD_DEMO_HPP

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
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
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr cmd_pub_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr cmd_mode_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr takeoff_land_pub_;

    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr trigger_sub_;

    rclcpp::TimerBase::SharedPtr timer_;

    /* -------------------- 回调函数 -------------------- */
    void timerCallback();
    void odomCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
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
    double takeoff_height_{-2.0};
};

#endif