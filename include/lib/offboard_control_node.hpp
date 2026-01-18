#ifndef OFFBOARD_CONTROL_NODE_HPP
#define OFFBOARD_CONTROL_NODE_HPP

#include <chrono>
#include <memory>
#include <cmath>
#include <array>
#include <vector>
#include <set>
#include <queue>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/int32.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include "std_msgs/msg/bool.hpp"

class OffboardControlNode : public rclcpp::Node {
public:
    OffboardControlNode();
    virtual void control_loop();

protected:
    int mod_;
    int setpoint_counter_;
    rclcpp::Time last_request_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::array<double, 3> current_position_;
    std::array<double, 3> current_velocity_;
    std::array<double, 3> target_position_;
    px4_msgs::msg::VehicleStatus current_state_;
    bool armed_;
    bool in_offboard_mode_;
    uint64_t timestamp_;
    
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr state_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
    
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr mod_pub_;

    bool start_fly();
    void state_cb(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
    void odom_cb(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
    void publish_offboard_control_mode(bool position = true, bool velocity = false);
    void publish_trajectory_setpoint(double yaw = 0.0);
    void publish_velocity_setpoint(double vx, double vy, double vz);
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
    bool reached_target(double pos_tol = 0.1);
    void land();
    void publish_mod();
    void arm();
    void disarm();
    void switch_to_offboard();
};

#endif // OFFBOARD_CONTROL_NODE_HPP