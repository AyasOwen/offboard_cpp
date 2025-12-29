#ifndef OFFBOARD_CONTROL_NODE_HPP
#define OFFBOARD_CONTROL_NODE_HPP

#include <chrono>
#include <memory>
#include <cmath>
#include <array>
#include <vector>
#include <set>
#include <queue>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
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
    mavros_msgs::msg::State current_state_;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pos_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vel_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_pub_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_client_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr mod_pub_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;


    bool start_fly();
    void state_cb(const mavros_msgs::msg::State::SharedPtr msg);
    void pos_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void vel_cb(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
    void publish_setpoint(double yaw = 0.0);
    void publish_velocity(double vx, double vy, double vz);
    bool reached_target(double pos_tol = 0.1);
    void land();
    void publish_mod();
};

#endif // OFFBOARD_CONTROL_NODE_HPP