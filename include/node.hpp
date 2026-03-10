#ifndef NODE_HPP
#define NODE_HPP

#include <lib/CtrlFSM.hpp>
#include "rclcpp/qos.hpp"

class OffboardControlNode : public rclcpp::Node {
public:
    OffboardControlNode();
    void init(const std::shared_ptr<OffboardControlNode>& self);

private:

    Param_t param;
    std::unique_ptr<CtrlFSM> fsm;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;

    // 订阅器
    rclcpp::Subscription<mavros_msgs::msg::RCIn>::SharedPtr rc_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub;
    rclcpp::Subscription<mavros_msgs::msg::PositionTarget>::SharedPtr offboard_sub;
    rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr offboard_mode_sub;
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr takeoff_land_sub;
    rclcpp::Subscription<mavros_msgs::msg::ExtendedState>::SharedPtr land_detected_sub;

    // 定时器
    rclcpp::TimerBase::SharedPtr timer;
};

#endif // NODE_HPP