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

    // 订阅器
    rclcpp::Subscription<px4_msgs::msg::RcChannels>::SharedPtr rc_sub;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr state_sub;
    rclcpp::Subscription<px4_msgs::msg::TrajectorySetpoint>::SharedPtr offboard_sub;
    rclcpp::Subscription<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_sub;
    rclcpp::Subscription<px4_msgs::msg::BatteryStatus>::SharedPtr battery_sub;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr takeoff_land_sub;
    rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr land_detected_sub;

    // 定时器
    rclcpp::TimerBase::SharedPtr timer;
};

#endif // NODE_HPP