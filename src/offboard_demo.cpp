#include "offboard_demo.hpp"

OffboardNode_Demo::OffboardNode_Demo() : OffboardControlNode() {
    RCLCPP_INFO(this->get_logger(), "OffboardNode_Demo started.");
}

void OffboardNode_Demo::control_loop() {
    auto now = this->now();
    if(start_fly()){
        switch (mod_) {
        case 0:
            target_position_ = {0.0, 0.0, -1.5};  // 上升到1.5米（NED坐标系）
            publish_offboard_control_mode();
            publish_trajectory_setpoint();
            if (reached_target()) {
                RCLCPP_INFO(this->get_logger(), "已起飞至 %.2f 米", -target_position_[2]);
                mod_ = 1;
            }
            break;
        case 1:
            target_position_ = {1, 0.0, -1.5};  // 移动到北方1米
            publish_offboard_control_mode();
            publish_trajectory_setpoint();
            if (reached_target()) {
                RCLCPP_INFO(this->get_logger(), "mod = 2");
                mod_ = 2;
            }
            break;
        case 2:
            target_position_ = {1, 1, -1.5};  // 移动到东方1米
            publish_offboard_control_mode();
            publish_trajectory_setpoint();
            if (reached_target()) {
                RCLCPP_INFO(this->get_logger(), "mod = 3");
                mod_ = 3;
            }
            break;
        case 3:
            publish_offboard_control_mode(false, true);
            publish_velocity_setpoint(0.0, 0.0, 0.3);  // 向下（下降）速度0.3 m/s
            if (current_position_[2] > 0.5 && std::abs(current_velocity_[2]) < 0.05) {  // Z > 0.5米表示离地小于0.5米
                RCLCPP_INFO(this->get_logger(), "即将降落...");
                mod_ = 4;
            }
            break;
        case 4: {
            land();
            mod_ = 5;
            break;
        }
        default:
            break;
        }

        if (mod_ < 3)
            publish_offboard_control_mode();
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OffboardNode_Demo>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
