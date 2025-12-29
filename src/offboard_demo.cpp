#include "offboard_demo.hpp"

OffboardNode_Demo::OffboardNode_Demo() : OffboardControlNode() {
    RCLCPP_INFO(this->get_logger(), "OffboardNode_Demo started.");

}

void OffboardNode_Demo::control_loop() {
    auto now = this->now();
    if(start_fly()){
        switch (mod_) {
        case 0:
            target_position_ = {0.0, 0.0, 1.5};
            publish_setpoint();
            if (reached_target()) {
                RCLCPP_INFO(this->get_logger(), "已起飞至 %.2f 米", target_position_[2]);
                mod_ = 1;
            }
            break;
        case 1:
            target_position_ = {1, 0.0, 1.5};
            publish_setpoint();
            if (reached_target()) {
                RCLCPP_INFO(this->get_logger(), "mod = 2");
                mod_ = 2;
            }
            break;
        case 2:
            target_position_ = {1, 1, 1.5};
            publish_setpoint();
            if (reached_target()) {
                RCLCPP_INFO(this->get_logger(), "mod = 3");
                mod_ = 3;
            }
            break;
        case 3:
            publish_velocity(0.0, 0.0, -0.3);
            if (current_position_[2] < 0.5 && std::abs(current_velocity_[2]) < 0.05) {
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
            publish_setpoint();
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OffboardNode_Demo>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
