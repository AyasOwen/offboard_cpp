#include "offboard_demo.hpp"

OffboardDemoNode::OffboardDemoNode() : Node("offboard_demo_node") {
    // 初始化发布者
    cmd_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        "/offboard/cmd", 10);
    cmd_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
        "/offboard/cmd_mode", 10);
    takeoff_land_pub_ = this->create_publisher<std_msgs::msg::UInt8>(
        "/offboard/takeoff_land", 10);
    
    // 初始化订阅者
    auto qos_px4 = rclcpp::QoS(rclcpp::KeepLast(1))
                       .best_effort()
                       .durability_volatile();
    odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
        "/fmu/out/vehicle_odometry", qos_px4,
        std::bind(&OffboardDemoNode::odomCallback, this, std::placeholders::_1));

    trigger_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/offboard/trigger", qos_px4,
        std::bind(&OffboardDemoNode::triggerodomCallback, this, std::placeholders::_1));
    
    // 初始化状态
    mission_state_ = MissionState::IDLE;
    state_start_time_ = this->now();
    
    // 定义航点 (NED坐标系: 北-东-地, z为负表示向上)
    // 方形飞行路径
    waypoints_ = {
        {2.0, 0.0, -2.0, 0.0},      // 航点1: 向北2米
        {2.0, 2.0, -2.0, M_PI/2},   // 航点2: 向东2米
        {0.0, 2.0, -2.0, M_PI},     // 航点3: 向南2米
        {0.0, 0.0, -2.0, -M_PI/2}   // 航点4: 向西2米，回到起点
    };
    
    // 创建定时器 (10Hz)
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&OffboardDemoNode::timerCallback, this));
    
    RCLCPP_INFO(this->get_logger(), "Offboard Demo Node 启动");
    RCLCPP_INFO(this->get_logger(), "等待 5 秒后开始任务...");
}

void OffboardDemoNode::timerCallback() {
    auto now = this->now();
    double elapsed = (now - state_start_time_).seconds();
    if (trigger_){
        switch (mission_state_) {
            case MissionState::IDLE: {
                // 等待5秒让系统初始化
                if (elapsed > 5.0) {
                    RCLCPP_INFO(this->get_logger(), "开始起飞!");
                    publishTakeoffCommand();
                    mission_state_ = MissionState::TAKEOFF;
                    state_start_time_ = now;
                }
                break;
            }
            
            case MissionState::TAKEOFF: {
                // 持续发送起飞命令和位置控制模式
                publishControlMode(true, false, false);
                publishCommand(0.0, 0.0, takeoff_height_, 0.0);  // 起飞到1.5米高度
                
                // 检查是否到达起飞高度
                if (position_received_ && reachedTarget(0.0, 0.0, takeoff_height_)) {
                    RCLCPP_INFO(this->get_logger(), "起飞完成，开始飞向航点1");
                    mission_state_ = MissionState::WAYPOINT_1;
                    state_start_time_ = now;
                } 
                else if (elapsed > 15.0) {
                    // 超时保护
                    RCLCPP_WARN(this->get_logger(), "起飞超时，强制进入下一状态");
                    mission_state_ = MissionState::WAYPOINT_1;
                    state_start_time_ = now;
                }
                break;
            }
            
            case MissionState::WAYPOINT_1: {
                auto& wp = waypoints_[0];
                publishControlMode(true, false, false);
                publishCommand(wp[0], wp[1], wp[2], wp[3]);
                
                if (position_received_ && reachedTarget(wp[0], wp[1], wp[2])) {
                    RCLCPP_INFO(this->get_logger(), "到达航点1，飞向航点2");
                    mission_state_ = MissionState::WAYPOINT_2;
                    state_start_time_ = now;
                } 
                else if (elapsed > 15.0) {
                    RCLCPP_WARN(this->get_logger(), "航点1超时，强制进入下一状态");
                    mission_state_ = MissionState::WAYPOINT_2;
                    state_start_time_ = now;
                }
                break;
            }
            
            case MissionState::WAYPOINT_2: {
                auto& wp = waypoints_[1];
                publishControlMode(true, false, false);
                publishCommand(wp[0], wp[1], wp[2], wp[3]);
                
                if (position_received_ && reachedTarget(wp[0], wp[1], wp[2])) {
                    RCLCPP_INFO(this->get_logger(), "到达航点2，飞向航点3");
                    mission_state_ = MissionState::WAYPOINT_3;
                    state_start_time_ = now;
                } 
                else if (elapsed > 15.0) {
                    RCLCPP_WARN(this->get_logger(), "航点2超时，强制进入下一状态");
                    mission_state_ = MissionState::WAYPOINT_3;
                    state_start_time_ = now;
                }
                break;
            }
            
            case MissionState::WAYPOINT_3: {
                auto& wp = waypoints_[2];
                publishControlMode(true, false, false);
                publishCommand(wp[0], wp[1], wp[2], wp[3]);
                
                if (position_received_ && reachedTarget(wp[0], wp[1], wp[2])) {
                    RCLCPP_INFO(this->get_logger(), "到达航点3，飞向航点4");
                    mission_state_ = MissionState::WAYPOINT_4;
                    state_start_time_ = now;
                } 
                else if (elapsed > 15.0) {
                    RCLCPP_WARN(this->get_logger(), "航点3超时，强制进入下一状态");
                    mission_state_ = MissionState::WAYPOINT_4;
                    state_start_time_ = now;
                }
                break;
            }
            
            case MissionState::WAYPOINT_4: {
                auto& wp = waypoints_[3];
                publishControlMode(true, false, false);
                publishCommand(wp[0], wp[1], wp[2], wp[3]);
                
                if (position_received_ && reachedTarget(wp[0], wp[1], wp[2])) {
                    RCLCPP_INFO(this->get_logger(), "完成方形飞行，准备降落");
                    mission_state_ = MissionState::LAND;
                    state_start_time_ = now;
                } 
                else if (elapsed > 15.0) {
                    RCLCPP_WARN(this->get_logger(), "航点4超时，强制进入降落状态");
                    mission_state_ = MissionState::LAND;
                    state_start_time_ = now;
                }
                break;
            }

            case MissionState::RETURN_HOME: {
                publishControlMode(true, false, false);
                publishCommand(0.0, 0.0, takeoff_height_, 0.0);

                if (position_received_ && reachedTarget(0.0, 0.0, takeoff_height_)) {
                    RCLCPP_INFO(this->get_logger(), "返回起点完成，准备降落");
                    mission_state_ = MissionState::LAND;
                    state_start_time_ = now;
                } else if (elapsed > 15.0) {
                    RCLCPP_WARN(this->get_logger(), "返回起点超时，强制进入降落状态");
                    mission_state_ = MissionState::LAND;
                    state_start_time_ = now;
                }
                break;
            }
            
            case MissionState::LAND: {
                RCLCPP_INFO(this->get_logger(), "发送降落命令");
                publishLandCommand();
                mission_state_ = MissionState::DONE;
                state_start_time_ = now;
                break;
            }
            
            case MissionState::DONE: {
                // 任务完成，停止发布
                if (elapsed < 2.0) {
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                        "任务完成！");
                }
                break;
            }
        }
    }
}

void OffboardDemoNode::publishCommand(double x, double y, double z, double yaw) {
    px4_msgs::msg::TrajectorySetpoint msg;
    msg.timestamp = this->now().nanoseconds() / 1000;
    
    msg.position[0] = static_cast<float>(x);
    msg.position[1] = static_cast<float>(y);
    msg.position[2] = static_cast<float>(z);
    
    msg.velocity[0] = NAN;
    msg.velocity[1] = NAN;
    msg.velocity[2] = NAN;
    
    msg.acceleration[0] = NAN;
    msg.acceleration[1] = NAN;
    msg.acceleration[2] = NAN;
    
    msg.jerk[0] = NAN;
    msg.jerk[1] = NAN;
    msg.jerk[2] = NAN;
    
    msg.yaw = static_cast<float>(yaw);
    msg.yawspeed = NAN;
    
    cmd_pub_->publish(msg);
}

void OffboardDemoNode::publishControlMode(bool position, bool velocity, bool acceleration) {
    px4_msgs::msg::OffboardControlMode msg;
    msg.timestamp = this->now().nanoseconds() / 1000;
    msg.position = position;
    msg.velocity = velocity;
    msg.acceleration = acceleration;
    msg.attitude = false;
    msg.body_rate = false;
    
    cmd_mode_pub_->publish(msg);
}

void OffboardDemoNode::publishTakeoffCommand() {
    std_msgs::msg::UInt8 msg;
    msg.data = 1;  // 1 = takeoff
    takeoff_land_pub_->publish(msg);
}

void OffboardDemoNode::publishLandCommand() {
    std_msgs::msg::UInt8 msg;
    msg.data = 2;  // 2 = land
    takeoff_land_pub_->publish(msg);
}

void OffboardDemoNode::odomCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
    // 更新当前位置
    current_position_[0] = msg->position[0];
    current_position_[1] = msg->position[1];
    current_position_[2] = msg->position[2];
    position_received_ = true;
}

void OffboardDemoNode::triggerodomCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    trigger_ = msg->data;
}

bool OffboardDemoNode::reachedTarget(double target_x, double target_y, double target_z) {
    // 计算当前位置与目标位置的距离
    double dx = current_position_[0] - target_x;
    double dy = current_position_[1] - target_y;
    double dz = current_position_[2] - target_z;
    double distance = std::sqrt(dx*dx + dy*dy + dz*dz);
    
    // 打印当前距离（用于调试）
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "当前位置: (%.2f, %.2f, %.2f), 目标: (%.2f, %.2f, %.2f), 距离: %.2fm",
        current_position_[0], current_position_[1], current_position_[2],
        target_x, target_y, target_z, distance);
    
    return distance < position_threshold_;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<OffboardDemoNode>();
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}

