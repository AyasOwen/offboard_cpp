#include "offboard_demo.hpp"

OffboardDemoNode::OffboardDemoNode() : Node("offboard_demo_node") {
    // 初始化订阅者
    auto qos_px4 = rclcpp::QoS(rclcpp::KeepLast(1))
                       .best_effort()
                       .durability_volatile();
    
    // 初始化发布者
    cmd_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        "/offboard/cmd", qos_px4);
    cmd_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
        "/offboard/cmd_mode", qos_px4);
    takeoff_land_pub_ = this->create_publisher<std_msgs::msg::UInt8>(
        "/offboard/takeoff_land", qos_px4);
    
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
        switch (mission_state_) {
            case MissionState::IDLE: {
                // 等待5秒让系统初始化后，发送起飞命令
                if (!takeoff_command_sent_ && elapsed > 5.0){
                    RCLCPP_INFO(this->get_logger(), "开始起飞!");
                    publishTakeoffCommand();
                    takeoff_command_sent_ = true;
                    takeoff_start_z_ = current_position_[2];
                    takeoff_stable_started_ = false;
                    state_start_time_ = now;
                }

                // 起飞完成检测：速度逼近静止并持续一段时间
                if (takeoff_command_sent_){
                    publishTakeoffCommand();

                    const bool altitude_changed = std::abs(current_position_[2] - takeoff_start_z_) > 0.3;
                    if (position_received_ && altitude_changed && isVelocityNearZero()){
                        if (!takeoff_stable_started_){
                            takeoff_stable_start_ = now;
                            takeoff_stable_started_ = true;
                        }
                        else if ((now - takeoff_stable_start_).seconds() > stable_time_){
                            RCLCPP_INFO(this->get_logger(), "起飞完成（速度稳定），进入航点任务");
                            mission_state_ = MissionState::WAYPOINT_1;
                            state_start_time_ = now;
                        }
                    }
                    else{
                        takeoff_stable_started_ = false;
                    }
                }
                break;
            }
            
            case MissionState::WAYPOINT_1: {
                if (trigger_){
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
                }
                break;
            }
            
            case MissionState::WAYPOINT_2: {
                if (trigger_){
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
                }
                break;
            }
            
            case MissionState::WAYPOINT_3: {
                if (trigger_){
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
                }
                break;
            }
            
            case MissionState::WAYPOINT_4: {
                if (trigger_){
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
                }
                break;
            }

            case MissionState::RETURN_HOME: {
                if (trigger_){
                    publishControlMode(true, false, false);
                    publishCommand(0.0, 0.0, takeoff_height_, 0.0);

                    if (position_received_ && reachedTarget(0.0, 0.0, takeoff_height_)) {
                        RCLCPP_INFO(this->get_logger(), "返回起点完成，准备降落");
                        mission_state_ = MissionState::LAND;
                        state_start_time_ = now;
                    } 
                    else if (elapsed > 15.0) {
                        RCLCPP_WARN(this->get_logger(), "返回起点超时，强制进入降落状态");
                        mission_state_ = MissionState::LAND;
                        state_start_time_ = now;
                    }
                }
                break;
            }
            
            case MissionState::LAND: {
                publishLandCommand();
                const bool near_ground = position_received_ && std::abs(current_position_[2]) < 0.15;

                if (near_ground && isVelocityNearZero()){
                    if (!land_stable_started_){
                        land_stable_start_ = now;
                        land_stable_started_ = true;
                    }
                    else if ((now - land_stable_start_).seconds() > stable_time_){
                        RCLCPP_INFO(this->get_logger(), "降落完成（速度稳定）");
                        mission_state_ = MissionState::DONE;
                        state_start_time_ = now;
                    }
                }
                else{
                    land_stable_started_ = false;
                }
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

    // 更新当前速度
    current_velocity_[0] = msg->velocity[0];
    current_velocity_[1] = msg->velocity[1];
    current_velocity_[2] = msg->velocity[2];

    position_received_ = true;
}

void OffboardDemoNode::triggerodomCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    trigger_ = msg->data;
}

bool OffboardDemoNode::reachedTarget(double target_x, double target_y, double target_z) {
    // 计算误差
    double dx = current_position_[0] - target_x;
    double dy = current_position_[1] - target_y;
    double dz = current_position_[2] - target_z;

    double dist_sq = dx*dx + dy*dy + dz*dz;
    bool reached = dist_sq < (position_threshold_ * position_threshold_);

    if (reached) {
        RCLCPP_INFO(this->get_logger(), "目标到达！误差: %.2fm", std::sqrt(dist_sq));
    }

    return reached;
}

bool OffboardDemoNode::isVelocityNearZero() const {
    const double vx = current_velocity_[0];
    const double vy = current_velocity_[1];
    const double vz = current_velocity_[2];
    const double vel_sq = vx * vx + vy * vy + vz * vz;
    return vel_sq < (velocity_threshold_ * velocity_threshold_);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<OffboardDemoNode>();
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}

