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
        std::bind(&OffboardDemoNode::triggerCallback, this, std::placeholders::_1));
    
    // 初始化状态
    state_start_time_ = now();
    
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

void OffboardDemoNode::timerCallback(){
    switch (mission_state_){
        case MissionState::IDLE:
            handleIdle();
            break;

        case MissionState::WAYPOINT:
            handleWaypoint();
            break;

        case MissionState::RETURN_HOME:
            handleReturnHome();
            break;

        case MissionState::LAND:
            handleLand();
            break;

        case MissionState::DONE:
            break;
    }
}

void OffboardDemoNode::changeState(MissionState new_state) {
    mission_state_ = new_state;
    state_start_time_ = now();
    stable_started_ = false;

    RCLCPP_INFO(get_logger(), "State changed");
}

void OffboardDemoNode::handleIdle() {
    double elapsed = (now() - state_start_time_).seconds();

    if (!takeoff_sent_ && elapsed > 3.0){
        publishTakeoffCommand();
        takeoff_sent_ = true;
    }

    if (position_received_ && isVelocityNearZero()){
        if (!stable_started_)
        {
            stable_start_time_ = now();
            stable_started_ = true;
        }
        else if ((now() - stable_start_time_).seconds() > stable_time_){
            current_wp_index_ = 0;
            changeState(MissionState::WAYPOINT);
        }
    }
}

void OffboardDemoNode::handleWaypoint(){
    if (!trigger_) return;

    auto &wp = waypoints_[current_wp_index_];

    publishControlMode();
    publishCommand(wp[0], wp[1], wp[2], wp[3]);

    if (position_received_ && reachedTarget(wp[0], wp[1], wp[2])){
        RCLCPP_INFO(get_logger(), "Reached waypoint %ld", current_wp_index_ + 1);

        current_wp_index_++;

        if (current_wp_index_ >= waypoints_.size()){
            changeState(MissionState::RETURN_HOME);
        }
    }
}

void OffboardDemoNode::handleReturnHome() {
    publishControlMode();
    publishCommand(0.0, 0.0, takeoff_height_, 0.0);

    if (position_received_ && reachedTarget(0.0, 0.0, takeoff_height_)) {
        changeState(MissionState::LAND);
    }
}

void OffboardDemoNode::handleLand(){
    publishLandCommand();

    if (position_received_ &&
        std::abs(current_position_[2]) < 0.15 &&
        isVelocityNearZero()) {
        changeState(MissionState::DONE);
        RCLCPP_INFO(get_logger(), "Mission Completed");
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

void OffboardDemoNode::triggerCallback(const std_msgs::msg::Bool::SharedPtr msg) {
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

