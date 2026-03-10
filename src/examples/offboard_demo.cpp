#include "examples/offboard_demo.hpp"

namespace {
uint16_t default_position_mask() {
    using PT = mavros_msgs::msg::PositionTarget;
    return PT::IGNORE_VX | PT::IGNORE_VY | PT::IGNORE_VZ |
           PT::IGNORE_AFX | PT::IGNORE_AFY | PT::IGNORE_AFZ |
           PT::IGNORE_YAW_RATE;
}
}

OffboardDemoNode::OffboardDemoNode() : Node("offboard_demo_node") {
    // 初始化订阅者
    auto qos_px4 = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();
    auto qos_cmd = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
    
    // 初始化发布者
    cmd_pub_ = this->create_publisher<mavros_msgs::msg::PositionTarget>(
        "offboard/cmd", qos_cmd);
    cmd_mode_pub_ = this->create_publisher<std_msgs::msg::UInt16>(
        "offboard/cmd_mode", qos_cmd);
    takeoff_land_pub_ = this->create_publisher<std_msgs::msg::UInt8>(
        "offboard/takeoff_land", qos_cmd);
    
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "mavros/local_position/odom", qos_px4,
        std::bind(&OffboardDemoNode::odomCallback, this, std::placeholders::_1));

    trigger_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "offboard/trigger", qos_px4,
        std::bind(&OffboardDemoNode::triggerCallback, this, std::placeholders::_1));
    
    // 初始化状态
    state_start_time_ = now();
    
    // 定义航点 (ENU坐标系: 东-北-天, z为正表示向上)
    // 方形飞行路径
    waypoints_ = {
        {0.0, 2.0, 2.0, M_PI/2},    // 航点1: 向北2米
        {2.0, 2.0, 2.0, 0.0},       // 航点2: 向东2米
        {2.0, 0.0, 2.0, -M_PI/2},   // 航点3: 向南2米
        {0.0, 0.0, 2.0, M_PI}       // 航点4: 向西2米，回到起点
    };
    
    // 创建定时器 (10Hz)
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&OffboardDemoNode::timerCallback, this));
    
    RCLCPP_INFO(this->get_logger(), "Offboard Demo Node 启动");
    RCLCPP_INFO(this->get_logger(), "等待 5 秒后开始任务...");
}

void OffboardDemoNode::timerCallback() {
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

    if (!takeoff_command_sent_ && elapsed > 5.0) {
        RCLCPP_INFO(get_logger(), "开始起飞");
        takeoff_start_z_ = current_position_[2];
        takeoff_command_sent_ = true;
        takeoff_stable_started_ = false;
    }

    if (takeoff_command_sent_) {
        publishTakeoffCommand();

        const bool altitude_changed =
            std::abs(current_position_[2] - takeoff_start_z_) > 0.3;

        if (position_received_ &&
            altitude_changed &&
            isVelocityNearZero()) {
            if (!takeoff_stable_started_) {
                stable_start_time_ = now();
                takeoff_stable_started_ = true;
            }
            else if ((now() - stable_start_time_).seconds() > stable_time_) {
                RCLCPP_INFO(get_logger(), "起飞完成");

                current_wp_index_ = 0;
                changeState(MissionState::WAYPOINT);
            }
        }
        else {
            takeoff_stable_started_ = false;
        }
    }
}

void OffboardDemoNode::handleWaypoint(){
    if (!trigger_) return;

    auto &wp = waypoints_[current_wp_index_];

    publishControlMode();
    publishCommand(wp[0], wp[1], wp[2], wp[3]);

    if (position_received_ && reachedTarget(wp[0], wp[1], wp[2])) {
        RCLCPP_INFO(get_logger(), "Reached waypoint %ld", current_wp_index_ + 1);

        current_wp_index_++;

        if (current_wp_index_ >= waypoints_.size()) {
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

void OffboardDemoNode::handleLand() {
    publishLandCommand();

    const bool near_ground =
        position_received_ &&
        std::abs(current_position_[2]) < 0.15;

    if (near_ground && isVelocityNearZero()) {
        if (!land_stable_started_) {
            stable_start_time_ = now();
            land_stable_started_ = true;
        }
        else if ((now() - stable_start_time_).seconds() > stable_time_) {
            RCLCPP_INFO(get_logger(), "降落完成（稳定）");
            changeState(MissionState::DONE);
        }
    }
    else {
        land_stable_started_ = false;
    }
}

void OffboardDemoNode::publishCommand(double x, double y, double z, double yaw) {
    mavros_msgs::msg::PositionTarget msg;
    msg.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_ENU;
    msg.type_mask = default_position_mask();
    msg.position.x = static_cast<float>(x);
    msg.position.y = static_cast<float>(y);
    msg.position.z = static_cast<float>(z);
    msg.yaw = static_cast<float>(yaw);
    cmd_pub_->publish(msg);
}

void OffboardDemoNode::publishControlMode(bool position, bool velocity, bool acceleration) {
    std_msgs::msg::UInt16 msg;
    using PT = mavros_msgs::msg::PositionTarget;
    uint16_t mask = 0;
    if (!position) mask |= PT::IGNORE_PX | PT::IGNORE_PY | PT::IGNORE_PZ;
    if (!velocity) mask |= PT::IGNORE_VX | PT::IGNORE_VY | PT::IGNORE_VZ;
    if (!acceleration) mask |= PT::IGNORE_AFX | PT::IGNORE_AFY | PT::IGNORE_AFZ;
    mask |= PT::IGNORE_YAW_RATE;
    msg.data = mask;
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

void OffboardDemoNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    const auto &p = msg->pose.pose.position;
    const auto &v = msg->twist.twist.linear;
    current_position_[0] = p.x;  // ENU East
    current_position_[1] = p.y;  // ENU North
    current_position_[2] = p.z;  // ENU Up

    current_velocity_[0] = v.x;
    current_velocity_[1] = v.y;
    current_velocity_[2] = v.z;

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

