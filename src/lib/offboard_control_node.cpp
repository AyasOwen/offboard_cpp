#include "lib/offboard_control_node.hpp" // 包含自己定义的头文件

using namespace std::chrono_literals; // 可以在 .cpp 文件中安全地使用

OffboardControlNode::OffboardControlNode() : Node("offboard_control_node") {
    current_state_ = mavros_msgs::msg::State();
    current_position_.fill(0.0);
    current_velocity_.fill(0.0);
    target_position_ = {0.0, 0.0, 0.0};
    setpoint_counter_ = 0;
    mod_ = 0;

    state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
        "mavros/state", 10, std::bind(&OffboardControlNode::state_cb, this, std::placeholders::_1));

    pos_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "mavros/local_position/pose", rclcpp::SensorDataQoS(), std::bind(&OffboardControlNode::pos_cb, this, std::placeholders::_1));

    vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        "mavros/local_position/velocity_local", rclcpp::SensorDataQoS(), std::bind(&OffboardControlNode::vel_cb, this, std::placeholders::_1));

    local_pos_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("mavros/setpoint_position/local", 10);
    vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 10);
    mod_pub_ = this->create_publisher<std_msgs::msg::UInt8>("px4/mod", 10);

    arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");
    mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");
    set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");

    last_request_ = this->now();
    timer_ = this->create_wall_timer(50ms, std::bind(&OffboardControlNode::control_loop, this));
    RCLCPP_INFO(this->get_logger(), "OffboardControlNode started.");
}

void OffboardControlNode::state_cb(const mavros_msgs::msg::State::SharedPtr msg) {
    current_state_ = *msg;
}

void OffboardControlNode::pos_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    current_position_ = {msg->pose.position.x, msg->pose.position.y, msg->pose.position.z};
}

void OffboardControlNode::vel_cb(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
    current_velocity_ = {msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z};
}

void OffboardControlNode::publish_setpoint(double yaw) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = this->now();
    pose.pose.position.x = target_position_[0];
    pose.pose.position.y = target_position_[1];
    pose.pose.position.z = target_position_[2];

    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();

    local_pos_pub_->publish(pose);
}

void OffboardControlNode::publish_velocity(double vx, double vy, double vz) {
    geometry_msgs::msg::TwistStamped twist;
    twist.header.stamp = this->now();
    twist.twist.linear.x = vx;
    twist.twist.linear.y = vy;
    twist.twist.linear.z = vz;
    vel_pub_->publish(twist);
}

bool OffboardControlNode::reached_target(double pos_tol) {
    for (int i = 0; i < 3; ++i) {
        if (std::abs(current_position_[i] - target_position_[i]) > pos_tol ||
            std::abs(current_velocity_[i]) > 0.09)
            return false;
    }
    return true;
}

void OffboardControlNode::control_loop() {
    auto now = this->now();

    // 1. 检查是否连接飞控
    if (!current_state_.connected) {
        RCLCPP_WARN(this->get_logger(), "等待飞控连接...");
        return;
    }

    // 2. 切 OFFBOARD 前，先预热 setpoint
    if (setpoint_counter_ < 100) {
        publish_setpoint();  // 不断发布当前位置 setpoint
        setpoint_counter_++;
        if (setpoint_counter_ == 100) {
            RCLCPP_INFO(this->get_logger(), "已完成 setpoint 预热，准备进入 OFFBOARD 模式");
        }
        return;  // 直接返回，不执行状态机
    }

    // 3. 若进入 OFFBOARD 模式
    if (current_state_.mode == "OFFBOARD") {
        // 3.1 若未解锁，每隔5秒尝试一次
        if (!current_state_.armed &&
            (now - last_request_).seconds() > 5.0) {
            if (arming_client_->service_is_ready()) {
                auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
                req->value = true;
                arming_client_->async_send_request(req);
                RCLCPP_INFO(this->get_logger(), "请求解锁...");
                last_request_ = now;
            } else {
                RCLCPP_WARN(this->get_logger(), "解锁服务未就绪，等待...");
            }
            return;
        }

        // 3.2 若已解锁，开始执行任务状态机
        if (1) {
        RCLCPP_WARN(this->get_logger(), "解锁，等待...");
            switch (mod_) {
                case 0:
                    target_position_ = {0.0, 0.0, 1.2};
                    publish_setpoint();
                    if (reached_target()) {
                        RCLCPP_INFO(this->get_logger(), "已起飞至 %.2f 米", target_position_[2]);
                        mod_ = 1;
                        publish_mod();
                    }
                    break;

                case 1:
                    target_position_ = {0.3, 0.0, 1.2};
                    publish_setpoint();
                    if (reached_target()) {
                        RCLCPP_INFO(this->get_logger(), "mod = 2");
                        mod_ = 2;
                        publish_mod();
                    }
                    break;

                case 2:
                    target_position_ = {0.3, 0.3, 1.2};
                    publish_setpoint();
                    if (reached_target()) {
                        RCLCPP_INFO(this->get_logger(), "mod = 3");
                        mod_ = 3;
                        publish_mod();
                    }
                    break;

                case 3:
                    publish_velocity(0.0, 0.0, -0.3);
                    if (current_position_[2] < 0.5 &&
                        std::abs(current_velocity_[2]) < 0.05) {
                        RCLCPP_INFO(this->get_logger(), "即将降落...");
                        mod_ = 4;
                        publish_mod();
                    }
                    break;

                case 4: {
                    land();
                    mod_ = 5;
                    publish_mod();
                    break;
                }

                case 5:
                    // 任务完成
                    break;

                default:
                    break;
            }

            // 在前3阶段持续发布 setpoint
            if (mod_ < 3) {
                publish_setpoint();
            }
        }
    }
}


void OffboardControlNode::land(){
    auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    req->custom_mode = "AUTO.LAND";
    mode_client_->async_send_request(req);
    RCLCPP_INFO(this->get_logger(), "已请求 AUTO.LAND");
}

bool OffboardControlNode::start_fly(){
    auto now = this->now();

    if (!current_state_.connected)
    {
        RCLCPP_WARN(this->get_logger(), "等待飞控连接...");
        return false;
    }

    if (setpoint_counter_ < 100)
    {
        publish_setpoint();
        setpoint_counter_++;
        return false;
    }

    if (!current_state_.armed &&
        (now - last_request_).seconds() > 5.0 && current_state_.mode == "OFFBOARD")
    {
        auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        req->value = true;
        arming_client_->async_send_request(req);
        RCLCPP_INFO(this->get_logger(), "请求解锁...");
        last_request_ = now;
        return false;
    }
    if (!current_state_.armed)
        return false;
    
    return true;
}

void OffboardControlNode::publish_mod()
{
    std_msgs::msg::UInt8 msg;
    msg.data = static_cast<uint8_t>(mod_);
    mod_pub_->publish(msg);
}