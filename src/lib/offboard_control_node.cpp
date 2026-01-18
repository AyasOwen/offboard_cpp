#include "lib/offboard_control_node.hpp"

using namespace std::chrono_literals;
using namespace px4_msgs::msg;

OffboardControlNode::OffboardControlNode() : Node("offboard_control_node") {
    current_state_ = VehicleStatus();
    current_position_.fill(0.0);
    current_velocity_.fill(0.0);
    target_position_ = {0.0, 0.0, 0.0};
    setpoint_counter_ = 0;
    mod_ = 0;
    armed_ = false;
    in_offboard_mode_ = false;
    timestamp_ = 0;

    // PX4 的 /fmu/out 话题在不同环境里 durability 可能是 VOLATILE。
    // 如果订阅端强制使用 TRANSIENT_LOCAL，而发布端是 VOLATILE，会直接 QoS 不兼容 -> 收不到任何状态/里程计。
    // 这会导致本节点一直认为“未进入OFFBOARD/未解锁”，看起来就像卡在 arm。
    auto qos_px4 = rclcpp::QoS(rclcpp::KeepLast(10))
                       .best_effort()
                       .durability_volatile();

    state_sub_ = this->create_subscription<VehicleStatus>(
        "/fmu/out/vehicle_status", qos_px4, std::bind(&OffboardControlNode::state_cb, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<VehicleOdometry>(
        "/fmu/out/vehicle_odometry", qos_px4, std::bind(&OffboardControlNode::odom_cb, this, std::placeholders::_1));

    offboard_control_mode_pub_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    trajectory_setpoint_pub_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
    vehicle_command_pub_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
    mod_pub_ = this->create_publisher<std_msgs::msg::UInt8>("px4/mod", 10);

    last_request_ = this->now();
    // 用 lambda 调用虚函数，确保派生类 override 的 control_loop() 会被正确调用
    timer_ = this->create_wall_timer(50ms, [this]() { this->control_loop(); });
    RCLCPP_INFO(this->get_logger(), "OffboardControlNode started.");
}

void OffboardControlNode::state_cb(const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
    current_state_ = *msg;
    armed_ = (msg->arming_state == VehicleStatus::ARMING_STATE_ARMED);
    in_offboard_mode_ = (msg->nav_state == VehicleStatus::NAVIGATION_STATE_OFFBOARD);

    RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "vehicle_status rx: nav_state=%u arming_state=%u (armed=%d offboard=%d)",
        (unsigned)msg->nav_state, (unsigned)msg->arming_state, (int)armed_, (int)in_offboard_mode_);
}

void OffboardControlNode::odom_cb(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
    current_position_ = {msg->position[0], msg->position[1], msg->position[2]};
    current_velocity_ = {msg->velocity[0], msg->velocity[1], msg->velocity[2]};
    timestamp_ = msg->timestamp;

    RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "vehicle_odometry rx: t=%lu pos=[%.2f %.2f %.2f] vel=[%.2f %.2f %.2f]",
        (unsigned long)msg->timestamp,
        (double)msg->position[0], (double)msg->position[1], (double)msg->position[2],
        (double)msg->velocity[0], (double)msg->velocity[1], (double)msg->velocity[2]);
}

void OffboardControlNode::publish_offboard_control_mode(bool position, bool velocity) {
    OffboardControlMode msg{};
    msg.position = position;
    msg.velocity = velocity;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    offboard_control_mode_pub_->publish(msg);
}

void OffboardControlNode::publish_trajectory_setpoint(double yaw) {
    TrajectorySetpoint msg{};
    msg.position = {(float)target_position_[0], (float)target_position_[1], (float)target_position_[2]};
    // 位置控制时，把不使用的字段设为 NaN，避免 PX4 认为这些字段也是有效指令
    const float nan = std::numeric_limits<float>::quiet_NaN();
    msg.velocity = {nan, nan, nan};
    msg.acceleration = {nan, nan, nan};
    msg.jerk = {nan, nan, nan};
    msg.yawspeed = nan;
    msg.yaw = (float)yaw;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_pub_->publish(msg);
}

void OffboardControlNode::publish_velocity_setpoint(double vx, double vy, double vz) {
    TrajectorySetpoint msg{};
    const float nan = std::numeric_limits<float>::quiet_NaN();
    msg.position = {nan, nan, nan};
    msg.velocity = {(float)vx, (float)vy, (float)vz};
    msg.acceleration = {nan, nan, nan};
    msg.jerk = {nan, nan, nan};
    msg.yaw = nan;
    msg.yawspeed = nan;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_pub_->publish(msg);
}

bool OffboardControlNode::reached_target(double pos_tol) {
    for (int i = 0; i < 3; ++i) {
        if (std::abs(current_position_[i] - target_position_[i]) > pos_tol ||
            std::abs(current_velocity_[i]) > 0.09)
            return false;
    }
    return true;
}

void OffboardControlNode::publish_vehicle_command(uint16_t command, float param1, float param2) {
    VehicleCommand msg{};
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    vehicle_command_pub_->publish(msg);
}

void OffboardControlNode::arm() {
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
    RCLCPP_INFO(this->get_logger(), "Arm command sent");
}

void OffboardControlNode::disarm() {
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
    RCLCPP_INFO(this->get_logger(), "Disarm command sent");
}

void OffboardControlNode::switch_to_offboard() {
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
    RCLCPP_INFO(this->get_logger(), "Switch to OFFBOARD mode");
}

void OffboardControlNode::control_loop() {
    auto now = this->now();

    // 1. 检查是否收到飞控状态（通过检查是否有有效的里程表数据）
    static bool received_first_odometry = false;
    if (!received_first_odometry) {
        // 首次需要收到里程表数据
        if (timestamp_ == 0) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                "等待飞控连接... (检查MicroXRCEDDS Agent是否启动)");
            return;
        }
        received_first_odometry = true;
        RCLCPP_INFO(this->get_logger(), "飞控已连接，timestamp: %lu", timestamp_);
    }

    // 2. 切 OFFBOARD 前，先预热 setpoint
    if (setpoint_counter_ < 100) {
        publish_offboard_control_mode();
        publish_trajectory_setpoint();  // 不断发布当前位置 setpoint
        setpoint_counter_++;
        if (setpoint_counter_ == 100) {
            RCLCPP_INFO(this->get_logger(), "已完成 setpoint 预热，准备进入 OFFBOARD 模式");
        }
        return;  // 直接返回，不执行状态机
    }

    // 3. 尝试进入 OFFBOARD 模式（持续发送setpoint）
    if (!in_offboard_mode_) {
        publish_offboard_control_mode();  // ⭐ 持续发送
        publish_trajectory_setpoint();      // ⭐ 持续发送
        if ((now - last_request_).seconds() > 5.0) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "尝试切换到 OFFBOARD 模式");
            switch_to_offboard();
            last_request_ = now;
        }
        return;  // 等待进入offboard模式
    }

    // 4. 若进入 OFFBOARD 模式，尝试解锁（持续发送setpoint）
    if (!armed_) {
        publish_offboard_control_mode();  // ⭐ 持续发送
        publish_trajectory_setpoint();      // ⭐ 持续发送
        if ((now - last_request_).seconds() > 2.0) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "尝试解锁");
            arm();
            last_request_ = now;
        }
        return;  // 等待解锁
    }

    // 5. 已解锁，开始执行任务状态机
    if (armed_ && in_offboard_mode_) {
        RCLCPP_INFO_ONCE(this->get_logger(), "已进入OFFBOARD模式并解锁，执行任务...");
            switch (mod_) {
                case 0:
                    target_position_ = {0.0, 0.0, -1.2};  // 上升到1.2米（NED坐标系）
                    publish_offboard_control_mode();
                    publish_trajectory_setpoint();
                    if (reached_target()) {
                        RCLCPP_INFO(this->get_logger(), "已起飞至 %.2f 米", -target_position_[2]);
                        mod_ = 1;
                        publish_mod();
                    }
                    break;

                case 1:
                    target_position_ = {0.3, 0.0, -1.2};  // 移动到北方0.3米
                    publish_offboard_control_mode();
                    publish_trajectory_setpoint();
                    if (reached_target()) {
                        RCLCPP_INFO(this->get_logger(), "mod = 2");
                        mod_ = 2;
                        publish_mod();
                    }
                    break;

                case 2:
                    target_position_ = {0.3, 0.3, -1.2};  // 移动到东方0.3米
                    publish_offboard_control_mode();
                    publish_trajectory_setpoint();
                    if (reached_target()) {
                        RCLCPP_INFO(this->get_logger(), "mod = 3");
                        mod_ = 3;
                        publish_mod();
                    }
                    break;

                case 3:
                    publish_offboard_control_mode(false, true);
                    publish_velocity_setpoint(0.0, 0.0, 0.3);  // 向下（下降）速度0.3 m/s
                    if (current_position_[2] > 0.5 &&  // Z > 0.5米表示离地面小于0.5米
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

            // 在前3阶段持续发布 offboard control mode
            if (mod_ < 3) {
                publish_offboard_control_mode();
            }
    }
}


void OffboardControlNode::land(){
    auto req = std::make_shared<VehicleCommand>();
    req->command = VehicleCommand::VEHICLE_CMD_NAV_LAND;
    req->param1 = 0;
    req->param2 = 0;
    req->target_system = 1;
    req->target_component = 1;
    req->source_system = 1;
    req->source_component = 1;
    req->from_external = true;
    req->timestamp = this->get_clock()->now().nanoseconds() / 1000;
    vehicle_command_pub_->publish(*req);
    RCLCPP_INFO(this->get_logger(), "已请求降落");
}

bool OffboardControlNode::start_fly(){
    auto now = this->now();

    // 1. 检查是否收到飞控状态
    static bool received_first_odometry = false;
    if (!received_first_odometry) {
        if (timestamp_ == 0) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                "等待飞控连接... (检查MicroXRCEDDS Agent是否启动)");
            return false;
        }
        received_first_odometry = true;
        RCLCPP_INFO(this->get_logger(), "飞控已连接");
    }

    // 2. 预热setpoint
    if (setpoint_counter_ < 100)
    {
        publish_offboard_control_mode();
        publish_trajectory_setpoint();
        setpoint_counter_++;
        return false;
    }

    // 3. 切换到OFFBOARD模式（持续发送setpoint）
    if (!in_offboard_mode_) {
        publish_offboard_control_mode();  // ⭐ 持续发送
        publish_trajectory_setpoint();      // ⭐ 持续发送
        if ((now - last_request_).seconds() > 2.0) {
            switch_to_offboard();
            last_request_ = now;
        }
        return false;
    }

    // 4. 解锁（持续发送setpoint）
    if (!armed_) {
        publish_offboard_control_mode();  // ⭐ 持续发送
        publish_trajectory_setpoint();      // ⭐ 持续发送
        if ((now - last_request_).seconds() > 2.0) {
            arm();
            last_request_ = now;
        }
        return false;
    }
    
    return true;  // 已连接、已预热、已进入offboard、已解锁
}


void OffboardControlNode::publish_mod()
{
    std_msgs::msg::UInt8 msg;
    msg.data = static_cast<uint8_t>(mod_);
    mod_pub_->publish(msg);
}
