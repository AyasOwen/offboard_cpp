#include "lib/input.hpp"

double normalize_angle(double angle) {
    while (angle > M_PI) {
        angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }
    return angle;
}


// RC_Data_t
// 初始化
RC_Data_t::RC_Data_t(const rclcpp::Node::SharedPtr& node) : node_(node){
    rcv_stamp =  node_ -> now();
    mode = -1.0;
    last_mode = -1.0;
    for (int i = 0; i < 4; ++i){
        ch[i] = 0.0;
    }
    is_hover_mode = false;
    enter_hover_mode = false;
}

// 检测 RC 是否合法
void RC_Data_t::check_validity(){
    if (mode >= -1.1 && mode <= 1.1 ){
        // pass
    }
    else{
        RCLCPP_ERROR(node_->get_logger(), "RC data validity check fail. mode=%f", mode);
    }
}

// 检测摇杆是否回正
bool RC_Data_t::check_centered(){
    return abs(ch[0]) < 1e-5 && abs(ch[1]) < 1e-5 && abs(ch[2]) < 1e-5 && abs(ch[3]) < 1e-5;
}

// 映射遥控器
void RC_Data_t::feed(mavros_msgs::msg::RCIn::SharedPtr pMsg, const Param_t& param){
    msg = *pMsg;
    rcv_stamp =  node_ -> now();

    if (msg.channels.size() < 10) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
            "RC channel count too low: %zu", msg.channels.size());
        return;
    }

    // 提取遥控器通道数据（根据实际通道映射调整）
    for(int i = 0; i < 4; i++){
        ch[i] = (static_cast<double>(msg.channels[i]) - 1500.0) / 500.0;
        // 对处于死区的数据进行处理
        // 防止对摇杆过度敏感
        if (ch[i] > DEAD_ZONE){
            ch[i] = (ch[i] - DEAD_ZONE) / (1 - DEAD_ZONE);
        }
        else if (ch[i] < - DEAD_ZONE){
            ch[i] = (ch[i] + DEAD_ZONE) / (1 - DEAD_ZONE);
        }
        else{
            ch[i] = 0.0;
        }
    }

    mode = (static_cast<double>(msg.channels[param.rc_debug.ch_mode]) - 1000.0) / 1000.0;
    gear = (static_cast<double>(msg.channels[param.rc_debug.ch_gear]) - 1000.0) / 1000.0;
    #ifdef TEXT_RC
        double mock_mode = 0.0;
        double mock_gear = 0.0;
        node_->get_parameter_or("mock_rc_mode", mock_mode, 0.0);
        node_->get_parameter_or("mock_rc_gear", mock_gear, 0.0);
        mode = mock_mode;
        gear = mock_gear;
    #endif
    // 这里归一到了 [0, 1] ，如有别的需求，自行进行修改
    p = (static_cast<double>(msg.channels[param.rc_debug.ch_p]) - 1000.0) / 1000.0;
    i = (static_cast<double>(msg.channels[param.rc_debug.ch_i]) - 1000.0) / 1000.0;
    d = (static_cast<double>(msg.channels[param.rc_debug.ch_d]) - 1000.0) / 1000.0;

    // 检测模式切换
    if (!have_init_last_mode) {
        last_mode = mode;
        have_init_last_mode = true;
    }

    if (!have_init_last_gear)
    {
        have_init_last_gear = true;
        last_gear = gear;
    }

    // 检测是否进入悬停模式
    if (mode > API_MODE_THRESHOLD_VALUE){
        if (last_mode < API_MODE_THRESHOLD_VALUE){
            enter_hover_mode = true;
        }
        else{
            enter_hover_mode = false;
        }
        is_hover_mode = true;
    }
    else{
        is_hover_mode = false;
    }

    // 只有在悬浮模式
    if (is_hover_mode)
    {
        if (last_gear < GEAR_SHIFT_VALUE && gear > GEAR_SHIFT_VALUE){
            enter_offboard = true;
        }
        else if (gear < GEAR_SHIFT_VALUE){
            enter_offboard = false;
        }
        if (gear > GEAR_SHIFT_VALUE){
            is_offboard = true;
        }
        else{
            is_offboard = false;
        }
    }

    last_mode = mode;
    last_gear = gear;
}

// Odom_Data_t
// 初始化
Odom_Data_t::Odom_Data_t(const rclcpp::Node::SharedPtr& node) : node_(node){
    rcv_stamp =  node_ -> now();
    q.setIdentity();
    recv_new_msg = false;
}

// 回调
void Odom_Data_t::feed(nav_msgs::msg::Odometry::SharedPtr pMsg, const Param_t& param){
    rcv_stamp =  node_ -> now();
    msg = *pMsg;
    recv_new_msg = true;
    bool is_first_msg = (rcv_stamp.nanoseconds() == 0);     // 判断是不是第一帧

    const auto &pos = msg.pose.pose.position;
    const auto &vel = msg.twist.twist.linear;
    const auto &ang_vel = msg.twist.twist.angular;
    const auto &quat = msg.pose.pose.orientation;

    // MAVROS local_position/odom is ENU; use ENU directly.
    Eigen::Vector3d new_p(pos.x, pos.y, pos.z);

    //  突变检查逻辑
    if (is_first_msg) {
        pos_jump = false; // 第一帧跳过检查
    } 
    else {
        if ((new_p - p).norm() > param.odom_pos_jump) {
            RCLCPP_ERROR(node_->get_logger(), "Odom 数据发生跳变!");
            pos_jump = true;
        } 
        else {
            pos_jump = false;
        }
    }

    // 更新 p 缓存值
    p = new_p;

    // 提取 v, q, w
    v << vel.x, vel.y, vel.z;

    const double yaw_enu = std::atan2(
        2.0 * (quat.w * quat.z + quat.x * quat.y),
        1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z));
    q = Eigen::AngleAxisd(yaw_enu, Eigen::Vector3d::UnitZ());

    w << ang_vel.x, ang_vel.y, ang_vel.z;

    // 处理机体坐标系速度，如果 Odom 里的速度是相对于机体坐标系的，需要旋转到世界坐标系
    #ifdef VEL_IN_BODY 
        v = q * v; 

        static int count = 0;
        if (count++ % 500 == 0) {
            RCLCPP_WARN(node_->get_logger(), "VEL_IN_BODY is enabled! Velocity has been rotated.");
        }
    #endif

    // 检查频率
    static int one_min_count = 9999;
    static rclcpp::Time last_clear_count_time = node_->now();;

    if ((rcv_stamp - last_clear_count_time).seconds() > 1.0) {
        if (one_min_count < 100 && last_clear_count_time.nanoseconds() != 0) {
            RCLCPP_WARN(node_->get_logger(), "Odom frequency too low: %d Hz", one_min_count);
        }
        one_min_count = 0;
        last_clear_count_time = rcv_stamp;
    }
    one_min_count++;
}

// State_Data_t
// 初始化
State_Data_t::State_Data_t(const rclcpp::Node::SharedPtr& node) : node_(node){
}

// 记录当前模式
void State_Data_t::feed(mavros_msgs::msg::State::SharedPtr pMsg){
    current_state = *pMsg;
}

// OffboardMode_Data_t
// 初始化
Offboard_Data_t::Offboard_Data_t(const rclcpp::Node::SharedPtr& node) : node_(node){
    rcv_stamp =  node_ -> now();
}

// 缓存接收到的目标点
void Offboard_Data_t::feed(mavros_msgs::msg::PositionTarget::SharedPtr pMsg){
    msg = *pMsg;
    rcv_stamp =  node_ -> now();

    // PositionTarget uses xyz vectors and ignores jerk.
    p << msg.position.x, msg.position.y, msg.position.z;
    v << msg.velocity.x, msg.velocity.y, msg.velocity.z;
    a << msg.acceleration_or_force.x, msg.acceleration_or_force.y, msg.acceleration_or_force.z;
    j.setZero();
    yaw = msg.yaw;
    yaw_rate = msg.yaw_rate;
}

// OffboardMode_Data_t
// 初始化
Offboard_Mode_Data_t::Offboard_Mode_Data_t(const rclcpp::Node::SharedPtr& node) : node_(node){
    rcv_stamp =  node_ -> now();
}

// 回调
void Offboard_Mode_Data_t::feed(std_msgs::msg::UInt16::SharedPtr pMsg){
    mask = pMsg->data;
    rcv_stamp =  node_ -> now();
}

// Battery_Data_t
//初始化
Battery_Data_t::Battery_Data_t(const rclcpp::Node::SharedPtr& node) : node_(node){
    rcv_stamp =  node_ -> now();
}

// 获取电池电量信息
void Battery_Data_t::feed(sensor_msgs::msg::BatteryState::SharedPtr pMsg){
    msg = *pMsg;
    rcv_stamp =  node_ -> now();
    volt = msg.voltage;
    percentage = msg.remaining;
    flyTime = 0.0;
    warning = 0;
}

// Takeoff_Land_Data_t
// 初始化
Takeoff_Land_Data_t::Takeoff_Land_Data_t(const rclcpp::Node::SharedPtr& node) : node_(node){
    rcv_stamp =  node_ -> now();
}

// 获取起飞信息
void Takeoff_Land_Data_t::feed_takeoff_land(std_msgs::msg::UInt8::SharedPtr pMsg){
    rcv_stamp =  node_ -> now();
    triggered = true;
    takeoff_land_cmd = pMsg->data;
}

void Takeoff_Land_Data_t::feed_landed(mavros_msgs::msg::ExtendedState::SharedPtr pMsg){
    land_msg = *pMsg;
    landed = (pMsg->landed_state == mavros_msgs::msg::ExtendedState::LANDED_STATE_ON_GROUND);
}
