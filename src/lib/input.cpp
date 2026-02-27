#include "lib/input.hpp"


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
// 未完成，待修改
void RC_Data_t::feed(px4_msgs::msg::RcChannels::SharedPtr pMsg, const Param_t& param){
    msg = *pMsg;
    rcv_stamp =  node_ -> now();

    // 提取遥控器通道数据（根据实际通道映射调整）
    for(int i = 0; i < 4; i++){
        ch[i] = ((double)msg.channels[i] - 1500.0) / 500.0;
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

    mode = ((double)msg.channels[param.rc_debug.ch_mode] - 1000.0) / 1000.0;
    gear = ((double)msg.channels[param.rc_debug.ch_gear] - 1000.0) / 1000.0;
    // 这里归一到了 [0, 1] ，如有别的需求，自行进行修改
    p = ((double)msg.channels[param.rc_debug.ch_p] - 1000.0) / 1000.0;
    i = ((double)msg.channels[param.rc_debug.ch_i] - 1000.0) / 1000.0;
    d = ((double)msg.channels[param.rc_debug.ch_d] - 1000.0) / 1000.0;

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
void Odom_Data_t::feed(px4_msgs::msg::VehicleOdometry::SharedPtr pMsg, const Param_t& param){
    rcv_stamp =  node_ -> now();
    msg = *pMsg;
    recv_new_msg = true;
    bool is_first_msg = (rcv_stamp.nanoseconds() == 0);     // 判断是不是第一帧

    Eigen::Vector3d new_p(msg.position[0], msg.position[1], msg.position[2]);

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
    v << msg.velocity[0], msg.velocity[1], msg.velocity[2];
    q = Eigen::Quaterniond(msg.q[0], msg.q[1], msg.q[2], msg.q[3]);
    w << msg.angular_velocity[0], msg.angular_velocity[1], msg.angular_velocity[2];

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
void State_Data_t::feed(px4_msgs::msg::VehicleStatus::SharedPtr pMsg){
    current_state = *pMsg;
}

// OffboardMode_Data_t
// 初始化
Offboard_Data_t::Offboard_Data_t(const rclcpp::Node::SharedPtr& node) : node_(node){
    rcv_stamp =  node_ -> now();
}

// 缓存接收到的目标点
void Offboard_Data_t::feed(px4_msgs::msg::TrajectorySetpoint::SharedPtr pMsg){
    msg = *pMsg;
    rcv_stamp =  node_ -> now();

    // 提取 p, v, a, j, yaw, yaw_rate
    p << msg.position[0], msg.position[1], msg.position[2];
    v << msg.velocity[0], msg.velocity[1], msg.velocity[2];
    a << msg.acceleration[0], msg.acceleration[1], msg.acceleration[2];
    j << msg.jerk[0], msg.jerk[1], msg.jerk[2];
    yaw = msg.yaw;
    yaw_rate = msg.yawspeed;
}

// OffboardMode_Data_t
// 初始化
Offboard_Mode_Data_t::Offboard_Mode_Data_t(const rclcpp::Node::SharedPtr& node) : node_(node){
    rcv_stamp =  node_ -> now();
}

// 回调
void Offboard_Mode_Data_t::feed(px4_msgs::msg::OffboardControlMode::SharedPtr pMsg){
    msg = *pMsg;
    rcv_stamp =  node_ -> now();
}

// Battery_Data_t
//初始化
Battery_Data_t::Battery_Data_t(const rclcpp::Node::SharedPtr& node) : node_(node){
    rcv_stamp =  node_ -> now();
}

// 获取电池电量信息
void Battery_Data_t::feed(px4_msgs::msg::BatteryStatus::SharedPtr pMsg){
    msg = *pMsg;
    rcv_stamp =  node_ -> now();
    volt = msg.voltage_filtered_v;
    percentage = msg.remaining;
    flyTime = msg.time_remaining_s;
    warning = msg.warning;
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

void Takeoff_Land_Data_t::feed_landed(px4_msgs::msg::VehicleLandDetected::SharedPtr pMsg){
    land_msg = *pMsg;
    landed = pMsg->landed;
}
