#include <lib/CtrlFSM.hpp>

/* 
如需花式降落（如斜向下 45° 降落）
请自行在 OFFBOARD 里进行操作
即自行给 cmd 话题发送对应的降落逻辑
并在降落后使用 arm_to_disarm() 进行无人机上锁

	         系统启动
	            |
	            |
	            v
	-------> 位置控制 <-------------------
	|         ^   |   \                 |
	|         |   |    \                |
	|         |   |     > 自动起飞       |
	|         |   |      /              |
	|         |   |     /               |
	|         |   |    /                |
	|         |   v   /                 |
	|       自动悬停 <                   |
	|         ^   |  \                  |
	|         |   |   \                 |
	|         |	  |    > 自动降落 -------|
	|         |   |          ^          |
	|         |   v          |          |
	-------- OFFBOARD--->定高模式--------
*/


CtrlFSM::CtrlFSM(const Param_t& param, const rclcpp::Node::SharedPtr& node)
:   param_(param), 
    node_(node), 
    rc_data(node),
    odom_data(node),
    state_data(node),
    offboard_data(node),
    offboard_mode_data(node),
    battery_data(node),
    takeoff_land_data(node){
	state = POSITION;
	
	// 初始化悬停目标位置为零
	hover_pose.setZero();
} 


void CtrlFSM::FSM(){
    // 获取当前时间戳
    rclcpp::Time now_time = node_ -> now();

    px4_msgs::msg::TrajectorySetpoint des = odom_data.msg;
    px4_msgs::msg::OffboardControlMode mode;

    mode.position = true;
    mode.velocity = true;
    mode.acceleration = false;
    mode.attitude = false;
    mode.body_rate = false;
    mode.timestamp = now_time.nanoseconds() / 1000;
    des.timestamp = now_time.nanoseconds() / 1000;

    static bool print_once_flag_positon = true;
    static bool pos_mode_flag = true;
    static bool alt_mode_flag = true;
    
    switch(state){
        case POSITION:{
            if (!odom_is_received(now_time)){
                if (alt_mode_flag){
                    if (print_once_flag_positon){
                        RCLCPP_INFO(node_->get_logger(), "定位不可用，切换为定高模式！");
                        print_once_flag_positon = false;
                    }
                    if (switch_to_altctl(now_time)){
                        print_once_flag_positon = true;
                        alt_mode_flag = false;
                    }
                    pos_mode_flag = true; 
                }
                break;
            }

            if (odom_is_received(now_time) && pos_mode_flag){
                if (print_once_flag_positon){
                    RCLCPP_INFO(node_->get_logger(), "定位可用，切换为定点模式！");
                    print_once_flag_positon = false;
                }
                if (switch_to_position(now_time)){
                    print_once_flag_positon = true;
                    pos_mode_flag = false;
                }
                alt_mode_flag = true;
            }
            

            // 遥控器进入自动悬浮模式
            if (rc_data.enter_hover_mode){
                // 检测传感器是否在线
                if (!odom_is_received(now_time)){
                    RCLCPP_WARN(node_->get_logger(), "传感器定位丢失，拒绝进入悬浮模式！");
                    break;
                }
                // 检测是否有外部命令
                if (offboard_is_received(now_time)){
                    RCLCPP_WARN(node_->get_logger(), "检测到外部命令，拒绝进入悬浮模式！");
                    break;
                }

                if (odom_data.v.norm() > 3.0){
                    RCLCPP_WARN(node_->get_logger(), "速度=%fm/s，定位故障！拒绝进入悬浮模式！", odom_data.v.norm());
                    break;
                }

                state = AUTO_HOVER;
                // 将当前位置设为悬停目标位置
                set_hover_pos();
                // 切换至 OFFBOARD 模式
                switch_to_offboard(now_time, true);
                RCLCPP_INFO(node_->get_logger(), "进入悬浮模式！");
            }

            else if (takeoff_land_data.triggered && param_.takeoff_land.enable && takeoff_land_data.takeoff_land_cmd == 1){
                // 检测传感器是否在线
                if (!odom_is_received(now_time)){
                    RCLCPP_WARN(node_->get_logger(), "传感器定位丢失，拒绝起飞！");
                    break;
                }
                // 检测是否有外部命令
                if (offboard_is_received(now_time) || offboard_mode_is_received(now_time)){
                    RCLCPP_WARN(node_->get_logger(), "检测到外部命令，拒绝起飞！");
                    break;
                }
                // 起飞前必须静止
                if (odom_data.v.norm() > 0.1){
                    RCLCPP_WARN(node_->get_logger(), "速度=%fm/s，拒绝起飞！", odom_data.v.norm());
                    break;
                }
                if (!takeoff_land_data.landed){
                    RCLCPP_WARN(node_->get_logger(), "检测到无人机为着陆，拒绝起飞！");
                    break;
                }

                if (rc_is_received(now_time)){
                    // 遥控器必须不处于悬停模式 && 命令模式 && 且所有摇杆居中
                    if (!rc_data.is_hover_mode || !rc_data.is_offboard || !rc_data.check_centered())
                    {
                        RCLCPP_WARN(node_->get_logger(), "请将摇杆回正并将模式扳机处于正确位置！");
                        break;
                    }
                }
                takeoff_start_time = now_time;
                set_start_pose_for_takeoff_land();
                if(switch_to_offboard(now_time, true)){
                    if(param_.takeoff_land.enable_arm){
                        takeoff_land.toggle_takeoff_land_time = now_time;
                        if(arm_to_disarm(now_time, true)){
                            state = AUTO_TAKEOFF;
                            RCLCPP_INFO(node_->get_logger(), "进入自动起飞模式！");
                        }
                    }
                }
            }
            break;
        }

        case AUTO_HOVER:{
            // 当遥控器不处于悬浮模式，或传感器失效，返回 POSITON
            if (!rc_data.is_hover_mode || !odom_is_received(now_time)){
                state = POSITION;
                switch_to_offboard(now_time, false);
                RCLCPP_WARN(node_->get_logger(), "返回 POSITION 模式！");
            }

            // 当遥控器处于命令模式，且收到外部命令
            else if (rc_data.is_offboard && offboard_is_received(now_time))
            {
                state = OFFBOARD;
                RCLCPP_INFO(node_->get_logger(), "收到外部命令，进入 OFFBOARD 控制！");
            }
            
            // 收到着陆信号
            else if (takeoff_land_data.triggered && takeoff_land_data.takeoff_land_cmd == 2){
                state = AUTO_LAND;
                set_start_pose_for_takeoff_land();
                RCLCPP_INFO(node_->get_logger(), "进入 AUTO_LAND 模式！");
            }

            else{
                des = get_hover_des(now_time);
                if (rc_data.enter_command_mode || (delay_trigger.first && now_time > delay_trigger.second)){
                    trigger_pub->publish(odom_data.msg);
                }
            }
            break;
        }

        case OFFBOARD:{
            // 遥控器离开悬浮模式或传感器失效
            if (!rc_data.is_hover_mode || !odom_is_received(now_time)){
                RCLCPP_WARN(node_->get_logger(), "返回 POSITION 模式！");
                state = POSITION;
                switch_to_offboard(now_time, false);
            }
            // 遥控器离开 OFFBOARD 模式或外部停止发送命令
            else if (!rc_data.is_offboard || !offboard_is_received(now_time)){
                state = AUTO_HOVER;
                RCLCPP_WARN(node_->get_logger(), "返回 AUTO_HOVER 模式！");
                set_hover_pos();
                des = get_hover_des(now_time);
            }

            else{
                des = offboard_data.msg;
                mode = offboard_mode_data.msg;
                des.timestamp = now_time.nanoseconds() / 1000;
                mode.timestamp = now_time.nanoseconds() / 1000;
            }
            
            // 拒绝从命令模式进行着陆
            if (takeoff_land_data.triggered && takeoff_land_data.takeoff_land_cmd == 2)
            {
                RCLCPP_WARN(node_->get_logger(), "拒绝着陆. 必须先停止发送命令, 返回悬浮模式后再着陆！");
            }
            break;
        }

        case AUTO_TAKEOFF:{
            static bool print_once_flag = true;
            if ((now_time - takeoff_start_time).seconds() < 3.0)
            {
                if(print_once_flag){
                    RCLCPP_INFO(node_->get_logger(), "无人机准备起飞，注意安全！");
                    print_once_flag = false;
                }
            }
            // 如果到达期望高度，进入悬浮模式
            else if (odom_data.p[2] <= (start_pose[2] - param_.takeoff_land.height))
            {
                state = AUTO_HOVER;
                set_hover_pos();  // 设置当前位置为悬停点
                RCLCPP_INFO(node_->get_logger(), "完成自动起飞，进入悬浮模式！");
                print_once_flag = true;
            }
            else{
                des = get_takeoff_des(now_time);
            }
            break;
        }

        case AUTO_LAND:{
            // 如果定位丢失或遥控器没有处于悬浮模式，则取消着陆，返回位置模式
            if (!rc_data.is_hover_mode || !odom_is_received(now_time)){
                RCLCPP_WARN(node_->get_logger(), "着陆取消，返回 POSITION！");
                state = POSITION;
                switch_to_offboard(now_time, false);
            }
            else{
                // 避免频繁调用服务
                static double last_trial_time = 0.0;
                if (now_time.seconds() - last_trial_time > 1.0){
                    if (land(now_time)){
                        state = POSITION;
                        switch_to_offboard(now_time, false);        // 退回先前的模式
                    }
                    last_trial_time = now_time.seconds();
                }
            }
            break;
        }

        case WANRING:{
            // 如果定位丢失，则强制着陆
            static double warn_count = -1.0;
            static bool warn_hov_once = true;
            RCLCPP_WARN_ONCE(node_->get_logger(), "准备紧急降落，请停止无人机飞行！");
            if (warn_count < 0){
                warn_count = now_time.seconds();
            }
            if(state_data.current_state.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD){
                if (warn_hov_once){
                    set_hover_pos();
                    warn_hov_once = false;
                }
                else{
                    des = get_hover_des(now_time);
                    if (now_time.seconds() - warn_count > 3.0){
                        if (land(now_time)){
                            state = NONE;
                            warn_hov_once = true;
                            warn_count = -1.0;
                        }
                        warn_count = now_time.seconds();
                    }
                }
            }
            else{
                if (land(now_time)){
                    state = NONE;
                    warn_hov_once = true;
                    warn_count = -1.0;
                }
            }
            break;
        }

        default:{
            break;
        }
    }

    if (battery_is_received(now_time)){
        if (battery_data.warning >= 2 || battery_data.percentage < 0.15 || battery_data.volt < param_.low_voltage){
            RCLCPP_WARN(node_->get_logger(), "电量过低，当前电量：%f，准备紧急降落！", battery_data.volt);
            state = WANRING;

        }
        else if(battery_data.volt < param_.low_voltage * 1.15 + 0.5){
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000, "当前电量：%f", battery_data.volt);
        }
    }

    offboard_pub -> publish(des);
    offboard_mode_pub -> publish(mode);
    rc_data.enter_hover_mode = false;
	rc_data.enter_offboard = false;
	takeoff_land_data.triggered = false;
}

// 判断 RC 数据是否有效
bool CtrlFSM::rc_is_received(rclcpp::Time& now_time){
    return (now_time - rc_data.rcv_stamp).seconds() < param_.msg_timeout.rc;
}

// 判断 Odom 数据是否有效
bool CtrlFSM::odom_is_received(rclcpp::Time& now_time){
    //  只有当 Odom 数据没有发生跳变且没有超时时才有效
    return !odom_data.pos_jump && (now_time - odom_data.rcv_stamp).seconds() < param_.msg_timeout.odom;
}

// 判断 Offboard 数据是否有效
bool CtrlFSM::offboard_is_received(rclcpp::Time& now_time){
    return (now_time - offboard_data.rcv_stamp).seconds() < param_.msg_timeout.offboard;
}

// 判断 OffboardMode 数据是否有效
bool CtrlFSM::offboard_mode_is_received(rclcpp::Time& now_time){
    return (now_time - offboard_mode_data.rcv_stamp).seconds() < param_.msg_timeout.offboardMode;
}

// 判断 Battery 数据是否有效
bool CtrlFSM::battery_is_received(rclcpp::Time& now_time){
    return (now_time - battery_data.rcv_stamp).seconds() < param_.msg_timeout.bat;
}

px4_msgs::msg::TrajectorySetpoint CtrlFSM::get_hover_des(rclcpp::Time& now_time){
    px4_msgs::msg::TrajectorySetpoint des;

    des.timestamp = now_time.nanoseconds() / 1000;
    des.position[0] = hover_pose[0];
    des.position[1] = hover_pose[1];
    des.position[2] = hover_pose[2];

    des.velocity[0] = NAN;
    des.velocity[1] = NAN;
    des.velocity[2] = NAN;

    des.acceleration[0] = NAN;
    des.acceleration[1] = NAN;
    des.acceleration[2] = NAN;

    des.jerk[0] = NAN;
    des.jerk[1] = NAN;
    des.jerk[2] = NAN;

    des.yaw = hover_pose[3];
    des.yawspeed = NAN;

    return des;
}

void CtrlFSM::publish_vehicle_command(rclcpp::Time& now_time, uint16_t command, float param1, float param2){
    px4_msgs::msg::VehicleCommand msg;
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = now_time.nanoseconds() / 1000;
    vehicle_command_pub -> publish(msg);
}

double CtrlFSM::get_yaw_from_odom(){
    double qw = odom_data.q.w();
    double qx = odom_data.q.x();
    double qy = odom_data.q.y();
    double qz = odom_data.q.z();
    return std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
}

// 无人机解锁 / 上锁
bool CtrlFSM::arm_to_disarm(rclcpp::Time& now_time, bool arm)
{
    // 如果已有请求在进行
    if (arm_in_progress){
        // 如果目标发生变化 → 重新发命令
        if (arm != arm_target){
            publish_vehicle_command(now_time, px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,arm);

            arm_start_time = now_time;
            arm_target = arm;

            RCLCPP_WARN(node_->get_logger(), "检测到新的 arm 请求，重新发送命令");
            return false;
        }

        // 成功判断
        if (arm_target &&
            state_data.current_state.arming_state ==
            px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED){
            RCLCPP_INFO(node_->get_logger(), "无人机解锁成功！");
            arm_in_progress = false;
            return true;
        }

        if (!arm_target &&
            state_data.current_state.arming_state ==
            px4_msgs::msg::VehicleStatus::ARMING_STATE_STANDBY){
            RCLCPP_INFO(node_->get_logger(), "无人机上锁成功！");
            arm_in_progress = false;
            return true;
        }

        // 超时判断
        if ((now_time - arm_start_time).seconds() > 3.0){
            RCLCPP_WARN(node_->get_logger(), "无人机上锁 / 解锁超时失败！");
            arm_in_progress = false;
            return false;
        }

        return false;
    }

    // 没有进行中的请求 → 发命令
    publish_vehicle_command(now_time, px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, arm);

    arm_start_time = now_time;
    arm_target = arm;
    arm_in_progress = true;

    return false;
}

// 保存着陆 / 起飞时的位置
void CtrlFSM::set_start_pose_for_takeoff_land(){
    start_pose.head<3>() = odom_data.p;
    start_pose[3] = get_yaw_from_odom();
}

bool CtrlFSM::land(rclcpp::Time& now_time){
    // 如果已有请求在进行
    if (land_in_progress){
        // 成功判断
        if (takeoff_land_data.landed){
            RCLCPP_INFO(node_->get_logger(), "成功着陆！");
            land_in_progress = false;
            return true;
        }

        // 当无人机超过 2s 高度没有发生变化，则判断超时
        if ((now_time - land_start_time).seconds() > 3.0 && std::abs(odom_data.p[2] - start_pose[2]) < 0.1){
            RCLCPP_WARN(node_->get_logger(), "命令超时，重新尝试降落！");
            land_in_progress = false;
            return false;
        }
        return false;
    }

    // 没有进行中的请求 → 发命令
    publish_vehicle_command(now_time, px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND, 0);
    land_start_time = now_time;
    land_in_progress = true;
    RCLCPP_INFO(node_->get_logger(), "正在请求 PX4 自主着陆...");

    return false;
}


px4_msgs::msg::TrajectorySetpoint CtrlFSM::get_takeoff_des(rclcpp::Time& now_time) {
    double delta_t = (now_time - takeoff_start_time).seconds();

    px4_msgs::msg::TrajectorySetpoint des;

    des.timestamp = now_time.nanoseconds() / 1000;
    des.position[0] = start_pose[0];
    des.position[1] = start_pose[1];
    des.position[2] = start_pose[2] - delta_t * param_.takeoff_land.speed;

    // 设置期望速度（水平静止，垂直上升）
    des.velocity[0] = 0.0f;
    des.velocity[1] = 0.0f;
    des.velocity[2] = -param_.takeoff_land.speed;  // 负值表示向上（NED坐标系）
    
    // 加速度和jerk交给控制器计算
    des.acceleration[0] = NAN;
    des.acceleration[1] = NAN;
    des.acceleration[2] = NAN;
    
    des.jerk[0] = NAN;
    des.jerk[1] = NAN;
    des.jerk[2] = NAN;
    
    des.yaw = start_pose[3]; 
    des.yawspeed = NAN;

    return des;
}

// 进入 / 退出 Offboard 模式
bool CtrlFSM::switch_to_offboard(rclcpp::Time& now_time, bool on_off){
    // 如果已有请求在进行
    if (mode_in_progress){
        // 成功判断
        if (on_off && state_data.current_state.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD){
            RCLCPP_INFO(node_->get_logger(), "成功进入 Offboard 模式！");
            mode_in_progress = false;
            return true;
        }
        else if (!on_off && state_data.current_state.nav_state == state_data.state_before_offboard.nav_state)
        {
            RCLCPP_INFO(node_->get_logger(), "成功退出 Offboard 模式！");
            mode_in_progress = false;
            return true;
        }

        else if (!on_off && state_data.current_state.nav_state != px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD)
        {
            RCLCPP_INFO(node_->get_logger(), "Odom 数据异常，Offboard 模式降级，当前模式：%s！", mode_to_string(status_to_mode(state_data.current_state)).c_str());
            mode_in_progress = false;
            return true;
        }
        
        // 超时判断
        else if ((now_time - mode_start_time).seconds() > 3.0)
        {
            if (on_off){
                RCLCPP_WARN(node_->get_logger(), "飞控拒绝进入 Offboard 模式！");
            }
            else{
                RCLCPP_WARN(node_->get_logger(), "飞控拒绝退出 Offboard 模式！");
            }
            mode_in_progress = false;
            return false;
        }     
    }

    mode_start_time = now_time;
    if (on_off){
        state_data.state_before_offboard = state_data.current_state;        // 保存先前模式的状态
        // 如果当前模式已经是 Offboard，则将先前模式设为位置模式，防止递归
        if (state_data.current_state.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD){
            state_data.state_before_offboard.nav_state = px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_POSCTL;
            RCLCPP_INFO(node_->get_logger(), "已经处于 Offboard 模式中，请勿重复切换！");
            return true;
        }

        publish_vehicle_command(now_time, px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, mode_to_com(Mode_t::OFFBOARD));
    }
    else{
        publish_vehicle_command(now_time, px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, mode_to_com(status_to_mode(state_data.state_before_offboard.nav_state)));
    }
    mode_in_progress = true;
    return false;
}

// 进入 ALTCTL 模式
bool CtrlFSM::switch_to_altctl(rclcpp::Time& now_time){
    // 如果已有请求在进行
    if (altctl_in_progress){
        // 成功判断
        if (state_data.current_state.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_ALTCTL){
            RCLCPP_INFO(node_->get_logger(), "成功进入 ALTCTL 模式！");
            altctl_in_progress = false;
            return true;
        }

        else if (state_data.current_state.nav_state != px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD &&
                 state_data.current_state.nav_state != px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_POSCTL){
            RCLCPP_INFO(node_->get_logger(), "Odom 数据异常，ALTCTL 模式降级，当前模式：%s！", mode_to_string(status_to_mode(state_data.current_state)).c_str());
            altctl_in_progress = false;
            return true;
        }
        
        // 超时判断
        else if ((now_time - altctl_start_time).seconds() > 3.0){
            RCLCPP_WARN(node_->get_logger(), "飞控拒绝进入 ALTCTL 模式，重新尝试！");
            altctl_in_progress = false;
            return false;
        }
    }

    altctl_start_time = now_time;
    publish_vehicle_command(now_time, px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, mode_to_com(Mode_t::ALTCTL));
    altctl_in_progress = true;
    return false;
}

// 进入 POSITION 模式
bool CtrlFSM::switch_to_position(rclcpp::Time& now_time){
    // 如果已有请求在进行
    if (position_in_progress){
        // 成功判断
        if (state_data.current_state.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_POSCTL){
            RCLCPP_INFO(node_->get_logger(), "成功进入 POSITON 模式！");
            position_in_progress = false;
            return true;
        }
        
        // 超时判断
        else if ((now_time - position_start_time).seconds() > 3.0)
        {
            RCLCPP_WARN(node_->get_logger(), "飞控拒绝进入 POSITON 模式，重新尝试！");
            position_in_progress = false;
            return false;
        }
    }

    position_start_time = now_time;
    publish_vehicle_command(now_time, px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, mode_to_com(Mode_t::POSCTL));
    position_in_progress = true;
    return false;
}

float CtrlFSM::mode_to_com(Mode_t mode){
    switch (mode){
        case Mode_t::MANUAL:{
            return 1.0;
        }
        case Mode_t::ALTCTL:{
            return 2.0;
        }
        case Mode_t::POSCTL:{
            return 3.0;
        }
        case Mode_t::OFFBOARD:{
            return 6.0;
        }
        default:{
            return 0.0;
        }
    }
}

Mode_t CtrlFSM::status_to_mode(const px4_msgs::msg::VehicleStatus& status){
    switch (status.nav_state){
        case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_MANUAL:{
            return Mode_t::MANUAL;
        }
        case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_ALTCTL:{
            return Mode_t::ALTCTL;
        }
        case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_POSCTL:{
            return Mode_t::POSCTL;
        }
        case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD:{
            return Mode_t::OFFBOARD;
        }
        default:{
            return Mode_t::UNKNOWN;
        }
    }
}

std::string CtrlFSM::mode_to_string(Mode_t mode){
    switch (mode){
        case Mode_t::MANUAL:{
            return "MANUAL";
        }
        case Mode_t::ALTCTL:{
            return "ALTCTL";
        }
        case Mode_t::POSCTL:{
            return "POSCTL";
        }
        case Mode_t::OFFBOARD:{
            return "OFFBOARD";
        }
        default:{
            return "UNKNOWN";
        }
    }
}

void CtrlFSM::set_hover_pos(){
    hover_pose.head<3>() = odom_data.p;
    hover_pose[3] = get_yaw_from_odom();
}
