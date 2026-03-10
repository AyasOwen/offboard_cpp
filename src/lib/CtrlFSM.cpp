#include <lib/CtrlFSM.hpp>
#include <algorithm>
#include <cmath>

namespace {
uint16_t make_type_mask(bool position, bool velocity, bool acceleration, bool yaw, bool yaw_rate) {
	using PT = mavros_msgs::msg::PositionTarget;
	uint16_t mask = 0;
	if (!position) {
		mask |= PT::IGNORE_PX | PT::IGNORE_PY | PT::IGNORE_PZ;
	}
	if (!velocity) {
		mask |= PT::IGNORE_VX | PT::IGNORE_VY | PT::IGNORE_VZ;
	}
	if (!acceleration) {
		mask |= PT::IGNORE_AFX | PT::IGNORE_AFY | PT::IGNORE_AFZ;
	}
	if (!yaw) {
		mask |= PT::IGNORE_YAW;
	}
	if (!yaw_rate) {
		mask |= PT::IGNORE_YAW_RATE;
	}
	return mask;
}
}  // namespace

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


CtrlFSM::CtrlFSM(Param_t& param, const rclcpp::Node::SharedPtr& node)
: node_(node),
  param_(param),
  rc_data(node),
  odom_data(node),
  state_data(node),
  offboard_data(node),
  offboard_mode_data(node),
  battery_data(node),
  takeoff_land_data(node) {
	state = POSITION;
	hover_pose.setZero();
}

void CtrlFSM::FSM() {
	rclcpp::Time now_time = node_->now();

	mavros_msgs::msg::PositionTarget des{};
	std_msgs::msg::UInt16 mode_msg{};
	std_msgs::msg::Bool trigger_flag;

	des.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_ENU;
	des.type_mask = make_type_mask(true, true, false, true, false);
	des.position.x = odom_data.p[0];
	des.position.y = odom_data.p[1];
	des.position.z = odom_data.p[2];
	des.yaw = static_cast<float>(get_yaw_from_odom());

	mode_msg.data = des.type_mask;

	static bool print_once_flag_positon = true;
	static bool pos_mode_flag = true;
	static bool alt_mode_flag = true;

	switch (state) {
		case POSITION: {
			if (!odom_is_received(now_time)) {
				if (alt_mode_flag) {
					if (print_once_flag_positon) {
						RCLCPP_INFO(node_->get_logger(), "定位不可用，切换为定高模式！");
						print_once_flag_positon = false;
					}
					if (switch_to_altctl(now_time)) {
						print_once_flag_positon = true;
						alt_mode_flag = false;
					}
					pos_mode_flag = true;
				}
				break;
			}

			if (odom_is_received(now_time) && pos_mode_flag) {
				if (print_once_flag_positon) {
					RCLCPP_INFO(node_->get_logger(), "定位可用，切换为定点模式！");
					print_once_flag_positon = false;
				}
				if (switch_to_position(now_time)) {
					print_once_flag_positon = true;
					pos_mode_flag = false;
				}
				alt_mode_flag = true;
			}

			if (rc_data.enter_hover_mode) {
				if (!odom_is_received(now_time)) {
					RCLCPP_WARN(node_->get_logger(), "传感器定位丢失，拒绝进入悬浮模式！");
					break;
				}
				if (offboard_is_received(now_time)) {
					RCLCPP_WARN(node_->get_logger(), "检测到外部命令，拒绝进入悬浮模式！");
					break;
				}

				if (odom_data.v.norm() > 3.0) {
					RCLCPP_WARN(node_->get_logger(), "速度=%fm/s，定位故障！拒绝进入悬浮模式！", odom_data.v.norm());
					break;
				}

				state = AUTO_HOVER;
				set_hover_pos();
				switch_to_offboard(now_time, true);
				RCLCPP_INFO(node_->get_logger(), "进入悬浮模式！");
			}

			else if (takeoff_land_data.triggered && param_.takeoff_land.enable && takeoff_land_data.takeoff_land_cmd == 1) {
				if (!odom_is_received(now_time)) {
					RCLCPP_WARN(node_->get_logger(), "传感器定位丢失，拒绝起飞！");
					break;
				}
				if (offboard_is_received(now_time) || offboard_mode_is_received(now_time)) {
					RCLCPP_WARN(node_->get_logger(), "检测到外部命令，拒绝起飞！");
					break;
				}
				if (odom_data.v.norm() > 0.1) {
					RCLCPP_WARN(node_->get_logger(), "速度=%fm/s，拒绝起飞！", odom_data.v.norm());
					break;
				}
				if (!landed) {
					RCLCPP_WARN(node_->get_logger(), "检测到无人机未着陆，拒绝起飞！");
					break;
				}

				if (rc_is_received(now_time)) {
					if (!rc_data.is_hover_mode || !rc_data.is_offboard || !rc_data.check_centered()) {
						RCLCPP_WARN(node_->get_logger(), "请将摇杆回正并将模式扳机处于正确位置！");
						break;
					}
				}

				set_start_pose_for_takeoff_land();
				if (switch_to_offboard(now_time, true)) {
					if (param_.takeoff_land.enable_arm) {
						takeoff_start_time = now_time;
						if (arm_to_disarm(now_time, true)) {
							state = AUTO_TAKEOFF;
							RCLCPP_INFO(node_->get_logger(), "进入自动起飞模式！");
						}
					}
				}
			}
			break;
		}

		case AUTO_HOVER: {
			if (!rc_data.is_hover_mode || !odom_is_received(now_time)) {
				state = POSITION;
				switch_to_offboard(now_time, false);
				RCLCPP_WARN(node_->get_logger(), "返回 POSITION 模式！");
			}

			else if (rc_data.is_offboard && offboard_is_received(now_time)) {
				state = OFFBOARD;
				RCLCPP_INFO(node_->get_logger(), "收到外部命令，进入 OFFBOARD 控制！");
			}

			else if (takeoff_land_data.triggered && takeoff_land_data.takeoff_land_cmd == 2) {
				state = AUTO_LAND;
				set_start_pose_for_takeoff_land();
				RCLCPP_INFO(node_->get_logger(), "进入 AUTO_LAND 模式！");
			}

			else {
				des = get_hover_des(now_time);
				mode_msg.data = des.type_mask;
				if (rc_data.enter_offboard || (delay_trigger.first && now_time > delay_trigger.second)) {
					trigger_flag.data = true;
					trigger_pub->publish(trigger_flag);
				}
			}
			break;
		}

		case OFFBOARD: {
			if (!rc_data.is_hover_mode || !odom_is_received(now_time)) {
				RCLCPP_WARN(node_->get_logger(), "返回 POSITION 模式！");
				state = POSITION;
				trigger_flag.data = false;
				trigger_pub->publish(trigger_flag);
				switch_to_offboard(now_time, false);
			}
			else if (!rc_data.is_offboard || !offboard_is_received(now_time)) {
				state = AUTO_HOVER;
				RCLCPP_WARN(node_->get_logger(), "返回 AUTO_HOVER 模式！");
				trigger_flag.data = false;
				trigger_pub->publish(trigger_flag);
				set_hover_pos();
				des = get_hover_des(now_time);
				mode_msg.data = des.type_mask;
			}
			else {
				des = offboard_data.msg;
				des.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_ENU;
				if (offboard_mode_is_received(now_time)) {
					des.type_mask = offboard_mode_data.mask;
				}
				mode_msg.data = des.type_mask;
			}

			if (takeoff_land_data.triggered && takeoff_land_data.takeoff_land_cmd == 2) {
				RCLCPP_WARN(node_->get_logger(), "拒绝着陆. 必须先停止发送命令, 返回悬浮模式后再着陆！");
			}
			break;
		}

		case AUTO_TAKEOFF: {
			static bool print_once_flag = true;
			if ((now_time - takeoff_start_time).seconds() < 3.0) {
				if (print_once_flag) {
					RCLCPP_INFO(node_->get_logger(), "无人机准备起飞，注意安全！");
					print_once_flag = false;
				}
			}
			else if (odom_data.p[2] >= (start_pose[2] + param_.takeoff_land.height)) {
				state = AUTO_HOVER;
				set_hover_pos();
				RCLCPP_INFO(node_->get_logger(), "完成自动起飞，进入悬浮模式！");
				print_once_flag = true;
			}
			else {
				des = get_takeoff_des(now_time);
				mode_msg.data = des.type_mask;
			}
			break;
		}

		case AUTO_LAND: {
			if (!rc_data.is_hover_mode || !odom_is_received(now_time)) {
				RCLCPP_WARN(node_->get_logger(), "着陆取消，返回 POSITION！");
				state = POSITION;
				switch_to_offboard(now_time, false);
			}
			else {
				static double last_trial_time = 0.0;
				if (now_time.seconds() - last_trial_time > 1.0) {
					if (land(now_time)) {
						state = POSITION;
						switch_to_offboard(now_time, false);
					}
					last_trial_time = now_time.seconds();
				}
			}
			break;
		}

		case WANRING: {
			static double warn_count = -1.0;
			static bool warn_hov_once = true;
			RCLCPP_WARN_ONCE(node_->get_logger(), "准备紧急降落，请停止无人机飞行！");
			if (warn_count < 0) {
				warn_count = now_time.seconds();
			}
			if (state_data.current_state.mode == "OFFBOARD") {
				if (warn_hov_once) {
					set_hover_pos();
					warn_hov_once = false;
				}
				else {
					des = get_hover_des(now_time);
					mode_msg.data = des.type_mask;
					if (now_time.seconds() - warn_count > 3.0) {
						set_start_pose_for_takeoff_land();
						if (land(now_time)) {
							state = NONE;
							warn_hov_once = true;
							warn_count = -1.0;
						}
						warn_count = now_time.seconds();
					}
				}
			}
			else {
				if (land(now_time)) {
					state = NONE;
					warn_hov_once = true;
					warn_count = -1.0;
				}
			}
			break;
		}

		default: {
			break;
		}
	}

	land_detector(des, now_time);

	if (battery_is_received(now_time)) {
		if (battery_data.volt > 1.0 && battery_data.percentage > 0 &&
			(battery_data.percentage < 0.15 || battery_data.volt < param_.low_voltage)) {
			RCLCPP_WARN(node_->get_logger(), "电量过低，当前电量：%f，准备紧急降落！", battery_data.volt);
			set_start_pose_for_takeoff_land();
			state = WANRING;
		}
		else if (battery_data.volt < param_.low_voltage * 1.1 + 0.25) {
			RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000, "当前电量：%f", battery_data.volt);
		}
	}

	offboard_pub->publish(des);
	offboard_mode_pub->publish(mode_msg);
	rc_data.enter_hover_mode = false;
	rc_data.enter_offboard = false;
	takeoff_land_data.triggered = false;
}

bool CtrlFSM::rc_is_received(rclcpp::Time& now_time) {
	return (now_time - rc_data.rcv_stamp).seconds() < param_.msg_timeout.rc;
}

bool CtrlFSM::odom_is_received(rclcpp::Time& now_time) {
	return !odom_data.pos_jump && (now_time - odom_data.rcv_stamp).seconds() < param_.msg_timeout.odom;
}

bool CtrlFSM::offboard_is_received(rclcpp::Time& now_time) {
	return (now_time - offboard_data.rcv_stamp).seconds() < param_.msg_timeout.offboard;
}

bool CtrlFSM::offboard_mode_is_received(rclcpp::Time& now_time) {
	return (now_time - offboard_mode_data.rcv_stamp).seconds() < param_.msg_timeout.offboardMode;
}

bool CtrlFSM::battery_is_received(rclcpp::Time& now_time) {
	return (now_time - battery_data.rcv_stamp).seconds() < param_.msg_timeout.bat;
}

mavros_msgs::msg::PositionTarget CtrlFSM::get_hover_des(rclcpp::Time&) {
	mavros_msgs::msg::PositionTarget des;
	des.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_ENU;
	des.type_mask = make_type_mask(true, false, false, true, false);

	des.position.x = hover_pose[0];
	des.position.y = hover_pose[1];
	des.position.z = hover_pose[2];
	des.yaw = hover_pose[3];
	return des;
}

double CtrlFSM::get_yaw_from_odom() {
	double qw = odom_data.q.w();
	double qx = odom_data.q.x();
	double qy = odom_data.q.y();
	double qz = odom_data.q.z();
	return std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
}

bool CtrlFSM::request_mode_change(const std::string &mode) {
	if (!set_mode_client->wait_for_service(std::chrono::milliseconds(300))) {
		RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, "等待 /mavros/set_mode 服务中");
		return false;
	}
	auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
	req->custom_mode = mode;
	auto future = set_mode_client->async_send_request(req);
	auto ret = rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(1));
	if (ret != rclcpp::FutureReturnCode::SUCCESS) {
		RCLCPP_ERROR(node_->get_logger(), "set_mode 调用超时或失败");
		return false;
	}
	return future.get()->mode_sent;
}

bool CtrlFSM::request_arm(bool arm) {
	if (!arm_client->wait_for_service(std::chrono::milliseconds(300))) {
		RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, "等待 /mavros/cmd/arming 服务中");
		return false;
	}
	auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
	req->value = arm;
	auto future = arm_client->async_send_request(req);
	auto ret = rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(1));
	if (ret != rclcpp::FutureReturnCode::SUCCESS) {
		RCLCPP_ERROR(node_->get_logger(), "arming 调用超时或失败");
		return false;
	}
	return future.get()->success;
}

bool CtrlFSM::request_land() {
	if (!land_client->wait_for_service(std::chrono::milliseconds(300))) {
		RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, "等待 /mavros/cmd/land 服务中");
		return false;
	}
	auto req = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
	req->altitude = 0.0;
	req->latitude = 0.0;
	req->longitude = 0.0;
	req->min_pitch = 0.0;
	req->yaw = 0.0;
	auto future = land_client->async_send_request(req);
	auto ret = rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(1));
	if (ret != rclcpp::FutureReturnCode::SUCCESS) {
		RCLCPP_ERROR(node_->get_logger(), "land 调用超时或失败");
		return false;
	}
	return future.get()->success;
}

bool CtrlFSM::arm_to_disarm(rclcpp::Time& now_time, bool arm) {
	(void)now_time;
	if (!request_arm(arm)) {
		return false;
	}
	if (arm) {
		RCLCPP_INFO(node_->get_logger(), "发送解锁请求成功");
	} else {
		RCLCPP_INFO(node_->get_logger(), "发送上锁请求成功");
	}
	return true;
}

void CtrlFSM::set_start_pose_for_takeoff_land() {
	start_pose.head<3>() = odom_data.p;
	start_pose[3] = get_yaw_from_odom();
}

bool CtrlFSM::land(rclcpp::Time& now_time) {
	if (land_in_progress) {
		if (takeoff_land_data.landed || landed) {
			RCLCPP_INFO(node_->get_logger(), "成功着陆！");
			land_in_progress = false;
			return true;
		}

		if ((now_time - land_start_time).seconds() > 3.0 && std::abs(odom_data.p[2] - start_pose[2]) < 0.1) {
			RCLCPP_WARN(node_->get_logger(), "命令超时，重新尝试降落！");
			land_in_progress = false;
			return false;
		}
		return false;
	}

	request_land();
	land_start_time = now_time;
	land_in_progress = true;
	RCLCPP_INFO(node_->get_logger(), "正在请求 PX4 自主着陆...");
	return false;
}

void CtrlFSM::land_detector(const mavros_msgs::msg::PositionTarget& des, const rclcpp::Time& now_time) {
	static State_t last_state = POSITION;
	if (last_state == POSITION && state != POSITION) {
		landed = false;
	}
	last_state = state;

	if (state == POSITION && !state_data.current_state.armed) {
		landed = true;
		return;
	}

	constexpr double POSITION_DEVIATION_C = 0.5;
	constexpr double VELOCITY_THR_C = 0.1;
	constexpr double TIME_KEEP_C = 3.0;

	static rclcpp::Time time_C12_reached(0, 0, RCL_ROS_TIME);
	static bool time_initialized = false;
	static bool is_last_C12_satisfy = false;

	if (!time_initialized || time_C12_reached.get_clock_type() != now_time.get_clock_type()) {
		time_C12_reached = now_time;
		time_initialized = true;
		is_last_C12_satisfy = false;
	}

	if (takeoff_land_data.landed) {
		time_C12_reached = now_time;
		is_last_C12_satisfy = false;
	}
	else {
		const bool C12_satisfy =
			(des.position.z - odom_data.p[2]) > POSITION_DEVIATION_C &&
			odom_data.v.norm() < VELOCITY_THR_C;

		if (C12_satisfy && !is_last_C12_satisfy) {
			time_C12_reached = now_time;
		}
		else if (C12_satisfy && is_last_C12_satisfy) {
			if ((now_time - time_C12_reached).seconds() > TIME_KEEP_C) {
				landed = true;
			}
		}
		is_last_C12_satisfy = C12_satisfy;
	}
}

mavros_msgs::msg::PositionTarget CtrlFSM::get_takeoff_des(rclcpp::Time& now_time) {
	double delta_t = (now_time - takeoff_start_time).seconds();
	double speed = std::abs(param_.takeoff_land.speed);
	if (speed < 1e-6) {
		speed = 0.3;
	}

	double target_z = start_pose[2] + param_.takeoff_land.height;
	double z_cmd = start_pose[2] + delta_t * speed;
	if (z_cmd > target_z) {
		z_cmd = target_z;
	}

	double remaining = std::max(0.0, target_z - odom_data.p[2]);
	double smooth_acc = std::max(0.2, speed);
	double vel_mag = std::min(speed, std::sqrt(2.0 * smooth_acc * remaining));
	float vz_cmd = 0.0f;
	if (remaining > 1e-3) {
		vz_cmd = static_cast<float>(vel_mag);
	}

	mavros_msgs::msg::PositionTarget des;
	des.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_ENU;
	des.type_mask = make_type_mask(true, true, false, true, false);

	des.position.x = start_pose[0];
	des.position.y = start_pose[1];
	des.position.z = static_cast<float>(z_cmd);
	des.velocity.x = 0.0f;
	des.velocity.y = 0.0f;
	des.velocity.z = vz_cmd;
	des.yaw = start_pose[3];
	return des;
}

bool CtrlFSM::switch_to_offboard(rclcpp::Time& now_time, bool on_off) {
	(void)now_time;
	if (on_off) {
		state_data.state_before_offboard = state_data.current_state;
		if (state_data.current_state.mode == "OFFBOARD") {
			state_data.state_before_offboard.mode = "POSCTL";
			RCLCPP_INFO(node_->get_logger(), "已经处于 Offboard 模式中，请勿重复切换！");
			return true;
		}
		if (!request_mode_change("OFFBOARD")) {
			RCLCPP_WARN(node_->get_logger(), "飞控拒绝进入 Offboard 模式！");
			return false;
		}
		return true;
	}
	const std::string mode = state_data.state_before_offboard.mode.empty() ? "POSCTL" : state_data.state_before_offboard.mode;
	if (!request_mode_change(mode)) {
		RCLCPP_WARN(node_->get_logger(), "飞控拒绝退出 Offboard 模式！");
		return false;
	}
	return true;
}

bool CtrlFSM::switch_to_altctl(rclcpp::Time& now_time) {
	(void)now_time;
	if (!request_mode_change("ALTCTL")) {
		RCLCPP_WARN(node_->get_logger(), "飞控拒绝进入 ALTCTL 模式，重新尝试！");
		return false;
	}
	return true;
}

bool CtrlFSM::switch_to_position(rclcpp::Time& now_time) {
	(void)now_time;
	if (!request_mode_change("POSCTL")) {
		RCLCPP_WARN(node_->get_logger(), "飞控拒绝进入 POSITON 模式，重新尝试！");
		return false;
	}
	return true;
}

void CtrlFSM::set_hover_pos() {
	hover_pose.head<3>() = odom_data.p;
	hover_pose[3] = get_yaw_from_odom();
}
