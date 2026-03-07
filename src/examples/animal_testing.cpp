#include "examples/animal_testing.hpp"

AnimalTestingNode::AnimalTestingNode() : Node("animal_testing_node") {
    // 定义 QoS 策略
    auto qos_px4 = rclcpp::QoS(rclcpp::KeepLast(1))
                       .best_effort()
                       .durability_volatile();
    
    // 初始化发布者
    cmd_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        "offboard/cmd", qos_px4);
    cmd_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
        "offboard/cmd_mode", qos_px4);
    takeoff_land_pub_ = this->create_publisher<std_msgs::msg::UInt8>(
        "offboard/takeoff_land", qos_px4);
    current_id_pub_ = this->create_publisher<std_msgs::msg::Int32>(
        "/current_map_id", 10);
    path_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>(
        "/final_path", 10);
    
    // 初始化订阅者
    odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
        "fmu/out/vehicle_odometry", qos_px4,
        std::bind(&AnimalTestingNode::odomCallback, this, std::placeholders::_1));
    
    no_fly_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
        "/no_fly_zone", 10,
        std::bind(&AnimalTestingNode::noFlyCallback, this, std::placeholders::_1));
    
    start_signal_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/start_flight", 10,
        std::bind(&AnimalTestingNode::startSignalCallback, this, std::placeholders::_1));

    trigger_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "offboard/trigger", qos_px4,
        std::bind(&AnimalTestingNode::triggerCallback, this, std::placeholders::_1));
    
    // 初始化状态
    state_start_time_ = now();
    
    // 创建定时器 (10Hz)
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&AnimalTestingNode::timerCallback, this));
    
    RCLCPP_INFO(this->get_logger(), "Animal Testing Node 启动");
}

void AnimalTestingNode::timerCallback() {
    // 发布当前位置对应的地图ID
    if (position_received_) {
        float x = current_position_[0];
        float y = current_position_[1];
        int current_id = getMapIDFromPos(x, y);
        
        if (current_id > 0) {
            std_msgs::msg::Int32 msg;
            msg.data = current_id;
            current_id_pub_->publish(msg);
        }
    }

    switch (mission_state_) {
        case MissionState::PLANNING:
            handlePlanning();
            break;

        case MissionState::WAIT_GCS:
            handleWaitGCS();
            break;

        case MissionState::TAKEOFF:
            handleTakeoff();
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

void AnimalTestingNode::changeState(MissionState new_state) {
    mission_state_ = new_state;
    state_start_time_ = now();
    stable_started_ = false;
    RCLCPP_INFO(get_logger(), "状态切换");
}

/* -------------------- 任务逻辑 -------------------- */

void AnimalTestingNode::handlePlanning() {
    if (forbidden_ids_ready_) {
        current_path_ = planFullTraversal();
        path_index_ = 0;

        if (current_path_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "未找到有效路径，终止任务");
            changeState(MissionState::DONE);
        } else {
            RCLCPP_INFO(this->get_logger(), "路径规划完成，等待地面站信号...");
            changeState(MissionState::WAIT_GCS);
        }
    }
}

void AnimalTestingNode::handleWaitGCS() {
    if (start_signal_from_gcs_) {
        start_signal_from_gcs_ = false;
        RCLCPP_INFO(this->get_logger(), "收到地面站信号，开始起飞");
        changeState(MissionState::TAKEOFF);
    }
}

void AnimalTestingNode::handleTakeoff() {
    if (!takeoff_command_sent_) {
        takeoff_start_z_ = current_position_[2];
        takeoff_command_sent_ = true;
        RCLCPP_INFO(get_logger(), "发送起飞命令");
    }

    publishTakeoffCommand();

    const bool altitude_changed =
        std::abs(current_position_[2] - takeoff_start_z_) > 0.3;

    if (position_received_ && altitude_changed && isVelocityNearZero()) {
        if (!stable_started_) {
            stable_start_time_ = now();
            stable_started_ = true;
        } else if ((now() - stable_start_time_).seconds() > stable_time_) {
            RCLCPP_INFO(get_logger(), "起飞完成，开始路径遍历");
            changeState(MissionState::WAYPOINT);
        }
    } else {
        stable_started_ = false;
    }
}

void AnimalTestingNode::handleWaypoint() {
    if (!trigger_) {
        return;
    }

    if (path_index_ >= current_path_.size()) {
        RCLCPP_INFO(this->get_logger(), "所有航点遍历完成");
        changeState(MissionState::RETURN_HOME);
        return;
    }

    // 获取目标位置
    float pos[2];
    int target_id = current_path_[path_index_];
    getPos(target_id, pos);
    
    publishControlMode();
    publishCommand(pos[0], pos[1], flight_height_);

    if (position_received_ && reachedTarget(pos[0], pos[1], flight_height_)) {
        auto [row, col] = idToRC(target_id);
        RCLCPP_INFO(get_logger(), "到达航点 %ld (ID: %d, A%dB%d)", 
                    path_index_ + 1, target_id, col + 1, row + 1);
        
        // 发布当前地图ID
        std_msgs::msg::Int32 msg;
        msg.data = target_id;
        current_id_pub_->publish(msg);
        
        path_index_++;
    }
}

void AnimalTestingNode::handleReturnHome() {
    if (!trigger_) {
        return;
    }

    publishControlMode();
    publishCommand(0.0, 0.0, flight_height_);

    if (position_received_ && reachedTarget(0.0, 0.0, flight_height_)) {
        RCLCPP_INFO(get_logger(), "返回原点完成，准备降落");
        changeState(MissionState::LAND);
    }
}

void AnimalTestingNode::handleLand() {
    publishLandCommand();

    const bool near_ground =
        position_received_ &&
        std::abs(current_position_[2]) < 0.15;

    if (near_ground && isVelocityNearZero()) {
        if (!land_stable_started_) {
            stable_start_time_ = now();
            land_stable_started_ = true;
        } else if ((now() - stable_start_time_).seconds() > stable_time_) {
            RCLCPP_INFO(get_logger(), "降落完成");
            changeState(MissionState::DONE);
        }
    } else {
        land_stable_started_ = false;
    }
}

/* -------------------- 回调函数 -------------------- */

void AnimalTestingNode::odomCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
    // NED坐标系
    current_position_[0] = msg->position[0];
    current_position_[1] = msg->position[1];
    current_position_[2] = msg->position[2];

    current_velocity_[0] = msg->velocity[0];
    current_velocity_[1] = msg->velocity[1];
    current_velocity_[2] = msg->velocity[2];

    position_received_ = true;
}

void AnimalTestingNode::noFlyCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
    if (msg->data.size() != 3) {
        RCLCPP_WARN(this->get_logger(), "接收到无效数量的禁飞区");
        return;
    }

    std::set<int> new_forbidden_ids(msg->data.begin(), msg->data.end());

    if (forbidden_ids_ready_ && new_forbidden_ids == forbidden_ids_) {
        return;
    }

    forbidden_ids_ = std::move(new_forbidden_ids);
    forbidden_ids_ready_ = true;

    std::string msg_str = "更新禁飞区ID:";
    for (int id : forbidden_ids_)
        msg_str += " " + std::to_string(id);
    RCLCPP_INFO(this->get_logger(), "%s", msg_str.c_str());
}

void AnimalTestingNode::startSignalCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    start_signal_from_gcs_ = msg->data;
    if (start_signal_from_gcs_) {
        RCLCPP_INFO(this->get_logger(), "收到地面站启动信号");
    }
}

void AnimalTestingNode::triggerCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    trigger_ = msg->data;
}

/* -------------------- 发布接口 -------------------- */

void AnimalTestingNode::publishCommand(double x, double y, double z, double yaw) {
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

void AnimalTestingNode::publishControlMode(bool position, bool velocity, bool acceleration) {
    px4_msgs::msg::OffboardControlMode msg;
    msg.timestamp = this->now().nanoseconds() / 1000;
    msg.position = position;
    msg.velocity = velocity;
    msg.acceleration = acceleration;
    msg.attitude = false;
    msg.body_rate = false;
    
    cmd_mode_pub_->publish(msg);
}

void AnimalTestingNode::publishTakeoffCommand() {
    std_msgs::msg::UInt8 msg;
    msg.data = 1;  // 1 = takeoff
    takeoff_land_pub_->publish(msg);
}

void AnimalTestingNode::publishLandCommand() {
    std_msgs::msg::UInt8 msg;
    msg.data = 2;  // 2 = land
    takeoff_land_pub_->publish(msg);
}

/* -------------------- 地图工具 -------------------- */

void AnimalTestingNode::getPos(int map_id, float pos[2]) {
    int row = map_id / 10 - 1;
    int col = map_id % 10 - 1;
    
    if (row < 0 || row >= ROWS || col < 0 || col >= COLS) {
        RCLCPP_ERROR(this->get_logger(), "无效的地图ID: %d", map_id);
        pos[0] = pos[1] = 0;
        return;
    }
    
    // NED坐标系：北(x)从4.0到0.0，东(y)从0.0到-3.0
    pos[0] = 4.0f - row * 0.5f;
    pos[1] = -col * 0.5f;
}

std::pair<int, int> AnimalTestingNode::idToRC(int id) {
    return {id / 10 - 1, id % 10 - 1};
}

int AnimalTestingNode::rcToID(int row, int col) {
    return (row + 1) * 10 + (col + 1);
}

int AnimalTestingNode::getMapIDFromPos(float x, float y) {
    // 从坐标反推行列
    int row = static_cast<int>((4.0f - x + 0.25f) / 0.5f);
    int col = static_cast<int>((-y + 0.25f) / 0.5f);

    if (row < 0 || row >= ROWS || col < 0 || col >= COLS) {
        return -1;
    }

    return rcToID(row, col);
}

/* -------------------- 路径规划 -------------------- */

std::vector<int> AnimalTestingNode::planPath(int start_id, int target_id) {
    auto [sr, sc] = idToRC(start_id);
    auto [tr, tc] = idToRC(target_id);

    std::vector<std::vector<bool>> visited(ROWS, std::vector<bool>(COLS, false));
    std::vector<std::vector<std::pair<int, int>>> parent(ROWS, std::vector<std::pair<int, int>>(COLS, {-1, -1}));

    std::vector<std::vector<bool>> grid(ROWS, std::vector<bool>(COLS, true));
    for (int id : forbidden_ids_) {
        auto [r, c] = idToRC(id);
        grid[r][c] = false;
    }

    std::queue<std::pair<int, int>> q;
    q.push({sr, sc});
    visited[sr][sc] = true;

    const int dr[4] = {-1, 1, 0, 0};
    const int dc[4] = {0, 0, -1, 1};

    bool found = false;
    while (!q.empty()) {
        auto [r, c] = q.front();
        q.pop();

        if (r == tr && c == tc) {
            found = true;
            break;
        }

        for (int i = 0; i < 4; ++i) {
            int nr = r + dr[i];
            int nc = c + dc[i];
            if (nr >= 0 && nr < ROWS && nc >= 0 && nc < COLS && 
                grid[nr][nc] && !visited[nr][nc]) {
                visited[nr][nc] = true;
                parent[nr][nc] = {r, c};
                q.push({nr, nc});
            }
        }
    }

    std::vector<int> path;
    if (!found) {
        RCLCPP_ERROR(this->get_logger(), "未找到从 %d 到 %d 的有效路径", start_id, target_id);
        return path;
    }

    int r = tr, c = tc;
    while (!(r == sr && c == sc)) {
        path.push_back(rcToID(r, c));
        std::tie(r, c) = parent[r][c];
    }
    path.push_back(start_id);
    std::reverse(path.begin(), path.end());
    
    return path;
}

std::vector<int> AnimalTestingNode::planFullTraversal() {
    // 生成所有非禁飞区的格点
    std::set<int> all_ids;
    for (int row = 0; row < ROWS; ++row) {
        for (int col = 0; col < COLS; ++col) {
            int id = rcToID(row, col);
            if (forbidden_ids_.find(id) == forbidden_ids_.end()) {
                all_ids.insert(id);
            }
        }
    }

    if (all_ids.empty()) {
        RCLCPP_ERROR(this->get_logger(), "没有可用格点，规划失败");
        return {};
    }

    // 贪心遍历：从91开始，每次选择最近的未访问点
    std::vector<int> final_path;
    int current = 91;
    std::set<int> unvisited = all_ids;
    unvisited.erase(current);
    final_path.push_back(current);

    while (!unvisited.empty()) {
        std::vector<int> best_segment;
        int next_target = -1;

        for (int candidate : unvisited) {
            std::vector<int> path = planPath(current, candidate);
            if (!path.empty() && (best_segment.empty() || path.size() < best_segment.size())) {
                best_segment = path;
                next_target = candidate;
            }
        }

        if (best_segment.empty()) {
            RCLCPP_ERROR(this->get_logger(), "剩余点不可达，提前结束");
            break;
        }

        final_path.insert(final_path.end(), best_segment.begin() + 1, best_segment.end());
        current = next_target;
        unvisited.erase(next_target);
    }

    // 返回起点
    std::vector<int> back_path = planPath(current, 91);
    if (!back_path.empty()) {
        final_path.insert(final_path.end(), back_path.begin() + 1, back_path.end());
    } else {
        RCLCPP_ERROR(this->get_logger(), "无法返回起点");
    }

    // 路径压缩
    std::vector<int> compressed_path = compressPath(final_path);
    RCLCPP_INFO(this->get_logger(), "路径压缩完成: %lu 点压缩为 %lu 点", 
                final_path.size(), compressed_path.size());

    // 发布最终路径
    std_msgs::msg::Int32MultiArray msg;
    msg.data = compressed_path;
    path_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "已发布压缩路径");

    return compressed_path;
}

std::vector<int> AnimalTestingNode::compressPath(const std::vector<int>& path) {
    if (path.size() <= 2)
        return path;

    RCLCPP_INFO(this->get_logger(), "[路径压缩] 原始路径大小: %lu", path.size());

    // 去除共线点
    std::vector<int> compressed;
    compressed.push_back(path[0]);

    for (size_t i = 1; i + 1 < path.size(); ++i) {
        auto [r_prev, c_prev] = idToRC(path[i - 1]);
        auto [r_curr, c_curr] = idToRC(path[i]);
        auto [r_next, c_next] = idToRC(path[i + 1]);

        int v1_r = r_curr - r_prev;
        int v1_c = c_curr - c_prev;
        int v2_r = r_next - r_curr;
        int v2_c = c_next - c_curr;

        // 如果不共线或者是重复点，保留
        if (v1_r * v2_c != v1_c * v2_r || path[i - 1] == path[i + 1]) {
            compressed.push_back(path[i]);
        }
    }

    compressed.push_back(path.back());
    RCLCPP_INFO(this->get_logger(), "[路径压缩] 压缩后大小: %lu", compressed.size());

    // 插入中点以避免过长的直线段
    std::vector<int> result;
    for (size_t i = 0; i + 1 < compressed.size(); ++i) {
        int start_id = compressed[i];
        int end_id = compressed[i + 1];
        result.push_back(start_id);

        auto [r1, c1] = idToRC(start_id);
        auto [r2, c2] = idToRC(end_id);

        int dr = r2 - r1;
        int dc = c2 - c1;

        // 如果距离较大，插入中点
        if (std::abs(dr) >= 2 || std::abs(dc) >= 4) {
            int mid_r = (r1 + r2) / 2;
            int mid_c = (c1 + c2) / 2;
            int mid_id = rcToID(mid_r, mid_c);

            if (forbidden_ids_.count(mid_id) == 0) {
                result.push_back(mid_id);
                RCLCPP_DEBUG(this->get_logger(), "[中点插入] %d 到 %d 插入 %d (A%dB%d)",
                            start_id, end_id, mid_id, mid_c + 1, mid_r + 1);
            } else {
                RCLCPP_WARN(this->get_logger(), "[中点被阻] %d 到 %d 中点 %d 是禁飞区", 
                           start_id, end_id, mid_id);
            }
        }
    }

    result.push_back(compressed.back());

    RCLCPP_INFO(this->get_logger(), "[路径压缩+中点] 最终路径大小: %lu", result.size());

    return result;
}

/* -------------------- 碰撞检测 -------------------- */

bool AnimalTestingNode::isPathBlockedByCoords(float x1, float y1, float x2, float y2) {
    for (int id : forbidden_ids_) {
        float pos[2];
        getPos(id, pos);
        float cx = pos[0];
        float cy = pos[1];
        float min_x = cx - 0.25f, max_x = cx + 0.25f;
        float min_y = cy - 0.25f, max_y = cy + 0.25f;

        if (lineIntersectsRect(x1, y1, x2, y2, min_x, max_x, min_y, max_y))
            return true;
    }
    return false;
}

bool AnimalTestingNode::lineIntersectsRect(float x1, float y1, float x2, float y2,
                                          float min_x, float max_x, float min_y, float max_y) {
    auto in_rect = [&](float x, float y) {
        return x >= min_x && x <= max_x && y >= min_y && y <= max_y;
    };

    if (in_rect(x1, y1) || in_rect(x2, y2))
        return true;

    int steps = 20;
    for (int i = 1; i < steps; ++i) {
        float t = static_cast<float>(i) / steps;
        float x = x1 + (x2 - x1) * t;
        float y = y1 + (y2 - y1) * t;
        if (in_rect(x, y))
            return true;
    }

    return false;
}

/* -------------------- 工具函数 -------------------- */

bool AnimalTestingNode::reachedTarget(double target_x, double target_y, double target_z) {
    double dx = current_position_[0] - target_x;
    double dy = current_position_[1] - target_y;
    double dz = current_position_[2] - target_z;

    double dist_sq = dx*dx + dy*dy + dz*dz;
    return dist_sq < (position_threshold_ * position_threshold_);
}

bool AnimalTestingNode::isVelocityNearZero() const {
    const double vx = current_velocity_[0];
    const double vy = current_velocity_[1];
    const double vz = current_velocity_[2];
    const double vel_sq = vx * vx + vy * vy + vz * vz;
    return vel_sq < (velocity_threshold_ * velocity_threshold_);
}

/* -------------------- Main -------------------- */

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<AnimalTestingNode>();
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}
