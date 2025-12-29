#include "2025_Ti_main.hpp"

OffboardNode_2025::OffboardNode_2025() : OffboardControlNode()
{
    // 创建订阅
    no_fly_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
        "/no_fly_zone", 10,
        std::bind(&OffboardNode_2025::no_fly_callback, this, std::placeholders::_1));

    start_signal_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/start_flight", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg)
        {
            start_signal_from_gcs_ = msg->data;
            if (start_signal_from_gcs_)
                RCLCPP_INFO(this->get_logger(), "Start signal received from GCS");
        });

    current_id_pub_ = this->create_publisher<std_msgs::msg::Int32>("/current_map_id", 10);
    path_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("/final_path", 10);

    pos_flag = true;
    pos[0] = 0.f;
    pos[1] = 0.f;
    mod_ = 0;
    path_index_ = 0;

    RCLCPP_INFO(this->get_logger(), "OffboardNode_2025 started.");
}

void OffboardNode_2025::get_pos(int map, float pos[2])
{
    int row = map / 10 - 1;
    int col = map % 10 - 1;
    if (row < 0 || row >= 9 || col < 0 || col >= 7)
    {
        RCLCPP_ERROR(this->get_logger(), "Invalid map ID: %d", map);
        pos[0] = pos[1] = 0;
        return;
    }
    pos[0] = 4.f - row * 0.5f;
    pos[1] = - col * 0.5f;
    // pos[0] = 0.9f - row * 0.1f;
    // pos[1] = - col * 0.1f;
}

bool OffboardNode_2025::is_path_blocked_by_coords(float x1, float y1, float x2, float y2)
{
    for (int id : forbidden_ids_)
    {
        float pos[2];
        get_pos(id, pos);
        float cx = pos[0];
        float cy = pos[1];
        float min_x = cx - 0.25f, max_x = cx + 0.25f;
        float min_y = cy - 0.25f, max_y = cy + 0.25f;

        if (line_intersects_rect(x1, y1, x2, y2, min_x, max_x, min_y, max_y))
            return true;
    }
    return false;
}

bool OffboardNode_2025::line_intersects_rect(float x1, float y1, float x2, float y2,
                                             float min_x, float max_x, float min_y, float max_y)
{
    auto in_rect = [&](float x, float y)
    {
        return x >= min_x && x <= max_x && y >= min_y && y <= max_y;
    };

    if (in_rect(x1, y1) || in_rect(x2, y2))
        return true;

    int steps = 20;
    for (int i = 1; i < steps; ++i)
    {
        float t = static_cast<float>(i) / steps;
        float x = x1 + (x2 - x1) * t;
        float y = y1 + (y2 - y1) * t;
        if (in_rect(x, y))
            return true;
    }

    return false;
}

void OffboardNode_2025::no_fly_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
{
    if (msg->data.size() != 3)
    {
        RCLCPP_WARN(this->get_logger(), "Received invalid number of no-fly zones");
        return;
    }

    std::set<int> new_forbidden_ids(msg->data.begin(), msg->data.end());

    if (forbidden_ids_ready_ && new_forbidden_ids == forbidden_ids_)
    {
        return;
    }

    forbidden_ids_ = std::move(new_forbidden_ids);
    forbidden_ids_ready_ = true;

    std::string msg_str = "Updated no-fly zone IDs:";
    for (int id : forbidden_ids_)
        msg_str += std::to_string(id) + " ";
    RCLCPP_INFO(this->get_logger(), "%s", msg_str.c_str());
}

std::vector<int> OffboardNode_2025::plan_path(int start_id, int target_id)
{
    float start_pos[2], target_pos[2];
    get_pos(start_id, start_pos);
    get_pos(target_id, target_pos);

    //if (!is_path_blocked_by_coords(start_pos[1], start_pos[0], target_pos[1], target_pos[0]))
    //{
    //    return {start_id, target_id}; 
    //}

    constexpr int ROWS = 9;
    constexpr int COLS = 7;
    std::vector<std::vector<bool>> visited(ROWS, std::vector<bool>(COLS, false));
    std::vector<std::vector<std::pair<int, int>>> parent(ROWS, std::vector<std::pair<int, int>>(COLS, {-1, -1}));

    std::vector<std::vector<bool>> grid(ROWS, std::vector<bool>(COLS, true));
    for (int id : forbidden_ids_)
    {
        auto [r, c] = id_to_rc(id);
        grid[r][c] = false;
    }

    auto [sr, sc] = id_to_rc(start_id);
    auto [tr, tc] = id_to_rc(target_id);

    std::queue<std::pair<int, int>> q;
    q.push({sr, sc});
    visited[sr][sc] = true;

    const int dr[4] = {-1, 1, 0, 0};
    const int dc[4] = {0, 0, -1, 1};

    bool found = false;
    while (!q.empty())
    {
        auto [r, c] = q.front();
        q.pop();

        if (r == tr && c == tc)
        {
            found = true;
            break;
        }

        for (int i = 0; i < 4; ++i)
        {
            int nr = r + dr[i];
            int nc = c + dc[i];
            if (nr >= 0 && nr < ROWS && nc >= 0 && nc < COLS && grid[nr][nc] && !visited[nr][nc])
            {
                visited[nr][nc] = true;
                parent[nr][nc] = {r, c};
                q.push({nr, nc});
            }
        }
    }

    std::vector<int> path;
    if (!found)
    {
        RCLCPP_ERROR(this->get_logger(), "No valid path found from %d to %d", start_id, target_id);
        return path;
    }

    int r = tr, c = tc;
    while (!(r == sr && c == sc))
    {
        path.push_back(rc_to_id(r, c));
        std::tie(r, c) = parent[r][c];
    }
    path.push_back(start_id);
    std::reverse(path.begin(), path.end());
    return path;
}

std::pair<int, int> OffboardNode_2025::id_to_rc(int id)
{
    return {id / 10 - 1, id % 10 - 1};
}

int OffboardNode_2025::rc_to_id(int row, int col)
{
    return (row + 1) * 10 + (col + 1);
}

std::vector<int> OffboardNode_2025::plan_full_traversal()
{
    constexpr int ROWS = 9;
    constexpr int COLS = 7;
    std::set<int> all_ids;
    for (int row = 0; row < ROWS; ++row)
        for (int col = 0; col < COLS; ++col)
        {
            int id = rc_to_id(row, col);
            if (forbidden_ids_.find(id) == forbidden_ids_.end())
                all_ids.insert(id);
        }

    if (all_ids.empty())
    {
        RCLCPP_ERROR(this->get_logger(), "No available grid, planning failed");
        return {};
    }

    std::vector<int> final_path;
    int current = 91;
    std::set<int> unvisited = all_ids;
    unvisited.erase(current);
    final_path.push_back(current);

    while (!unvisited.empty())
    {
        std::vector<int> best_segment;
        int next_target = -1;

        for (int candidate : unvisited)
        {
            std::vector<int> path = plan_path(current, candidate);
            if (!path.empty() && (best_segment.empty() || path.size() < best_segment.size()))
            {
                best_segment = path;
                next_target = candidate;
            }
        }

        if (best_segment.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Remaining points unreachable, ending early");
            break;
        }

        final_path.insert(final_path.end(), best_segment.begin() + 1, best_segment.end());
        current = next_target;
        unvisited.erase(next_target);
    }

    std::vector<int> back_path = plan_path(current, 91);
    if (!back_path.empty())
    {
        final_path.insert(final_path.end(), back_path.begin() + 1, back_path.end());
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Unable to return to start");
    }

    std::vector<int> compressed_path = compress_path(final_path);
    RCLCPP_INFO(this->get_logger(), "Path compression complete: %lu points compressed to %lu points", final_path.size(), compressed_path.size());

    std_msgs::msg::Int32MultiArray msg;
    msg.data = compressed_path;
    path_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Published compressed path");

    return compressed_path;
}

void OffboardNode_2025::set_mod(uint8_t new_mod) {
    mod_ = new_mod;
    std_msgs::msg::UInt8 mod_msg;
    mod_msg.data = mod_;
    mod_pub_->publish(mod_msg);
}

std::vector<int> OffboardNode_2025::compress_path(const std::vector<int> &path)
{
    if (path.size() <= 2)
        return path;

    RCLCPP_INFO(this->get_logger(), "[Path Compression] Original path size: %lu", path.size());

    std::vector<int> compressed;
    compressed.push_back(path[0]);

    for (size_t i = 1; i + 1 < path.size(); ++i)
    {
        auto [r_prev, c_prev] = id_to_rc(path[i - 1]);
        auto [r_curr, c_curr] = id_to_rc(path[i]);
        auto [r_next, c_next] = id_to_rc(path[i + 1]);

        int v1_r = r_curr - r_prev;
        int v1_c = c_curr - c_prev;
        int v2_r = r_next - r_curr;
        int v2_c = c_next - c_curr;

        if (v1_r * v2_c != v1_c * v2_r || path[i - 1] == path[i + 1])
        {
            compressed.push_back(path[i]);
        }
    }

    compressed.push_back(path.back());
    RCLCPP_INFO(this->get_logger(), "[Path Compression] Compressed size: %lu", compressed.size());

    std::vector<int> result;
    for (size_t i = 0; i + 1 < compressed.size(); ++i)
    {
        int start_id = compressed[i];
        int end_id = compressed[i + 1];
        result.push_back(start_id);

        auto [r1, c1] = id_to_rc(start_id);
        auto [r2, c2] = id_to_rc(end_id);

        int dr = r2 - r1;
        int dc = c2 - c1;

        if (std::abs(dr) >= 2 || std::abs(dc) >= 4)
        {
            int mid_r = (r1 + r2) / 2;
            int mid_c = (c1 + c2) / 2;
            int mid_id = rc_to_id(mid_r, mid_c);

            if (forbidden_ids_.count(mid_id) == 0) {
                result.push_back(mid_id);
                RCLCPP_DEBUG(this->get_logger(), "[Midpoint Insertion] %d to %d insert %d (A%dB%d)",
                            start_id, end_id, mid_id, mid_c + 1, mid_r + 1);
            } else {
                RCLCPP_WARN(this->get_logger(), "[Midpoint Blocked] %d to %d midpoint %d is forbidden", start_id, end_id, mid_id);
            }
        }
    }

    result.push_back(compressed.back());

    RCLCPP_INFO(this->get_logger(), "[Path Compression + Midpoint] Final path size: %lu", result.size());

    // Verify path integrity
    std::set<int> result_set(result.begin(), result.end());
    for (int id : path)
    {
        if (result_set.count(id) == 0 && forbidden_ids_.count(id) == 0)
        {
            auto [r, c] = id_to_rc(id);
            RCLCPP_WARN(this->get_logger(), "Path compression issue: missing %d (A%dB%d)", id, c + 1, r + 1);
        }
    }

    return result;
}





int OffboardNode_2025::get_map_id_from_pos(float x, float y)
{
    // Row decreases as x goes from 4.0 to 0.0
    int row = static_cast<int>((4.0f - x + 0.25f) / 0.5f);
    int col = static_cast<int>((-y + 0.25f) / 0.5f);

    if (row < 0 || row >= 9 || col < 0 || col >= 7)
    {
        RCLCPP_WARN(this->get_logger(), "Current position out of map range: x=%.2f y=%.2f, row=%d col=%d", x, y, row, col);
        return -1;
    }

    return rc_to_id(row, col);
}


void OffboardNode_2025::control_loop()
{
    auto now = this->now();

    float x = current_position_[0];
    float y = current_position_[1];
    int current_id = get_map_id_from_pos(x, y);

    if (current_id > 0) {
        std_msgs::msg::Int32 msg;
        msg.data = current_id;
        current_id_pub_->publish(msg);
        RCLCPP_DEBUG(this->get_logger(), "Current position x=%.2f y=%.2f maps to grid %d", x, y, current_id);
    } else {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "Current position out of map range: x=%.2f y=%.2f", x, y);
    }

    // 1. Wait for FCU connection
    if (!current_state_.connected) {
        RCLCPP_WARN(this->get_logger(), "Waiting for FCU connection...");
        return;
    }

    // 2. Pre-warm by sending 100 setpoints
    if (setpoint_counter_ < 100) {
        publish_setpoint();
        setpoint_counter_++;
        if (setpoint_counter_ == 100) {
            RCLCPP_INFO(this->get_logger(), "Setpoint pre-warming complete");
        }
        return;
    }

    switch (mod_) {
        case 0:  // Path planning
            if (forbidden_ids_ready_) {
                current_path_ = plan_full_traversal();
                path_index_ = 0;

                if (current_path_.empty()) {
                    RCLCPP_ERROR(this->get_logger(), "No valid path found, stopping mission");
                    set_mod(99);
                } else {
                    RCLCPP_INFO(this->get_logger(), "Path planning complete, waiting for GCS signal...");
                    set_mod(1);
                }
            }
            break;

        case 1:  // Wait for GCS signal
            if (start_signal_from_gcs_) {
                start_signal_from_gcs_ = false;
                last_request_ = now;
                set_mod(2);
            }
            break;

        case 2:  // Switch to OFFBOARD and arm
            if (current_state_.mode != "OFFBOARD") {
                if ((now - last_request_).seconds() > 2.0 && set_mode_client_->service_is_ready()) {
                    auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
                    req->custom_mode = "OFFBOARD";
                    set_mode_client_->async_send_request(req);
                    RCLCPP_INFO(this->get_logger(), "Requesting OFFBOARD mode...");
                    last_request_ = now;
                }
            } 
            else if (!current_state_.armed) {
                if ((now - last_request_).seconds() > 2.0 && arming_client_->service_is_ready()) {
                    auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
                    req->value = true;
                    arming_client_->async_send_request(req);
                    RCLCPP_INFO(this->get_logger(), "Arming...");
                    last_request_ = now;
                }
            } 
            else {
                RCLCPP_INFO(this->get_logger(), "OFFBOARD & armed complete, taking off...");
                set_mod(3);
            }
            break;

        case 3:  // Takeoff phase
            target_position_ = {0.0, 0.0, 1.2};
            publish_setpoint();
            if (reached_target()) {
                RCLCPP_INFO(this->get_logger(), "Takeoff complete, starting path traversal");
                set_mod(4);
            }
            break;

        case 4:  // Path traversal
            if (path_index_ >= current_path_.size()) {
                RCLCPP_INFO(this->get_logger(), "All waypoints traversed");
                set_mod(5);
                break;
            }

            else if (pos_flag) {
                get_pos(current_path_[path_index_], pos);
                target_position_ = {pos[0], pos[1], 1.2};
                pos_flag = false;
                RCLCPP_INFO(this->get_logger(), "Moving to grid %d at x=%.1f, y=%.1f", current_path_[path_index_], pos[0], pos[1]);
            }

            else if (reached_target()) {
                std_msgs::msg::Int32 msg;
                msg.data = current_path_[path_index_];
                current_id_pub_->publish(msg);
                path_index_++;
                pos_flag = true;
            }
            break;

        case 5:  // Return to home
            target_position_ = {0, 0, 1.2};
            //publish_setpoint();
            if (reached_target()) {
                RCLCPP_INFO(this->get_logger(), "Returned to home, preparing to land");
                set_mod(6);
            }
            break;

        case 6:  // Descending phase
            publish_velocity(0.0, 0.0, -0.3);
            if (current_position_[2] < 0.5 && std::abs(current_velocity_[2]) < 0.05) {
                RCLCPP_INFO(this->get_logger(), "Close to ground, executing LAND");
                set_mod(7);
            }
            break;

        case 7:  // Landing
            land();
            break;

        default:
            break;
    }

    // Continuously publish setpoint commands
    if (mod_ <= 5) {
        publish_setpoint();
    }
}



int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OffboardNode_2025>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
