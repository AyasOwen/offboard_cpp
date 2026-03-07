#ifndef ANIMAL_TESTING_HPP
#define ANIMAL_TESTING_HPP

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>

#include <vector>
#include <array>
#include <queue>
#include <set>
#include <utility>
#include <cmath>
#include <algorithm>

class AnimalTestingNode : public rclcpp::Node {
public:
    AnimalTestingNode();

private:
    /* -------------------- 任务状态 -------------------- */
    enum class MissionState {
        PLANNING,      // 路径规划
        WAIT_GCS,      // 等待地面站信号
        TAKEOFF,       // 起飞
        WAYPOINT,      // 航点飞行
        RETURN_HOME,   // 返回原点
        LAND,          // 降落
        DONE           // 任务完成
    };

    void changeState(MissionState new_state);

    /* -------------------- ROS 通信 -------------------- */
    // 发布器
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr cmd_pub_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr cmd_mode_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr takeoff_land_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr current_id_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr path_pub_;

    // 订阅器
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr no_fly_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_signal_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr trigger_sub_;

    rclcpp::TimerBase::SharedPtr timer_;

    /* -------------------- 回调函数 -------------------- */
    void timerCallback();
    void odomCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
    void noFlyCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg);
    void startSignalCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void triggerCallback(const std_msgs::msg::Bool::SharedPtr msg);

    /* -------------------- 发布接口 -------------------- */
    void publishCommand(double x, double y, double z, double yaw = 0.0);
    void publishControlMode(bool position = true, bool velocity = false, bool acceleration = false);
    void publishTakeoffCommand();
    void publishLandCommand();

    /* -------------------- 任务逻辑 -------------------- */
    void handlePlanning();
    void handleWaitGCS();
    void handleTakeoff();
    void handleWaypoint();
    void handleReturnHome();
    void handleLand();

    /* -------------------- 地图工具 -------------------- */
    void getPos(int map_id, float pos[2]);                    // 地图编号 → 中心坐标
    std::pair<int, int> idToRC(int id);                       // 编号转行列
    int rcToID(int row, int col);                             // 行列转编号
    int getMapIDFromPos(float x, float y);                    // 坐标 → 地图编号

    /* -------------------- 路径规划 -------------------- */
    std::vector<int> planPath(int start_id, int target_id);   // BFS 搜索路径
    std::vector<int> planFullTraversal();                     // 完整遍历路径
    std::vector<int> compressPath(const std::vector<int>& path);  // 路径压缩

    /* -------------------- 碰撞检测 -------------------- */
    bool isPathBlockedByCoords(float x1, float y1, float x2, float y2);
    bool lineIntersectsRect(float x1, float y1, float x2, float y2,
                           float min_x, float max_x, float min_y, float max_y);

    /* -------------------- 工具函数 -------------------- */
    bool reachedTarget(double x, double y, double z);
    bool isVelocityNearZero() const;

    /* -------------------- 数据成员 -------------------- */
    MissionState mission_state_{MissionState::PLANNING};
    rclcpp::Time state_start_time_;

    // 位置和速度
    std::array<double, 3> current_position_{0.0, 0.0, 0.0};
    std::array<double, 3> current_velocity_{0.0, 0.0, 0.0};
    bool position_received_{false};

    // 禁飞区
    std::set<int> forbidden_ids_;
    bool forbidden_ids_ready_{false};

    // 路径
    std::vector<int> current_path_;
    size_t path_index_{0};
    std::array<float, 2> target_pos_{0.0f, 0.0f};

    // 信号
    bool start_signal_from_gcs_{false};
    bool trigger_{false};

    // 起飞/降落判定
    bool stable_started_{false};
    rclcpp::Time stable_start_time_;
    bool takeoff_command_sent_{false};
    double takeoff_start_z_{0.0};
    bool land_stable_started_{false};

    // 参数
    double position_threshold_{0.1};
    double velocity_threshold_{0.12};
    double stable_time_{1.0};
    double flight_height_{-1.2};  // NED坐标系，负值表示向上

    // 地图参数
    static constexpr int ROWS = 9;
    static constexpr int COLS = 7;
};

#endif // ANIMAL_TESTING_HPP
