#ifndef TI_2025_HPP
#define TI_2025_HPP

#include "lib/offboard_control_node.hpp"

class OffboardNode_2025 : public OffboardControlNode
{
public:
    OffboardNode_2025();

protected:
    void control_loop() override;

    // ✅ 地图工具函数
    void get_pos(int map_id, float pos[2]); // 地图编号 → 中心坐标
    std::pair<int, int> id_to_rc(int id);   // 编号转行列
    int rc_to_id(int row, int col);         // 行列转编号

    // ✅ 路径规划（贪心 + BFS 路径规划）
    std::vector<int> plan_path(int start_id, int target_id);   // 从起点到目标点 BFS 搜索路径
    std::vector<int> plan_full_traversal();                    // 规划整个路径遍历非禁飞区

    // ✅ 控制状态
    int map_num = 0;      // 当前路径执行到第几个格子
    bool pos_flag = true; // 是否需要更新目标点
    float pos[2] = {0};   // 当前目标点坐标
    
    
    std::vector<int> current_path_;
    size_t path_index_ = 0;
    std::vector<int> full_path_; // 最终生成的完整飞行路径
    bool path_ready_ = false;

    // ✅ 禁飞区
    std::set<int> forbidden_ids_;  // 禁飞区集合
    bool forbidden_ids_ready_ = false;

    bool start_signal_from_gcs_ = false;  // 默认未启动
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_signal_sub_;

    // ✅ ROS接口
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr no_fly_sub_; // 接收禁飞区
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr current_id_pub_;         // 发布当前map_id
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr path_pub_; // 发布最终路径

    // ✅ 回调
    void no_fly_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg); // 接收禁飞区数据
    bool is_path_blocked_by_coords(float x1, float y1, float x2, float y2);
    bool line_intersects_rect(float x1, float y1, float x2, float y2,
                            float min_x, float max_x, float min_y, float max_y);
    std::vector<int> compress_path(const std::vector<int> &path);
    int get_map_id_from_pos(float x, float y);

    void set_mod(uint8_t new_mod);

};

#endif // TI_2025_HPP
