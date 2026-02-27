#include <lib/node.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    // 创建控制节点
    auto node = std::make_shared<OffboardControlNode>();
    
    // 初始化节点（必须在 shared_ptr 创建后调用）
    node->init(node);
    
    RCLCPP_INFO(node->get_logger(), "Offboard Control Node 启动成功！");
    
    // 运行节点
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}
