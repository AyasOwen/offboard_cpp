#include <node.hpp>

OffboardControlNode::OffboardControlNode() : Node("offboard_control_node"){
}

void OffboardControlNode::init(const std::shared_ptr<OffboardControlNode>& self) {
    // 获取参数配置
    param.getStaticParam(self);
    param.initDynamicParams(self);
    param_cb_ = this->add_on_set_parameters_callback(
        [this, self](const std::vector<rclcpp::Parameter>& parameters) {
            return param.updateDynamicParams(self, parameters);
        });
    #ifdef TEXT_RC
    this->declare_parameter("mock_rc_mode", 0.0);
    this->declare_parameter("mock_rc_gear", 0.0);
    #endif
    // 创建状态机
    fsm = std::make_unique<CtrlFSM>(param, self);

    // 定义 QoS 策略
    auto qos_sensor = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();
    auto qos_cmd = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
    
    // 初始化发布者
    fsm->offboard_pub = this->create_publisher<mavros_msgs::msg::PositionTarget>(
        "mavros/setpoint_raw/local", qos_cmd);
    fsm->trigger_pub = this->create_publisher<std_msgs::msg::Bool>(
        "offboard/trigger", qos_cmd);
    fsm->offboard_mode_pub = this->create_publisher<std_msgs::msg::UInt16>(
        "offboard/cmd_mode", qos_cmd);

    fsm->set_mode_client = this->create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");
    fsm->arm_client = this->create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");
    fsm->land_client = this->create_client<mavros_msgs::srv::CommandTOL>("mavros/cmd/land");

    // 初始化订阅者
    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        "mavros/local_position/odom", qos_sensor,
        [this](nav_msgs::msg::Odometry::SharedPtr msg) {
            fsm->odom_data.feed(msg, param);
        });
    
    state_sub = this->create_subscription<mavros_msgs::msg::State>(
        "mavros/state", qos_sensor,
        [this](mavros_msgs::msg::State::SharedPtr msg) {
            fsm->state_data.feed(msg);
        });

    rc_sub = this->create_subscription<mavros_msgs::msg::RCIn>(
        "mavros/rc/in", qos_sensor,
        [this](mavros_msgs::msg::RCIn::SharedPtr msg) {
            fsm->rc_data.feed(msg, param);
        });

    offboard_sub = this->create_subscription<mavros_msgs::msg::PositionTarget>(
        "offboard/cmd", qos_cmd,
        [this](mavros_msgs::msg::PositionTarget::SharedPtr msg) {
            fsm->offboard_data.feed(msg);
        });

    offboard_mode_sub = this->create_subscription<std_msgs::msg::UInt16>(
        "offboard/cmd_mode", qos_cmd,
        [this](std_msgs::msg::UInt16::SharedPtr msg) {
            fsm->offboard_mode_data.feed(msg);
        });

    battery_sub = this->create_subscription<sensor_msgs::msg::BatteryState>(
        "mavros/battery", qos_sensor,
        [this](sensor_msgs::msg::BatteryState::SharedPtr msg) {
            fsm->battery_data.feed(msg);
        });

    takeoff_land_sub = this->create_subscription<std_msgs::msg::UInt8>(
        "offboard/takeoff_land", qos_cmd,
        [this](std_msgs::msg::UInt8::SharedPtr msg) {
            fsm->takeoff_land_data.feed_takeoff_land(msg);
        });

    land_detected_sub = this->create_subscription<mavros_msgs::msg::ExtendedState>(
        "mavros/extended_state", qos_sensor,
        [this](mavros_msgs::msg::ExtendedState::SharedPtr msg) {
            fsm->takeoff_land_data.feed_landed(msg);
        });

    // 创建定时器，定期调用状态机
    timer = this->create_wall_timer(
        std::chrono::milliseconds(20),  // 50Hz
        [this]() {
            fsm->FSM();
        });

    RCLCPP_INFO(this->get_logger(), "Offboard Control Node initialized");
}

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