#include <lib/param.hpp>

Param_t::Param_t(){
}

void Param_t::getStaticParam(const std::shared_ptr<rclcpp::Node>& node){
    readStaticParam(node, "msg_timeout.rc", msg_timeout.rc);
    readStaticParam(node, "msg_timeout.odom", msg_timeout.odom);
    readStaticParam(node, "msg_timeout.offboard", msg_timeout.offboard);
    readStaticParam(node, "msg_timeout.offboardMode", msg_timeout.offboardMode);
    readStaticParam(node, "msg_timeout.bat", msg_timeout.bat);
    readStaticParam(node, "takeoff_land.enable", takeoff_land.enable);
    readStaticParam(node, "takeoff_land.enable_arm", takeoff_land.enable_arm);
    readStaticParam(node, "takeoff_land.speed", takeoff_land.speed);
    readStaticParam(node, "takeoff_land.height", takeoff_land.height);
    readStaticParam(node, "low_voltage", low_voltage);
    readStaticParam(node, "odom_pos_jump", odom_pos_jump);
    // 如需调整 ch 名称，请修改下面的三个参数
    readStaticParam(node, "rc_debug.ch_p", rc_debug.ch_p);
    readStaticParam(node, "rc_debug.ch_i", rc_debug.ch_i);
    readStaticParam(node, "rc_debug.ch_d", rc_debug.ch_d);
    readStaticParam(node, "rc_debug.ch_mode", rc_debug.ch_mode);
    readStaticParam(node, "rc_debug.ch_gear", rc_debug.ch_gear);
}

// 动态参数回调实例
DynamicParamNode::DynamicParamNode() : Node("dynamic_param_node"){
    // 声明参数
    params_.rc_debug.p = declare_parameter("rc_debug.p", params_.rc_debug.p);
    params_.rc_debug.i = declare_parameter("rc_debug.i", params_.rc_debug.i);
    params_.rc_debug.d = declare_parameter("rc_debug.d", params_.rc_debug.d);
    // 注册回调
    param_cb_ = add_on_set_parameters_callback(
        std::bind(&DynamicParamNode::updateDynamicParams, this, std::placeholders::_1));
}

rcl_interfaces::msg::SetParametersResult DynamicParamNode::updateDynamicParams(
    const std::vector<rclcpp::Parameter>& parameters){
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";

    for (const auto& param : parameters) {
        if (param.get_name() == "rc_debug.p"){
            params_.rc_debug.p = param.as_double();
        }
            
        else if (param.get_name() == "rc_debug.i"){
            params_.rc_debug.i = param.as_double();
        }
            
        else if (param.get_name() == "rc_debug.d"){
            params_.rc_debug.d = param.as_double();
        }
    }
    return result;
}

