#ifndef __PARAM_HPP
#define __PARAM_HPP

#include <rclcpp/rclcpp.hpp>


// 静态参数，启动后不能再修改
class Param_t{
public:
    // 静态参数
    // 超时参数
    struct MsgTimeOut{
        double rc;
        double odom;
        double offboard;
        double offboardMode;
        double bat;
    };

    // 起飞降落用参数
    struct TakeoffLand{
        bool enable;        // 是否启用自动起飞
        bool enable_arm;    // 是否启用起飞时自动解锁(arm)无人机
        double speed;       // 起飞降落速度
        double height;      // 起飞高度
    };

    double low_voltage;     // 低电压报警阈值。例如 4S 电池单节 3.3V 时报警，触发安全保护。
    double odom_pos_jump;   // Odom 位置数据跳变阈值
    // 实例化
    MsgTimeOut msg_timeout;
    TakeoffLand takeoff_land;

    //----------------------------------------------
    // 动态参数
    // 手柄映射调试用参数，根据需求自行修改
    struct RCDebug{
        // 这里以 PID 为例，ch 为遥控器对应通道
        // 同理也能改成 OpenCv 的 RGB阈值
        // p, i, d(也可以是其他名字)为动态参数
        double p;
        double i;
        double d;

        // ch 通道为静态参数
        // 通道映射到遥控器上不允许动态修改，非常危险
        int ch_p;
        int ch_i;
        int ch_d;
        int ch_mode;
        int ch_gear;
    };

    // 实例化
    RCDebug rc_debug;


    Param_t();
    void getStaticParam(const std::shared_ptr<rclcpp::Node>& node);

private:
    // 读取静态参数的模板
    template<typename TName, typename TVal>
    void readStaticParam(const std::shared_ptr<rclcpp::Node>& node, TName& name, TVal& val){
        try
        {
            val = node->declare_parameter<TVal>(name, val);

            rclcpp::Parameter param(name, val);
            RCLCPP_INFO(node->get_logger(),
                        "Read param %s: %s",
                        name.c_str(),
                        param.value_to_string().c_str());
        }

        catch (const std::exception& e)
        {
            RCLCPP_FATAL(node->get_logger(),
                        "Failed to read essential parameter %s: %s",
                        name.c_str(), e.what());
            throw;
        }       
    }
};

// 动态参数回调 Node
class DynamicParamNode : public rclcpp::Node{
public:
    DynamicParamNode();

private:
    Param_t params_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
    rcl_interfaces::msg::SetParametersResult updateDynamicParams(
        const std::vector<rclcpp::Parameter>& parameters);
};

#endif  // PARAM_HPP
