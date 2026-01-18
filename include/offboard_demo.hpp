#ifndef DEMO_HPP
#define DEMO_HPP

#include "lib/offboard_control_node.hpp"


class OffboardNode_Demo : public OffboardControlNode {
public:
    OffboardNode_Demo();

protected:
    void control_loop() override;
};

#endif // DEMO_HPP
