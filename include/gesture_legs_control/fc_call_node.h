#ifndef FC_CALL_NODE_H
#define FC_CALL_NODE_H

#include <string>
#include <queue>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "RobotLegs.h"

#define RIGHT_LEG_PIN  32U
#define LEFT_LEG_PIN   33U

class FcCallNode : public rclcpp::Node{
public:
    FcCallNode();
    ~FcCallNode(){};
private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr fc_msg_subscription_ = nullptr;
    std::unique_ptr<RobotLegs> robotLegs_ptr_ = nullptr;
    void FcMsgCallback(const std_msgs::msg::String::ConstSharedPtr msg);
    void RightControl(int angle);
    void LeftControl(int angle);
    std::unordered_map<std::string, std::function<void(int)>> func_map_;
    
};

#endif //FC_CALL_NODE_H