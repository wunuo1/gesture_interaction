#ifndef GESTURE_CONTROL_NODE_H
#define GESTURE_CONTROL_NODE_H

#include <string>
#include <queue>
#include "rclcpp/rclcpp.hpp"


#include "ai_msgs/msg/capture_targets.hpp"
#include "ai_msgs/msg/perception_targets.hpp"


#include "RobotLegs.h"


enum class GestureCtrlType 
{
    ThumbUp = 2,      // 点赞
    Victory = 3,      // V手势
    Palm = 5,         // 手掌
    Okay = 11,        //  OK手势
    ThumbRight = 12,  // 大拇指向左
    ThumbLeft = 13,   // 大拇指向右
    Awesome = 14,      // 666手势
    None = 0,       // 无手势
};


class GestureControlNode : public rclcpp::Node 
{
    public:
        GestureControlNode();
        ~GestureControlNode();


    private:
       
        int gesture_value_ = 0;
        size_t queue_len_limit_ = 25;
        bool need_exchange_ = false;
        std::vector<int> gesture_value_vector;
        std::queue<int> gesture_value_queue;
        GestureCtrlType gesture_ctrl_type_ = GestureCtrlType::None;
        // std::string ai_msg_sub_topic_name_ = "/hobot_hand_gesture_detection";
        
        std::string ai_msg_sub_topic_name_ = "/tros_perc_fusion";
        rclcpp::Subscription<ai_msgs::msg::PerceptionTargets>::SharedPtr smart_subscription_ = nullptr;

        std::unique_ptr<RobotLegs> robotLegs_ptr_ = nullptr;
        // RobotLegs robotLegs;


        std::thread gesture_control_thread_;
        void gesture_control_loop();
        void SmartTopicCallback(const ai_msgs::msg::PerceptionTargets::ConstSharedPtr msg);
};

#endif  // GESTURE_STRATEGY_NODE_H