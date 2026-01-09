/*
 * 手势说明表：
 * 
 * 手势名称      说明                数值
 * --------     ----------------   ----
 * ThumbUp      竖起大拇指           2
 * Victory      "V"手势             3
 * Mute         "嘘"手势            4
 * Palm         手掌                5
 * Okay         OK手势              11
 * ThumbLeft    大拇指向左           12
 * ThumbRight   大拇指向右           13
 * Awesome      666手势             14
 */
#include <rclcpp/rclcpp.hpp>
#include "gesture_control_node.h"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    RCLCPP_INFO(rclcpp::get_logger("GestureControlNode"), "Gesture Control Node is starting...");

    auto gesture_control_node = std::make_shared<GestureControlNode>();
    rclcpp::spin(gesture_control_node);
    rclcpp::shutdown();
    return 0;
}