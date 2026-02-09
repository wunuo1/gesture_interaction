#include "fc_call_node.h"
#include <regex>

void FcCallNode::LeftControl(int angle) {
  
  robotLegs_ptr_->servoControl(0, 0 - ((angle - 30) / 15.0));
}

void FcCallNode::RightControl(int angle) {
  robotLegs_ptr_->servoControl(1, 0 + ((angle - 30) / 15.0));
}

FcCallNode::FcCallNode(): rclcpp::Node("fc_call_node"){
  robotLegs_ptr_ = std::make_unique<ActuatorsControl>(LEFT_LEG_PIN, RIGHT_LEG_PIN);
  robotLegs_ptr_->initializeLegs();
  func_map_["left_control"]  = [this](int angle){ this->LeftControl(angle) ;};
  func_map_["right_control"] = [this](int angle){ this->RightControl(angle); };
  fc_msg_subscription_ =
      this->create_subscription<std_msgs::msg::String>(
          "fc_text", rclcpp::QoS(5).reliable(),
          std::bind(&FcCallNode::FcMsgCallback, this, std::placeholders::_1));
}

void FcCallNode::FcMsgCallback(const std_msgs::msg::String::ConstSharedPtr msg){
  auto input = msg->data;
  if (input.size() < 2 || input.front() != '{' || input.back() != '}') {
      return;
  }
  std::string content = input.substr(1, input.size() - 2);
  static const std::regex pattern(R"(\[([^\]]+)\]\[([^\]]+)\])");
  for (auto it = std::sregex_iterator(content.begin(), content.end(), pattern);
        it != std::sregex_iterator(); ++it) {

      auto fun_name = (*it)[1].str();
      auto fun_param = std::stoi((*it)[2].str());
      if (func_map_.count(fun_name)) {
          func_map_[fun_name](fun_param);  // ✅ 使用变量调用函数
      }
  }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    RCLCPP_INFO(rclcpp::get_logger("FcCallNode"), "Function call Node is starting...");

    auto function_call_node = std::make_shared<FcCallNode>();
    rclcpp::spin(function_call_node);
    rclcpp::shutdown();
    return 0;
}