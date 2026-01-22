# 功能介绍

gesture_legs_control package是MagicBox用于控制舵机的功能包，实现功能如下：

- gesture_legs_control：接收手势消息，实现对应动作

- function_call_control：接收大模型输出的指令，控制对应舵机，需配合大模型使用

# 编译

- 编译命令：

```shell
cd /userdata/MagicBox/app/ros_ws
colcon build --packages-select gesture_legs_control
```
# 运行


```shell
cd /userdata/MagicBox
ros2 launch gesture_legs_control gesture_legs_control.launch.py
```

# 手势说明
|手势      |手部动作   |触发动作  |
|----------|----------|---------|
|ThumbUp   |竖起大拇指 |耳朵摇晃  |
|Victory   |“V”手势    |双脚撑起 |
|ThumbLeft |大拇指向左 |举起左手  |
|ThumbRight|大拇指向右 |举起右手  |
|Okay      |OK手势     |灯光闪烁 |