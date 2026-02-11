#ifndef ACTUATORS_CONTROL_H
#define ACTUATORS_CONTROL_H

#include <vector>
#include <string>
#include <Python.h>

#include "lighting_control/ws2812b.h"
#include "servo_control/hobot_pwm.h"
/* Macro config define */
#define RIGHT_LEG_PIN  33U
#define LEFT_LEG_PIN   32U

enum class ActuatorsState 
{
    NONE = -1, // 未初始化
    Relaxed = 0,   // 放松
    LiftLeft = 1,  // 抬起左腿
    LiftRight = 2, // 抬起右腿
    Stand = 3,     // 站立
    ShakeLeft = 4, // 抖动左腿
    ShakeRight = 5,// 抖动右腿
    ShakeEars = 6, // 抖动耳朵
    FlashingLight = 7     // 灯光闪烁
};

class ActuatorsControl 
{
    public:
        // 构造函数和析构函数
        ActuatorsControl(int leftLegPin, int rightLegPin);
        ~ActuatorsControl();
        
        // 初始化函数
        bool initializeLegs();
        
        // 基本动作控制
        void liftLeftLeg(int speed = 100);
        void liftRightLeg(int speed = 100);

        void lowerLeftLeg(int speed = 100);
        void lowerRightLeg(int speed = 100);

        void standStraight();
        void relaxLegs();
        
        // 复合动作
        void shakeEars(int speed = 100);
        
        // 灯光控制
        void flashingLight(int speed = 100);

        // 舵机控制接口
        void servoControl(int index, float duty);

        // 状态查询
        std::string getCurrentPose() const;


        
    private:
        // 引脚配置
        int leftLegPin;
        int rightLegPin;
        ActuatorsState robot_legs_state = ActuatorsState::NONE;   
        WS2812B lamp_;
        HobotPWM pwmController_;
};



#endif // ACTUATORS_CONTROL_H
