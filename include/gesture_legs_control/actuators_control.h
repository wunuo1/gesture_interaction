#ifndef ROBOT_LEGS_H
#define ROBOT_LEGS_H

#include <vector>
#include <string>
#include <Python.h>

#include "lighting_control/ws2812b.h"
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


class HobotPWM {
public:
    HobotPWM() {
        // 初始化 Python 解释器
        Py_Initialize();

        // 导入 Hobot.GPIO
        pGPIO = PyImport_ImportModule("Hobot.GPIO");
        if (!pGPIO) {
            PyErr_Print();
            throw std::runtime_error("Failed to import Hobot.GPIO");
        }

        // 设置 warnings=False
        PyObject* pSetWarnings = PyObject_GetAttrString(pGPIO, "setwarnings");
        if (pSetWarnings && PyCallable_Check(pSetWarnings)) {
            PyObject_CallFunction(pSetWarnings, "O", Py_False);
        }
        Py_XDECREF(pSetWarnings);

        // 设置 mode: GPIO.setmode(GPIO.BOARD)
        PyObject* pSetMode = PyObject_GetAttrString(pGPIO, "setmode");
        PyObject* pBOARD = PyObject_GetAttrString(pGPIO, "BOARD");
        if (pSetMode && PyCallable_Check(pSetMode) && pBOARD) {
            PyObject_CallFunctionObjArgs(pSetMode, pBOARD, NULL);
        }
        Py_XDECREF(pSetMode);
        Py_XDECREF(pBOARD);

        // 获取 PWM 类
        pPWMClass = PyObject_GetAttrString(pGPIO, "PWM");
        if (!pPWMClass) {
            throw std::runtime_error("Failed to get PWM class");
        }
    }

    ~HobotPWM() {
        // 停止所有 PWM
        for (auto& pwm : pwmObjects) {
            PyObject* pStop = PyObject_GetAttrString(pwm, "stop");
            if (pStop) {
                PyObject_CallObject(pStop, NULL);
                Py_XDECREF(pStop);
            }
            Py_XDECREF(pwm);
        }
        pwmObjects.clear();

        // 清理 GPIO
        PyObject* pCleanup = PyObject_GetAttrString(pGPIO, "cleanup");
        if (pCleanup) {
            PyObject_CallObject(pCleanup, NULL);
            Py_XDECREF(pCleanup);
        }

        Py_XDECREF(pPWMClass);
        Py_XDECREF(pGPIO);

        // 结束 Python
        Py_Finalize();
    }

    // 添加 PWM
    void addPWM(int pin, int freq) {
        PyObject* pwm = PyObject_CallFunction(pPWMClass, "ii", pin, freq);
        if (!pwm) {
            PyErr_Print();
            throw std::runtime_error("Failed to create PWM object");
        }
        pwmObjects.push_back(pwm);
    }

    // 启动 PWM 并设置占空比
    void startPWM(int index, float duty) {
        if (index < 0 || index >= pwmObjects.size()) return;

        PyObject* pChangeDuty = PyObject_GetAttrString(pwmObjects[index], "ChangeDutyCycle");
        PyObject* pStart = PyObject_GetAttrString(pwmObjects[index], "start");
        if (pChangeDuty && pStart) {
            PyObject_CallFunction(pChangeDuty, "f", duty);
            PyObject_CallFunction(pStart, "f", duty);
        }
        Py_XDECREF(pChangeDuty);
        Py_XDECREF(pStart);
    }

    // 修改占空比
    void setDutyCycle(int index, float duty) {
        if (index < 0 || index >= pwmObjects.size()) return;
        PyObject* pChangeDuty = PyObject_GetAttrString(pwmObjects[index], "ChangeDutyCycle");
        if (pChangeDuty) {
            PyObject_CallFunction(pChangeDuty, "f", duty);
        }
        Py_XDECREF(pChangeDuty);
    }

private:
    PyObject* pGPIO = nullptr;
    PyObject* pPWMClass = nullptr;
    std::vector<PyObject*> pwmObjects;
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
        WS2812B strip;
        HobotPWM pwmController_;
};



#endif // ROBOT_LEGS_H
