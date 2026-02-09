#include "actuators_control.h"
#include <thread>
#include <chrono>
#include <iostream>


#define LEFT_INITIAL_POSITON  9.0
#define RIGHT_INITIAL_POSITON  7.0

static int high_edge_time = 0;   // uint :0.1ms
static int cycle_time     = 200; // uint :0.1ms


ActuatorsControl::ActuatorsControl(int leftLegPin, int rightLegPin) 
    : leftLegPin(leftLegPin), rightLegPin(rightLegPin),strip(4)
{}

ActuatorsControl::~ActuatorsControl() 
{}

bool ActuatorsControl::initializeLegs()
{

    pwmController_.addPWM(leftLegPin, 50);
    pwmController_.addPWM(rightLegPin, 50);

    pwmController_.startPWM(0, LEFT_INITIAL_POSITON);
    pwmController_.startPWM(1, RIGHT_INITIAL_POSITON);

    relaxLegs(); // 抖动结束后放松腿部

    return true;
}

void ActuatorsControl::servoControl(int index, float duty){
    switch(index){
        case 0:
            pwmController_.setDutyCycle(0, LEFT_INITIAL_POSITON + duty);
            break;
        case 1:
            pwmController_.setDutyCycle(1, RIGHT_INITIAL_POSITON + duty);
            break;
        default:
            std::cerr <<"Only 0 and 1 are supported"<< std::endl;
    }
    

}

void ActuatorsControl::liftLeftLeg(int speed) 
{
    if( robot_legs_state != ActuatorsState::Relaxed){};

    if(robot_legs_state != ActuatorsState::LiftLeft){
        pwmController_.setDutyCycle(0, LEFT_INITIAL_POSITON - 5.0);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        robot_legs_state = ActuatorsState::LiftLeft;
    }

}

void ActuatorsControl::liftRightLeg(int speed) 
{

    if(robot_legs_state != ActuatorsState::Relaxed){};

    if(robot_legs_state != ActuatorsState::LiftRight){
        pwmController_.setDutyCycle(1, RIGHT_INITIAL_POSITON + 5.0);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        robot_legs_state = ActuatorsState::LiftRight;
    }


}

void ActuatorsControl::lowerLeftLeg(int speed) 
{

    if( robot_legs_state != ActuatorsState::Relaxed){};
    if(robot_legs_state != ActuatorsState::LiftRight){
        pwmController_.setDutyCycle(0, LEFT_INITIAL_POSITON);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

}

void ActuatorsControl::lowerRightLeg(int speed) 
{
    if( robot_legs_state != ActuatorsState::Relaxed){};

    if(robot_legs_state != ActuatorsState::LiftLeft){
        pwmController_.setDutyCycle(1, RIGHT_INITIAL_POSITON); 
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

}

void ActuatorsControl::standStraight() 
{
    
    if (robot_legs_state != ActuatorsState::Stand){
        pwmController_.setDutyCycle(0, LEFT_INITIAL_POSITON);
        pwmController_.setDutyCycle(1, RIGHT_INITIAL_POSITON);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        pwmController_.setDutyCycle(0, LEFT_INITIAL_POSITON + 2.0);
        pwmController_.setDutyCycle(1, RIGHT_INITIAL_POSITON - 2.0);
        robot_legs_state = ActuatorsState::Stand;
    }

}


void ActuatorsControl::relaxLegs() 
{
    if (robot_legs_state == ActuatorsState::Stand){
        for(int i = 0; i < 8; i++){
            pwmController_.setDutyCycle(0, LEFT_INITIAL_POSITON + 2.0 - i * 0.25);
            pwmController_.setDutyCycle(1, RIGHT_INITIAL_POSITON - 2.0 + i * 0.25);
            std::this_thread::sleep_for(std::chrono::milliseconds(35));
        }
        robot_legs_state = ActuatorsState::Relaxed;
    }
    pwmController_.setDutyCycle(0, LEFT_INITIAL_POSITON);
    pwmController_.setDutyCycle(1, RIGHT_INITIAL_POSITON);
    robot_legs_state = ActuatorsState::Relaxed;
}



void ActuatorsControl::flashingLight(int speed) 
{

    relaxLegs();
    strip.clear();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    strip.set_all_same_color(0, 255, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    robot_legs_state = ActuatorsState::FlashingLight;

}


void ActuatorsControl::shakeEars(int speed) 
{
    if( robot_legs_state != ActuatorsState::Relaxed){};

    pwmController_.setDutyCycle(0, LEFT_INITIAL_POSITON - 3.0);
    pwmController_.setDutyCycle(1, RIGHT_INITIAL_POSITON + 3.0);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    for(int i = 0; i < 5; i ++) 
    {
        pwmController_.setDutyCycle(0, LEFT_INITIAL_POSITON - 6.0);
        pwmController_.setDutyCycle(1, RIGHT_INITIAL_POSITON + 5.5);  
        std::this_thread::sleep_for(std::chrono::milliseconds(60));

        pwmController_.setDutyCycle(0, LEFT_INITIAL_POSITON - 3.0);
        pwmController_.setDutyCycle(1, RIGHT_INITIAL_POSITON + 3.0);
        std::this_thread::sleep_for(std::chrono::milliseconds(60));
    }  
    robot_legs_state = ActuatorsState::ShakeEars;
}
