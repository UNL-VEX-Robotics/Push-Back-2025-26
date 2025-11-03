#pragma once

#include "vex.h"
#include "neblib/devices.hpp"

class Intake
{
private:
    vex::motor &topMotor;
    vex::motor &middleMotor;
    vex::motor_group frontMotors;
    vex::motor_group mainMotors;

    neblib::Cylinder &hood;
    neblib::Cylinder &lift;
    neblib::Cylinder &front;

    vex::optical &colorSensor;

    double velocity;
    bool running;

public:
    Intake(vex::motor_group &&frontMotors, vex::motor_group &&mainMotors, vex::motor &topMotor, vex::motor &middleMotor, neblib::Cylinder &hood, neblib::Cylinder &lift, neblib::Cylinder &front, vex::optical &colorSensor);

    void startLoop();
    void stopLoop();

    void setSpeed(double velocity);
};