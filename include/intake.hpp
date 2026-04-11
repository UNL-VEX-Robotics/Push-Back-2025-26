
#pragma once

#include "vex.h"
#include "neblib/devices/cylinder.hpp"

class Intake
{
private:
    vex::motor &topMotor;
    vex::motor &middleMotor;
    vex::motor_group frontMotors;
    vex::motor_group mainMotors;

    vex::optical &colorSensor;
    neblib::Cylinder &hood;

    double velocity;
    bool running;

    bool frontRunning;

public:
    Intake(
        vex::motor_group &&frontMotors,
        vex::motor_group &&mainMotors,
        vex::motor &topMotor,
        vex::motor &middleMotor,
        vex::optical &colorSensor,
        neblib::Cylinder &hood);

    void startLoop();
    void stopLoop();

    void setSpeed(double velocity);
    void toggleFront(bool toggle);
};
