#pragma once

#include "vex.h"

class Lever
{
private:
    vex::motor &lever;

    double velocity;

    bool running;
    
    unsigned int timeStuck;

public:
    Lever(vex::motor &lever);

    void startLoop();
    void stopLoop();

    bool isUp();

    void setVelocity(const double velocity);
};