#include "intake.hpp"

#include <iostream>

Lever::Lever(vex::motor &lever)
    : lever(lever),
      velocity(0.0),
      running(false),
      timeStuck(0u)
{
}

void Lever::startLoop()
{
    running = true;
    while (running)
    {
        if (velocity == 0)
        {
            lever.stop(vex::brakeType::coast);
        }
        else{
            if (abs(lever.velocity(vex::velocityUnits::pct)) < 0.05 * abs(velocity))
                timeStuck += 10;
            
            if (timeStuck > 250)
                lever.stop(vex::brakeType::brake);
            else
                lever.spin(vex::directionType::fwd, velocity, vex::velocityUnits::pct);
        }

        vex::task::sleep(10);
    }
}

void Lever::stopLoop()
{
    running = false;
}

bool Lever::isUp()
{
    if (velocity < 250 && timeStuck > 100)
        return false;
    return true;
}

void Lever::setVelocity(const double velocity)
{
    if (this->velocity != velocity)
    {
        this->velocity = velocity;
        timeStuck = 0;
    }
}
