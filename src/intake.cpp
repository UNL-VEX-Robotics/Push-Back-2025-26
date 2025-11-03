#include "intake.hpp"

Intake::Intake(vex::motor_group &&frontMotors, vex::motor_group &&mainMotors, vex::motor &topMotor, vex::motor &middleMotor, neblib::Cylinder &hood, neblib::Cylinder &lift, neblib::Cylinder &front, vex::optical &colorSensor): topMotor(topMotor), middleMotor(middleMotor), frontMotors(frontMotors), mainMotors(mainMotors), hood(hood), lift(lift), front(front), colorSensor(colorSensor), velocity(0.0), running(false) {}

void Intake::startLoop()
{
    running = true;
    int msStopped = 0;
    while (running)
    {
        // match load rollers only when not scoring through the top
        if (!hood.getState()) frontMotors.spin(vex::directionType::fwd, velocity, vex::velocityUnits::pct);
        else frontMotors.stop(vex::brakeType::coast);

        // main motors always
        mainMotors.spin(vex::directionType::fwd, velocity, vex::velocityUnits::pct);

        // middle motor only if reversed or scoring through top or if velocity can keep up
        if (velocity <= 0 || hood.getState())
        {
            middleMotor.spin(vex::directionType::fwd, velocity, vex::velocityUnits::pct);
            msStopped = 0;
        }
        else if (msStopped < 250)
        {
            middleMotor.spin(vex::directionType::fwd, velocity, vex::velocityUnits::pct);

            if (middleMotor.velocity(vex::velocityUnits::pct) < 0.25 * velocity) msStopped += 5;
            else msStopped = 0;
        }
        else middleMotor.stop(vex::brakeType::coast);

        if (hood.getState()) topMotor.spin(vex::directionType::fwd, velocity, vex::velocityUnits::pct);
        else topMotor.stop(vex::brakeType::coast);

        vex::task::sleep(5);
    }
}

void Intake::stopLoop() { running = false; }

void Intake::setSpeed(double velocity) { this->velocity = velocity; }