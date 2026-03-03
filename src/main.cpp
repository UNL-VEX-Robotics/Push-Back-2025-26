/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       closm                                                     */
/*    Created:      5/22/2025, 10:39:20 AM                                    */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "neblib/standard_drive.hpp"
#include "intake.hpp"
#include <iostream>

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here
brain Brain;
controller controller1(primary);

vex::motor leverMotor(PORT6, ratio36_1, true);
vex::motor intakeMotor(PORT13, ratio6_1, true);

vex::motor left1(PORT16, ratio6_1, false);
vex::motor left2(PORT17, ratio6_1, true);
vex::motor left3(PORT18, ratio6_1, true);
vex::motor left4(PORT19, ratio6_1, false);
vex::motor left5(PORT20, ratio6_1, true);
vex::motor_group leftMotors(left1, left2, left3, left4, left5);

vex::motor right1(PORT1, ratio6_1, true);
vex::motor right2(PORT2, ratio6_1, false);
vex::motor right3(PORT3, ratio6_1, false);
vex::motor right4(PORT4, ratio6_1, true);
vex::motor right5(PORT5, ratio6_1, false);
vex::motor_group rightMotors(right1, right2, right3, right4, right5);

vex::rotation parallelRotation(PORT11);
neblib::RotationTrackerWheel parallelTrackerWheel(parallelRotation, 2.0);

vex::inertial imu(PORT12);

neblib::StandardDrive standardDrive(leftMotors, rightMotors, nullptr, parallelTrackerWheel, imu);

vex::led wing(Brain.ThreeWirePort.F);
vex::led matchload(Brain.ThreeWirePort.G);
vex::led lift(Brain.ThreeWirePort.H);

neblib::Cylinder wingCylinder(wing);
neblib::Cylinder matchloadCylinder(matchload);
neblib::Cylinder liftCylinder(lift);

Lever lever(leverMotor);


/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void)
{

    // All activities that occur before the competition starts
    // Example: clearing encoders, setting servo positions, ...
}


void autonomous(void)
{
    
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

double standardizeExp(double input, double exp)
{
    return neblib::sign(input) * (std::pow(std::abs(input), exp) / std::pow(100, exp - 1));
}

void usercontrol(void)
{
    neblib::launchTask(std::bind(&Lever::startLoop, &lever));

    bool rightWasPressing = false;
    bool bWasPressing = false;
    bool r2WasPressing = false;
    while (true)
    {
        if (controller1.ButtonR1.pressing())
            lever.setVelocity(100);
        else    
            lever.setVelocity(-100);

        if (controller1.ButtonL2.pressing())
            intakeMotor.spin(reverse, 100, percent);
        else if (controller1.ButtonL1.pressing() && !lever.isUp())
            intakeMotor.spin(forward, 100, percent);
        else    
            intakeMotor.stop(brake);

        if (controller1.ButtonRight.pressing() && !rightWasPressing)
            wingCylinder.toggle();
        if (controller1.ButtonB.pressing() && !bWasPressing)
            matchloadCylinder.toggle();
        if (controller1.ButtonR2.pressing() && !r2WasPressing)
            liftCylinder.toggle();

        standardDrive.arcadeDrive(controller1.Axis3.position(percent), controller1.Axis1.position(percent) * 0.7, vex::velocityUnits::pct);

        r2WasPressing = controller1.ButtonR2.pressing();
        bWasPressing = controller1.ButtonB.pressing();
        rightWasPressing = controller1.ButtonRight.pressing();
        task::sleep(10);

    }
}

//
// Main will set up the competition functions and callbacks.
//
int main()
{
    // Set up callbacks for autonomous and driver control periods.
    Competition.autonomous(autonomous);
    Competition.drivercontrol(usercontrol);

    // Run the pre-autonomous function.
    pre_auton();

    // Prevent main from exiting with an infinite loop.
    while (true)
    {
        wait(100, msec);
    }
}
