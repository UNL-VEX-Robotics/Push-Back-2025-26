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
#include "path_generator.hpp"
#include <iostream>

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here
brain Brain;
controller controller1(primary);

motor leftDrive1(PORT13, ratio6_1, false);
motor leftDrive2(PORT11, ratio6_1, true);
motor leftDrive3(PORT18, ratio6_1, true);
motor leftDrive4(PORT19, ratio6_1, false);
motor leftDrive5(PORT20, ratio6_1, true);

motor rightDrive1(PORT6, ratio6_1, true);
motor rightDrive2(PORT7, ratio6_1, false);
motor rightDrive3(PORT8, ratio6_1, false);
motor rightDrive4(PORT9, ratio6_1, true);
motor rightDrive5(PORT10, ratio6_1, false);

motor_group leftDrive = vex::motor_group(leftDrive1, leftDrive2, leftDrive3, leftDrive4, leftDrive5);
motor_group rightDrive = vex::motor_group(rightDrive1, rightDrive2, rightDrive3, rightDrive4, rightDrive5);



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

PathGenerator p(leftDrive, rightDrive, 11.0, 86.39, 20.0);

void autonomous(void)
{
  p.followPath({
    Point(-36, -48, 0),
    Point(-36, 0, 0),
    Point(0, 36, 90),
    Point(36, 0, 180),
    Point(0, -36, 270),
    Point(-24, -36, 270),
    Point(-36, -48, 180)
  });
  leftDrive.stop(hold);
  rightDrive.stop(hold);
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

  while (true)
  {
    double leftOutput = standardizeExp(controller1.Axis3.position(percent), 1) + standardizeExp(controller1.Axis1.position(percent), 1) * 0.7;
    double rightOutput = standardizeExp(controller1.Axis3.position(percent), 1) - standardizeExp(controller1.Axis1.position(percent), 1) * 0.7;
    leftDrive.spin(forward, 0.12 * leftOutput, volt);
    rightDrive.spin(forward, 0.12 * rightOutput, volt);
    if (leftOutput == 0) leftDrive.stop(hold);
    if (rightOutput == 0) rightDrive.stop(hold);

    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Wattage: ");
    Brain.Screen.print(leftDrive.power());
    Brain.Screen.print(", ");
    Brain.Screen.print(rightDrive.power());

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
