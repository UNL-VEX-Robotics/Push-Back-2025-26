/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       closm                                                     */
/*    Created:      5/22/2025, 10:39:20 AM                                    */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "neblib/xdrive.hpp"
#include "intake.hpp"
#include <iostream>

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here
brain Brain;
controller controller1(primary);

vex::motor frontLeftTop = vex::motor(PORT1, ratio6_1, false);
vex::motor frontLeftBottom = vex::motor(PORT2, ratio6_1, true);
vex::motor frontRightTop = vex::motor(PORT4, ratio6_1, true);
vex::motor frontRightBottom = vex::motor(PORT3, ratio6_1, false);
vex::motor backLeftTop = vex::motor(PORT18, ratio6_1, false);
vex::motor backLeftBottom = vex::motor(PORT17, ratio6_1, true);
vex::motor backRightTop = vex::motor(PORT13, ratio6_1, true);
vex::motor backRightBottom = vex::motor(PORT12, ratio6_1, false);

vex::motor firstStage = vex::motor(PORT5, ratio6_1, false);
vex::motor secondStage = vex::motor(PORT14, ratio6_1, true);
vex::motor thirdStage = vex::motor(PORT15, ratio6_1, true);
vex::motor leftRoller = vex::motor(PORT21, ratio6_1, false); // placeholder
vex::motor rightRoller = vex::motor(PORT21, ratio6_1, true); // placeholder

vex::rotation parallelRotation = vex::rotation(PORT6);
vex::rotation perpendicularRotation = vex::rotation(PORT8);
vex::distance leftDistance = vex::distance(PORT16);
vex::distance rightDistance = vex::distance(PORT9);
vex::inertial imu = vex::inertial(PORT10);
vex::optical colorSensor = vex::optical(PORT11);

std::vector<neblib::Line> obstacles = {
  neblib::Line(neblib::Point(-72.0, -72.0), neblib::Point(72.0, -72.0)),
  neblib::Line(neblib::Point(72.0, -72.0), neblib::Point(72.0, 72.0)),
  neblib::Line(neblib::Point(-72.0, 72.0), neblib::Point(72.0, 72.0)),
  neblib::Line(neblib::Point(-72.0, -72.0), neblib::Point(-72.0, 72.0))
};
neblib::MCL mcl = neblib::MCL({new neblib::Distance(leftDistance, 0.0, 0.0, 0.0), new neblib::Distance(rightDistance, 0.0, 0.0, 0.0)}, std::unique_ptr<neblib::TrackerWheel>(new neblib::RotationTrackerWheel(parallelRotation, 2.0)), 0.0, std::unique_ptr<neblib::TrackerWheel>(new neblib::RotationTrackerWheel(perpendicularRotation, 2.0)), 0.0, imu, 250, obstacles, 1.0, 0.05);
neblib::XDrive xDrive = neblib::XDrive(vex::motor_group(frontLeftTop, frontLeftBottom), vex::motor_group(frontRightTop, frontRightBottom), vex::motor_group(backLeftTop, backLeftBottom), vex::motor_group(backRightTop, backRightBottom), &mcl, imu);

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
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


void usercontrol(void) {
 
  while (true)
  {
    task::sleep(10);
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}

