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
#include "neblib/auton_selector.hpp"
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
vex::motor leftRoller = vex::motor(PORT19, ratio6_1, false);
vex::motor rightRoller = vex::motor(PORT7, ratio6_1, true); 

vex::rotation parallelRotation = vex::rotation(PORT6);
vex::rotation perpendicularRotation = vex::rotation(PORT8);
vex::distance leftDistance = vex::distance(PORT16);
vex::distance rightDistance = vex::distance(PORT9);
vex::inertial imu = vex::inertial(PORT10);
vex::optical colorSensor = vex::optical(PORT11);

vex::led hood = vex::led(Brain.ThreeWirePort.A);
vex::led lift = vex::led(Brain.ThreeWirePort.B);
vex::led front = vex::led(Brain.ThreeWirePort.C);

neblib::Cylinder liftCylinders = neblib::Cylinder(lift);
neblib::Cylinder hoodCylinder = neblib::Cylinder(hood);
neblib::Cylinder frontCylinders = neblib::Cylinder(front);

std::vector<neblib::Line> obstacles = {
  neblib::Line(neblib::Point(-72.0, -72.0), neblib::Point(72.0, -72.0)),
  neblib::Line(neblib::Point(72.0, -72.0), neblib::Point(72.0, 72.0)),
  neblib::Line(neblib::Point(-72.0, 72.0), neblib::Point(72.0, 72.0)),
  neblib::Line(neblib::Point(-72.0, -72.0), neblib::Point(-72.0, 72.0))
};
neblib::MCL mcl = neblib::MCL({new neblib::Distance(leftDistance, -5.9375, 0.8125, 270.0), new neblib::Distance(rightDistance, 5.9375, 0.8125, 90.0)}, std::unique_ptr<neblib::TrackerWheel>(new neblib::RotationTrackerWheel(parallelRotation, 2.0)), 3.25, std::unique_ptr<neblib::TrackerWheel>(new neblib::RotationTrackerWheel(perpendicularRotation, 2.0)), 0.875, imu, 250, obstacles, 1.0, 0.05);
neblib::XDrive xDrive = neblib::XDrive(vex::motor_group(frontLeftTop, frontLeftBottom), vex::motor_group(frontRightTop, frontRightBottom), vex::motor_group(backLeftTop, backLeftBottom), vex::motor_group(backRightTop, backRightBottom), &mcl, imu);
Intake intake = Intake(vex::motor_group(leftRoller, rightRoller), vex::motor_group(firstStage), thirdStage, secondStage, hoodCylinder, liftCylinders, frontCylinders, colorSensor);

neblib::Page redPage = neblib::Page(neblib::Button(0, 0, 160, 50, vex::color(155, 155, 155), vex::color(75, 75, 75), vex::color(255, 255, 255), vex::color(0, 0, 0), "Red"), {
  neblib::Button(10, 120, 160, 50, vex::color(0, 0, 0), vex::color(150, 0, 0), vex::color(255, 255, 255), vex::color(255, 255, 255), "Left Red AWP"),
  neblib::Button(310, 120, 160, 50, vex::color(0, 0, 0), vex::color(150, 0, 0), vex::color(255, 255, 255), vex::color(255, 255, 255), "Left Red Elim")
});
neblib::Page bluePage = neblib::Page(neblib::Button(160, 0, 160, 50, vex::color(155, 155, 155), vex::color(75, 75, 75), vex::color(255, 255, 255), vex::color(0, 0, 0), "Blue"), {
  neblib::Button(10, 120, 160, 50, vex::color(0, 0, 0), vex::color(0, 0, 150), vex::color(255, 255, 255), vex::color(255, 255, 255), "Left Blue AWP"),
  neblib::Button(310, 120, 160, 50, vex::color(0, 0, 0), vex::color(0, 0, 150), vex::color(255, 255, 255), vex::color(255, 255, 255), "Left Blue Elim")
});
neblib::Page skillsPage = neblib::Page(neblib::Button(320, 0, 160, 50, vex::color(155, 155, 155), vex::color(75, 75, 75), vex::color(255, 255, 255), vex::color(0, 0, 0), "Skills"), {
  neblib::Button(10, 120, 160, 50, vex::color(0, 0, 0), vex::color(150, 0, 0), vex::color(255, 255, 255), vex::color(255, 255, 255), "Left Skills")
});
neblib::AutonSelector selector = neblib::AutonSelector(Brain, { &redPage, &bluePage, &skillsPage }, neblib::Button(180, 120, 120, 50, vex::color(255, 255, 255), vex::color(255, 255, 255), vex::color(0, 0, 0), vex::color(255, 255, 255), "Calibrate"));

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
  selector.runSelector();

  Brain.Screen.clearScreen();
  Brain.Screen.setPenColor(vex::color(255, 255, 255));
  Brain.Screen.setFillColor(vex::color(0, 0, 0));

  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print("Calibrating Inertial...");

  imu.calibrate();
  do { task::sleep(2); } while (imu.isCalibrating());

  Brain.Screen.setPenColor(vex::color(0, 255, 0));
  Brain.Screen.setCursor(2, 1);
  Brain.Screen.print("Calibration Successful!");

  task::sleep(500);
  Brain.Screen.clearScreen(selector.getColor());
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
  /* REMOVE BEFORE COMPETITION, TESINT PURPOSES ONLY */
  imu.calibrate();
  do {task::sleep(5);} while (imu.isCalibrating());
  imu.setHeading(90, deg);
  /* REMOVE BEFORE COMPETITION, TESINT PURPOSES ONLY */

  neblib::launchTask(std::bind(&Intake::startLoop, &intake));

  bool L1WasPressing = false;
  bool R1WasPressing = false;
  bool aWasPressing = false;
  while (true)
  {
    double intakeVelocity = 0.0;
    if (controller1.ButtonR2.pressing()) intakeVelocity = -100.0;
    if (controller1.ButtonL2.pressing()) intakeVelocity = 100.0;
    if (controller1.ButtonL1.pressing() && !L1WasPressing) liftCylinders.toggle();
    if (controller1.ButtonR1.pressing() && !R1WasPressing) hoodCylinder.toggle();
    if (controller1.ButtonA.pressing() && !aWasPressing) frontCylinders.toggle();
    intake.setSpeed(intakeVelocity);

    xDrive.driveGlobal(controller1.Axis3.position(percent), controller1.Axis4.position(percent), controller1.Axis1.position(percent));

    L1WasPressing = controller1.ButtonL1.pressing();
    R1WasPressing = controller1.ButtonR1.pressing();
    aWasPressing = controller1.ButtonA.pressing();

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

