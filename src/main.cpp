/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       closm                                                     */
/*    Created:      5/21/2025, 1:14:46 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "neblib/xdrive.hpp"
#include "neblib/devices/tracker_wheel.hpp"
#include "neblib/auton_selector.hpp"
#include <iostream>

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here
brain Brain;

vex::controller controller1(primary);

vex::motor frontLeftTop = vex::motor(PORT1, ratio6_1, false);
vex::motor frontLeftBottom = vex::motor(PORT2, ratio6_1, true);
vex::motor frontRightTop = vex::motor(PORT4, ratio6_1, true);
vex::motor frontRightBottom = vex::motor(PORT3, ratio6_1, false);
vex::motor backLeftTop = vex::motor(PORT18, ratio6_1, false);
vex::motor backLeftBottom = vex::motor(PORT17, ratio6_1, true);
vex::motor backRightTop = vex::motor(PORT13, ratio6_1, true);
vex::motor backRightBottom = vex::motor(PORT12, ratio6_1, false);

vex::motor_group leftFront(frontLeftTop, frontLeftBottom);
vex::motor_group rightFront(frontRightTop, frontRightBottom);
vex::motor_group leftBack(backLeftTop, backLeftBottom);
vex::motor_group rightBack(backRightTop, backRightBottom);

vex::inertial imu(PORT10, vex::turnType::right);
vex::rotation parallelRotation(PORT6, true); // 6
vex::rotation perpendicularRotation(PORT8, true); //8

neblib::RotationTrackerWheel parallel(
    parallelRotation,
    2.05);
neblib::RotationTrackerWheel perpendicular(
    perpendicularRotation,
    2.0119);

neblib::Odometry odom(
    parallel,
    -4.0, // -4.0
    perpendicular,
    0.0, //0.0
    imu);
neblib::XDrive xDrive(
    leftFront,
    rightFront,
    leftBack,
    rightBack,
    imu,
    &odom);

neblib::PID linearPID(
    neblib::PID::Gains(
        0.4,
        0.005,
        0.8,
        0.45),
    neblib::PID::Behaviors(
        12.0,
        true),
    neblib::PID::ExitConditions(
        0.25,
        30));

neblib::PID angularPID(
    neblib::PID::Gains(
        0.15,
        0.005,
        0.2),
    neblib::PID::Behaviors(
        15.0,
        true),
    neblib::PID::ExitConditions(
        0.5,
        50));
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
    xDrive.setLinearController(&linearPID);
    xDrive.setAngularController(&angularPID);
}

void autonomous(void)
{
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

void usercontrol(void)
{
    odom.calibrate();
    odom.setPose(
        0.0,
        0.0,
        90.0);
    task a = neblib::launchTask(std::bind(&neblib::Odometry::begin, &odom));
    int out = xDrive.driveToPose(23.5, 0.0, 90.0, 2500);
    controller1.Screen.print(out);
    xDrive.turnTo(90.0);

    task::sleep(1000);
    xDrive.stop(coast);

    while (true)
    {
        // xDrive.driveGlobal(
        //     controller1.Axis3.position(percent),
        //     controller1.Axis4.position(percent),
        //     controller1.Axis1.position(percent),
        //     vex::velocityUnits::pct);

        const neblib::Pose p = odom.getPose();
        Brain.Screen.clearScreen();
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("X: ");
        Brain.Screen.print(p.x);
        Brain.Screen.setCursor(2, 1);
        Brain.Screen.print("Y: ");
        Brain.Screen.print(p.y);
        Brain.Screen.setCursor(3, 1);
        Brain.Screen.print("T: ");
        Brain.Screen.print(p.heading);
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
