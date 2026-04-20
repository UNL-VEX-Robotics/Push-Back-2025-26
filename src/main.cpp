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
#include "neblib/auton_selector.hpp"
#include <iostream>

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here
brain Brain;
controller controller1(primary);

vex::motor leverMotor(PORT5, ratio36_1, true);
vex::motor intakeMotor(PORT11, ratio6_1, true);

vex::motor left1(PORT14, ratio6_1, false);
vex::motor left2(PORT15, ratio6_1, true);
vex::motor left3(PORT16, ratio6_1, true);
vex::motor left4(PORT17, ratio6_1, false);
vex::motor left5(PORT18, ratio6_1, true);
vex::motor_group leftMotors(left1, left2, left3, left4, left5);

vex::motor right1(PORT6, ratio6_1, true);
vex::motor right2(PORT7, ratio6_1, false);
vex::motor right3(PORT8, ratio6_1, false);
vex::motor right4(PORT21, ratio6_1, true);
vex::motor right5(PORT10, ratio6_1, false);
vex::motor_group rightMotors(right1, right2, right3, right4, right5);

vex::rotation parallelRotation(PORT13);
neblib::RotationTrackerWheel parallelTrackerWheel(parallelRotation, 2.0);

vex::inertial imu(PORT12);
vex::distance dist(PORT1);

neblib::StandardDrive standardDrive(leftMotors, rightMotors, nullptr, parallelTrackerWheel, imu);

vex::led wing(Brain.ThreeWirePort.A);
vex::led wingExtend(Brain.ThreeWirePort.B);

vex::led matchload(Brain.ThreeWirePort.H);
vex::led lift(Brain.ThreeWirePort.G);

neblib::Cylinder wingCylinder(wing);
neblib::Cylinder wingExtendCylinder(wingExtend);

neblib::Cylinder matchloadCylinder(matchload);
neblib::Cylinder liftCylinder(lift);

Lever lever(leverMotor);

neblib::Page redPage = neblib::Page(neblib::Button(0, 0, 160, 50, vex::color(155, 155, 155), vex::color(75, 75, 75), vex::color(255, 255, 255), vex::color(0, 0, 0), "Red"), {
    neblib::Button(310, 60, 160, 50, vex::color(0, 0, 0), vex::color(150, 0, 0), vex::color(255, 255, 255), vex::color(255, 255, 255), "Red > AWP")});
neblib::Page bluePage = neblib::Page(neblib::Button(160, 0, 160, 50, vex::color(155, 155, 155), vex::color(75, 75, 75), vex::color(255, 255, 255), vex::color(0, 0, 0), "Blue"), {
    neblib::Button(10, 120, 160, 50, vex::color(0, 0, 0), vex::color(0, 0, 150), vex::color(255, 255, 255), vex::color(255, 255, 255), "Right Blue AWP"),
    neblib::Button(310, 120, 160, 50, vex::color(0, 0, 0), vex::color(0, 0, 150), vex::color(255, 255, 255), vex::color(255, 255, 255), "Right Blue Elims"),
    neblib::Button(310, 180, 160, 50, vex::color(0, 0, 0), vex::color(0, 0, 150), vex::color(255, 255, 255), vex::color(255, 255, 255), "Right Blue Quick")});
neblib::Page skillsPage = neblib::Page(neblib::Button(320, 0, 160, 50, vex::color(155, 155, 155), vex::color(75, 75, 75), vex::color(255, 255, 255), vex::color(0, 0, 0), "Skills"), {neblib::Button(10, 120, 160, 50, vex::color(0, 0, 0), vex::color(150, 0, 0), vex::color(255, 255, 255), vex::color(255, 255, 255), "Left Skills")});
neblib::AutonSelector selector = neblib::AutonSelector(Brain, {&redPage, &bluePage, &skillsPage}, neblib::Button(180, 120, 120, 50, vex::color(255, 255, 255), vex::color(255, 255, 255), vex::color(0, 0, 0), vex::color(255, 255, 255), "Calibrate"));

neblib::PID turnPID(0.1, 0.01, 0.5, 15.0, std::make_shared<neblib::PID::SettleTimeExitConditions>(neblib::PID::SettleTimeExitConditions(0.5, 30, 10)), true);
neblib::PID swingPID(0.125, 0.01, 0.25, 15.0, std::make_shared<neblib::PID::SettleTimeExitConditions>(neblib::PID::SettleTimeExitConditions(0.5, 30, 10)), true);
neblib::PID drivePID(0.5, 0.01, 1.5, 6.0, std::make_shared<neblib::PID::SettleTimeExitConditions>(neblib::PID::SettleTimeExitConditions(0.5, 50, 10)), true);
neblib::PID angularPID(0.5, 0.0, 0.0, 0.0, std::make_shared<neblib::PID::SettleTimeExitConditions>(neblib::PID::SettleTimeExitConditions(0.5, 50, 10)), true);

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
    standardDrive.setLinearPID(&drivePID);
    standardDrive.setAngularPID(&angularPID);
    standardDrive.setSwingPID(&swingPID);
    standardDrive.setTurnPID(&turnPID);

    selector.runSelector();
    Brain.Screen.clearScreen();
    Brain.Screen.setPenColor(vex::color(255, 255, 255));
    Brain.Screen.setFillColor(vex::color(0, 0, 0));

    Brain.Screen.clearScreen(yellow);

    waitUntil(!Brain.Screen.pressing());
    task::sleep(250);
    Brain.Screen.clearScreen(green);

    imu.startCalibration();
    do
    {
        task::sleep(5);
    } while (imu.isCalibrating());
    imu.setHeading(90, deg);

    Brain.Screen.clearScreen(selector.getColor());
    controller1.rumble(".");
}

void rightAWP()
{
    imu.setHeading(180, deg);

    // ----- Match Load ----- //
    standardDrive.driveFor(dist.objectDistance(inches) - 18.25, 1.5);
    lever.setVelocity(-100);
    wingCylinder.set(true);
    liftCylinder.set(true);
    matchloadCylinder.toggle();
    standardDrive.turnTo(270, 1.5);
    intakeMotor.spin(forward, 100, percent);
    int totalMatchLoadTime = int(1000 * standardDrive.driveFor(10, 1.5));
    task::sleep(1800 - totalMatchLoadTime);
    controller1.Screen.print("%.2f, ", dist.objectDistance(inches));

    // ----- Score Long Goal ----- //
    standardDrive.driveFor(-32, 270, 1.5);
    lever.setVelocity(100);
    task::sleep(50);
    intakeMotor.spin(reverse, 100, percent);
    task::sleep(700);
    lever.setVelocity(-100);
    vex::task([]() {
        task::sleep(250);
        intakeMotor.spin(forward, 100, percent);
        return 0;
    });

    // ----- Match Load ----- //
    standardDrive.driveFor(30, 270, -5, 5, 1.5);
    task::sleep(1000);
    standardDrive.driveFor(-12);
    standardDrive.turnTo(300, 1);
    matchloadCylinder.toggle();
    lever.setVelocity(100);
    task::sleep(50);
    intakeMotor.stop(coast);
    task::sleep(50);
    lever.setVelocity(50);
    task::sleep(500);
    lever.setVelocity(-100);
    standardDrive.turnTo(270);
    matchloadCylinder.toggle();
    task::sleep(100);
    standardDrive.driveFor(13.5);
    intakeMotor.spin(forward, 100, percent);
    task::sleep(3000);

    // ----- Score Middle Goal ----- //
    standardDrive.driveFor(-4);
    standardDrive.turnTo(45);
    matchloadCylinder.toggle();
    standardDrive.driveFor(44);
    intakeMotor.spin(reverse, 100, percent);
    task::sleep(3000);

    // // ----- Score Long Goal ----- //
    // standardDrive.driveFor(-32, 270, 1.5);
    // lever.setVelocity(100);
    // task::sleep(750);
    // lever.setVelocity(-100);
}

void rightSafe()
{
    
}

void rightMany()
{
}

void rightElims()
{
    
}

void skills()
{
    
}

void autonomous(void)
{
    auto auton = selector.getAuton();
    auto startTime = Brain.Timer.system();
    
    neblib::launchTask(std::bind(&Lever::startLoop, &lever));

    if (neblib::contains(auton, "> AWP"))
        rightAWP();
    else if (neblib::contains(auton, "Safe"))
        rightSafe();
    else if (neblib::contains(auton, "Elims"))
    {
        rightAWP();
        rightElims();
    }
    else if (neblib::contains(auton, "Many"))
    {
        rightSafe();
        rightElims();
    }
    else if (neblib::contains(auton, "Skills"))
    {
        skills();
    }
    
    lever.stopLoop();
    intakeMotor.stop(coast);

    controller1.Screen.print(0.001 * (Brain.Timer.system() - startTime));
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
    wingCylinder.set(true);

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
            wingExtendCylinder.toggle();
        if (controller1.ButtonY.pressing() && !bWasPressing)
            matchloadCylinder.toggle();
        if (controller1.ButtonR2.pressing() && !r2WasPressing)
            liftCylinder.toggle();

        standardDrive.arcadeDrive(controller1.Axis3.position(percent) * 0.12, controller1.Axis1.position(percent) * 0.7 * 0.12, vex::voltageUnits::volt);
        if (abs(controller1.Axis3.position(percent)) < 3 && abs(controller1.Axis1.position(percent)) < 3)
            standardDrive.stop(hold);

        r2WasPressing = controller1.ButtonR2.pressing();
        bWasPressing = controller1.ButtonY.pressing();
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
