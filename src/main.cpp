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
    neblib::Button(10, 120, 160, 50, vex::color(0, 0, 0), vex::color(150, 0, 0), vex::color(255, 255, 255), vex::color(255, 255, 255), "Right Red AWP"),
    neblib::Button(310, 120, 160, 50, vex::color(0, 0, 0), vex::color(150, 0, 0), vex::color(255, 255, 255), vex::color(255, 255, 255), "Right Red Elims"),
    neblib::Button(310, 180, 160, 50, vex::color(0, 0, 0), vex::color(150, 0, 0), vex::color(255, 255, 255), vex::color(255, 255, 255), "Right Red Many"),
    neblib::Button(10, 180, 160, 50, vex::color(0, 0, 0), vex::color(150, 0, 0), vex::color(255, 255, 255), vex::color(255, 255, 255), "Right Red Safe")});
neblib::Page bluePage = neblib::Page(neblib::Button(160, 0, 160, 50, vex::color(155, 155, 155), vex::color(75, 75, 75), vex::color(255, 255, 255), vex::color(0, 0, 0), "Blue"), {
    neblib::Button(10, 120, 160, 50, vex::color(0, 0, 0), vex::color(0, 0, 150), vex::color(255, 255, 255), vex::color(255, 255, 255), "Right Blue AWP"),
    neblib::Button(310, 120, 160, 50, vex::color(0, 0, 0), vex::color(0, 0, 150), vex::color(255, 255, 255), vex::color(255, 255, 255), "Right Blue Elims"),
    neblib::Button(310, 180, 160, 50, vex::color(0, 0, 0), vex::color(0, 0, 150), vex::color(255, 255, 255), vex::color(255, 255, 255), "Right Blue Quick")});
neblib::Page skillsPage = neblib::Page(neblib::Button(320, 0, 160, 50, vex::color(155, 155, 155), vex::color(75, 75, 75), vex::color(255, 255, 255), vex::color(0, 0, 0), "Skills"), {neblib::Button(10, 120, 160, 50, vex::color(0, 0, 0), vex::color(150, 0, 0), vex::color(255, 255, 255), vex::color(255, 255, 255), "Left Skills")});
neblib::AutonSelector selector = neblib::AutonSelector(Brain, {&redPage, &bluePage, &skillsPage}, neblib::Button(180, 120, 120, 50, vex::color(255, 255, 255), vex::color(255, 255, 255), vex::color(0, 0, 0), vex::color(255, 255, 255), "Calibrate"));

neblib::PID turnPID(0.1, 0.01, 0.5, 15.0, std::make_shared<neblib::PID::SettleTimeExitConditions>(neblib::PID::SettleTimeExitConditions(0.5, 30, 10)), true);
neblib::PID swingPID(0.125, 0.01, 0.25, 15.0, std::make_shared<neblib::PID::SettleTimeExitConditions>(neblib::PID::SettleTimeExitConditions(0.5, 30, 10)), true);
neblib::PID drivePID(0.5, 0.01, 1.5, 6.0, std::make_shared<neblib::PID::SettleTimeExitConditions>(neblib::PID::SettleTimeExitConditions(0.5, 50, 10)), true);
neblib::PID angularPID(0.2, 0.0, 0.0, 0.0, std::make_shared<neblib::PID::SettleTimeExitConditions>(neblib::PID::SettleTimeExitConditions(0.5, 50, 10)), true);

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

    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Calibrating Inertial...");

    waitUntil(!Brain.Screen.pressing());

    imu.startCalibration();
    do
    {
        task::sleep(5);
    } while (imu.isCalibrating());
    imu.setHeading(90, deg);

    while (!Brain.Screen.pressing())
    {
        Brain.Screen.clearScreen(selector.getColor());
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print(imu.heading(deg));
        task::sleep(10);
    }
}

void rightAWP()
{
    // Intake under long goal
    intakeMotor.spin(forward, 100, percent);
    standardDrive.driveFor(47, 2);
    standardDrive.driveFor(6, -3, 3, 2.5);
    task::sleep(250);

    // Score bottom middle
    standardDrive.driveFor(-26.25, 2);
    standardDrive.turnTo(45, 1.5);
    liftCylinder.toggle();
    standardDrive.driveFor(16, 2);
    intakeMotor.spin(reverse, 100, percent);
    task::sleep(1333);

    // Matchload
    vex::task([](){
        task::sleep(500);
        matchloadCylinder.toggle();
        return 0;
    });
    standardDrive.driveFor(-44, 2);
    standardDrive.turnTo(270, 2);
    intakeMotor.spin(forward, 100, percent);
    int totalTime = 1500;
    double t = standardDrive.driveFor(12, 3, 6, 0.001 * totalTime);
    task::sleep(totalTime - int(300.0 * t));

    // Score Long goal
    standardDrive.driveFor(-29.5, 272, 1.5);
    lever.setVelocity(70);
    task::sleep(250);
    intakeMotor.spin(reverse, 100, percent);
    task::sleep(1500);
}

void rightSafe()
{
    // Score bottom middle
    standardDrive.driveFor(26, 2);
    standardDrive.turnTo(45, 1.5);
    liftCylinder.toggle();
    standardDrive.driveFor(18.5, 2);
    intakeMotor.spin(reverse, 100, percent);
    task::sleep(500);

    // Matchload
    vex::task([](){
        task::sleep(500);
        matchloadCylinder.toggle();
        return 0;
    });
    standardDrive.driveFor(-47, 2);
    standardDrive.turnTo(270, 2);
    intakeMotor.spin(forward, 100, percent);
    int totalTime = 1285;
    double t = standardDrive.driveFor(9, 3, 6, 0.001 * totalTime);
    task::sleep(totalTime - int(1000.0 * t));

    // Score Long goal
    standardDrive.driveFor(-26, 271, 1.5);
    lever.setVelocity(80);
    task::sleep(250);
    intakeMotor.spin(reverse, 100, percent);
}

void rightMany()
{
    lever.setVelocity(-100);
    intakeMotor.spin(forward, 100, percent);
    standardDrive.driveFor(26, 2);
    task::sleep(1000);
    standardDrive.swingFor(left, -60, 1.25);
    lever.setVelocity(100);
    task::sleep(750);
    lever.setVelocity(-100);
    standardDrive.swingFor(left, 60, 2);
    intakeMotor.spin(forward, 100, percent);
    task::sleep(2500);

    // Score Long goal
    standardDrive.driveFor(-26, 272, 1.5);
    lever.setVelocity(35);
    task::sleep(250);
    intakeMotor.spin(reverse, 100, percent);
}

void rightElims()
{
    task::sleep(1000);
    wingCylinder.toggle();
    standardDrive.swingFor(right, 180, 2);
    standardDrive.driveFor(10);
    standardDrive.swingFor(right, 35, 2);
    standardDrive.driveFor(8);
    standardDrive.swingTo(left, 90, 2);
    wingCylinder.toggle();
    standardDrive.driveFor(8);
}

void skills()
{
    intakeMotor.spin(forward, 100, percent);
    wingCylinder.toggle();
}

void autonomous(void)
{
    auto auton = selector.getAuton();
    auto startTime = Brain.Timer.system();
    
    neblib::launchTask(std::bind(&Lever::startLoop, &lever));
    lever.setVelocity(-100);

    if (neblib::contains(auton, "AWP"))
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
    wingCylinder.toggle();



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
