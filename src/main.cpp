/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       closm                                                     */
/*    Created:      5/21/2025, 1:14:46 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "neblib/auton_selector.hpp"
#include "neblib/devices/cylinder.hpp"
#include "intake.hpp"
#include "neblib/xdrive.hpp"
#include <iostream>

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here
brain Brain;
vex::controller controller1(primary);

// ---------- Devices ---------- //
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

vex::motor firstStage = vex::motor(PORT5, ratio6_1, false);
vex::motor secondStage = vex::motor(PORT14, ratio6_1, true);
vex::motor thirdStage = vex::motor(PORT15, ratio6_1, true);
vex::motor leftRoller = vex::motor(PORT19, ratio6_1, false);
vex::motor rightRoller = vex::motor(PORT7, ratio6_1, true);

vex::optical colorSensor(PORT11);

vex::inertial imu(PORT10, vex::turnType::right);
vex::rotation parallelRotation(PORT6, true);      // 6
vex::rotation perpendicularRotation(PORT8, true); // 8
vex::distance rightDistance(PORT9);
vex::distance leftDistance(PORT16);

vex::led matchload(Brain.ThreeWirePort.C);
vex::led wing(Brain.ThreeWirePort.E);
vex::led lift(Brain.ThreeWirePort.B);
vex::led hood(Brain.ThreeWirePort.A);

neblib::Cylinder matchloadCylinder(matchload);
neblib::Cylinder wingCylinder(wing);
neblib::Cylinder liftCylinder(lift);
neblib::Cylinder hoodCylinder(hood);

Intake intake(
    vex::motor_group(leftRoller, rightRoller),
    vex::motor_group(firstStage),
    thirdStage,
    secondStage,
    colorSensor,
    hoodCylinder);

// ---------- Position Tracking ---------- //
neblib::RotationTrackerWheel parallel(
    parallelRotation,
    2.05);
neblib::RotationTrackerWheel perpendicular(
    perpendicularRotation,
    2.0119);

neblib::Odometry odom(
    parallel,
    -4.2, // -4.0
    perpendicular,
    0.0, // 0.0
    imu);

// ---------- Drivetrain ---------- //
neblib::XDrive xDrive(
    leftFront,
    rightFront,
    leftBack,
    rightBack,
    imu,
    &odom);

// ---------- PIDs ---------- //
neblib::PID linearPID(
    neblib::PID::Gains(
        0.6,
        0.004,
        1.1,
        0.8),
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

// ---------- Auton Selector ---------- //
neblib::Page redPage = neblib::Page(
    neblib::Button(
        0,
        0,
        160,
        50,
        vex::color(155, 155, 155),
        vex::color(50, 50, 50),
        vex::color(255, 255, 255),
        vex::color(0, 0, 0),
        "Red"),
    {neblib::Button(
         10,
         120,
         160,
         50,
         vex::color(0, 0, 0),
         vex::color(150, 0, 0),
         vex::color(255, 255, 255),
         vex::color(255, 255, 255),
         "Red < Mid"),
     neblib::Button(
         10,
         60,
         160,
         50,
         vex::color(0, 0, 0),
         vex::color(150, 0, 0),
         vex::color(255, 255, 255),
         vex::color(255, 255, 255),
         "Red < AWP"),
     neblib::Button(
         10,
         180,
         160,
         50,
         vex::color(0, 0, 0),
         vex::color(150, 0, 0),
         vex::color(255, 255, 255),
         vex::color(255, 255, 255),
         "Red < End"),
     neblib::Button(
         310,
         60,
         160,
         50,
         vex::color(0, 0, 0),
         vex::color(150, 0, 0),
         vex::color(255, 255, 255),
         vex::color(255, 255, 255),
         "Red > AWP"),
     neblib::Button(
         310,
         120,
         160,
         50,
         vex::color(0, 0, 0),
         vex::color(150, 0, 0),
         vex::color(255, 255, 255),
         vex::color(255, 255, 255),
         "Red > Mid"),
     neblib::Button(
         310,
         180,
         160,
         50,
         vex::color(0, 0, 0),
         vex::color(150, 0, 0),
         vex::color(255, 255, 255),
         vex::color(255, 255, 255),
         "Red > End"),
     neblib::Button(
         180,
         60,
         120,
         50,
         vex::color(0, 0, 0),
         vex::color(150, 0, 0),
         vex::color(255, 255, 255),
         vex::color(255, 255, 255),
         "Red > Far"),
     neblib::Button(
         180,
         180,
         120,
         50,
         vex::color(0, 0, 0),
         vex::color(150, 0, 0),
         vex::color(255, 255, 255),
         vex::color(255, 255, 255),
         "Red >^ Wing")});

neblib::Page bluePage = neblib::Page(
    neblib::Button(
        160,
        0,
        160,
        50,
        vex::color(155, 155, 155),
        vex::color(50, 50, 50),
        vex::color(255, 255, 255),
        vex::color(0, 0, 0),
        "Blue"),
    {neblib::Button(
         10,
         120,
         160,
         50,
         vex::color(0, 0, 0),
         vex::color(0, 0, 150),
         vex::color(255, 255, 255),
         vex::color(255, 255, 255),
         "Blue < Mid"),
     neblib::Button(
         10,
         60,
         160,
         50,
         vex::color(0, 0, 0),
         vex::color(0, 0, 150),
         vex::color(255, 255, 255),
         vex::color(255, 255, 255),
         "Blue < AWP"),
     neblib::Button(
         10,
         180,
         160,
         50,
         vex::color(0, 0, 0),
         vex::color(0, 0, 150),
         vex::color(255, 255, 255),
         vex::color(255, 255, 255),
         "Blue < End"),
     neblib::Button(
         310,
         60,
         160,
         50,
         vex::color(0, 0, 0),
         vex::color(0, 0, 150),
         vex::color(255, 255, 255),
         vex::color(255, 255, 255),
         "Blue > AWP"),
     neblib::Button(
         310,
         120,
         160,
         50,
         vex::color(0, 0, 0),
         vex::color(0, 0, 150),
         vex::color(255, 255, 255),
         vex::color(255, 255, 255),
         "Blue > Mid"),
     neblib::Button(
         310,
         180,
         160,
         50,
         vex::color(0, 0, 0),
         vex::color(0, 0, 150),
         vex::color(255, 255, 255),
         vex::color(255, 255, 255),
         "Blue > End"),
     neblib::Button(
         180,
         60,
         120,
         50,
         vex::color(0, 0, 0),
         vex::color(0, 0, 150),
         vex::color(255, 255, 255),
         vex::color(255, 255, 255),
         "Blue > Far"),
     neblib::Button(
         180,
         180,
         120,
         50,
         vex::color(0, 0, 0),
         vex::color(0, 0, 150),
         vex::color(255, 255, 255),
         vex::color(255, 255, 255),
         "Blue >^ Wing")});

neblib::Page skillsPage = neblib::Page(
    neblib::Button(
        320,
        0,
        160,
        50,
        vex::color(155, 155, 155),
        vex::color(50, 50, 50),
        vex::color(255, 255, 255),
        vex::color(0, 0, 0),
        "Skills"),
    {neblib::Button(
        10,
        120,
        160,
        50,
        vex::color(0, 0, 0),
        vex::color(150, 0, 0),
        vex::color(255, 255, 255),
        vex::color(255, 255, 255),
        "< Skills")});

neblib::AutonSelector autonSelector(
    Brain,
    {&redPage,
     &bluePage,
     &skillsPage},
    neblib::Button(
        180,
        120,
        120,
        50,
        vex::color(255, 255, 255),
        vex::color(255, 255, 255),
        vex::color(0, 0, 0),
        vex::color(255, 255, 255),
        "Calibrate"));
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

    autonSelector.runSelector();
    Brain.Screen.clearScreen(yellow);
    waitUntil(!Brain.Screen.pressing());
    task::sleep(500);
    Brain.Screen.clearScreen(green);
    odom.calibrate();
    Brain.Screen.clearScreen(autonSelector.getColor());
    controller1.rumble(".");
}

// ---------- Autonomous Helpers ---------- //
void senseColor(vex::color c, int timeout = infinity())
{
    colorSensor.setLightPower(100);
    colorSensor.setLight(ledState::on);

    int elapsedTime = 0;
    while (colorSensor.color() != c && elapsedTime < timeout)
    {
        task::sleep(10);
        elapsedTime += 10;
    }

    colorSensor.setLight(ledState::off);
}

void setScore()
{
    vex::task a = vex::task([]()
                            {
        xDrive.driveLocal(-3, 0, 0, volt);
        while (std::abs(thirdStage.velocity(percent)) < 25)
        {
            task::sleep(10);
        }
        xDrive.stop(hold);
        return 0; });
}

int startTime = 0;
void printTime()
{
    controller1.Screen.print(abs(int(Brain.Timer.system()) - int(startTime)));
    controller1.Screen.print(", ");
}

void leftStart(vex::color c)
{
    vex::color oppositeColor = (c == vex::color::red) ? vex::color::blue : vex::color::red;

    odom.setPose(-54.375, 65.5 - rightDistance.objectDistance(inches), 270.0);
    task a = neblib::launchTask(std::bind(&neblib::Odometry::begin, &odom));

    // ----- Match load ----- //
    xDrive.driveLocal(-10, 10, 0, volt);
    task::sleep(250);
    xDrive.driveTo(-56.2, 45, 1000); // 1500
    intake.setSpeed(100);
    liftCylinder.toggle();
    matchloadCylinder.toggle();
    xDrive.driveLocal(2.0, 0.0, 0.0);
    task::sleep(1500); // 2000

    // ----- Score long goal ----- //
    xDrive.driveTo(-36, 46.25, 1000); // 1500
    xDrive.turnFor(90 - imu.heading(deg));
    // odom.setPose(
    //     curPose.x,
    //     65.5 - leftDistance.objectDistance(inches),
    //     imu.heading(deg)
    // );
    intake.setSpeed(-100);
    task::sleep(100);
    intake.setSpeed(0);
    hoodCylinder.toggle();
    matchloadCylinder.toggle();
    auto curPose = odom.getPose();
    odom.setPose(curPose.x, 65.5 - leftDistance.objectDistance(inches), imu.heading(deg));

    xDrive.driveTo(-25, 49, 1000); // 1000 SCORE
    intake.setSpeed(100);
    setScore();
    xDrive.driveLocal(-3, 0, 0, volt);
    task::sleep(100);
    xDrive.stop(hold);
    senseColor(oppositeColor, 2000);
    intake.setSpeed(0);

    // ----- Match load ----- //
    vex::task([]()
              {
        task::sleep(100);
        intake.setSpeed(80);
        return 0; });
    int t = xDrive.driveTo(-40, 47, 1500); // 1500
    task::sleep(800 - t);
    intake.setSpeed(0);
    hoodCylinder.toggle();
    curPose = odom.getPose();
    xDrive.turnTo(270, 1000);
    // odom.setPose(
    //     curPose.x,
    //     65.5 - rightDistance.objectDistance(inches),
    //     imu.heading(deg));

    xDrive.driveToPose(-56.25, 47, 270, 1250);
    intake.setSpeed(100);
    matchloadCylinder.toggle();
    task::sleep(750);
    matchloadCylinder.toggle();
    task::sleep(250);
    matchloadCylinder.toggle();
    // xDrive.driveLocal(2.0, 0.0, 0.0);
    task::sleep(1500);
}

// ---------- Autonomous Routes ---------- //
void leftAWP(vex::color c)
{
    leftStart(c);

    // ----- Score Center ----- //
    xDrive.driveTo(-42, 46.5, 1000);
    liftCylinder.toggle();
    intake.setSpeed(0);
    matchloadCylinder.toggle();
    xDrive.turnTo(135);
    vex::task([]()
              {
        intake.setSpeed(-100);
        task::sleep(100);
        intake.setSpeed(0);
        task::sleep(300);
        hoodCylinder.toggle();
        return 0; });
    xDrive.driveToPose(-10, 12.25, 135, 3000, -6, 6);
    intake.setSpeed(50);
    setScore();
    task::sleep(3000);
    intake.setSpeed(0);
    xDrive.driveLocal(8, 0, 0, volt);
    task::sleep(100);
    xDrive.driveLocal(-8, 0, 0, volt);
    task::sleep(300);
    hoodCylinder.toggle();
    xDrive.driveToPose(-10.5, 12, 135, 3000, -6, 6);
    task::sleep(100);
}

void leftMid(vex::color c)
{
    leftStart(c);

    // ----- Wing ----- //
    xDrive.driveTo(-32, 38.5, 1500);
    matchloadCylinder.toggle();
    xDrive.driveTo(2, 40.5, 2000, -5, 5);
    wingCylinder.toggle();
    xDrive.driveTo(-24, 24, 1000);

    // ----- Score Center ----- //
    liftCylinder.toggle();
    intake.setSpeed(0);
    xDrive.turnTo(135);
    vex::task([]()
              {
        intake.setSpeed(-100);
        task::sleep(100);
        intake.setSpeed(0);
        task::sleep(300);
        hoodCylinder.toggle();
        return 0; });
    xDrive.driveToPose(-10, 13.75, 135, 3000, -6, 6);
    intake.setSpeed(50);
    setScore();
    task::sleep(3000);
    intake.setSpeed(0);
    xDrive.driveLocal(8, 0, 0, volt);
    task::sleep(100);
    xDrive.driveLocal(-8, 0, 0, volt);
    task::sleep(300);
    hoodCylinder.toggle();
    xDrive.driveToPose(-10.5, 13.75, 135, 3000, -6, 6);
    task::sleep(100);
}

void leftEnd(vex::color c)
{
    leftStart(c);

    // ----- Score Center ----- //
    xDrive.driveTo(-42, 46.5, 1000);
    liftCylinder.toggle();
    intake.setSpeed(0);
    matchloadCylinder.toggle();
    xDrive.turnTo(135);
    vex::task([]()
              {
        intake.setSpeed(-100);
        task::sleep(100);
        intake.setSpeed(0);
        task::sleep(300);
        hoodCylinder.toggle();
        return 0; });
    xDrive.driveToPose(-10, 12.25, 135, 3000, -6, 6);
    intake.setSpeed(50);
    setScore();
    task::sleep(3000);
    intake.setSpeed(0);
    xDrive.driveLocal(8, 0, 0, volt);
    task::sleep(50);

    // ----- Wing ----- //
    xDrive.driveTo(-32, 37.5, 1500);
    hoodCylinder.toggle();
    liftCylinder.toggle();
    xDrive.turnTo(270, 1000);
    xDrive.driveTo(2, 39, 2000, -5, 5);
}

void rightStart(vex::color c)
{
    vex::color oppositeColor = (c == vex::color::red) ? vex::color::blue : vex::color::red;

    odom.setPose(-54.375, -65.5 + leftDistance.objectDistance(inches), 270.0);
    task a = neblib::launchTask(std::bind(&neblib::Odometry::begin, &odom));

    // ----- Match Load ----- //
    xDrive.driveTo(-58, -46, 1000);
    intake.setSpeed(100);
    liftCylinder.toggle();
    matchloadCylinder.toggle();
    xDrive.driveLocal(2.0, 0.0, 0.0);
    task::sleep(1500);

    // ----- Score Long Goal ----- //
    xDrive.driveTo(-40, -46.25, 1000);
    xDrive.turnFor(90 - imu.heading(deg));
    intake.setSpeed(-100);
    task::sleep(100);
    intake.setSpeed(0);
    hoodCylinder.toggle();
    matchloadCylinder.toggle();

    xDrive.driveTo(-29, -47, 750);
    intake.setSpeed(100);
    setScore();
    senseColor(oppositeColor, 2000);
    intake.setSpeed(0);

    // ----- Match Load 2 ----- //
    vex::task([]()
              {
        task::sleep(100);
        intake.setSpeed(80);
        return 0; });
    int t = xDrive.driveTo(-42, -47, 1000);
    task::sleep(800 - t);
    intake.setSpeed(0);
    hoodCylinder.toggle();
    xDrive.turnTo(270, 1000);

    xDrive.driveToPose(-56.25, -46.5, 270, 1250);
    intake.setSpeed(100);
    matchloadCylinder.toggle();
    xDrive.driveLocal(2.0, 0.0, 0.0);
    task::sleep(2000);
}

void rightAWP(vex::color c)
{
    rightStart(c);

    // ----- Score Center ----- //
    xDrive.driveTo(-37, -41.5, 1000);
    liftCylinder.toggle();
    intake.setSpeed(0);
    matchloadCylinder.toggle();
    xDrive.turnTo(135);
    xDrive.driveTo(-18, 18, 3000); // 3000
    vex::task([]()
              {
        intake.setSpeed(-100);
        task::sleep(100);
        intake.setSpeed(0);
        task::sleep(300);
        hoodCylinder.toggle();
        return 0; });
    xDrive.driveToPose(-9, 9, 135, 1250, -6, 6);
    intake.setSpeed(50);
    setScore();
    task::sleep(3000);
    xDrive.driveLocal(-8, 0, 0, volt);
    task::sleep(250);
    intake.setSpeed(0);
    hoodCylinder.toggle();
    xDrive.driveToPose(-9, 9, 135, 1500);
    task::sleep(100);
}

void rightMid(vex::color c)
{
    rightStart(c);

    // ----- Wing Long Goal ----- //
    xDrive.driveTo(-35, -34.5, 1500);
    matchloadCylinder.toggle();
    xDrive.turnFor(90 - imu.heading(deg));
    xDrive.driveTo(-8, -38, 2000, -6, 6);
    wingCylinder.toggle();

    // ----- Score Center ----- //
    xDrive.driveTo(-30, 0, 1000);
    liftCylinder.toggle();
    intake.setSpeed(0);
    xDrive.turnTo(135);
    xDrive.driveTo(-20, 20, 1750); // 3000
    vex::task([]()
              {
        intake.setSpeed(-100);
        task::sleep(100);
        intake.setSpeed(0);
        task::sleep(300);
        hoodCylinder.toggle();
        return 0; });
    xDrive.driveToPose(-8.5, 9, 135, 1250, -6, 6);
    intake.setSpeed(50);
    setScore();
    task::sleep(3000);
    xDrive.driveLocal(-8, 0, 0, volt);
    task::sleep(250);
    intake.setSpeed(0);
    hoodCylinder.toggle();
    xDrive.driveToPose(-8.5, 9, 135, 1500);
    task::sleep(100);
}

void rightEnd(vex::color c)
{
    rightStart(c);

    // ----- Score Center ----- //
    xDrive.driveTo(-42, -46.5, 1000);
    liftCylinder.toggle();
    intake.setSpeed(0);
    matchloadCylinder.toggle();
    xDrive.turnTo(135, 1000);
    xDrive.driveToPose(-18, 18, 135, 2500); // 3000
    vex::task([]()
              {
        intake.setSpeed(-100);
        task::sleep(100);
        intake.setSpeed(0);
        task::sleep(300);
        hoodCylinder.toggle();
        return 0; });
    xDrive.driveToPose(-8.5, 9, 135, 1250, -6, 6);
    intake.setSpeed(50);
    setScore();
    task::sleep(3000);
    xDrive.driveLocal(-8, 0, 0, volt);
    task::sleep(250);
    intake.setSpeed(0);

    // ----- Wing ----- //
    xDrive.driveTo(-30, 0, 1000);
    hoodCylinder.toggle();
    liftCylinder.toggle();
    xDrive.turnTo(90, 1000);

    xDrive.driveToPose(-35, -34.25, 90, 1250);
    xDrive.driveTo(-8, -38, 2000, -6, 6);
}

void rightFar(vex::color c)
{
    rightStart(c);

    // ----- Score Center ----- //
    vex::task([]()
              {
            task::sleep(100);
            intake.setSpeed(-80);
            task::sleep(50);
            intake.setSpeed(0);
            task::sleep(250);
            liftCylinder.toggle();
            matchloadCylinder.toggle();
            hoodCylinder.toggle();
            return 0; });

    // xDrive.driveToPose(7, -24, 315, 3000, -7, 7);
    // xDrive.turnTo(315);
    xDrive.driveLocal(-3, 6, 0, volt);
    task::sleep(500);
    xDrive.driveToPose(15, -24, 315, 3000, -7, 7); // Drive towards mid goals
    xDrive.driveTo(12.25, -14.25, 2000, -6, 6);    // Scoot over
    intake.setSpeed(50);
    task::sleep(100);
    setScore();
    vex::color oppositeColor = (c == vex::color::red) ? vex::color::blue : vex::color::red;
    senseColor(oppositeColor, 3000);
    intake.setSpeed(0);
}

void rightFarWing(vex::color c)
{
    rightStart(c);

    // ----- Wing Long Goal ----- //
    xDrive.driveTo(-35, -34.5, 1500);
    matchloadCylinder.toggle();
    xDrive.turnFor(90 - imu.heading(deg));
    xDrive.driveTo(-8, -38, 800, -6, 6);
    wingCylinder.toggle();

    // ----- Score Center ----- //
    vex::task([]()
              {
            task::sleep(100);
            intake.setSpeed(-80);
            task::sleep(100);
            intake.setSpeed(0);
            task::sleep(375);
            liftCylinder.toggle();
            return 0; });

    task::sleep(50);
    xDrive.driveToPose(12, -20, 315, 2000, -7, 7); // Drive towards mid goals //(10,-24)

    // //For some reason this works if there is a robot sitting there and works if there is no bot
    // //Attempts to body slam the robot that is there (I don't think it actually listens but it works...)
    // //Also for some reason the sleeps were needed
    // task::sleep(100);
    // xDrive.driveTo(13, -14.25, 10000, -6, 6); //Scoot over, was 2000   //DELETE IF DONT USE
    // task::sleep(100);
    // xDrive.driveTo(10, -10, 1000, -6, 6); //tries to body slam          /DELETE IF DONT USE
    // task::sleep(100);

    // Actually aligns with the goal and scores
    vex::task([]()
              {
    task::sleep(20);
    hoodCylinder.toggle();
    return 0; });

    xDrive.driveTo(12, -14.5, 1150, -6, 6); // Scoot over, was 2000
    intake.setSpeed(50);
    task::sleep(100);
    setScore();
    task::sleep(3000);
    intake.setSpeed(0);
    // task::sleep(100);

    xDrive.driveLocal(-8, 0, 0, volt);
    task::sleep(250);
    intake.setSpeed(0);
    hoodCylinder.toggle();
    xDrive.driveToPose(12, -14.5, 315, 1500);
    task::sleep(100);
}

void autonomous(void)
{
    startTime = Brain.Timer.system();
    vex::task i = neblib::launchTask(std::bind(&Intake::startLoop, &intake));
    auto route = autonSelector.getAuton();
    if (neblib::contains(route, "< AWP"))
        leftAWP(autonSelector.getColor());
    else if (neblib::contains(route, "< Mid"))
        leftMid(autonSelector.getColor());
    else if (neblib::contains(route, "< End"))
        leftEnd(autonSelector.getColor());
    else if (neblib::contains(route, "> AWP"))
        rightAWP(autonSelector.getColor());
    else if (neblib::contains(route, "> Mid"))
        rightMid(autonSelector.getColor());
    else if (neblib::contains(route, "> End"))
        rightEnd(autonSelector.getColor());
    else if (neblib::contains(route, "> Far"))
        rightFar(autonSelector.getColor());
    else if (neblib::contains(route, ">^ Wing"))
        rightFarWing(autonSelector.getColor());
    else if (neblib::contains(route, "Skills"))
    {
        wingCylinder.toggle();
        intake.setSpeed(-100);
        task::sleep(1000);
    }

    printTime();

    i.stop();

    auto pose = odom.getPose();
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("X: %.2f", pose.x);
    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print("Y: %.2f", pose.y);
    Brain.Screen.setCursor(3, 1);
    Brain.Screen.print("H: %.2f", pose.heading);
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
    const char *auton = autonSelector.getAuton();
    if (neblib::contains(auton, "Skills"))
        imu.setHeading(270, deg);

    neblib::launchTask(std::bind(&Intake::startLoop, &intake));

    bool L1WasPressing = false;
    bool R1WasPressing = false;
    bool yWasPressing = false;
    bool rightWasPressing = false;
    bool downWasPressing = false;
    while (true)
    {
        double intakeVelocity = 0.0;
        if (controller1.ButtonR2.pressing())
            intakeVelocity = -80.0;
        if (controller1.ButtonL2.pressing())
            intakeVelocity = hoodCylinder.getState() ? 70 : 100.0;
        if (controller1.ButtonL1.pressing() && !L1WasPressing)
            liftCylinder.toggle();
        if (controller1.ButtonR1.pressing() && !R1WasPressing)
            hoodCylinder.toggle();
        if (controller1.ButtonY.pressing() && !yWasPressing)
            matchloadCylinder.toggle();
        wingCylinder.setState(!controller1.ButtonRight.pressing());
        if (controller1.ButtonLeft.pressing() || controller1.ButtonDown.pressing())
            intakeVelocity = 0.4 * intakeVelocity;
        intake.setSpeed(intakeVelocity);

        xDrive.driveGlobal(controller1.Axis3.position(percent) * 0.12, controller1.Axis4.position(percent) * 0.12, controller1.Axis1.position(percent) * 0.12, volt);

        L1WasPressing = controller1.ButtonL1.pressing();
        R1WasPressing = controller1.ButtonR1.pressing();
        yWasPressing = controller1.ButtonY.pressing();
        rightWasPressing = controller1.ButtonRight.pressing();
        downWasPressing = controller1.ButtonDown.pressing();

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
