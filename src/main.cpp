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
    neblib::Button(310, 60, 160, 50, vex::color(0, 0, 0), vex::color(150, 0, 0), vex::color(255, 255, 255), vex::color(255, 255, 255), "Red > AWP"),
    neblib::Button(310, 120, 160, 50, vex::color(0, 0, 0), vex::color(150, 0, 0), vex::color(255, 255, 255), vex::color(255, 255, 255), "Red > x2"),
    neblib::Button(310, 180, 160, 50, vex::color(0, 0, 0), vex::color(150, 0, 0), vex::color(255, 255, 255), vex::color(255, 255, 255), "Red > Wing"),
    neblib::Button(10, 60, 160, 50, vex::color(0, 0, 0), vex::color(150, 0, 0), vex::color(255, 255, 255), vex::color(255, 255, 255), "Red < AWP"),
    neblib::Button(10, 120, 160, 50, vex::color(0, 0, 0), vex::color(150, 0, 0), vex::color(255, 255, 255), vex::color(255, 255, 255), "Red < x2"),
    neblib::Button(10, 180, 160, 50, vex::color(0, 0, 0), vex::color(150, 0, 0), vex::color(255, 255, 255), vex::color(255, 255, 255), "Red < Wing"),
    neblib::Button(180, 60, 120, 50, vex::color(0, 0, 0), vex::color(150, 0, 0), vex::color(255, 255, 255), vex::color(255, 255, 255), "Red < Q")});
neblib::Page bluePage = neblib::Page(neblib::Button(160, 0, 160, 50, vex::color(155, 155, 155), vex::color(75, 75, 75), vex::color(255, 255, 255), vex::color(0, 0, 0), "Blue"), {
    neblib::Button(310, 60, 160, 50, vex::color(0, 0, 0), vex::color(0, 0, 150), vex::color(255, 255, 255), vex::color(255, 255, 255), "Blue > AWP"),
    neblib::Button(310, 120, 160, 50, vex::color(0, 0, 0), vex::color(0, 0, 150), vex::color(255, 255, 255), vex::color(255, 255, 255), "Blue > x2"),
    neblib::Button(310, 180, 160, 50, vex::color(0, 0, 0), vex::color(0, 0, 150), vex::color(255, 255, 255), vex::color(255, 255, 255), "Blue > Wing"),
    neblib::Button(10, 60, 160, 50, vex::color(0, 0, 0), vex::color(0, 0, 150), vex::color(255, 255, 255), vex::color(255, 255, 255), "Blue < AWP"),
    neblib::Button(10, 120, 160, 50, vex::color(0, 0, 0), vex::color(0, 0, 150), vex::color(255, 255, 255), vex::color(255, 255, 255), "Blue < x2"),
    neblib::Button(10, 180, 160, 50, vex::color(0, 0, 0), vex::color(0, 0, 150), vex::color(255, 255, 255), vex::color(255, 255, 255), "Blue < Wing"),
    neblib::Button(180, 60, 120, 50, vex::color(0, 0, 0), vex::color(0, 0, 150), vex::color(255, 255, 255), vex::color(255, 255, 255), "Blue < Q")});
neblib::Page skillsPage = neblib::Page(neblib::Button(320, 0, 160, 50, vex::color(155, 155, 155), vex::color(75, 75, 75), vex::color(255, 255, 255), vex::color(0, 0, 0), "Skills"), {neblib::Button(10, 120, 160, 50, vex::color(0, 0, 0), vex::color(150, 0, 0), vex::color(255, 255, 255), vex::color(255, 255, 255), "Left Skills")});
neblib::AutonSelector selector = neblib::AutonSelector(Brain, {&redPage, &bluePage, &skillsPage}, neblib::Button(180, 120, 120, 50, vex::color(255, 255, 255), vex::color(255, 255, 255), vex::color(0, 0, 0), vex::color(255, 255, 255), "Calibrate"));

neblib::PID turnPID(0.1, 0.0075, 0.5, 15.0, std::make_shared<neblib::PID::SettleTimeExitConditions>(neblib::PID::SettleTimeExitConditions(0.5, 30, 10)), true);
neblib::PID swingPID(0.15, 0.01, 0.25, 15.0, std::make_shared<neblib::PID::SettleTimeExitConditions>(neblib::PID::SettleTimeExitConditions(0.5, 30, 10)), true);
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

const double matchLoadDistance = 3.25;
void rightStart()
{
    imu.setHeading(180, deg);

    // ----- Match Load ----- //
    standardDrive.driveFor(dist.objectDistance(inches) - 18.5, 1.5);
    lever.setVelocity(-100);
    wingCylinder.set(true);
    liftCylinder.set(true);
    matchloadCylinder.toggle();
    standardDrive.turnTo(270, 1.5);
    intakeMotor.spin(forward, 100, percent);
    standardDrive.driveFor(dist.objectDistance(inches) - matchLoadDistance, 1.5);
    task::sleep(600);

    // ----- Score Long Goal ----- //
    standardDrive.driveFor(-32, 270, -7, 7, 1.5);
    lever.setVelocity(60);
    task::sleep(600);
    lever.setVelocity(-100);
    intakeMotor.spin(reverse, 100, percent);
    vex::task([]() {
        task::sleep(450);
        intakeMotor.spin(forward, 100, percent);
        return 0;
    });

    // ----- Match Load ----- //
    standardDrive.driveFor(30, 270, -5, 5, 1.5);
    task::sleep(1200);
    standardDrive.driveFor(-12, 1);
    task::sleep(100);
    matchloadCylinder.toggle();
    standardDrive.turnTo(180, 1);
    intakeMotor.spin(reverse, 100, percent);
    task::sleep(800);
    standardDrive.driveFor(dist.objectDistance(inches) - 17.5, 1.5);
    standardDrive.turnTo(270);
    matchloadCylinder.toggle();
    task::sleep(500);
    intakeMotor.spin(forward, 100, percent);
    standardDrive.driveFor(dist.objectDistance(inches) - matchLoadDistance, -6, 6, 2);
    task::sleep(2500);
}

void rightAWP()
{
    rightStart();

    // ----- Score Middle Goal ----- //
    standardDrive.driveFor(-4);
    standardDrive.turnTo(45);
    matchloadCylinder.toggle();
    standardDrive.driveFor(52, 2);
    standardDrive.driveFor(-4);
    intakeMotor.spin(reverse, 100, percent);
    task([](){
        lever.setVelocity(25);
        task::sleep(100);
        lever.setVelocity(-100);
        return 0;
    });
    task::sleep(3000);
    standardDrive.driveFor(4, 1.5);
}

void rightMany()
{
    rightStart();
    // ----- Score Long Goal ----- //
    standardDrive.driveFor(-30, 270, 1.25);
    lever.setVelocity(50);
    task::sleep(600);
    intakeMotor.spin(reverse, 100, percent);
    lever.setVelocity(-100);
    matchloadCylinder.toggle();
}

void rightWing()
{
    rightMany();   
    
    // standardDrive.turnFor(-45);
    // standardDrive.driveFor(8);
    // standardDrive.turnFor(45, 1.5);
    // standardDrive.driveFor(-25);

    standardDrive.swingFor(left, 70);
    standardDrive.swingFor(right, 70);
    standardDrive.driveFor(-27);
}

void leftStart()
{
    imu.setHeading(0, deg);
    

    // ----- Match Load ----- //
    standardDrive.driveFor(dist.objectDistance(inches) - 18.5, 1.5);
    lever.setVelocity(-100);
    wingCylinder.set(true);
    liftCylinder.set(true);
    matchloadCylinder.toggle();
    standardDrive.turnTo(270, 1.5);
    intakeMotor.spin(forward, 100, percent);
    standardDrive.driveFor(dist.objectDistance(inches) - matchLoadDistance, 1.5);
    task::sleep(300);

    // ----- Score Long Goal ----- //
    standardDrive.driveFor(-32, 270, -7, 7, 1.5);
    lever.setVelocity(50);
    task::sleep(600);
    lever.setVelocity(-100);
    intakeMotor.spin(reverse, 100, percent);
    vex::task([]() {
        task::sleep(450);
        intakeMotor.spin(forward, 100, percent);
        return 0;
    });

    // ----- Match Load ----- //
    standardDrive.driveFor(30, 270, -5, 5, 1.5);
    task::sleep(1200);
    standardDrive.driveFor(-12, 1);
    task::sleep(100);
    matchloadCylinder.toggle();
    standardDrive.turnTo(0, 1);
    intakeMotor.spin(reverse, 100, percent);
    task::sleep(800);
    standardDrive.driveFor(dist.objectDistance(inches) - 17.5, 1.5);
    standardDrive.turnTo(270);
    matchloadCylinder.toggle();
    task::sleep(500);
    intakeMotor.spin(forward, 100, percent);
    standardDrive.driveFor(dist.objectDistance(inches) - matchLoadDistance, -6, 6, 2);
    task::sleep(2500);
}

void leftAWP()
{
    leftStart();

    // ----- Score Bottom Goal ----- //
    standardDrive.driveFor(dist.objectDistance(inches) - 16, 2);
    standardDrive.turnTo(180, 2);
    matchloadCylinder.toggle();
    standardDrive.driveFor(91);
    task::sleep(250);
    standardDrive.turnTo(45);
    standardDrive.driveFor(43);
    intakeMotor.spin(reverse, 100, percent);
    task::sleep(3000);
}

void leftMany()
{
    leftStart();

    // ----- Score Long Goal ----- //
    standardDrive.driveFor(-30, 270, 1.25);
    lever.setVelocity(50);
    task::sleep(650);
    intakeMotor.spin(reverse, 100, percent);
    lever.setVelocity(-100);
    matchloadCylinder.toggle();
}

void leftWing()
{
    leftMany();

    standardDrive.swingFor(left, 68);
    standardDrive.swingFor(right, 68);
    standardDrive.driveFor(-31, 2);
}

void leftQuick()
{
    imu.setHeading(0, deg);
    

    // ----- Match Load ----- //
    standardDrive.driveFor(dist.objectDistance(inches) - 18.5, 1.5);
    lever.setVelocity(-100);
    wingCylinder.set(true);
    liftCylinder.set(true);
    matchloadCylinder.toggle();
    standardDrive.turnTo(270, 1.5);
    intakeMotor.spin(forward, 100, percent);
    standardDrive.driveFor(dist.objectDistance(inches) - matchLoadDistance, 1.5);
    task::sleep(200);

    // ----- Score Long Goal ----- //
    standardDrive.driveFor(-32, 270, -7, 7, 1.5);
    lever.setVelocity(50);
    task::sleep(600);
    lever.setVelocity(-100);
    intakeMotor.spin(reverse, 100, percent);
    vex::task([]() {
        task::sleep(450);
        intakeMotor.spin(forward, 100, percent);
        return 0;
    });
    matchloadCylinder.toggle();

    standardDrive.swingFor(left, 60);
    standardDrive.swingFor(right, 62);
    standardDrive.driveFor(-31, 2);
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
    else if (neblib::contains(auton, "> x2"))
        rightMany();
    else if (neblib::contains(auton, "> Wing"))
    {
        rightWing();
    }
    else if (neblib::contains(auton, "< AWP"))
    {
        leftAWP();
    }
    else if (neblib::contains(auton, "< x2"))
    {
        leftMany();
    }
    else if (neblib::contains(auton, "< Wing"))
    {
        leftWing();
    }
    else if (neblib::contains(auton, "< Q"))
    {
        leftQuick();
    }
    else if (neblib::contains(auton, "Skills"))
    {
        wingCylinder.toggle();
        intakeMotor.spin(reverse, 100, percent);
        task::sleep(10000);
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

bool macroFinished = false;

int macro()
{
    macroFinished = false;
    standardDrive.swingFor(left, 70);
    standardDrive.swingFor(right, 70);
    macroFinished = true;
    return 0;
}

int userDrive()
{
    while(true)
    {
        standardDrive.arcadeDrive(controller1.Axis3.position(percent) * 0.12, controller1.Axis1.position(percent) * 0.7 * 0.12, vex::voltageUnits::volt);
        if (abs(controller1.Axis3.position(percent)) < 3 && abs(controller1.Axis1.position(percent)) < 3)
            standardDrive.stop(hold);
        task::sleep(10);
    }
    return 0;
}

void usercontrol(void)
{
    neblib::launchTask(std::bind(&Lever::startLoop, &lever));
    wingCylinder.set(true);

    vex::task driveTask = vex::task(userDrive);

    bool rightWasPressing = false;
    bool bWasPressing = false;
    bool r2WasPressing = false;
    bool aWasPressing = false;
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
        if (controller1.ButtonY.pressing() && !bWasPressing)
            matchloadCylinder.toggle();
        if (controller1.ButtonR2.pressing() && !r2WasPressing)
            liftCylinder.toggle();

        wingExtendCylinder.set(!controller1.ButtonRight.pressing());
        if (controller1.ButtonA.pressing() != aWasPressing || macroFinished)
        {
            driveTask.stop();
            if (controller1.ButtonA.pressing() && !macroFinished)
                driveTask = task(macro);
            else
            {
                driveTask = task(userDrive);
                macroFinished = false;
            }
                
        }

        r2WasPressing = controller1.ButtonR2.pressing();
        bWasPressing = controller1.ButtonY.pressing();
        rightWasPressing = controller1.ButtonRight.pressing();
        aWasPressing = controller1.ButtonA.pressing();
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
