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

vex::rotation parallelRotation = vex::rotation(PORT6, false);
vex::rotation perpendicularRotation = vex::rotation(PORT8, true);
vex::distance leftDistance = vex::distance(PORT16);
vex::distance rightDistance = vex::distance(PORT9);
vex::inertial imu = vex::inertial(PORT10, vex::turnType::right);
vex::optical colorSensor = vex::optical(PORT11);

vex::led hood = vex::led(Brain.ThreeWirePort.A);
vex::led lift = vex::led(Brain.ThreeWirePort.B);
vex::led front = vex::led(Brain.ThreeWirePort.C);
vex::led wing = vex::led(Brain.ThreeWirePort.E);
vex::led rake = vex::led(Brain.ThreeWirePort.D);

neblib::Cylinder liftCylinders = neblib::Cylinder(lift);
neblib::Cylinder hoodCylinder = neblib::Cylinder(hood);
neblib::Cylinder frontCylinders = neblib::Cylinder(front);
neblib::Cylinder wingCylinder = neblib::Cylinder(wing);
neblib::Cylinder rakeCylinder = neblib::Cylinder(rake);

std::vector<neblib::Line> obstacles = {
    neblib::Line(neblib::Point(-72.0, -72.0), neblib::Point(72.0, -72.0)),
    neblib::Line(neblib::Point(72.0, -72.0), neblib::Point(72.0, 72.0)),
    neblib::Line(neblib::Point(-72.0, 72.0), neblib::Point(72.0, 72.0)),
    neblib::Line(neblib::Point(-72.0, -72.0), neblib::Point(-72.0, 72.0)),
    neblib::Line(neblib::Point(-6.625, -9.4375), neblib::Point(9.4375, 6.625)),
    neblib::Line(neblib::Point(-9.4375, -6.625), neblib::Point(6.625, 9.4375))};
// neblib::MCL mcl = neblib::MCL({new neblib::Distance(leftDistance, -5.9375, 0.8125, 270.0), new neblib::Distance(rightDistance, 5.9375, 0.8125, 90.0)}, std::unique_ptr<neblib::TrackerWheel>(new neblib::RotationTrackerWheel(parallelRotation, 2.0)), 3.25, std::unique_ptr<neblib::TrackerWheel>(new neblib::RotationTrackerWheel(perpendicularRotation, 2.0)), 0.25, imu, 250, obstacles, 1.0, 0.05);
neblib::RotationTrackerWheel parallel = neblib::RotationTrackerWheel(parallelRotation, 2.0);
neblib::RotationTrackerWheel perpendicular = neblib::RotationTrackerWheel(perpendicularRotation, 2.0);
neblib::Odometry odom = neblib::Odometry(parallel, 3.25, perpendicular, 0.25, imu);
neblib::XDrive xDrive = neblib::XDrive(vex::motor_group(frontLeftTop, frontLeftBottom), vex::motor_group(frontRightTop, frontRightBottom), vex::motor_group(backLeftTop, backLeftBottom), vex::motor_group(backRightTop, backRightBottom), &odom, imu);
Intake intake = Intake(vex::motor_group(leftRoller, rightRoller), vex::motor_group(firstStage), thirdStage, secondStage, hoodCylinder, liftCylinders, frontCylinders, colorSensor);

neblib::Page redPage = neblib::Page(neblib::Button(0, 0, 160, 50, vex::color(155, 155, 155), vex::color(75, 75, 75), vex::color(255, 255, 255), vex::color(0, 0, 0), "Red"), {neblib::Button(10, 120, 160, 50, vex::color(0, 0, 0), vex::color(150, 0, 0), vex::color(255, 255, 255), vex::color(255, 255, 255), "Left Red AWP"),
                                                                                                                                                                              neblib::Button(310, 120, 160, 50, vex::color(0, 0, 0), vex::color(150, 0, 0), vex::color(255, 255, 255), vex::color(255, 255, 255), "Left Red Elims"),
                                                                                                                                                                              neblib::Button(310, 180, 160, 50, vex::color(0, 0, 0), vex::color(150, 0, 0), vex::color(255, 255, 255), vex::color(255, 255, 255), "Left Red Quick")});
neblib::Page bluePage = neblib::Page(neblib::Button(160, 0, 160, 50, vex::color(155, 155, 155), vex::color(75, 75, 75), vex::color(255, 255, 255), vex::color(0, 0, 0), "Blue"), {neblib::Button(10, 120, 160, 50, vex::color(0, 0, 0), vex::color(0, 0, 150), vex::color(255, 255, 255), vex::color(255, 255, 255), "Left Blue AWP"),
                                                                                                                                                                                  neblib::Button(310, 120, 160, 50, vex::color(0, 0, 0), vex::color(0, 0, 150), vex::color(255, 255, 255), vex::color(255, 255, 255), "Left Blue Elims"),
                                                                                                                                                                                  neblib::Button(310, 180, 160, 50, vex::color(0, 0, 0), vex::color(0, 0, 150), vex::color(255, 255, 255), vex::color(255, 255, 255), "Left Blue Quick")});
neblib::Page skillsPage = neblib::Page(neblib::Button(320, 0, 160, 50, vex::color(155, 155, 155), vex::color(75, 75, 75), vex::color(255, 255, 255), vex::color(0, 0, 0), "Skills"), {neblib::Button(10, 120, 160, 50, vex::color(0, 0, 0), vex::color(150, 0, 0), vex::color(255, 255, 255), vex::color(255, 255, 255), "Right Skills")});
neblib::AutonSelector selector = neblib::AutonSelector(Brain, {&redPage, &bluePage, &skillsPage}, neblib::Button(180, 120, 120, 50, vex::color(255, 255, 255), vex::color(255, 255, 255), vex::color(0, 0, 0), vex::color(255, 255, 255), "Calibrate"));

neblib::PID turnPID = neblib::PID(0.125, 0.005, 0.125, 15.0, std::make_shared<neblib::PID::SettleTimeExitConditions>(neblib::PID::SettleTimeExitConditions(0.5, 50, 10)), true);
neblib::PID rotationalPID = neblib::PID(0.125, 0.0, 0.0, 0.0, std::make_shared<neblib::PID::SettleTimeExitConditions>(neblib::PID::SettleTimeExitConditions(0.5, 50, 10)), true);
neblib::PID drivePID = neblib::PID(0.5, 0.005, 0.75, 6.0, std::make_shared<neblib::PID::SettleTimeExitConditions>(neblib::PID::SettleTimeExitConditions(0.5, 50, 10)), true);

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
    selector.runSelector();
    Brain.Screen.clearScreen();
    Brain.Screen.setPenColor(vex::color(255, 255, 255));
    Brain.Screen.setFillColor(vex::color(0, 0, 0));

    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Calibrating Inertial...");

    imu.calibrate();
    do
    {
        task::sleep(2);
    } while (imu.isCalibrating());
    imu.setHeading(270, deg);

    auto auton = selector.getAuton();
    std::cout << "\n\nAuton: " << auton << "\n\n";
    std::cout << "\n\nAWP? " << (neblib::contains(auton, "AWP") ? "true" : "false") << "\n\n";
    std::cout << "\n\nTime " << Brain.Timer.time() << "\n\n";
    std::cout << "\n\nTemp " << leftRoller.temperature(percent) << "\n\n";

    Brain.Screen.setPenColor(vex::color(0, 255, 0));
    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print("Calibration Successful!");

    task::sleep(500);
    Brain.Screen.clearScreen(selector.getColor());
    Brain.Screen.setPenColor(black);
    Brain.Screen.setFillColor(selector.getColor());

    while (!Brain.Screen.pressing())
    {
        Brain.Screen.clearScreen(selector.getColor());
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print(imu.heading(deg));
        task::sleep(10);
    }
}

int runMCL()
{
    while (true)
    {
        neblib::Pose pose = odom.updatePose();
        // Brain.Screen.clearScreen();
        // Brain.Screen.setCursor(1, 1);
        // Brain.Screen.print(pose.x);
        // Brain.Screen.setCursor(2, 1);
        // Brain.Screen.print(pose.y);
        // Brain.Screen.setCursor(3, 1);
        // Brain.Screen.print(imu.heading(deg));
        task::sleep(10);
    }
}

void senseColor(vex::color c, int timeout)
{
    int t = 0;
    colorSensor.setLight(ledState::on);
    while (t < timeout)
    {
        if (colorSensor.color() == c && colorSensor.isNearObject())
        {
            break;
        }

        task::sleep(10);
        t += 10;
    }
}

int autoTime = 0;
int runTimer()
{
    while (true)
    {
        task::sleep(10);
        autoTime += 10;
    }
}

void leftSafe(vex::color c)
{
    odom.setPose(-55.0, 65.4 - rightDistance.objectDistance(inches), 270.0);
    vex::task m = vex::task(runMCL);

    // Match Loads
    xDrive.driveToPose(-56, 46.5, 270.0, -7, 7, 1.5);
    frontCylinders.toggle();
    xDrive.driveLocal(2, 0, 0, volt);
    intake.setSpeed(100);
    task::sleep(1000);
    xDrive.stop();
    task::sleep(1000);

    // Score upper middle goal
    xDrive.driveTo(-45, 46.25, 1.0);
    auto curPose = odom.getPose();
    xDrive.turnTo(135, 1.0);
    frontCylinders.toggle();
    intake.setSpeed(0);
    odom.setPose(curPose.x, curPose.y, imu.heading(deg));
    vex::task([]()
              {
    task::sleep(500);
    hoodCylinder.toggle();
    return 0; });
    xDrive.driveToPose(-11, 12, 135, -6, 6, 2.25);

    intake.setSpeed(65);
    vex::task([]()
              {
    xDrive.driveLocal(-4.5, 0, 0, volt);
    while (thirdStage.velocity(rpm) < 20)
    {
      task::sleep(2);
    }
    xDrive.stop();

    return 0; });
    senseColor(c == red ? blue : red, 2000);
    hoodCylinder.toggle();
    intake.setSpeed(-100);

    // Match Load
    xDrive.driveTo(-40, 46.25, -6, 6, 1.5);
    curPose = odom.getPose();
    xDrive.turnTo(270, 1.5);
    odom.setPose(curPose.x, 67 - rightDistance.objectDistance(inches), imu.heading(deg));
    intake.setSpeed(100);
    frontCylinders.toggle();
    xDrive.driveToPose(-57.0, 45.5, 270.0, 0.75);
    xDrive.driveLocal(2, 0, 0, volt);
    task::sleep(1000);
    xDrive.stop();
    task::sleep(1000);

    // Score long goal
    xDrive.driveTo(-40, 46.25, -6, 6, 1.5);
    intake.setSpeed(0);
    curPose = odom.getPose();
    xDrive.turnTo(90, 1.5);
    odom.setPose(curPose.x, 65.5 - leftDistance.objectDistance(inches), imu.heading(deg));
    liftCylinders.toggle();
    hoodCylinder.toggle();
    frontCylinders.toggle();

    xDrive.driveTo(-22, 48, -6, 6, 1.5);
    intake.setSpeed(100);
    task::sleep(10);
    vex::task([]()
              {
    xDrive.driveLocal(-4.5, 0, 0, volt);
    while (thirdStage.velocity(rpm) < 200)
    {
      task::sleep(2);
    }
    xDrive.stop();

    return 0; });
    task::sleep(2000);

    xDrive.driveLocal(8, 0, 0, volt);
    task::sleep(400);
}

void leftElims(vex::color c)
{
    leftSafe(c);
    intake.setSpeed(0);

    // Wing
    xDrive.driveToPose(-40, 30, 90, -6, 6, 1.5);
    xDrive.turnTo(270, 1.0);
    hoodCylinder.toggle();
    auto curPose = odom.getPose();
    odom.setPose(curPose.x, 64.7 - rightDistance.objectDistance(inches), imu.heading(deg));
    wingCylinder.toggle();
    xDrive.driveTo(-20, 36.75, -6, 6, 2);

    wingCylinder.toggle();
    task::sleep(50);
    auto curY = odom.getPose().y;
    xDrive.driveTo(-5, curY - 1, -6, 6, 3);
}

void leftQuick(vex::color c)
{
    odom.setPose(-55.0, 65.4 - rightDistance.objectDistance(inches), 270.0);
    vex::task m = vex::task(runMCL);

    // Match Loads
    xDrive.driveToPose(-56, 46.5, 270.0, -7, 7, 1.5);
    frontCylinders.toggle();
    xDrive.driveLocal(2, 0, 0, volt);
    intake.setSpeed(100);
    task::sleep(1000);
    xDrive.stop();
    task::sleep(1000);

    // Score long goal
    xDrive.driveTo(-40, 46.25, -6, 6, 1.5);
    intake.setSpeed(0);
    auto curPose = odom.getPose();
    xDrive.turnTo(90, 1.5);
    odom.setPose(curPose.x, 64.7 - leftDistance.objectDistance(inches), imu.heading(deg));
    liftCylinders.toggle();
    hoodCylinder.toggle();
    frontCylinders.toggle();

    xDrive.driveTo(-26, 49, -6, 6, 1.5);
    intake.setSpeed(100);
    task::sleep(10);
    vex::task([]()
              {
    xDrive.driveLocal(-4.5, 0, 0, volt);
    while (thirdStage.velocity(rpm) < 200)
    {
      task::sleep(2);
    }
    xDrive.stop();

    return 0; });
    senseColor(c == red ? blue : red, 2000);

    intake.setSpeed(0);

    // Wing
    xDrive.driveToPose(-40, 30, 90, -6, 6, 1.5);
    xDrive.turnTo(270, 1.0);
    intake.setSpeed(-100);
    hoodCylinder.toggle();
    curPose = odom.getPose();
    odom.setPose(curPose.x, 64.7 - rightDistance.objectDistance(inches), imu.heading(deg));
    wingCylinder.toggle();
    xDrive.driveTo(-20, 37.75, -6, 6, 2);

    wingCylinder.toggle();
    task::sleep(50);
    auto curY = odom.getPose().y;
    xDrive.driveTo(-5, curY - 1, -6, 6, 3);
}

void skills()
{
    odom.setPose(-48, 0, 270.0);
    vex::task m = vex::task(runMCL);

    // Intake from park red zone
    rakeCylinder.toggle();
    wingCylinder.toggle();
    task::sleep(750);
    xDrive.driveTo(-32, 0, -4, 4, 1.5);
    rakeCylinder.toggle();
    xDrive.driveTo(-36, 0, -3, 3, 1.0);

    // match load
    xDrive.driveLocal(-6, 0, 0, volt);
    task::sleep(100);
    xDrive.driveTo(-24, -24, -6, 6, 1.25);
    xDrive.driveTo(-54, -43, -6, 6, 1.25);
    auto curPose = odom.getPose();
    odom.setPose(curPose.x, -65.7 + leftDistance.objectDistance(inches), imu.heading(deg));\
    frontCylinders.toggle();
    intake.setSpeed(100);
    xDrive.driveTo(-58.5, -48, -6, 6, 1.5);
    xDrive.driveLocal(2.5, 0, 0, volt);
    task::sleep(1750);

    //Score red in long goal
    xDrive.driveTo(-40, -48, -6, 6, 2);
    curPose = odom.getPose();
    xDrive.turnTo(90, 1);
    odom.setPose(curPose.x, -65.7 + rightDistance.objectDistance(inches), imu.heading(deg));
    intake.setSpeed(0);
    liftCylinders.toggle();
    hoodCylinder.toggle();
    frontCylinders.toggle();
    xDrive.driveTo(-26, -47, -6, 6, 1.25);
    task([](){
        xDrive.driveLocal(-4.5, 0, 0, volt);
        waitUntil(thirdStage.velocity(percent) > 10);
        task::sleep(50);
        xDrive.stop(hold);
        return 0;
    });
    intake.setSpeed(100);
    senseColor(blue, 2000);
    
    // intake 4 blue from park zone
    intake.setSpeed(-100);
    task::sleep(250);
    intake.setSpeed(0);
    task([](){
        task::sleep(750);
        hoodCylinder.toggle();
        return 0;
    });
    xDrive.driveTo(-48, -24, -6, 6, 1.5);
    curPose = odom.getPose();
    xDrive.turnTo(0, 1);
    odom.setPose(-65.7 + leftDistance.objectDistance(inches), curPose.y, imu.heading(deg));
    liftCylinders.toggle();
    xDrive.driveTo(-44.5, -16, -5, 5, 1.5);
    intake.setSpeed(80);
    xDrive.driveTo(-44.5, 12, -2.5, 2.5, 3);
    
    // Score top goal
    curPose = odom.getPose();
    xDrive.turnTo(135, 1.25);
    odom.setPose(curPose.x, curPose.y, imu.heading(deg));
    intake.setSpeed(0);
    xDrive.driveLocal(0, -6, 0, volt);
    task::sleep(250);
    hoodCylinder.toggle();
    xDrive.driveTo(-16.75, 9, -5, 5, 2);
    intake.setSpeed(45);
    task([](){
        xDrive.driveLocal(-4.5, 0, 0, volt);
        waitUntil(thirdStage.velocity(percent) > 10);
        xDrive.stop(hold);
        return 0;
    });
    task::sleep(3000);

    // Pull form opposite park zone
    xDrive.driveLocal(-3, -6, 0, volt);
    intake.setSpeed(0);
    task::sleep(250);
    xDrive.driveTo(24, 18, -6, 6, 2);
    curPose = odom.getPose();
    xDrive.turnTo(90, 1);
    odom.setPose(curPose.x, -65.7 + rightDistance.objectDistance(inches), imu.heading(deg));
    xDrive.driveTo(36, 1.25, -6, 6, 2); 
    xDrive.driveLocal(3, 0, 0, volt);
    task::sleep(750);
    xDrive.stop(hold);
    curPose = odom.getPose();
    odom.setPose(48, curPose.y, imu.heading(deg));
    rakeCylinder.toggle();
    task::sleep(250);
    xDrive.driveTo(32, 0, -4, 4, 1.5);
    rakeCylinder.toggle();
    hoodCylinder.toggle();
    xDrive.driveTo(36, 0, -3, 3, 1.0);


    xDrive.driveLocal(-6, 0, 0, volt);
    task::sleep(100);
    xDrive.driveTo(24, -24, -6, 6, 1.25);
    curPose = odom.getPose();

    // Match load
    xDrive.driveTo(52, -45, -6, 6, 1.25);
    curPose = odom.getPose();
    odom.setPose(curPose.x, -65.7 + rightDistance.objectDistance(inches), imu.heading(deg));
    frontCylinders.toggle();
    intake.setSpeed(100);
    xDrive.driveTo(57.5, -48, -6, 6, 1.5);
    xDrive.driveLocal(2.5, 0, 0, volt);
    task::sleep(1750);

    // Score long goal
    xDrive.driveTo(46, -48, -6, 6, 2);
    curPose = odom.getPose();
    xDrive.turnTo(180, 1.5);
    odom.setPose(curPose.x, curPose.y, imu.heading(deg));
    frontCylinders.toggle();
    xDrive.driveTo(46, -68, -6, 6, 2);
    task::sleep(1000);
    xDrive.driveLocal(-3, 0, 0, volt);
    task::sleep(500);
    xDrive.driveTo(46, -68, -6, 6, 2);
    xDrive.driveTo(40, -48, -6, 6, 2);

    curPose = odom.getPose();
    xDrive.turnTo(270, 1.5);
    odom.setPose(curPose.x, -65.7 + leftDistance.objectDistance(inches), imu.heading(deg));
    intake.setSpeed(0);
    liftCylinders.toggle();
    hoodCylinder.toggle();
    xDrive.driveTo(26, -50, -6, 6, 1.25);
    task([](){
        xDrive.driveLocal(-4.5, 0, 0, volt);
        waitUntil(thirdStage.velocity(percent) > 10);
        xDrive.stop(hold);
        return 0;
    });
    intake.setSpeed(100);
    task::sleep(2000);

    xDrive.driveLocal(8, 0, 0, volt);
    task::sleep(500);
    xDrive.driveLocal(-12, 0, 0, volt);
    task::sleep(500);
    xDrive.stop(coast);
}

void autonomous(void)
{
    vex::task t = vex::task(runTimer);
    /* TESTING */
    // imu.calibrate();
    // do { task::sleep(5); } while (imu.isCalibrating());
    parallel.resetPosition();
    perpendicular.resetPosition();

    xDrive.setTurnPID(turnPID);
    xDrive.setRotationalPID(rotationalPID);
    xDrive.setLinearPID(drivePID);
    vex::task i = neblib::launchTask(std::bind(&Intake::startLoop, &intake));
    vex::task a = vex::task(runTimer);

    const char *auton = selector.getAuton();
    vex::color c = selector.getColor();

    int startTime = Brain.Timer.time();
    if (neblib::contains(auton, "AWP"))
    {
        leftSafe(c);
        xDrive.driveTo(-40, 46.25, -6, 6, 0.75);
        hoodCylinder.toggle();
        intake.setSpeed(0);
        xDrive.driveToPose(-26, 48, 90, -6, 6, 1.5);
    }
    else if (neblib::contains(auton, "Elims"))
    {
        leftElims(c);
    }
    else if (neblib::contains(auton, "Quick"))
    {
        leftQuick(c);
    }
    else if (neblib::contains(auton, "Skills"))
    {
        skills();
    }
    else
    {
        controller1.rumble(".");
        odom.setPose(0.0, 0.0, 90.0);
        vex::task t = vex::task(runMCL);
        xDrive.driveToPose(24.0, 0.0, 0.0);
        xDrive.driveToPose(48.0, 0.0, 180.0);
        t.stop();
        neblib::Pose pose = odom.getPose();
        controller1.Screen.print(pose.x);
        controller1.Screen.print(",");
        controller1.Screen.print(pose.y);
        controller1.Screen.print(",");
        controller1.Screen.print(pose.heading);
        controller1.Screen.print(",");
    }

    intake.setSpeed(0);
    intake.stopLoop();
    double seconds = (double)(Brain.Timer.time() - startTime) / 1000.0;
    controller1.Screen.print(seconds);

    if (!neblib::contains(auton, "Skills"))
    {
        while (autoTime < 29900)
            task::sleep(2);
        wingCylinder.set(true);
    }
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
    const char *auton = selector.getAuton();
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
            liftCylinders.toggle();
        if (controller1.ButtonR1.pressing() && !R1WasPressing)
            hoodCylinder.toggle();
        if (controller1.ButtonY.pressing() && !yWasPressing)
            frontCylinders.toggle();
        if (controller1.ButtonRight.pressing() && !rightWasPressing)
        {
            wingCylinder.toggle();
        }
        if (controller1.ButtonDown.pressing() && !downWasPressing)
            rakeCylinder.toggle();
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
