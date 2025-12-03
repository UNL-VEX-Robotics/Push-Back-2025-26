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
vex::rotation perpendicularRotation = vex::rotation(PORT8, false);
vex::distance leftDistance = vex::distance(PORT16);
vex::distance rightDistance = vex::distance(PORT9);
vex::inertial imu = vex::inertial(PORT10);
vex::optical colorSensor = vex::optical(PORT11);

vex::led hood = vex::led(Brain.ThreeWirePort.A);
vex::led lift = vex::led(Brain.ThreeWirePort.B);
vex::led front = vex::led(Brain.ThreeWirePort.C);
vex::led poke = vex::led(Brain.ThreeWirePort.D);

neblib::Cylinder liftCylinders = neblib::Cylinder(lift);
neblib::Cylinder hoodCylinder = neblib::Cylinder(hood);
neblib::Cylinder frontCylinders = neblib::Cylinder(front);
neblib::Cylinder pokeCylinder = neblib::Cylinder(poke);

std::vector<neblib::Line> obstacles = {
  neblib::Line(neblib::Point(-72.0, -72.0), neblib::Point(72.0, -72.0)),
  neblib::Line(neblib::Point(72.0, -72.0), neblib::Point(72.0, 72.0)),
  neblib::Line(neblib::Point(-72.0, 72.0), neblib::Point(72.0, 72.0)),
  neblib::Line(neblib::Point(-72.0, -72.0), neblib::Point(-72.0, 72.0)),
  neblib::Line(neblib::Point(-6.625, -9.4375), neblib::Point(9.4375, 6.625)),
  neblib::Line(neblib::Point(-9.4375, -6.625), neblib::Point(6.625, 9.4375))
};
//neblib::MCL mcl = neblib::MCL({new neblib::Distance(leftDistance, -5.9375, 0.8125, 270.0), new neblib::Distance(rightDistance, 5.9375, 0.8125, 90.0)}, std::unique_ptr<neblib::TrackerWheel>(new neblib::RotationTrackerWheel(parallelRotation, 2.0)), 3.25, std::unique_ptr<neblib::TrackerWheel>(new neblib::RotationTrackerWheel(perpendicularRotation, 2.0)), 0.25, imu, 250, obstacles, 1.0, 0.05);
neblib::RotationTrackerWheel parallel = neblib::RotationTrackerWheel(parallelRotation, 2.0440706731398);
neblib::RotationTrackerWheel perpendicular = neblib::RotationTrackerWheel(perpendicularRotation, 2.0170210492856);
neblib::Odometry odom = neblib::Odometry(parallel, -4.19372, perpendicular, -0.00011, imu);
neblib::XDrive xDrive = neblib::XDrive(vex::motor_group(frontLeftTop, frontLeftBottom), vex::motor_group(frontRightTop, frontRightBottom), vex::motor_group(backLeftTop, backLeftBottom), vex::motor_group(backRightTop, backRightBottom), &odom, imu);
Intake intake = Intake(vex::motor_group(leftRoller, rightRoller), vex::motor_group(firstStage), thirdStage, secondStage, hoodCylinder, liftCylinders, frontCylinders, colorSensor);

neblib::Page redPage = neblib::Page(neblib::Button(0, 0, 160, 50, vex::color(155, 155, 155), vex::color(75, 75, 75), vex::color(255, 255, 255), vex::color(0, 0, 0), "Red"), {
  neblib::Button(10, 120, 160, 50, vex::color(0, 0, 0), vex::color(150, 0, 0), vex::color(255, 255, 255), vex::color(255, 255, 255), "Left Red AWP"),
  neblib::Button(310, 120, 160, 50, vex::color(0, 0, 0), vex::color(150, 0, 0), vex::color(255, 255, 255), vex::color(255, 255, 255), "Left Red Elims")
});
neblib::Page bluePage = neblib::Page(neblib::Button(160, 0, 160, 50, vex::color(155, 155, 155), vex::color(75, 75, 75), vex::color(255, 255, 255), vex::color(0, 0, 0), "Blue"), {
  neblib::Button(10, 120, 160, 50, vex::color(0, 0, 0), vex::color(0, 0, 150), vex::color(255, 255, 255), vex::color(255, 255, 255), "Left Blue AWP"),
  neblib::Button(310, 120, 160, 50, vex::color(0, 0, 0), vex::color(0, 0, 150), vex::color(255, 255, 255), vex::color(255, 255, 255), "Left Blue Elims")
});
neblib::Page skillsPage = neblib::Page(neblib::Button(320, 0, 160, 50, vex::color(155, 155, 155), vex::color(75, 75, 75), vex::color(255, 255, 255), vex::color(0, 0, 0), "Skills"), {
  neblib::Button(10, 120, 160, 50, vex::color(0, 0, 0), vex::color(150, 0, 0), vex::color(255, 255, 255), vex::color(255, 255, 255), "Left Skills")
});
neblib::AutonSelector selector = neblib::AutonSelector(Brain, { &redPage, &bluePage, &skillsPage }, neblib::Button(180, 120, 120, 50, vex::color(255, 255, 255), vex::color(255, 255, 255), vex::color(0, 0, 0), vex::color(255, 255, 255), "Calibrate"));

neblib::PID turnPID = neblib::PID(0.125, 0.005, 0.125, 15.0, std::make_shared<neblib::PID::SettleTimeExitConditions>(neblib::PID::SettleTimeExitConditions(0.5, 50, 10)), true);
neblib::PID rotationalPID = neblib::PID(0.125, 0.0, 0.0, 0.0, std::make_shared<neblib::PID::SettleTimeExitConditions>(neblib::PID::SettleTimeExitConditions(0.5, 50, 10)), true);
neblib::PID drivePID = neblib::PID(0.5, 0.005, 0.5, 8.0, std::make_shared<neblib::PID::SettleTimeExitConditions>(neblib::PID::SettleTimeExitConditions(0.5, 50, 10)), true);

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
}

int runMCL()
{
  while (true)
  {
    neblib::Pose pose = odom.updatePose();
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print(pose.x);
    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print(pose.y);
    Brain.Screen.setCursor(3, 1);
    Brain.Screen.print(pose.heading);
    task::sleep(10);
  }
}

void senseColor(vex::color c, int timeout)
{
  int t = 0;
  colorSensor.setLight(ledState::on);
  while (t < timeout)
  {
    if (colorSensor.color() == c && colorSensor.isNearObject()) {
      break;
    }
    
    task::sleep(10);
    t += 10;
  }
}

void leftElims(vex::color c)
{
  odom.setPose(-55.0, 16.5, 270.0);
  vex::task m = vex::task(runMCL);

  // Match Loads
  xDrive.driveToPose(-57.0, 46.25, 270.0, 1.5);
  frontCylinders.toggle();
  xDrive.driveLocal(2, 0, 0, volt);
  intake.setSpeed(100);
  task::sleep(1000);
  xDrive.stop();
  task::sleep(1000);
  
  frontCylinders.toggle();
  intake.setSpeed(0);
  task::sleep(20);

  // Score middle
  vex::task t([]() {
    task::sleep(500);
    hoodCylinder.toggle();
    return 0;
  });
  xDrive.driveToPose(-7.5, 13.25, 135.0, -5, 5, 2.5);
  intake.setSpeed(-100.0);
  task::sleep(100);
  intake.setSpeed(50.0);
  xDrive.driveLocal(-4, 0, 0, volt);
  waitUntil(thirdStage.velocity(rpm) > 100);
  xDrive.stop();
  senseColor(c == red ? blue : red, 3000);
  neblib::Pose currentPose = odom.getPose();
  xDrive.driveLocal(0, 0, 8, volt);
  task::sleep(150);
  xDrive.stop(hold);
  odom.setPose(currentPose.x, currentPose.y, imu.heading(deg));
  task::sleep(1000);

  // // Intake under long goal
  // intake.setSpeed(0);
  // vex::task t1([]() {
  //   task::sleep(500);
  //   hoodCylinder.toggle();
  //   return 0;
  // });
  // xDrive.driveToPose(-16, 53.5, 85, -4, 4, 2.0);
  // intake.setSpeed(100);
  // xDrive.driveTo(-10, 53.5, -4.5, 4.5, 2.0);

  // // Match load
  // xDrive.driveLocal(-8, -8, 0, volt);
  // waitUntil(odom.getPose().y > 63);
  xDrive.driveTo(-48, 58, -6, 6, 2);
  currentPose = odom.getPose();
  xDrive.turnTo(270, 1);
  odom.setPose(currentPose.x, currentPose.y, imu.heading(deg));
  intake.setSpeed(100);
  xDrive.driveToPose(-60, 58.25, 270, -6, 6, 1.5);
  frontCylinders.toggle();
  xDrive.driveLocal(2, 0, 0, volt);
  task::sleep(1000);
  xDrive.stop();
  task::sleep(1250);

  // Score long Goal
  xDrive.driveTo(-48, 56, -6, 6, 1.5);
  frontCylinders.toggle();
  currentPose = odom.getPose();
  xDrive.turnTo(90, 1);
  odom.setPose(currentPose.x, currentPose.y, imu.heading(deg));
  intake.setSpeed(0);
  liftCylinders.toggle();
  hoodCylinder.toggle();
  xDrive.driveTo(-30, 56, -6, 6, 1.5);
  intake.setSpeed(-100.0);
  task::sleep(100);
  intake.setSpeed(100.0);
  xDrive.driveLocal(-3, 0, 0, volt);
  waitUntil(thirdStage.velocity(rpm) > 100);
  xDrive.stop();
  task::sleep(2000);
  xDrive.driveLocal(8, 0, 0, volt);
  task::sleep(150);
  xDrive.driveLocal(-6, 0, 0, volt);
  task::sleep(150);
  xDrive.stop(coast);
}

void leftAWP(vex::color c)
{
  leftElims(c);

  // Park
  intake.setSpeed(0);
  xDrive.driveToPose(-69.0, 42.0, 0.0, 1.5);
  hoodCylinder.toggle();
  liftCylinders.toggle();

  xDrive.driveLocal(-12.0, -4.0, 0.0, volt);
  waitUntil(imu.roll(deg) > -10.0);
  task::sleep(1500);
  waitUntil(imu.roll(deg) < -10.0);
  xDrive.stop();
}

void skills()
{
  odom.setPose(-55.0, 16.5, 270.0);
  vex::task m = vex::task(runMCL);

  // Match Loads
  xDrive.driveToPose(-57.0, 46.25, 270.0, 1.5);
  frontCylinders.toggle();
  xDrive.driveLocal(2, 0, 0, volt);
  intake.setSpeed(100);
  task::sleep(1000);
  xDrive.stop();
  task::sleep(1000);

  // Score opposite side
  frontCylinders.toggle();
  xDrive.driveToPose(-24, 58, 270, -6, 6, 1.5);
  xDrive.driveTo(36, 58, -6, 6, 2.0);
  vex::task t0 = vex::task([]() {
    liftCylinders.toggle();
    intake.setSpeed(0);
    task::sleep(10);
    hoodCylinder.toggle();
    return 0;
  });
  xDrive.driveTo(36, 46.5, -6, 6, 2);
  xDrive.driveTo(29.5, 46.5, -6, 6, 1.5);
  vex::task t1 = vex::task([]() {
    xDrive.driveLocal(-2.5, 0, 0, volt);
    int t = 0;
    while (thirdStage.velocity(rpm) < 50 && t < 2000) {
      task::sleep(5);
      t += 5;
    }
    xDrive.stop(hold);
    return 0;
  });
  intake.setSpeed(100);
  task::sleep(3000);
  controller1.rumble(".");

}

void autonomous(void) {
  /* TESTING */
  // imu.calibrate();
  // do { task::sleep(5); } while (imu.isCalibrating());
  parallel.resetPosition();
  perpendicular.resetPosition();

  xDrive.setTurnPID(turnPID);
  xDrive.setRotationalPID(rotationalPID);
  xDrive.setLinearPID(drivePID);
  vex::task i = neblib::launchTask(std::bind(&Intake::startLoop, &intake));

  const char* auton = selector.getAuton();
  vex::color c = selector.getColor();

  int startTime = Brain.Timer.time();
  if (neblib::contains(auton, "AWP")) leftAWP(c);
  else if (neblib::contains(auton, "Elims")) leftElims(c);
  else if (neblib::contains(auton, "Skills")) skills();
  else 
  {
    imu.calibrate();
    do { task::sleep(2); } while (imu.isCalibrating());
    odom.setPose(0.0, 0.0, 90.0);
    vex::task t = vex::task(runMCL);
    xDrive.driveToPose(24.0, 0.0, 0.0, -6, 6);
    controller1.rumble(".");
    xDrive.driveToPose(48.0, 0.0, 180.0, -6, 6);
    controller1.rumble(".");
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
  if (neblib::contains(selector.getAuton(), "skills")) imu.setHeading(270, deg);

  neblib::launchTask(std::bind(&Intake::startLoop, &intake));

  bool L1WasPressing = false;
  bool R1WasPressing = false;
  bool aWasPressing = false;
  bool xWasPressing = false;
  bool bWasPressing = false;
  while (true)
  {
    double intakeVelocity = 0.0;
    if (controller1.ButtonR2.pressing()) intakeVelocity = -80.0;
    if (controller1.ButtonL2.pressing()) intakeVelocity = hoodCylinder.getState() ? 70 : 100.0;
    if (controller1.ButtonL1.pressing() && !L1WasPressing) liftCylinders.toggle();
    if (controller1.ButtonR1.pressing() && !R1WasPressing) hoodCylinder.toggle();
    if (controller1.ButtonA.pressing() && !aWasPressing) frontCylinders.toggle();
    if (controller1.ButtonX.pressing() && !xWasPressing) pokeCylinder.toggle();
    if (controller1.ButtonB.pressing() && !bWasPressing) 
    {
      controller1.Screen.clearScreen();
      controller1.Screen.setCursor(1, 1);
      controller1.Screen.print("||%.5lf", parallel.getPosition() / neblib::toRad(imu.rotation(deg)));
      controller1.Screen.setCursor(2, 1);
      controller1.Screen.print(" T%.5lf", perpendicular.getPosition() / neblib::toRad(imu.rotation(deg)));
    }
    intake.setSpeed(intakeVelocity);

    xDrive.driveGlobal(controller1.Axis3.position(percent) * 0.12, controller1.Axis4.position(percent) * 0.12, controller1.Axis1.position(percent) * 0.12, volt);

    L1WasPressing = controller1.ButtonL1.pressing();
    R1WasPressing = controller1.ButtonR1.pressing();
    aWasPressing = controller1.ButtonA.pressing();
    xWasPressing = controller1.ButtonX.pressing();
    bWasPressing = controller1.ButtonB.pressing();

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

