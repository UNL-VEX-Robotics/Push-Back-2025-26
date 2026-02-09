#pragma once

#include <initializer_list>
#include <vector>

#include "neblib/util.hpp"
#include "vex.h"

struct Point
{
    double x;
    double y;
    double theta;

    Point(double x, double y, double theta);
};

struct CubicPolynomial
{
    double a;
    double b;
    double c;
    double d;

    CubicPolynomial(double a, double b, double c, double d);
};

struct Segment
{
    CubicPolynomial x;
    CubicPolynomial y;
    double length;
    double duration;

    double initialVelocity;
    double finalVelocity;
    double accelerationTime;
    double cruiseTime;
    double decelerationTime;
    double maxVelocity;

    double v(const double t);
    double w(const double t);
    double arcLength(const CubicPolynomial& x, const CubicPolynomial& y, int samples = 100);
    double radius(const double t);
    void calculateAcceleration(double maxAcceleration);
    double convertTime(double s);

    Segment(CubicPolynomial x, CubicPolynomial y, double trackWidth, double maxVelocity);
};

class PathGenerator
{
private:
    vex::motor_group *leftMotors;
    vex::motor_group *rightMotors;

    const double trackWidth;
    const double maxVelocity;
    const double maxAcceleration;

    inline double distance(const Point &p0, const Point &p1);
public:
    PathGenerator(vex::motor_group &leftMotors, vex::motor_group &rightMotors, const double trackWidth, const double maxVelocity, const double maxAcceleration);
    void followPath(std::initializer_list<Point> points);
};