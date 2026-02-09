#include "path_generator.hpp"

#include <iostream>
Point::Point(double x, double y, double theta):
    x(x),
    y(y),
    theta(theta) {}

CubicPolynomial::CubicPolynomial(double a, double b, double c, double d):
    a(a),
    b(b),
    c(c),
    d(d) {}

double Segment::v(const double t) { return hypot(x.b + 2*x.c*t + 3*x.d*t*t, y.b + 2*y.c*t + 3*y.d*t*t); }

double Segment::w(const double t)
{
    double dx = x.b + 2*x.c*t + 3*x.d*t*t;
    double dy = y.b + 2*y.c*t + 3*y.d*t*t;

    double ddx = 2*x.c + 6*x.d*t;
    double ddy = 2*y.c + 6*y.d*t;

    return -(dx * ddy - dy * ddx) / (dx*dx + dy*dy);
}

double Segment::arcLength(const CubicPolynomial& x, const CubicPolynomial& y, int samples)
{
    double L = 0.0;
    double previousT = 0.0;
    double previousX = x.a + x.b*previousT + x.c*previousT*previousT + x.d*previousT*previousT*previousT;
    double previousY = y.a + y.b*previousT + y.c*previousT*previousT + y.d*previousT*previousT*previousT;

    for (int i = 1; i <= samples; ++i)
    {
        double t = i / static_cast<double>(samples);
        double currentX = x.a + x.b*t + x.c*t*t + x.d*t*t*t;
        double currentY = y.a + y.b*t + y.c*t*t + y.d*t*t*t;
        L += hypot(currentX - previousX, currentY - previousY);
        previousX = currentX;
        previousY = currentY;
    }
    return L;
}

double Segment::radius(const double t)
{
    const double dx = x.b + 2 * x.c * t + 3 * x.d * t * t;
    const double ddx = 2 * x.c + 6 * x.d * t;
    const double dy = y.b + 2 * y.c * t + 3 * y.d * t * t;
    const double ddy = 2 * y.c + 6 * y.d * t;

    return abs(pow(dx*dx + dy*dy, 1.5) / (dx * ddy - dy * ddx));
}

double Segment::convertTime(double s)
{
    return s / duration;
}

Segment::Segment(CubicPolynomial x, CubicPolynomial y, double trackWidth, double maxVelocity):
    x(x),
    y(y),
    length(arcLength(x, y)),
    duration(length / maxVelocity)
{
    initialVelocity = std::min(maxVelocity / (1.0 + trackWidth / (2.0 * radius(0.0))), maxVelocity);
    finalVelocity = std::min(maxVelocity / (1.0 + trackWidth / (2.0 * radius(1.0))), maxVelocity);
}

inline double PathGenerator::distance(const Point &p0, const Point &p1) { return hypot(p1.x - p0.x, p1.y - p0.y); }

PathGenerator::PathGenerator(vex::motor_group &leftMotors, vex::motor_group &rightMotors, const double trackWidth, const double maxVelocity, const double maxAcceleration):
    leftMotors(&leftMotors),
    rightMotors(&rightMotors),
    trackWidth(trackWidth),
    maxVelocity(maxVelocity),
    maxAcceleration(maxAcceleration)
{}

void PathGenerator::followPath(std::initializer_list<Point> points)
{
    if (points.size() <= 1) 
    { 
        std::cout << "\n\nNot enough points!\n\n";
        return;
    }

    std::vector<Segment> path;
    path.reserve(points.size() - 1);

    for (auto i = points.begin(); std::next(i) != points.end(); ++i)
    {
        auto p0 = *i;
        auto p1 = *std::next(i);

        const double distance = this->distance(p0, p1);
        const double dx0 = distance * sin(neblib::toRad(p0.theta));
        const double dx1 = distance * sin(neblib::toRad(p1.theta));
        const double dy0 = distance * cos(neblib::toRad(p0.theta));
        const double dy1 = distance * cos(neblib::toRad(p1.theta));

        path.push_back(Segment(
            CubicPolynomial(p0.x, dx0, 3 * p1.x - 3 * p0.x - 2 * dx0 - dx1, 2 * p0.x - 2 * p1.x + dx0 + dx1),
            CubicPolynomial(p0.y, dy0, 3 * p1.y - 3 * p0.y - 2 * dy0 - dy1, 2 * p0.y - 2 * p1.y + dy0 + dy1),
            this->trackWidth,
            this->maxVelocity
        ));
    }

    for (size_t i = 0; i < path.size(); ++i)
    {
        if (i == 0) path.at(i).initialVelocity = 0.0;

        path.at(i).finalVelocity = std::min(path.at(i).finalVelocity, path.at(i + 1).initialVelocity);

        if (i + 1 == path.size()) path.at(i + 1).finalVelocity = 0.0;
    }

    for (size_t i = 0; i < path.size(); ++i)
    {
        auto &segment = path.at(i);

        std::cout << "\n\nSegment " << i + 1 << ": X(t) = " << segment.x.a << " + " << segment.x.b << "t + " << segment.x.c << "t^2 + " << segment.x.d << "t^3, Y(t) = " << segment.y.a << " + " << segment.y.b << "t + " << segment.y.c << "t^2 + " << segment.y.d << "t^3";
        for (int j = 0; j < segment.duration * 1000; j += 10)
        {
            const double linearVelocity = segment.v(segment.convertTime(j / 1000.0)) / segment.duration;
            const double angularVelocity = segment.w(segment.convertTime(j / 1000.0)) / segment.duration;

            double leftVelocity = linearVelocity + angularVelocity * trackWidth / 2.0;
            double rightVelocity = linearVelocity - angularVelocity * trackWidth / 2.0;

            const double maxWheel = std::max(abs(leftVelocity), abs(rightVelocity));
            if (maxWheel > maxVelocity)
            {
                const double scale = maxVelocity / maxWheel;
                leftVelocity *= scale;
                rightVelocity *= scale;
            }


            const double leftOutput = leftVelocity / maxVelocity * 100.0;
            const double rightOutput = rightVelocity / maxVelocity * 100.0;

            leftMotors->spin(vex::directionType::fwd, leftOutput, vex::velocityUnits::pct);
            rightMotors->spin(vex::directionType::fwd, rightOutput, vex::velocityUnits::pct);

            vex::task::sleep(10);
        }
    }
}