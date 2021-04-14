#include "vector"
#include "iostream"
#include "math.h"

#include "trajectory.hpp"
//#include "spline.hpp"
#include "iostream"

using namespace std;

const double DEADBAND = 0.0001;

bool keyPointsTest()
{ //true is fail
    Spline spline = Spline();
    double px0 = 0, py0 = 0, dr0 = 10, dtheta0 = 3.1415 / 2, ddr0 = 10, ddtheta0 = 3.1415 / 2;
    double px1 = 10, py1 = 10, dr1 = 10, dtheta1 = 3.1415 / 2, ddr1 = 10, ddtheta1 = 3.1415 / 2;
    spline.addSegment(WayPoint(px0, py0, dr0, dtheta0, ddr0, ddtheta0));
    spline.addSegment(WayPoint(px1, py1, dr1, dtheta1, ddr1, ddtheta1));
    Trajectory trajectory = Trajectory(spline, RotaryPath(), Kinematics(10.0, 8.0, 10, 10, 10));
    trajectory.calculate(.005, .0001);
    vector<double> v1 = trajectory.getKeyPoints();
    try
    {
        if (abs(v1.at(0) - 0) > DEADBAND)
            return true;
        if (abs(v1.at(1) - 0.238929) > DEADBAND)
            return true;
        if (abs(v1.at(2) - 0.91106) > DEADBAND)
            return true;
        if (abs(v1.at(3) - 0.99009) > DEADBAND)
            return true;
        if (abs(v1.at(4) - 1) > DEADBAND)
            return true;
    }
    catch (int e)
    {
        return true;
    }
    return false;
}

bool generationTest()
{
    Spline spline = Spline();
    double px0 = 0, py0 = 0, dr0 = 10, dtheta0 = 3.1415 / 2, ddr0 = 10, ddtheta0 = 3.1415 / 2;
    double px1 = 10, py1 = 10, dr1 = 10, dtheta1 = 3.1415 / 2, ddr1 = 10, ddtheta1 = 3.1415 / 2;
    spline.addSegment(WayPoint(px0, py0, dr0, dtheta0, ddr0, ddtheta0));
    spline.addSegment(WayPoint(px1, py1, dr1, dtheta1, ddr1, ddtheta1));
    Trajectory trajectory = Trajectory(spline, RotaryPath(), Kinematics(8.0, 1.0, 5.0, 1, 4.0));
    trajectory.calculate(.005, .0001);
    return false;
}

int main()
{
    if (keyPointsTest())
    {
        cout << "Test failed" << endl;
    }
    else
    {
        cout << "Test passed" << endl;
    }
}