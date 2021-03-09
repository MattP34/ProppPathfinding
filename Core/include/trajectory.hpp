#include "vector"
#include "string"
#include "iostream"
#include "math.h"

#include "spline.hpp"

struct Kinematics
{
    public:
        double vMax, omegaMax, aMax, alphaMax, aCentrMax;

        Kinematics(double vMax, double omegaMax, double aMax, double alphaMax, double aCentrMax);
};

class Trajectory
{
    private:
        Kinematics kinematics;
        Spline spline;
    public:
        Trajectory(Spline spline, Kinematics kinematics);
        Trajectory(vector<WayPoint> wayPoints, Kinematics kinematics);
};