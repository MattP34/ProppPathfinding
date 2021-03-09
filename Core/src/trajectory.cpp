#include "spline.hpp"
#include "trajectory.hpp"

Kinematics::Kinematics(double vMax, double omegaMax, double aMax, double alphaMax, double aCentrMax) {
    this->vMax = vMax;
    this->omegaMax = omegaMax;
    this->aMax = aMax;
    this->alphaMax = alphaMax;
    this->aCentrMax = aCentrMax;
}

Trajectory::Trajectory(Spline spline, Kinematics kinematics) {
    this->kinematics = kinematics;
    this->spline = spline;
}

Trajectory::Trajectory(vector<WayPoint> wayPoints, Kinematics kinematics) {
    this->kinematics = kinematics;
    this->spline = Spline(wayPoints);
}

