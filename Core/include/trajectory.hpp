#include "vector"
#include "string"
#include "iostream"
#include "math.h"

#include "spline.hpp"
#include "math_util.hpp"

#ifndef TRAJECTORY_HPP
#define TRAJECTORY_HPP

struct Kinematics
{
    public:
        double vMax, omegaMax, aMax, alphaMax, aCentrMax;

        Kinematics();
        Kinematics(double vMax, double omegaMax, double aMax, double alphaMax, double aCentrMax);
};

struct Kinematics2
{
    public:
        double kS, kV, kA, robotWidth, robotLength, aCentrMax;

        Kinematics2();
        Kinematics2(double kS, double kV, double kA, double robotWidth, double robotLength, double aCentrMax);
};

struct MotionState
{
    public:
        double time,x,y,angle,xVel,yVel,omega,accel,alpha,disp;

        MotionState();
        MotionState(double time, double x,double y,double angle,double xVel,double yVel,double omega,double accel,double alpha,double disp);

        double getSpeed();
        double getAccel();

        string toString();
};

class Trajectory
{
    private:
        Kinematics kinematics;
        Spline spline;
        vector<double> keyPoints;
        vector<MotionState> profile;
        vector<vector<MotionState> > storedProfiles;
        vector<double> keyPointVelocity;
        vector<double> keyPointDisplacement;

        double findPointDCurvature(double val, double start, double end, double findSize);
        double getVelocityOnCurve(double u);
        double getTangentialAccelLeft(double u, double velocity);
        void findKeyPoints(double iterateSize, double findSize, double initialVelocity, double finalVelocity);
        int profileBetweenPoints(int startIndex, double iterationTime, double integralColumns);
        void iterate(vector<MotionState> &p, double &u1, double &u2, bool reversed, double iterationTime, double integralColumns,double &prevDisp);

    public:
        Trajectory();
        Trajectory(Spline spline, Kinematics kinematics);
        Trajectory(vector<WayPoint> wayPoints, Kinematics kinematics);

        void calculate(double iterateSize, double findSize);
        vector<double> getKeyPoints();
        vector<MotionState> *getProfile();
};

#endif