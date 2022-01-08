/**
 * @file spline.hpp
 * @author Matthew Propp34 (https://github.com/MattP34)
 * @brief Defines the functionality for quintic hermite splines and paths of rotation.
 * @version 0.1
 * 
 */
#include <vector>
#include <string>

#ifndef SPLINE_HPP
#define SPLINE_HPP

using namespace std;

/**
 * Struct representing a single waypoint for defining splines
 */
struct WayPoint
{
private:
    double x, y, dx, dy, ddx, ddy;

public:
    WayPoint();
    WayPoint(double x, double y, double dr, double dtheta, double ddr, double ddtheta);
    double getX();
    double getY();
    double getdx();
    double getdy();
    double getddx();
    double getddy();

    string toString();
};

struct Quintic
{
private:
    double a, b, c, d, e, f;

public:
    Quintic(double p0, double v0, double a0, double p1, double v1, double ao);

    double getValue(double x);
    double getDerivativePoint(double x);
    double getAccelPoint(double x);
    double getJerkPoint(double x);
    double getFourthPoint(double x);

    string getEquation();

    static Quintic quinticXfromWayPoint(WayPoint start, WayPoint end);
    static Quintic quinticYfromWayPoint(WayPoint start, WayPoint end);
};

class Spline
{
private:
    vector<Quintic> piecewiseX;
    vector<Quintic> piecewiseY;
    bool noWayPoint;
    WayPoint tempWayPoint;

public:
    Spline();
    Spline(Quintic segmentX, Quintic segmentY);
    Spline(vector<Quintic> piecewiseX, vector<Quintic> piecewiseY);
    Spline(vector<WayPoint> wayPoints);

    void addSegment(Quintic segmentX, Quintic segmentY);
    void addSegment(WayPoint wayPoint);
    void getValue(double u, double outputArray[2]);
    void getDerivativePoint(double u, double outputArray[2]);
    void getAccelPoint(double u, double outputArray[2]);
    void getJerkPoint(double u, double outputArray[2]);
    void getFourthPoint(double u, double outputArray[2]);

    int getLength();

    double getValueX(double u);
    double getDerivativePointX(double u);
    double getAccelPointX(double u);
    double getJerkPointX(double u);
    double getFourthPointX(double u);

    double getValueY(double u);
    double getDerivativePointY(double u);
    double getAccelPointY(double u);
    double getJerkPointY(double u);
    double getFourthPointY(double u);

    double getSpeed(double u);
    double getAccel(double u);

    double getXVelocityComponent(double u);
    double getYVelocityComponent(double u);
    double getXAccelComponent(double u);
    double getYAccelComponent(double u);

    double getDisplacement(double uInitial, double uFinal, double columns);
    double getUOfDisplacement(double uInitial, double target, double columnSize);

    double getRadius(double u);
    double getDRadius(double u);
    double getDDRadius(double u);
    double getAccelForCurve(double u);

    string getEquations();
};

class RotaryWayPoint {
    public:
        double theta;
        bool used;
        RotaryWayPoint();
        RotaryWayPoint(double theta, bool used);
};

class RotaryPath {
    public: 
        vector<RotaryWayPoint> angles;
        RotaryPath();
        RotaryPath(vector<RotaryWayPoint> angles);
};

#endif