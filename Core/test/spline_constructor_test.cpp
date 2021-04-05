#include "vector"
#include "iostream"
#include "math.h"

#include "spline.hpp"

using namespace std;

const double DEADBAND = 0.00001;

bool splineCheck(Spline spline, double px0, double dx0, double ddx0, double px1, double dx1, double ddx1,
                 double px2, double dx2, double ddx2, double py0, double dy0, double ddy0, double py1, double dy1, double ddy1,
                 double py2, double dy2, double ddy2)
{
    if (abs(spline.getValueX(0) - px0) > DEADBAND)
    {
        cout << "test 1" << endl;
        return true;
    }
    if (abs(spline.getValueY(0) - py0) > DEADBAND)
    {
        cout << "test 2" << endl;
        return true;
    }
    if (abs(spline.getValueX(1) - px1) > DEADBAND)
    {
        cout << "test 3" << endl;
        return true;
    }
    if (abs(spline.getValueY(1) - py1) > DEADBAND)
    {
        cout << "test 4" << endl;
        return true;
    }
    if (abs(spline.getValueX(2) - px2) > DEADBAND)
    {
        cout << "test 5" << endl;
        return true;
    }
    if (abs(spline.getValueY(2) - py2) > DEADBAND)
    {
        cout << "test 6" << endl;
        return true;
    }
    if (abs(spline.getDerivativePointX(0) - dx0) > DEADBAND)
    {
        cout << "test 11" << endl;
        return true;
    }
    if (abs(spline.getDerivativePointY(0) - dy0) > DEADBAND)
    {
        cout << "test 21" << endl;
        return true;
    }
    if (abs(spline.getDerivativePointX(1) - dx1) > DEADBAND)
    {
        cout << "test 31" << endl;
        return true;
    }
    if (abs(spline.getDerivativePointY(1) - dy1) > DEADBAND)
    {
        cout << "test 41" << endl;
        return true;
    }
    if (abs(spline.getDerivativePointX(2) - dx2) > DEADBAND)
    {
        cout << "test 51" << endl;
        return true;
    }
    if (abs(spline.getDerivativePointY(2) - dy2) > DEADBAND)
    {
        cout << "test 61" << endl;
        return true;
    }
    if (abs(spline.getAccelPointX(0) - ddx0) > DEADBAND)
    {
        cout << "test 12" << endl;
        return true;
    }
    if (abs(spline.getAccelPointY(0) - ddy0) > DEADBAND)
    {
        cout << "test 22" << endl;
        return true;
    }
    if (abs(spline.getAccelPointX(1) - ddx1) > DEADBAND)
    {
        cout << "test 32" << endl;
        return true;
    }
    if (abs(spline.getAccelPointY(1) - ddy1) > DEADBAND)
    {
        cout << "test 4" << endl;
        return true;
    }
    if (abs(spline.getAccelPointX(2) - ddx2) > DEADBAND)
    {
        cout << "test 52" << endl;
        return true;
    }
    if (abs(spline.getAccelPointY(2) - ddy2) > DEADBAND)
    {
        cout << "test 62" << endl;
        return true;
    }
    double outputArray[2];
    spline.getValue(0, outputArray);
    if (abs(outputArray[0] - px0) > DEADBAND)
    {
        cout << "test 7" << endl;
        return true;
    }
    if (abs(outputArray[1] - py0) > DEADBAND)
    {
        cout << "test 8" << endl;
        return true;
    }
    spline.getValue(1, outputArray);
    if (abs(outputArray[0] - px1) > DEADBAND)
    {
        cout << "test 9" << endl;
        return true;
    }
    if (abs(outputArray[1] - py1) > DEADBAND)
    {
        cout << "test 10" << endl;
        return true;
    }
    spline.getValue(2, outputArray);
    if (abs(outputArray[0] - px2) > DEADBAND)
    {
        cout << "test 11" << endl;
        return true;
    }
    if (abs(outputArray[1] - py2) > DEADBAND)
    {
        cout << "test 12" << endl;
        return true;
    }
    spline.getDerivativePoint(0, outputArray);
    if (abs(outputArray[0] - dx0) > DEADBAND)
    {
        cout << "test 13" << endl;
        return true;
    }
    if (abs(outputArray[1] - dy0) > DEADBAND)
    {
        cout << "test 14" << endl;
        return true;
    }
    spline.getDerivativePoint(1, outputArray);
    if (abs(outputArray[0] - dx1) > DEADBAND)
    {
        cout << "test 15" << endl;
        return true;
    }
    if (abs(outputArray[1] - dy1) > DEADBAND)
    {
        cout << "test 16" << endl;
        return true;
    }
    spline.getDerivativePoint(2, outputArray);
    if (abs(outputArray[0] - dx2) > DEADBAND)
    {
        cout << "test 17" << endl;
        return true;
    }
    if (abs(outputArray[1] - dy2) > DEADBAND)
    {
        cout << "test 18" << endl;
        return true;
    }
    spline.getAccelPoint(0, outputArray);
    if (abs(outputArray[0] - ddx0) > DEADBAND)
    {
        cout << "test 19" << endl;
        return true;
    }
    if (abs(outputArray[1] - ddy0) > DEADBAND)
    {
        cout << "test 20" << endl;
        return true;
    }
    spline.getAccelPoint(1, outputArray);
    if (abs(outputArray[0] - ddx1) > DEADBAND)
    {
        cout << "test 21" << endl;
        return true;
    }
    if (abs(outputArray[1] - ddy1) > DEADBAND)
    {
        cout << "test 22" << endl;
        return true;
    }
    spline.getAccelPoint(2, outputArray);
    if (abs(outputArray[0] - ddx2) > DEADBAND)
    {
        cout << "test 23" << endl;
        return true;
    }
    if (abs(outputArray[1] - ddy2) > DEADBAND)
    {
        cout << "test 24" << endl;
        return true;
    }
    return false;
}

bool splineTest1()
{ //true if failed
    Spline spline = Spline();
    double px0 = 2, dx0 = -0.5, ddx0 = .75, py0 = -13, dy0 = 9.1, ddy0 = -6.2;
    double px1 = 1023, dx1 = -43, ddx1 = 0.84, py1 = 0, dy1 = -5, ddy1 = 3.25;
    double px2 = .351, dx2 = -45.9, ddx2 = -0.435, py2 = 35, dy2 = -2.35, ddy2 = 3;
    spline.addSegment(Quintic(px0, dx0, ddx0, px1, dx1, ddx1), Quintic(py0, dy0, ddy0, py1, dy1, ddy1));
    spline.addSegment(Quintic(px1, dx1, ddx1, px2, dx2, ddx2), Quintic(py1, dy1, ddy1, py2, dy2, ddy2));
    return splineCheck(spline, px0, dx0, ddx0, px1, dx1, ddx1, px2, dx2, ddx2, py0, dy0, ddy0, py1, dy1, ddy1, py2, dy2, ddy2);
}

bool splineTest2()
{
    double px0 = 2, dx0 = -0.5, ddx0 = .75, py0 = -13, dy0 = 9.1, ddy0 = -6.2;
    double px1 = 1023, dx1 = -43, ddx1 = 0.84, py1 = 0, dy1 = -5, ddy1 = 3.25;
    double px2 = .351, dx2 = -45.9, ddx2 = -0.435, py2 = 35, dy2 = -2.35, ddy2 = 3;
    Spline spline = Spline(Quintic(px0, dx0, ddx0, px1, dx1, ddx1), Quintic(py0, dy0, ddy0, py1, dy1, ddy1));
    spline.addSegment(Quintic(px1, dx1, ddx1, px2, dx2, ddx2), Quintic(py1, dy1, ddy1, py2, dy2, ddy2));
    return splineCheck(spline, px0, dx0, ddx0, px1, dx1, ddx1, px2, dx2, ddx2, py0, dy0, ddy0, py1, dy1, ddy1, py2, dy2, ddy2);
}

bool splineTest3()
{
    double px0 = 2, dx0 = -0.5, ddx0 = .75, py0 = -13, dy0 = 9.1, ddy0 = -6.2;
    double px1 = 1023, dx1 = -43, ddx1 = 0.84, py1 = 0, dy1 = -5, ddy1 = 3.25;
    double px2 = .351, dx2 = -45.9, ddx2 = -0.435, py2 = 35, dy2 = -2.35, ddy2 = 3;
    vector<Quintic> piecewiseX = vector<Quintic>();
    vector<Quintic> piecewiseY = vector<Quintic>();
    piecewiseX.push_back(Quintic(px0, dx0, ddx0, px1, dx1, ddx1));
    piecewiseY.push_back(Quintic(py0, dy0, ddy0, py1, dy1, ddy1));
    piecewiseX.push_back(Quintic(px1, dx1, ddx1, px2, dx2, ddx2));
    piecewiseY.push_back(Quintic(py1, dy1, ddy1, py2, dy2, ddy2));
    Spline spline = Spline(piecewiseX, piecewiseY);
    return splineCheck(spline, px0, dx0, ddx0, px1, dx1, ddx1, px2, dx2, ddx2, py0, dy0, ddy0, py1, dy1, ddy1, py2, dy2, ddy2);
}

double xFromPolar(double r, double theta)
{
    return r * cos(theta);
}

double yFromPolar(double r, double theta)
{
    return r * sin(theta);
}

bool splineTest4()
{ //true if failed
    Spline spline = Spline();
    double px0 = 2, py0 = -0.697, dr0 = 69.0, dtheta0 = 2.1, ddr0 = -.986, ddtheta0 = 0;
    double px1 = -37.2, py1 = 8.498, dr1 = 7, dtheta1 = .5, ddr1 = 9.7, ddtheta1 = 6;
    double px2 = 0, py2 = 98.9, dr2 = -.065, dtheta2 = 0, ddr2 = 3.7, ddtheta2 = 4.56;
    spline.addSegment(WayPoint(px0, py0, dr0, dtheta0, ddr0, ddtheta0));
    spline.addSegment(WayPoint(px1, py1, dr1, dtheta1, ddr1, ddtheta1));
    spline.addSegment(WayPoint(px2, py2, dr2, dtheta2, ddr2, ddtheta2));
    return splineCheck(spline, px0, xFromPolar(dr0, dtheta0), xFromPolar(ddr0, ddtheta0), px1, xFromPolar(dr1, dtheta1), xFromPolar(ddr1, ddtheta1), px2, xFromPolar(dr2, dtheta2), xFromPolar(ddr2, ddtheta2),
                       py0, yFromPolar(dr0, dtheta0), yFromPolar(ddr0, ddtheta0), py1, yFromPolar(dr1, dtheta1), yFromPolar(ddr1, ddtheta1), py2, yFromPolar(dr2, dtheta2), yFromPolar(ddr2, ddtheta2));
}

bool splineTest5()
{
    vector<WayPoint> wayPoints = vector<WayPoint>();
    double px0 = 2, py0 = -0.697, dr0 = 69.0, dtheta0 = 2.1, ddr0 = -.986, ddtheta0 = 0;
    double px1 = -37.2, py1 = 8.498, dr1 = 7, dtheta1 = .5, ddr1 = 9.7, ddtheta1 = 6;
    double px2 = 0, py2 = 98.9, dr2 = -.065, dtheta2 = 0, ddr2 = 3.7, ddtheta2 = 4.56;
    wayPoints.push_back(WayPoint(px0, py0, dr0, dtheta0, ddr0, ddtheta0));
    wayPoints.push_back(WayPoint(px1, py1, dr1, dtheta1, ddr1, ddtheta1));
    wayPoints.push_back(WayPoint(px2, py2, dr2, dtheta2, ddr2, ddtheta2));
    Spline spline = Spline(wayPoints);
    return splineCheck(spline, px0, xFromPolar(dr0, dtheta0), xFromPolar(ddr0, ddtheta0), px1, xFromPolar(dr1, dtheta1), xFromPolar(ddr1, ddtheta1), px2, xFromPolar(dr2, dtheta2), xFromPolar(ddr2, ddtheta2),
                       py0, yFromPolar(dr0, dtheta0), yFromPolar(ddr0, ddtheta0), py1, yFromPolar(dr1, dtheta1), yFromPolar(ddr1, ddtheta1), py2, yFromPolar(dr2, dtheta2), yFromPolar(ddr2, ddtheta2));
}

int main()
{
    if (splineTest1())
    {
        cout << "Test failed 1" << endl;
    }
    else if (splineTest2())
    {
        cout << "Test failed 2" << endl;
    }
    else if (splineTest3())
    {
        cout << "Test failed 3" << endl;
    }
    else if (splineTest4())
    {
        cout << "Test failed 4" << endl;
    }
    else if (splineTest5())
    {
        cout << "Test failed 5" << endl;
    }
    else
    {
        cout << "Test passed" << endl;
    }
}