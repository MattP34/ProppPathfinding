#include "vector"
#include "iostream"
#include "math.h"

#include "spline.hpp"

using namespace std;

const double DEADBAND = 0.0001;

bool integralTest()
{
    Spline spline = Spline();
    double px0 = 2, py0 = -0.697, dr0 = 69.0, dtheta0 = 2.1, ddr0 = -.986, ddtheta0 = 0;
    double px1 = -37.2, py1 = 8.498, dr1 = 7, dtheta1 = .5, ddr1 = 9.7, ddtheta1 = 6;
    double px2 = 0, py2 = 98.9, dr2 = -.065, dtheta2 = 0, ddr2 = 3.7, ddtheta2 = 4.56;
    spline.addSegment(WayPoint(px0, py0, dr0, dtheta0, ddr0, ddtheta0));
    spline.addSegment(WayPoint(px1, py1, dr1, dtheta1, ddr1, ddtheta1));
    spline.addSegment(WayPoint(px2, py2, dr2, dtheta2, ddr2, ddtheta2));
    if (abs(spline.getDisplacement(0, 1.5, 10000) - 96.2796157) > DEADBAND)
    {
        cout << "test1 1" << endl;
        return true;
    }
    if (abs(spline.getDisplacement(0, 1, 10000) - 46.4762698) > DEADBAND)
    {
        cout << "test1 2" << endl;
        return true;
    }
    if (abs(spline.getDisplacement(.4, 1.8, 10000) - 113.34664) > DEADBAND)
    {
        cout << "test1 3" << endl;
        return true;
    }
    if (abs(spline.getDisplacement(1.8, .4, 10000) + 113.34664) > DEADBAND)
    {
        cout << "test1 4" << endl;
        return true;
    }
    return false;
}

bool integralTest2()
{
    Spline spline = Spline();
    double px0 = 2, py0 = -0.697, dr0 = 69.0, dtheta0 = 2.1, ddr0 = -.986, ddtheta0 = 0;
    double px1 = -37.2, py1 = 8.498, dr1 = 7, dtheta1 = .5, ddr1 = 9.7, ddtheta1 = 6;
    double px2 = 0, py2 = 98.9, dr2 = -.065, dtheta2 = 0, ddr2 = 3.7, ddtheta2 = 4.56;
    spline.addSegment(WayPoint(px0, py0, dr0, dtheta0, ddr0, ddtheta0));
    spline.addSegment(WayPoint(px1, py1, dr1, dtheta1, ddr1, ddtheta1));
    spline.addSegment(WayPoint(px2, py2, dr2, dtheta2, ddr2, ddtheta2));
    if (abs(spline.getUOfDisplacement(0, 96.2796157, .000001) - 1.5) > DEADBAND)
    {
        cout << spline.getUOfDisplacement(0, 96.2796157, .000001) << endl;
        cout << "test2 1" << endl;
        return true;
    }
    if (abs(spline.getUOfDisplacement(0, 46.4762698, .000001) - 1) > DEADBAND)
    {
        cout << "test2 2" << endl;
        return true;
    }
    if (abs(spline.getUOfDisplacement(.4, 113.34664, .000001) - 1.8) > DEADBAND)
    {
        cout << "test2 3" << endl;
        return true;
    }
    if (abs(spline.getUOfDisplacement(1.8, -113.34664, -.000001) - 0.4) > DEADBAND)
    {
        cout << "test2 4" << endl;
        return true;
    }
    return false;
}

double helperMaxCurveV(Spline spline, double u)
{
    return sqrt(pow(pow(spline.getDerivativePointX(u), 2) + pow(spline.getDerivativePointY(u), 2), 1.5) / abs(spline.getDerivativePointX(u) * spline.getAccelPointY(u) - spline.getAccelPointX(u) * spline.getDerivativePointY(u)));
}

double helperAccelCurve(Spline spline, double u)
{
    return sqrt(pow(pow(spline.getDerivativePointX(u), 2) + pow(spline.getDerivativePointY(u), 2), 1.5) / (abs(spline.getDerivativePointX(u) * spline.getAccelPointY(u) - spline.getDerivativePointY(u) * spline.getAccelPointX(u)))) / sqrt(pow(spline.getDerivativePointX(u), 2) + pow(spline.getDerivativePointY(u), 2));
}

bool accelCurveTest()
{
    Spline spline = Spline();
    double px0 = 2, py0 = -0.697, dr0 = 69.0, dtheta0 = 2.1, ddr0 = -.986, ddtheta0 = 0;
    double px1 = -37.2, py1 = 8.498, dr1 = 7, dtheta1 = .5, ddr1 = 9.7, ddtheta1 = 6;
    double px2 = 0, py2 = 98.9, dr2 = -.065, dtheta2 = 0, ddr2 = 3.7, ddtheta2 = 4.56;
    double delta = 0.000001;
    double errorPercentage = 0.01;
    spline.addSegment(WayPoint(px0, py0, dr0, dtheta0, ddr0, ddtheta0));
    spline.addSegment(WayPoint(px1, py1, dr1, dtheta1, ddr1, ddtheta1));
    spline.addSegment(WayPoint(px2, py2, dr2, dtheta2, ddr2, ddtheta2));
    if (abs(spline.getAccelForCurve(0) - helperAccelCurve(spline, 0) * (helperMaxCurveV(spline, delta) - helperMaxCurveV(spline, 0)) / delta) > abs(spline.getAccelForCurve(0)) * errorPercentage)
    {
        cout << "test3 1" << endl;
        return true;
    }
    if (abs(spline.getAccelForCurve(1) - helperAccelCurve(spline, 1) * (helperMaxCurveV(spline, 1 + delta) - helperMaxCurveV(spline, 1)) / delta) > abs(spline.getAccelForCurve(1)) * errorPercentage)
    {
        cout << "test3 2" << endl;
        return true;
    }
    if (abs(spline.getAccelForCurve(2) - helperAccelCurve(spline, 2) * (helperMaxCurveV(spline, 2) - helperMaxCurveV(spline, 2 - delta)) / delta) > abs(spline.getAccelForCurve(2)) * errorPercentage)
    {
        cout << "test3 3" << endl;
        return true;
    }
    if (abs(spline.getAccelForCurve(0.5) - helperAccelCurve(spline, 0.5) * (helperMaxCurveV(spline, 0.5) - helperMaxCurveV(spline, 0.5 - delta)) / delta) > abs(spline.getAccelForCurve(0.5)) * errorPercentage)
    {
        cout << "test3 4" << endl;
        return true;
    }
    return false;
}

bool maxCurveVTest()
{
    Spline spline = Spline();
    double px0 = 2, py0 = -0.697, dr0 = 69.0, dtheta0 = 2.1, ddr0 = -.986, ddtheta0 = 0;
    double px1 = -37.2, py1 = 8.498, dr1 = 7, dtheta1 = .5, ddr1 = 9.7, ddtheta1 = 6;
    double px2 = 0, py2 = 98.9, dr2 = -.065, dtheta2 = 0, ddr2 = 3.7, ddtheta2 = 4.56;
    double delta = 0.000001;
    double errorPercentage = 0.02;
    spline.addSegment(WayPoint(px0, py0, dr0, dtheta0, ddr0, ddtheta0));
    spline.addSegment(WayPoint(px1, py1, dr1, dtheta1, ddr1, ddtheta1));
    spline.addSegment(WayPoint(px2, py2, dr2, dtheta2, ddr2, ddtheta2));
    double deltaU = spline.getUOfDisplacement(0, helperMaxCurveV(spline, 0) * delta, 0.0000000001);
    if (abs(spline.getAccelForCurve(0) - (helperMaxCurveV(spline, 0 + deltaU) - helperMaxCurveV(spline, 0)) / delta) > abs(spline.getAccelForCurve(0)) * errorPercentage)
    {
        cout << "test4 1" << endl;
        return true;
    }
    deltaU = spline.getUOfDisplacement(1, helperMaxCurveV(spline, 1) * delta, 0.0000000001) - 1;
    if (abs(spline.getAccelForCurve(1) - (helperMaxCurveV(spline, 1 + deltaU) - helperMaxCurveV(spline, 1)) / delta) > abs(spline.getAccelForCurve(1)) * errorPercentage)
    {
        cout << "test4 2" << endl;
        return true;
    }
    deltaU = spline.getUOfDisplacement(2, helperMaxCurveV(spline, 2) * -delta, -0.0000000001) - 2;
    if (abs(spline.getAccelForCurve(2) - (helperMaxCurveV(spline, 2) - helperMaxCurveV(spline, 2 + deltaU)) / delta) > abs(spline.getAccelForCurve(2)) * errorPercentage)
    {
        cout << "test4 3" << endl;
        return true;
    }
    deltaU = spline.getUOfDisplacement(0.5, helperMaxCurveV(spline, 0.5) * delta, 0.0000000001) - 0.5;
    if (abs(spline.getAccelForCurve(0.5) - (helperMaxCurveV(spline, 0.5 + deltaU) - helperMaxCurveV(spline, 0.5)) / delta) > abs(spline.getAccelForCurve(0.5)) * errorPercentage)
    {
        cout << "test4 4" << endl;
        return true;
    }
    return false;
}

int main()
{
    if (integralTest())
    {
        cout << "Test failed" << endl;
    }
    else if (integralTest2())
    {
        cout << "Test failed" << endl;
    }
    else if (accelCurveTest())
    {
        cout << "Test failed" << endl;
    }
    else if (maxCurveVTest())
    {
        cout << "Test failed" << endl;
    }
    else
    {
        cout << "Test passed" << endl;
    }
}