/**
 * @file math_util.hpp
 * @author Matthew Propp34 (https://github.com/MattP34)
 * @brief Defines math operations including points and the rotation piecewise class.
 * @version 0.1
 */

#ifndef MATH_UTIL_HPP
#define MATH_UTIL_HPP

using namespace std;

struct Point
{
public:
    Point();
    Point(double x, double y);

    double x, y;
};

bool sgn(double x);

class RotationPieceWise {
    vector<double> points;
    vector<double> values;
    vector<double> disp;
    vector<double> angles;
    public:
        RotationPieceWise();
        RotationPieceWise(vector<double> points, vector<double> values, vector<double> disp, vector<double> angles);
        double getValue(double x);
        double getAngle(double currDisp);
};

#endif