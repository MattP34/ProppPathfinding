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

class PieceWise {
    vector<double> points;
    vector<double> values;
    vector<double> disp;
    vector<double> angles;
    public:
        PieceWise();
        PieceWise(vector<double> points, vector<double> values, vector<double> disp, vector<double> angles);
        double getValue(double x);
        double getAngle(double currDisp);
};

#endif