/**
 * @file math_util.hpp
 * @author Matthew Propp34 (https://github.com/MattP34)
 * @brief Defines math operations including points and the rotation piecewise class.
 * @version 0.1
 */
#include "vector"
#include "math.h"
#include "math_util.hpp"
#include "iostream"

using namespace std;

Point::Point(double x, double y)
{
    this->x = x;
    this->y = y;
}

bool sgn(double x)
{
    if (x >= 0)
        return true;
    return false;
}

RotationPieceWise::RotationPieceWise() {
    this->points = vector<double>();
    this->values = vector<double>();
}

RotationPieceWise::RotationPieceWise(vector<double> points, vector<double> values, vector<double> disp, vector<double> angles) {
    this->points = points;
    this->values = values;
    this->disp = disp;
    this->angles = angles;
}

double RotationPieceWise::getValue(double x) {
    for(int i = 0; i < this->points.size()-1; i++) {
        if(x < this->points.at(i)) {
            return this->values.at(i);
        }
    }
    if(this->values.size() == 0) return 0;
    return this->values.at(this->values.size()-1);
}

double RotationPieceWise::getAngle(double currDisp) {
    double prevDisp = -1, nextDisp = -1;
    double prev = 0.0, next = 0.0;
    if(this->disp.size() <= 1) {
        return 0;
    }
    if(currDisp < this->disp.at(0)) {
        return this->angles.at(0);
    }
    for(int i = 1; i < this->disp.size(); i++) {
        if(currDisp < this->disp.at(i)) {
            //cout << "test " << currDisp << " " << this->disp.at(i) << " " << this->disp.at(i-1) << endl;
            prevDisp = this->disp.at(i-1);
            nextDisp = this->disp.at(i);
            prev = this->angles.at(i-1);
            next = this->angles.at(i);
            break;
        }
    }
    if(currDisp > this->disp.at(this->disp.size()-1) && this->angles.size() >= 2) {
        return this->angles.at(this->angles.size()-1);
        /*prevDisp = this->disp.at(this->disp.size()-2);
        nextDisp = this->disp.at(this->disp.size()-1);
        prev = this->angles.at(this->angles.size()-2);
        next = this->angles.at(this->angles.size()-1);*/
    }
    if(prevDisp == -1) {
        return 0;
    }
    double deltaAngle = next-prev;
    double deltaDisp = nextDisp-prevDisp;
    if(deltaAngle > M_PI) {
        deltaAngle = -2*M_PI+deltaAngle;
    } else if(deltaAngle < -M_PI) {
        deltaAngle = 2*M_PI+deltaAngle;
    }
    double angle = deltaAngle*(currDisp-prevDisp)/(deltaDisp)+prev;
    if(angle > M_PI) {
        angle = -2*M_PI+angle;
    } else if(angle < -M_PI) {
        angle = 2*M_PI+angle;
    }
    return angle;
}