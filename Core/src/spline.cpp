#include "vector"
#include "string"
#include "iostream"
#include "math.h"

#include "spline.hpp"

using namespace std;

Quintic::Quintic(double p0, double v0, double a0, double p1, double v1, double a1)
{
    this->a = -6 * p0 + -3 * v0 + -1.0 / 2 * a0 + 1.0 / 2 * a1 + -3 * v1 + 6 * p1;
    this->b = 15 * p0 + 8 * v0 + 3.0 / 2 * a0 + -1 * a1 + 7 * v1 + -15 * p1;
    this->c = -10 * p0 + -6 * v0 + -3.0 / 2 * a0 + 1.0 / 2 * a1 + -4 * v1 + 10 * p1;
    this->d = 1.0 / 2 * a0;
    this->e = 1 * v0;
    this->f = 1 * p0;
}

double Quintic::getValue(double x)
{
    return this->a * pow(x, 5) + this->b * pow(x, 4) + this->c * pow(x, 3) + this->d * pow(x, 2) + this->e * pow(x, 1) + this->f;
}

double Quintic::getDerivativePoint(double x)
{
    return this->a * 5 * pow(x, 4) + this->b * 4 * pow(x, 3) + this->c * 3 * pow(x, 2) + this->d * 2 * pow(x, 1) + this->e;
}

double Quintic::getAccelPoint(double x)
{
    return this->a * 20 * pow(x, 3) + this->b * 12 * pow(x, 2) + this->c * 6 * pow(x, 1) + this->d * 2;
}

double Quintic::getJerkPoint(double x)
{
    return this->a * 60 * pow(x, 2) + this->b * 24 * pow(x, 1) + this->c * 6;
}

double Quintic::getFourthPoint(double x)
{
    return this->a * 120 * pow(x, 1) + this->b * 24;
}

string Quintic::getEquation()
{
    return to_string(this->a) + "t^5+" + to_string(this->b) + "t^4+" + to_string(this->c) + "t^3+" + to_string(this->d) + "t^2+" + to_string(this->e) + "t+" + to_string(this->f);
}

Quintic Quintic::quinticXfromWayPoint(WayPoint start, WayPoint end)
{
    return Quintic(start.getX(), start.getdx(), start.getddx(), end.getX(), end.getdx(), end.getddx());
}

Quintic Quintic::quinticYfromWayPoint(WayPoint start, WayPoint end)
{
    return Quintic(start.getY(), start.getdy(), start.getddy(), end.getY(), end.getdy(), end.getddy());
}

WayPoint::WayPoint(double x, double y, double dr, double dtheta, double ddr, double ddtheta)
{
    this->x = x;
    this->y = y;
    this->dx = cos(dtheta) * dr;
    this->dy = sin(dtheta) * dr;
    this->ddx = cos(ddtheta) * ddr;
    this->ddy = sin(ddtheta) * ddr;
}

WayPoint::WayPoint()
{
    this->x = 0;
    this->y = 0;
    this->dx = 0;
    this->dy = 0;
    this->ddx = 0;
    this->ddy = 0;
}

double WayPoint::getX()
{
    return this->x;
}

double WayPoint::getY()
{
    return this->y;
}

double WayPoint::getdx()
{
    return this->dx;
}

double WayPoint::getdy()
{
    return this->dy;
}

double WayPoint::getddx()
{
    return this->ddx;
}

double WayPoint::getddy()
{
    return this->ddy;
}

string WayPoint::toString()
{
    return "x:" + to_string(x) + " y:" + to_string(x);
}

Spline::Spline()
{
    this->piecewiseX = vector<Quintic>();
    this->piecewiseY = vector<Quintic>();
    this->tempWayPoint = WayPoint();
    this->noWayPoint = true;
}

Spline::Spline(Quintic segmentX, Quintic segmentY)
{
    this->piecewiseX = vector<Quintic>();
    this->piecewiseY = vector<Quintic>();
    this->piecewiseX.push_back(segmentX);
    this->piecewiseY.push_back(segmentY);
    this->noWayPoint = false;
    this->tempWayPoint = WayPoint(segmentX.getValue(1), segmentY.getValue(1), segmentX.getDerivativePoint(1),
                                  segmentY.getDerivativePoint(1), segmentX.getAccelPoint(1), segmentY.getAccelPoint(1));
}

Spline::Spline(vector<Quintic> piecewiseX, vector<Quintic> piecewiseY)
{
    this->piecewiseX = piecewiseX;
    this->piecewiseY = piecewiseY;
    Quintic segmentX = piecewiseX.at(piecewiseX.size() - 1);
    Quintic segmentY = piecewiseY.at(piecewiseY.size() - 1);
    this->noWayPoint = false;
    this->tempWayPoint = WayPoint(segmentX.getValue(1), segmentY.getValue(1), segmentX.getDerivativePoint(1),
                                  segmentY.getDerivativePoint(1), segmentX.getAccelPoint(1), segmentY.getAccelPoint(1));
}

Spline::Spline(vector<WayPoint> wayPoints)
{
    this->piecewiseX = vector<Quintic>();
    this->piecewiseY = vector<Quintic>();
    this->tempWayPoint = WayPoint();
    this->noWayPoint = true;
    for (int i = 0; i < wayPoints.size(); i++)
    {
        this->addSegment(wayPoints.at(i));
    }
}

void Spline::addSegment(Quintic segmentX, Quintic segmentY)
{
    this->piecewiseX.push_back(segmentX);
    this->piecewiseY.push_back(segmentY);
    this->tempWayPoint = WayPoint(segmentX.getValue(1), segmentY.getValue(1), segmentX.getDerivativePoint(1),
                                  segmentY.getDerivativePoint(1), segmentX.getAccelPoint(1), segmentY.getAccelPoint(1));
}

void Spline::addSegment(WayPoint wayPoint)
{
    if (this->noWayPoint)
    {
        this->tempWayPoint = wayPoint;
        this->noWayPoint = false;
        return;
    }
    this->piecewiseX.push_back(Quintic::quinticXfromWayPoint(this->tempWayPoint, wayPoint));
    this->piecewiseY.push_back(Quintic::quinticYfromWayPoint(this->tempWayPoint, wayPoint));
    this->tempWayPoint = wayPoint;
}

int Spline::getLength()
{
    return this->piecewiseX.size();
}

void Spline::getValue(double u, double outputArray[2])
{
    if (u >= this->piecewiseX.size())
    {
        outputArray[0] = this->piecewiseX.at(this->piecewiseX.size() - 1).getValue(1);
        outputArray[1] = this->piecewiseY.at(this->piecewiseY.size() - 1).getValue(1);
        return;
    }
    Quintic quinticX = this->piecewiseX.at((int)u);
    Quintic quinticY = this->piecewiseY.at((int)u);
    outputArray[0] = quinticX.getValue(u - (int)u);
    outputArray[1] = quinticY.getValue(u - (int)u);
}

void Spline::getDerivativePoint(double u, double outputArray[2])
{
    if (u >= this->piecewiseX.size())
    {
        outputArray[0] = this->piecewiseX.at(this->piecewiseX.size() - 1).getDerivativePoint(1);
        outputArray[1] = this->piecewiseY.at(this->piecewiseY.size() - 1).getDerivativePoint(1);
        return;
    }
    Quintic quinticX = this->piecewiseX.at((int)u);
    Quintic quinticY = this->piecewiseY.at((int)u);
    outputArray[0] = quinticX.getDerivativePoint(u - (int)u);
    outputArray[1] = quinticY.getDerivativePoint(u - (int)u);
}

void Spline::getAccelPoint(double u, double outputArray[2])
{
    if (u >= this->piecewiseX.size())
    {
        outputArray[0] = this->piecewiseX.at(this->piecewiseX.size() - 1).getAccelPoint(1);
        outputArray[1] = this->piecewiseY.at(this->piecewiseY.size() - 1).getAccelPoint(1);
        return;
    }
    Quintic quinticX = this->piecewiseX.at((int)u);
    Quintic quinticY = this->piecewiseY.at((int)u);
    outputArray[0] = quinticX.getAccelPoint(u - (int)u);
    outputArray[1] = quinticY.getAccelPoint(u - (int)u);
}

void Spline::getJerkPoint(double u, double outputArray[2])
{
    if (u >= this->piecewiseX.size())
    {
        outputArray[0] = this->piecewiseX.at(this->piecewiseX.size() - 1).getJerkPoint(1);
        outputArray[1] = this->piecewiseY.at(this->piecewiseY.size() - 1).getJerkPoint(1);
        return;
    }
    Quintic quinticX = this->piecewiseX.at((int)u);
    Quintic quinticY = this->piecewiseY.at((int)u);
    outputArray[0] = quinticX.getJerkPoint(u - (int)u);
    outputArray[1] = quinticY.getJerkPoint(u - (int)u);
}

void Spline::getFourthPoint(double u, double outputArray[2])
{
    if (u >= this->piecewiseX.size())
    {
        outputArray[0] = this->piecewiseX.at(this->piecewiseX.size() - 1).getFourthPoint(1);
        outputArray[1] = this->piecewiseY.at(this->piecewiseY.size() - 1).getFourthPoint(1);
        return;
    }
    Quintic quinticX = this->piecewiseX.at((int)u);
    Quintic quinticY = this->piecewiseY.at((int)u);
    outputArray[0] = quinticX.getFourthPoint(u - (int)u);
    outputArray[1] = quinticY.getFourthPoint(u - (int)u);
}

double Spline::getValueX(double u)
{
    if (u >= this->piecewiseX.size())
    {
        return this->piecewiseX.at(this->piecewiseX.size() - 1).getValue(1);
    }
    Quintic quinticX = this->piecewiseX.at((int)u);
    return quinticX.getValue(u - (int)u);
}

double Spline::getDerivativePointX(double u)
{
    if (u >= this->piecewiseX.size())
    {
        return this->piecewiseX.at(this->piecewiseX.size() - 1).getDerivativePoint(1);
    }
    Quintic quinticX = this->piecewiseX.at((int)u);
    return quinticX.getDerivativePoint(u - (int)u);
}

double Spline::getAccelPointX(double u)
{
    if (u >= this->piecewiseX.size())
    {
        return this->piecewiseX.at(this->piecewiseX.size() - 1).getAccelPoint(1);
    }
    Quintic quinticX = this->piecewiseX.at((int)u);
    return quinticX.getAccelPoint(u - (int)u);
}

double Spline::getJerkPointX(double u)
{
    if (u >= this->piecewiseX.size())
    {
        return this->piecewiseX.at(this->piecewiseX.size() - 1).getJerkPoint(1);
    }
    Quintic quinticX = this->piecewiseX.at((int)u);
    return quinticX.getJerkPoint(u - (int)u);
}

double Spline::getFourthPointX(double u)
{
    if (u >= this->piecewiseX.size())
    {
        return this->piecewiseX.at(this->piecewiseX.size() - 1).getFourthPoint(1);
    }
    Quintic quinticX = this->piecewiseX.at((int)u);
    return quinticX.getFourthPoint(u - (int)u);
}

double Spline::getValueY(double u)
{
    if (u >= this->piecewiseY.size())
    {
        return this->piecewiseY.at(this->piecewiseX.size() - 1).getValue(1);
    }
    Quintic quinticY = this->piecewiseY.at((int)u);
    return quinticY.getValue(u - (int)u);
}

double Spline::getDerivativePointY(double u)
{
    if (u >= this->piecewiseY.size())
    {
        return this->piecewiseY.at(this->piecewiseX.size() - 1).getDerivativePoint(1);
    }
    Quintic quinticY = this->piecewiseY.at((int)u);
    return quinticY.getDerivativePoint(u - (int)u);
}

double Spline::getAccelPointY(double u)
{
    if (u >= this->piecewiseY.size())
    {
        return this->piecewiseY.at(this->piecewiseX.size() - 1).getAccelPoint(1);
    }
    Quintic quinticY = this->piecewiseY.at((int)u);
    return quinticY.getAccelPoint(u - (int)u);
}

double Spline::getJerkPointY(double u)
{
    if (u >= this->piecewiseY.size())
    {
        return this->piecewiseY.at(this->piecewiseY.size() - 1).getJerkPoint(1);
    }
    Quintic quinticY = this->piecewiseY.at((int)u);
    return quinticY.getJerkPoint(u - (int)u);
}

double Spline::getFourthPointY(double u)
{
    if (u >= this->piecewiseY.size())
    {
        return this->piecewiseY.at(this->piecewiseY.size() - 1).getFourthPoint(1);
    }
    Quintic quinticY = this->piecewiseY.at((int)u);
    return quinticY.getFourthPoint(u - (int)u);
}

double Spline::getSpeed(double u)
{
    return sqrt(pow(this->getDerivativePointX(u), 2) + pow(this->getDerivativePointY(u), 2));
}

double Spline::getAccel(double u)
{
    return sqrt(pow(this->getAccelPointX(u), 2) + pow(this->getAccelPointY(u), 2));
}

double Spline::getXVelocityComponent(double u)
{
    return getDerivativePointX(u) / getSpeed(u);
}

double Spline::getYVelocityComponent(double u)
{
    return getDerivativePointY(u) / getSpeed(u);
}

double Spline::getXAccelComponent(double u)
{
    return getAccelPointX(u) / getSpeed(u);
}

double Spline::getYAccelComponent(double u)
{
    return getAccelPointY(u) / getSpeed(u);
}

double Spline::getDisplacement(double uInitial, double uFinal, double columns)
{
    double iterate = (uFinal - uInitial) / columns;
    double currX = uInitial;
    double sum = (this->getSpeed(uInitial) + this->getSpeed(uFinal)) * iterate;
    for (int i = 1; i < columns; i++)
    {
        currX += iterate;
        sum += this->getSpeed(currX) * 2 * iterate;
    }
    return sum / 2;
}

double Spline::getUOfDisplacement(double uInitial, double target, double columnSize)
{
    double currX = uInitial;
    double sum = 0;
    while ((target < 0 || sum < target) && (target > 0 || sum > target) && (currX + columnSize <= this->piecewiseX.size() || currX + columnSize >= 0))
    {
        sum += (this->getSpeed(currX) + this->getSpeed(currX + columnSize)) * columnSize / 2;
        currX += columnSize;
    }
    return currX;
}

double Spline::getRadius(double u)
{
    double outputArray[2];
    double dx, dy, ddx, ddy, dddx, dddy;
    this->getDerivativePoint(u, outputArray);
    dx = outputArray[0];
    dy = outputArray[1];
    this->getAccelPoint(u, outputArray);
    ddx = outputArray[0];
    ddy = outputArray[1];
    this->getJerkPoint(u, outputArray);
    dddx = outputArray[0];
    dddy = outputArray[1];
    return pow(pow(dx, 2) + pow(dy, 2), 3.0 / 2.0) / abs(dx * ddy - dy * ddx);
}

double Spline::getDRadius(double u)
{
    double outputArray[2];
    double dx, dy, ddx, ddy, dddx, dddy;
    this->getDerivativePoint(u, outputArray);
    dx = outputArray[0];
    dy = outputArray[1];
    this->getAccelPoint(u, outputArray);
    ddx = outputArray[0];
    ddy = outputArray[1];
    this->getJerkPoint(u, outputArray);
    dddx = outputArray[0];
    dddy = outputArray[1];
    //double top1 = -6*pow(dy*ddx-dx*ddy,2)*(dx*ddx+dy*ddy);
    //double top2 = 2*(pow(dx,2)+pow(dy,2))*(-dy*ddx+dx*ddy)*(-dy*dddx+dx*dddy);
    //double bottom = 2*pow(pow(dx,2)+pow(dy,2),5.0/2.0)*sqrt(pow(dy*ddx-dx*ddy,2));
    double top1 = sqrt(pow(dx, 2) + pow(dy, 2));
    double top2 = pow(dy * ddx - dx * ddy, 2) * (dx * ddx + dy * ddy);
    double top3 = 2 * (pow(dx, 2) + pow(dy, 2)) * (-dy * ddx + dx * ddy) * (-dy * dddx + dx * dddy);
    double bottom = 2 * (pow(abs(dy * ddx - dx * ddy), 3));
    return top1 * (6 * top2 - top3) / bottom;
}

double Spline::getDDRadius(double u)
{
    double outputArray[2];
    double dx, dy, ddx, ddy, dddx, dddy, ddddx, ddddy;
    this->getDerivativePoint(u, outputArray);
    dx = outputArray[0];
    dy = outputArray[1];
    this->getAccelPoint(u, outputArray);
    ddx = outputArray[0];
    ddy = outputArray[1];
    this->getJerkPoint(u, outputArray);
    dddx = outputArray[0];
    dddy = outputArray[1];
    this->getFourthPoint(u, outputArray);
    ddddx = outputArray[0];
    ddddy = outputArray[1];
    double top1 = -6 * (pow(dx, 2) + pow(dy, 2)) * (-dy * ddx + dx * ddy) * (dx * ddx + dy * ddy) * (-dy * dddx + dx * dddy);
    double top2 = 3 * pow(dy * ddx - dx * ddy, 2) * (pow(dx * ddx + dy * ddy, 2) + ((pow(dx, 2) + pow(dy, 2)) * (pow(ddx, 2) + pow(ddy, 2) + dx * dddx + dy + dddy)));
    double top3 = pow(pow(dx, 2) + pow(dy, 2), 2);
    double top4 = (2 * pow(dy * dddx - dx * dddy, 2) - ((dy * ddx - dx * ddy) * (ddy * dddx - ddx * dddy + dy * ddddx - dx * ddddy)));
    double bottom = sqrt(pow(dx, 2) + pow(dy, 2)) * pow(abs(dy * ddx - dx * ddy), 3);
    return (top1 + top2 + (top3 * top4)) / bottom;
}

double Spline::getAccelForCurve(double u)
{
    double top1 = 3 * (this->getDerivativePointY(u) * this->getAccelPointX(u) - this->getDerivativePointX(u) * this->getAccelPointY(u)) * (this->getDerivativePointX(u) * this->getAccelPointX(u) + this->getDerivativePointY(u) * this->getAccelPointY(u));
    double top2 = -this->getDerivativePointY(u) * (pow(this->getDerivativePointX(u), 2) + pow(this->getDerivativePointY(u), 2)) * this->getJerkPointX(u);
    double top3 = this->getDerivativePointX(u) * (pow(this->getDerivativePointX(u), 2) + pow(this->getDerivativePointY(u), 2)) * this->getJerkPointY(u);
    double bottom = 2 * (this->getDerivativePointY(u) * this->getAccelPointX(u) - this->getDerivativePointX(u) * this->getAccelPointY(u)) * sqrt(pow(this->getDerivativePointY(u) * this->getAccelPointX(u) - this->getDerivativePointX(u) * this->getAccelPointY(u), 2));

    return (top1 + top2 + top3) / bottom;
}

string Spline::getEquations()
{
    string str = "";
    for (int i = 0; i < this->piecewiseX.size(); i++)
    {
        str += this->piecewiseX.at(i).getEquation() + "\n";
    }
    str += "\n";
    for (int i = 0; i < this->piecewiseY.size(); i++)
    {
        str += this->piecewiseY.at(i).getEquation() + "\n";
    }
    return str;
}