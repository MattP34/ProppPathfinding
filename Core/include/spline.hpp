#include "vector"

using namespace std;

struct WayPoint
{
private:
    double x,y,dx,dy,ddx,ddy;
public:
    WayPoint();
    WayPoint(double x, double y, double dr, double dtheta, double ddr, double ddtheta);
    double getX();
    double getY();
    double getdx();
    double getdy();
    double getddx();
    double getddy();
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

    double getValueX(double u);
    double getDerivativePointX(double u);
    double getAccelPointX(double u);

    double getValueY(double u);
    double getDerivativePointY(double u);
    double getAccelPointY(double u);

    double getSpeed(double u);
    
    double getDisplacement(double uInitial, double uFinal, double columns);
};