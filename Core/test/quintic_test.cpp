#include "spline.hpp"
#include "iostream"
#include "math.h"

using namespace std;

int main()
{
    double p0=4,v0=2.1,a0=-2.3,p1=-34,v1=6.4,a1=0.5;
    Quintic q = Quintic(p0,v0,a0,p1,v1,a1);
    if(abs(q.getValue(0) - p0) > 0.0001) {
        cout << q.getValue(0) << endl;
        cout << "Test failed 1" << endl;
    } else if(abs(q.getValue(1) - p1) > 0.0001) {
        cout << q.getValue(1) << endl;
        cout << "Test failed 2" << endl;
    } else if(abs(q.getDerivativePoint(0) - v0) > 0.0001) {
        cout << q.getDerivativePoint(0) << endl;
        cout << "Test failed 3" << endl;
    } else if(abs(q.getDerivativePoint(1) - v1) > 0.0001) {
        cout << q.getDerivativePoint(1) << endl;
        cout << "Test failed 4" << endl;
    } else if(abs(q.getAccelPoint(0) - a0) > 0.0001) {
        cout << q.getAccelPoint(0) << endl;
        cout << "Test failed 5" << endl;
    } else if(abs(q.getAccelPoint(1) - a1) > 0.0001) {
        cout << q.getAccelPoint(1) << endl;
        cout << "Test failed 6" << endl;
    } else {
        cout << "Test passed" << endl;
    }
}