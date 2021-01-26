#include "vector"
#include "iostream"
#include "math.h"

#include "spline.hpp"

using namespace std;

bool splineTest1() { //true if failed
    Spline spline = Spline();
    double px0=2,dx0=-0.5,ddx0=.75,py0=-13,dy0=9.1,ddy0=-6.2;
    double px1=1023,dx1=-43,ddx1=0.84,py1=0,dy1=-5,ddy1=3.25;
    double px2=.351,dx2=-45.9,ddx2=-0.435,py2=35,dy2=-2.35,ddy2=3;
    spline.addSegment(Quintic(px0,dx0,ddx0,px1,dx1,ddx1), Quintic(py0,dy0,ddy0,py1,dy1,ddy1));
    spline.addSegment(Quintic(px1,dx1,ddx1,px2,dx2,ddx2), Quintic(py1,dy1,ddy1,py2,dy2,ddy2));
    if(abs(spline.getValueX(0)-px0) > 0.00001) {
        cout << "test 1" << endl;
        return true;
    }
    if(abs(spline.getValueY(0)-py0) > 0.00001) {
        cout << "test 2" << endl;
        return true;
    }
    if(abs(spline.getValueX(1)-px1) > 0.00001) {
        cout << "test 3" << endl;
        return true;
    }
    if(abs(spline.getValueY(1)-py1) > 0.00001) {
        cout << "test 4" << endl;
        return true;
    } 
    if(abs(spline.getValueX(2)-px2) > 0.00001) {
        cout << "test 5" << endl;
        return true;
    }
    if(abs(spline.getValueY(2)-py2) > 0.00001) {
        cout << "test 6" << endl;
        return true;
    }
    return false;
}

bool splineTest2() { //true if failed
    double px0=2,dx0=-0.5,ddx0=.75,py0=-13,dy0=9.1,ddy0=-6.2;
    double px1=1023,dx1=-43,ddx1=0.84,py1=0,dy1=-5,ddy1=3.25;
    double px2=.351,dx2=-45.9,ddx2=-0.435,py2=35,dy2=-2.35,ddy2=3;
    Spline spline = Spline(Quintic(px0,dx0,ddx0,px1,dx1,ddx1), Quintic(py0,dy0,ddy0,py1,dy1,ddy1));
    spline.addSegment(Quintic(px1,dx1,ddx1,px2,dx2,ddx2), Quintic(py1,dy1,ddy1,py2,dy2,ddy2));
    if(abs(spline.getValueX(0)-px0) > 0.00001) {
        cout << "test 1" << endl;
        return true;
    }
    if(abs(spline.getValueY(0)-py0) > 0.00001) {
        cout << "test 2" << endl;
        return true;
    }
    if(abs(spline.getValueX(1)-px1) > 0.00001) {
        cout << "test 3" << endl;
        return true;
    }
    if(abs(spline.getValueY(1)-py1) > 0.00001) {
        cout << "test 4" << endl;
        return true;
    } 
    if(abs(spline.getValueX(2)-px2) > 0.00001) {
        cout << "test 5" << endl;
        return true;
    }
    if(abs(spline.getValueY(2)-py2) > 0.00001) {
        cout << "test 6" << endl;
        return true;
    }
    return false;
}

int main() {
    if(splineTest1()) {
        cout << "Test failed 1" << endl;
    } else if(splineTest2()) {
        cout << "Test failed 2" << endl;
    }
    cout << "Test passed" << endl;
}