#include "trajectory.hpp"
#include "math.h"

Kinematics::Kinematics() {
    this->vMax = 0;
    this->omegaMax = 0;
    this->aMax = 0;
    this->alphaMax = 0;
    this->aCentrMax = 0;
}

Kinematics::Kinematics(double vMax, double omegaMax, double aMax, double alphaMax, double aCentrMax) {
    this->vMax = vMax;
    this->omegaMax = omegaMax;
    this->aMax = aMax;
    this->alphaMax = alphaMax;
    this->aCentrMax = aCentrMax;
}

MotionState::MotionState() {
    this->time = 0;
    this->x = 0;
    this->y = 0;
    this->angle = 0;
    this->xVel = 0;
    this->yVel = 0;
    this->omega = 0;
    this->accel = 0;
    this->alpha = 0;
}

MotionState::MotionState(double time, double x,double y,double angle,double xVel,double yVel,double omega,double accel,double alpha) {
    this->time = time;
    this->x = x;
    this->y = y;
    this->angle = angle;
    this->xVel = xVel;
    this->yVel = yVel;
    this->omega = omega;
    this->accel = accel;
    this->alpha = alpha;
}

double MotionState::getSpeed() {
    return hypot(this->xVel,this->yVel);
}

double MotionState::getAccel() {
    return this->accel;
}

string MotionState::toString() {
    //return "t:"+to_string(time)+" x:"+to_string(x)+" y:"+to_string(y)+" v:"+to_string(getSpeed())+" a:"+to_string(accel)+" xv:"+to_string(xVel)+" yv:"+to_string(yVel);
    return to_string(time)+","+to_string(x)+","+to_string(y)+","+to_string(angle)+","+to_string(getSpeed())+","+to_string(atan2(yVel,xVel)*180.0/3.1415)+","+to_string(omega)+","+to_string(accel)+","+to_string(alpha);
}

Trajectory::Trajectory() {
    this->kinematics = Kinematics();
    this->spline = Spline();
    this->keyPoints = vector<double>();
    this->profile = vector<MotionState>();
    this->storedProfiles = vector<vector<MotionState> >();
    this->keyPointVelocity = vector<double>();
}

Trajectory::Trajectory(Spline spline, Kinematics kinematics) {
    this->kinematics = kinematics;
    this->spline = spline;
    this->keyPoints = vector<double>();
    this->profile = vector<MotionState>();
    this->storedProfiles = vector<vector<MotionState> >();
    this->keyPointVelocity = vector<double>();
}

Trajectory::Trajectory(vector<WayPoint> wayPoints, Kinematics kinematics) {
    this->kinematics = kinematics;
    this->spline = Spline(wayPoints);
    this->keyPoints = vector<double>();
    this->profile = vector<MotionState>();
    this->storedProfiles = vector<vector<MotionState> >();
    this->keyPointVelocity = vector<double>();
}

double Trajectory::findPointDCurvature(double val, double start, double end, double findSize) {
    bool pos = sgn(this->spline.getDRadius(start)-val);
    //cout << pos << endl;
    for(double u = start; u < end; u+= findSize) {
        //cout << "u:" + to_string(u)+ "R:" + to_string(this->spline.getRadius(u)) + "DR:" + to_string(this->spline.getDRadius(u)) + "DDR:" + to_string(this->spline.getDDRadius(u)) + "" << endl;
        if(pos == sgn(this->spline.getDRadius(u)-val)) continue;
        if(pos == sgn(this->spline.getDDRadius(u))) return -1;
        return u-findSize/2.0; //this code is to linear estimate instead of middle u-((this->spline.getDCurvature(u)-this->spline.getDCurvature(u-findSize))
    }
    return end;
}

void Trajectory::findKeyPoints(double iterateSize, double findSize, double initialVelocity, double finalVelocity) {
    bool pos = sgn(this->spline.getDRadius(0));
    this->keyPoints.clear();
    this->keyPointVelocity.clear();
    this->keyPoints.push_back(0);
    this->keyPointVelocity.push_back(initialVelocity);
    this->storedProfiles.push_back(vector<MotionState>());
    for(double u = 0; u < this->spline.getLength(); u+=iterateSize) {
        if(pos == sgn(this->spline.getDRadius(u))) continue;
        pos = !pos;
        double val = findPointDCurvature(0,u-iterateSize,u,findSize);
        //cout << "u:" + to_string(u) + " R:" + to_string(spline.getRadius(u)) + " dR:" +to_string(spline.getDRadius(u)) + " ddR:" + to_string(spline.getDDRadius(u))<< endl; 
        if(val == -1) continue;
        this->keyPoints.push_back(val);
        this->keyPointVelocity.push_back(min(getVelocityOnCurve(val),this->kinematics.vMax));
        this->storedProfiles.push_back(vector<MotionState>());
        this->storedProfiles.push_back(vector<MotionState>());
    }
    this->keyPoints.push_back(this->spline.getLength());
    this->keyPointVelocity.push_back(finalVelocity);
    this->storedProfiles.push_back(vector<MotionState>());
}

double Trajectory::getVelocityOnCurve(double u) {
    return sqrt(this->spline.getRadius(u)*this->kinematics.aCentrMax);
}

double Trajectory::getTangentialAccelLeft(double u, double speed) {
    double radius = this->spline.getRadius(u);
    if(radius == INFINITY || radius == -INFINITY) {
        //cout << "hi" + to_string(radius) << endl;
        return this->kinematics.aMax;
    }
    //cout << "value check:" + to_string(pow(speed,2)/radius/this->kinematics.aCentrMax) << endl;
    //cout << "test " + to_string(sqrt(max(1-pow(speed,2)/radius/this->kinematics.aCentrMax,0.0))) << endl;
    //cout << "test2 " + to_string(max(1-pow(speed,2)/radius/this->kinematics.aCentrMax,0.0)) << endl;
    return sqrt(max(1-pow(pow(speed,2)/radius/this->kinematics.aCentrMax,2),0.0))*this->kinematics.aMax;
}

void Trajectory::iterate(vector<MotionState> &p, double &u1, double &u2, bool reversed, double iterationTime, double integralColumns) {
    MotionState prev = p.at(p.size()-1);
    u1 = u2;
    int posCoef = 1;
    if(reversed) posCoef = -1;
    double velocity = prev.getSpeed()+prev.getAccel()*(posCoef*iterationTime);
    //cout << "prev:" + to_string(prev.getSpeed()) + " now:" + to_string(velocity) + " accel:" + to_string(prev.accel) << endl;
    double accel;
    u2 = this->spline.getUOfDisplacement(u1,velocity*posCoef*iterationTime,(1/(this->spline.getSpeed(u1)))*velocity*posCoef*iterationTime/integralColumns);
    if(velocity >= this->kinematics.vMax) { //not perfect for prev accel will be too high
        velocity = this->kinematics.vMax;
        accel = 0;
    } else {
        accel = min(getTangentialAccelLeft(u1,velocity),(min(getVelocityOnCurve(u2),this->kinematics.vMax)-velocity)/iterationTime)*posCoef;
    }
    p.push_back(MotionState(prev.time+posCoef*iterationTime,spline.getValueX(u1),spline.getValueY(u1),0,
        spline.getXVelocityComponent(u1)*velocity,spline.getYVelocityComponent(u1)*velocity,0,
        accel,0));
}

int Trajectory::profileBetweenPoints(int startIndex, double iterationTime, double integralColumns) {
    vector<MotionState> p1 = vector<MotionState>();
    vector<MotionState> p2 = vector<MotionState>();
    MotionState prev;
    double u1 = this->keyPoints.at(startIndex);
    double u2 = this->keyPoints.at(startIndex+1);
    double u12, u22;
    double velocity, accel;
    velocity = this->keyPointVelocity.at(startIndex);
    accel = min(getTangentialAccelLeft(u1,velocity),this->kinematics.aMax);
    p1.push_back(MotionState(0,spline.getValueX(u1),spline.getValueY(u1),0,
        spline.getXVelocityComponent(u1)*velocity,spline.getYVelocityComponent(u1)*velocity,0,
        accel,0));
    velocity = this->keyPointVelocity.at(startIndex+1);
    accel = -min(getTangentialAccelLeft(u2,velocity),this->kinematics.aMax);
    p2.push_back(MotionState(0,spline.getValueX(u2),spline.getValueY(u2),0,
        spline.getXVelocityComponent(u2)*velocity,spline.getYVelocityComponent(u2)*velocity,0,
        accel,0));
    prev = p1.at(0);
    u12 = this->spline.getUOfDisplacement(u1,prev.getSpeed()*iterationTime,(1/(this->spline.getSpeed(u1)))*prev.getSpeed()*iterationTime/integralColumns);
    prev = p2.at(0);
    //cout << "test4 " + to_string(u2) << endl;
    u22 = this->spline.getUOfDisplacement(u2,prev.getSpeed()*-iterationTime,-(1/(this->spline.getSpeed(u2)))*prev.getSpeed()*iterationTime/integralColumns);
    //cout << "test3" << endl;
    //int counter = 0;
    while(((u2-u1)*min(this->spline.getSpeed(u2),this->spline.getSpeed(u1)))/max(p1.at(p1.size()-1).getSpeed(),p2.at(p2.size()-1).getSpeed()) > iterationTime) { //when the points are too further than the iteration time
        //cout << "u1:" + to_string(u1) + " u2:" + to_string(u2) << endl;
        if(p1.at(p1.size()-1).getSpeed() < p2.at(p2.size()-1).getSpeed()) {
            iterate(p1,u1,u12,false,iterationTime,integralColumns);
        } else {
            iterate(p2,u2,u22,true,iterationTime,integralColumns);
        }
        //cout << "test 7" << endl;
    }
    this->storedProfiles.at(startIndex*2).clear();
    if(p1.size() > 1) {
        for(int i = 0; i < p1.size(); i++) { //add to total profile
            this->storedProfiles.at(startIndex*2).push_back(p1.at(i));
        }
    } else {
        u1 = this->keyPoints.at(startIndex);
        double timeBetween = this->spline.getDisplacement(u2,u1,integralColumns)/p2.at(p2.size()-1).getSpeed();
        prev = p2.at(p2.size()-1);
        velocity = (prev.getSpeed()+prev.getAccel()*timeBetween);
        accel = min(getTangentialAccelLeft(u1,velocity),this->kinematics.aMax);
        this->storedProfiles.at(startIndex*2).push_back(MotionState(0,this->spline.getValueX(u1),this->spline.getValueY(u1),0,
            velocity*this->spline.getXVelocityComponent(u1),velocity*this->spline.getYVelocityComponent(u1),0,
            accel,0));
        if(!((prev.getSpeed()*.99 <= this->keyPointVelocity.at(startIndex) && this->keyPointVelocity.at(startIndex) <= velocity*1.01)
            || (prev.getSpeed()*1.01 >= this->keyPointVelocity.at(startIndex) && this->keyPointVelocity.at(startIndex) >= velocity*0.99))) {
            this->keyPointVelocity.at(startIndex) = velocity;
            return -1;
        }
        this->keyPointVelocity.at(startIndex) = velocity;
    }
    this->storedProfiles.at(startIndex*2+1).clear();
    if(p2.size() > 1) {
        for(int i = p2.size()-1; i >= 0; i--) {
            this->storedProfiles.at(startIndex*2+1).push_back(p2.at(i));
        }
    } else {
        u2 = this->keyPoints.at(startIndex+1);
        double timeBetween = this->spline.getDisplacement(u1,u2,integralColumns)/p1.at(p1.size()-1).getSpeed();
        prev = p1.at(p1.size()-1);
        velocity = (prev.getSpeed()+prev.getAccel()*timeBetween);
        accel = min(getTangentialAccelLeft(u2,velocity),this->kinematics.aMax);
        this->storedProfiles.at(startIndex*2+1).push_back(MotionState(0,this->spline.getValueX(u2),this->spline.getValueY(u2),0,
            velocity*this->spline.getXVelocityComponent(u2),velocity*this->spline.getYVelocityComponent(u2),0,
            accel,0));
        if(!((prev.getSpeed()*.99 <= this->keyPointVelocity.at(startIndex+1) && this->keyPointVelocity.at(startIndex+1) <= velocity*1.01)
            || (prev.getSpeed()*1.01 >= this->keyPointVelocity.at(startIndex+1) && this->keyPointVelocity.at(startIndex+1) >= velocity*0.99))) {
            this->keyPointVelocity.at(startIndex+1) = velocity;
            return 1;
        }
        this->keyPointVelocity.at(startIndex+1) = velocity;
    }
    return 0;
}

void Trajectory::calculate(double iterateSize, double findSize) {
    int returnVal = 0;
    this->findKeyPoints(iterateSize, findSize,0,0);
    int counter = 0;
    for(int i = 0; i < this->keyPoints.size()-1; i++) {
        returnVal = profileBetweenPoints(i,.001,100);
        if(returnVal == -1) {
            if(i == 0) {
                cerr << "too fast first speed" << endl;
                return;
            }
            i-=2;
        }
    }
    double time = 0.0;
    double startTime = 0.0;
    for(int i = 0; i < this->storedProfiles.size(); i++) {
        startTime = this->storedProfiles.at(i).at(0).time-time;
        for(int j = 0; j < this->storedProfiles.at(i).size(); j++) {
            this->profile.push_back(MotionState(this->storedProfiles.at(i).at(j)));
            this->profile.at(this->profile.size()-1).time = this->profile.at(this->profile.size()-1).time-startTime;
            //cout << "hi" + this->storedProfiles.at(i).at(j).toString() << endl;
        }
        i++;
        startTime = this->storedProfiles.at(i).at(0).time-this->profile.at(this->profile.size()-1).time;
        for(int j = 0; j < this->storedProfiles.at(i).size()-1 || (j < this->storedProfiles.at(i).size() && i == this->storedProfiles.size()-1); j++) {
            this->profile.push_back(MotionState(this->storedProfiles.at(i).at(j)));
            this->profile.at(this->profile.size()-1).time = this->profile.at(this->profile.size()-1).time-startTime;
        }
        time = this->profile.at(this->profile.size()-1).time;
    }
    /*for(int i = 0; i < this->profile.size(); i++) {
        cout << this->profile.at(i).toString() << endl;
        //cout << to_string(i*.001) + "," + to_string(this->profile.at(i).getSpeed()) << endl;
    }*/
}

vector<double> Trajectory::getKeyPoints() {
    return this->keyPoints;
}

vector<MotionState> *Trajectory::getProfile() {
    return &(this->profile);
}