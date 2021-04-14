#include "trajectory.hpp"
#include "math.h"

Kinematics::Kinematics()
{
    this->vMax = 0;
    this->omegaMax = 0;
    this->aMax = 0;
    this->alphaMax = 0;
    this->aCentrMax = 0;
}

Kinematics::Kinematics(double vMax, double omegaMax, double aMax, double alphaMax, double aCentrMax)
{
    this->vMax = vMax;
    this->omegaMax = omegaMax;
    this->aMax = aMax;
    this->alphaMax = alphaMax;
    this->aCentrMax = aCentrMax;
}

Kinematics2::Kinematics2()
{
    this->voltageMax = 0.0;
    this->kS = 0.0;
    this->kV = 0.0;
    this->kA = 0.0;
    this->aMax = 0.0;
    this->robotWidth = 0.0;
    this->robotLength = 0.0;
    this->aCentrMax = 0.0;
}

Kinematics2::Kinematics2(double voltageMax, double kS, double kV, double kA, double aMax, double robotWidth, double robotLength, double aCentrMax)
{
    this->voltageMax = voltageMax;
    this->kS = kS;
    this->kV = kV;
    this->kA = kA;
    this->aMax = aMax;
    this->robotWidth = robotWidth;
    this->robotLength = robotLength;
    this->aCentrMax = aCentrMax;
}

MotionState::MotionState()
{
    this->time = 0;
    this->x = 0;
    this->y = 0;
    this->angle = 0;
    this->xVel = 0;
    this->yVel = 0;
    this->rotationPercentage = 0;
    this->accel = 0;
}

MotionState::MotionState(double time, double x, double y, double angle, double xVel, double yVel, double rotationPercentage, double accel, double disp)
{
    this->time = time;
    this->x = x;
    this->y = y;
    this->angle = angle;
    this->xVel = xVel;
    this->yVel = yVel;
    this->rotationPercentage = rotationPercentage;
    this->accel = accel;
    this->disp = disp;
}

double MotionState::getSpeed()
{
    return hypot(this->xVel, this->yVel);
}

double MotionState::getAccel()
{
    return this->accel;
}

string MotionState::toString()
{
    //return "t:"+to_string(time)+" x:"+to_string(x)+" y:"+to_string(y)+" v:"+to_string(getSpeed())+" a:"+to_string(accel)+" xv:"+to_string(xVel)+" yv:"+to_string(yVel);
    return to_string(time) + "," + to_string(x) + "," + to_string(y) + "," + to_string(angle) + "," + to_string(getSpeed()) + "," + to_string(atan2(yVel, xVel) * 180.0 / 3.1415) + "," + to_string(rotationPercentage) + "," + to_string(accel) + "," + to_string(disp);
}

Trajectory::Trajectory()
{
    this->kinematics = Kinematics();
    this->spline = Spline();
    this->keyPoints = vector<double>();
    this->profile = vector<MotionState>();
    this->storedProfiles = vector<vector<MotionState> >();
    this->keyPointVelocity = vector<double>();
    this->keyPointDisplacement = vector<double>();
}

Trajectory::Trajectory(Spline spline, RotaryPath rotaryPath, Kinematics kinematics)
{
    this->kinematics = kinematics;
    this->spline = spline;
    this->rotaryPath = rotaryPath;
    this->keyPoints = vector<double>();
    this->profile = vector<MotionState>();
    this->storedProfiles = vector<vector<MotionState> >();
    this->keyPointVelocity = vector<double>();
    this->keyPointDisplacement = vector<double>();
}

Trajectory::Trajectory(vector<WayPoint> wayPoints, RotaryPath rotaryPath, Kinematics kinematics)
{
    this->kinematics = kinematics;
    this->spline = Spline(wayPoints);
    this->rotaryPath = rotaryPath;
    this->keyPoints = vector<double>();
    this->profile = vector<MotionState>();
    this->storedProfiles = vector<vector<MotionState> >();
    this->keyPointVelocity = vector<double>();
    this->keyPointDisplacement = vector<double>();
}

double Trajectory::findPointDCurvature(double val, double start, double end, double findSize)
{
    bool pos = sgn(this->spline.getDRadius(start) - val);
    //cout << pos << endl;
    for (double u = start; u < end; u += findSize)
    {
        //cout << "u:" + to_string(u)+ "R:" + to_string(this->spline.getRadius(u)) + "DR:" + to_string(this->spline.getDRadius(u)) + "DDR:" + to_string(this->spline.getDDRadius(u)) + "" << endl;
        if (pos == sgn(this->spline.getDRadius(u) - val))
            continue;
        if (pos == sgn(this->spline.getDDRadius(u)))
            return -1;
        return u - findSize / 2.0; //this code is to linear estimate instead of middle u-((this->spline.getDCurvature(u)-this->spline.getDCurvature(u-findSize))
    }
    return end;
}

void Trajectory::findKeyPoints(double iterateSize, double findSize, double initialVelocity, double finalVelocity)
{
    bool pos = sgn(this->spline.getDRadius(0));
    this->keyPoints.clear();
    this->keyPointVelocity.clear();
    this->keyPoints.push_back(0);
    this->keyPointVelocity.push_back(initialVelocity);
    this->keyPointDisplacement.push_back(0);
    this->storedProfiles.push_back(vector<MotionState>());
    for (double u = 0; u < this->spline.getLength(); u += iterateSize)
    {
        if (pos == sgn(this->spline.getDRadius(u)))
            continue;
        pos = !pos;
        double val = findPointDCurvature(0, u - iterateSize, u, findSize);
        //cout << "u:" + to_string(u) + " R:" + to_string(spline.getRadius(u)) + " dR:" +to_string(spline.getDRadius(u)) + " ddR:" + to_string(spline.getDDRadius(u))<< endl;
        if (val == -1)
            continue;
        this->keyPoints.push_back(val);
        this->keyPointVelocity.push_back(min(getVelocityOnCurve(val), this->kinematics.vMax));        this->keyPointDisplacement.push_back(this->keyPointDisplacement.at(this->keyPointDisplacement.size() - 1) + this->spline.getDisplacement(this->keyPoints.at(this->keyPoints.size() - 1), val, (val - this->keyPoints.at(this->keyPoints.size() - 1) / iterateSize)));
        this->storedProfiles.push_back(vector<MotionState>());
        this->storedProfiles.push_back(vector<MotionState>());
    }
    this->keyPoints.push_back(this->spline.getLength());
    this->keyPointVelocity.push_back(finalVelocity);
    this->keyPointDisplacement.push_back(this->keyPointDisplacement.at(this->keyPointDisplacement.size() - 1) + this->spline.getDisplacement(this->keyPoints.at(this->keyPoints.size() - 1), this->spline.getLength(), (this->spline.getLength() - this->keyPoints.at(this->keyPoints.size() - 1) / iterateSize)));
    this->storedProfiles.push_back(vector<MotionState>());
}

double Trajectory::getVelocityOnCurve(double u)
{
    double aCentr = this->kinematics.aCentrMax;
    return sqrt(this->spline.getRadius(u) * aCentr);
}

double Trajectory::getTangentialAccelLeft(double u, double speed)
{
    double radius = this->spline.getRadius(u);
    if (radius == INFINITY || radius == -INFINITY) {
        return this->kinematics.aMax;
    }
    double aCentr = this->kinematics.aCentrMax;
    double aMax = this->kinematics.aMax;
    return sqrt(max(1 - pow(pow(speed, 2) / radius / aCentr, 2), 0.0)) * aMax;
}

void Trajectory::iterate(vector<MotionState> &p, double &u1, double &u2, bool reversed, double iterationTime, double integralColumns, double &prevDisp)
{
    MotionState prev = p.at(p.size() - 1);
    u1 = u2;
    int posCoef = 1;
    if (reversed)
        posCoef = -1;
    double velocity = prev.getSpeed() + prev.getAccel() * (posCoef * iterationTime);
    //cout << "prev:" + to_string(prev.getSpeed()) + " now:" + to_string(velocity) + " accel:" + to_string(prev.accel) << endl;
    double accel;
    u2 = this->spline.getUOfDisplacement(u1, velocity * posCoef * iterationTime, (1 / (this->spline.getSpeed(u1))) * velocity * posCoef * iterationTime / integralColumns);
    if (velocity >= this->kinematics.vMax)
    { //not perfect for prev accel will be too high
        velocity = this->kinematics.vMax;
        accel = 0;
    }
    else
    {
        accel = min(getTangentialAccelLeft(u1, velocity), (min(getVelocityOnCurve(u2), this->kinematics.vMax) - velocity) / iterationTime) * posCoef;
    }
    prevDisp += velocity * posCoef * iterationTime;
    p.push_back(MotionState(prev.time + posCoef * iterationTime, spline.getValueX(u1), spline.getValueY(u1), 0,
                            spline.getXVelocityComponent(u1) * velocity, spline.getYVelocityComponent(u1) * velocity, 0,
                            accel, prevDisp));
}

int Trajectory::profileBetweenPoints(int startIndex, double iterationTime, double integralColumns)
{
    vector<MotionState> p1 = vector<MotionState>();
    vector<MotionState> p2 = vector<MotionState>();
    MotionState prev;
    double u1 = this->keyPoints.at(startIndex);
    double u2 = this->keyPoints.at(startIndex + 1);
    double disp1 = this->keyPointDisplacement.at(startIndex);
    double disp2 = this->keyPointDisplacement.at(startIndex + 1);
    double u12, u22;
    double velocity, accel;
    velocity = this->keyPointVelocity.at(startIndex);
    accel = min(getTangentialAccelLeft(u1, velocity), this->kinematics.aMax);
    p1.push_back(MotionState(0, spline.getValueX(u1), spline.getValueY(u1), 0,
                             spline.getXVelocityComponent(u1) * velocity, spline.getYVelocityComponent(u1) * velocity, 0,
                             accel, disp1));
    velocity = this->keyPointVelocity.at(startIndex + 1);
    accel = -min(getTangentialAccelLeft(u2, velocity), this->kinematics.aMax);
    p2.push_back(MotionState(0, spline.getValueX(u2), spline.getValueY(u2), 0,
                             spline.getXVelocityComponent(u2) * velocity, spline.getYVelocityComponent(u2) * velocity, 0,
                             accel, disp2));
    prev = p1.at(0);
    u12 = this->spline.getUOfDisplacement(u1, prev.getSpeed() * iterationTime, (1 / (this->spline.getSpeed(u1))) * prev.getSpeed() * iterationTime / integralColumns);
    prev = p2.at(0);
    //cout << "test4 " + to_string(u2) << endl;
    u22 = this->spline.getUOfDisplacement(u2, prev.getSpeed() * -iterationTime, -(1 / (this->spline.getSpeed(u2))) * prev.getSpeed() * iterationTime / integralColumns);
    //cout << "test3" << endl;
    //int counter = 0;
    while (((u2 - u1) * min(this->spline.getSpeed(u2), this->spline.getSpeed(u1))) / max(p1.at(p1.size() - 1).getSpeed(), p2.at(p2.size() - 1).getSpeed()) > iterationTime)
    { //when the points are too further than the iteration time
        //cout << "u1:" + to_string(u1) + " u2:" + to_string(u2) << endl;
        if (p1.at(p1.size() - 1).getSpeed() < p2.at(p2.size() - 1).getSpeed())
        {
            iterate(p1, u1, u12, false, iterationTime, integralColumns, disp1);
        }
        else
        {
            iterate(p2, u2, u22, true, iterationTime, integralColumns, disp2);
        }
        //cout << "test 7" << endl;
    }
    this->storedProfiles.at(startIndex * 2).clear();
    if (p1.size() > 1)
    {
        for (int i = 0; i < p1.size(); i++)
        { //add to total profile
            this->storedProfiles.at(startIndex * 2).push_back(p1.at(i));
        }
    }
    else
    {
        u1 = this->keyPoints.at(startIndex);
        double timeBetween = this->spline.getDisplacement(u2, u1, integralColumns) / p2.at(p2.size() - 1).getSpeed();
        prev = p2.at(p2.size() - 1);
        velocity = (prev.getSpeed() + prev.getAccel() * timeBetween);
        accel = min(getTangentialAccelLeft(u1, velocity), this->kinematics.aMax);
        this->storedProfiles.at(startIndex * 2).push_back(MotionState(0, this->spline.getValueX(u1), this->spline.getValueY(u1), 0, velocity * this->spline.getXVelocityComponent(u1), velocity * this->spline.getYVelocityComponent(u1), 0, accel, this->keyPointDisplacement.at(startIndex)));
        if (!((prev.getSpeed() * .99 <= this->keyPointVelocity.at(startIndex) && this->keyPointVelocity.at(startIndex) <= velocity * 1.01) || (prev.getSpeed() * 1.01 >= this->keyPointVelocity.at(startIndex) && this->keyPointVelocity.at(startIndex) >= velocity * 0.99)))
        {
            this->keyPointVelocity.at(startIndex) = velocity;
            return -1;
        }
        this->keyPointVelocity.at(startIndex) = velocity;
    }
    this->storedProfiles.at(startIndex * 2 + 1).clear();
    if (p2.size() > 1)
    {
        for (int i = p2.size() - 1; i >= 0; i--)
        {
            this->storedProfiles.at(startIndex * 2 + 1).push_back(p2.at(i));
        }
        //cout << "bye" << endl;
    }
    else
    {
        u2 = this->keyPoints.at(startIndex + 1);
        double timeBetween = this->spline.getDisplacement(u1, u2, integralColumns) / p1.at(p1.size() - 1).getSpeed();
        prev = p1.at(p1.size() - 1);
        velocity = (prev.getSpeed() + prev.getAccel() * timeBetween);
        accel = min(getTangentialAccelLeft(u2, velocity), this->kinematics.aMax);
        this->storedProfiles.at(startIndex * 2 + 1).push_back(MotionState(0, this->spline.getValueX(u2), this->spline.getValueY(u2), 0, velocity * this->spline.getXVelocityComponent(u2), velocity * this->spline.getYVelocityComponent(u2), 0, accel, this->keyPointDisplacement.at(startIndex + 1)));
        if (!((prev.getSpeed() * .99 <= this->keyPointVelocity.at(startIndex + 1) && this->keyPointVelocity.at(startIndex + 1) <= velocity * 1.01) || (prev.getSpeed() * 1.01 >= this->keyPointVelocity.at(startIndex + 1) && this->keyPointVelocity.at(startIndex + 1) >= velocity * 0.99)))
        {
            this->keyPointVelocity.at(startIndex + 1) = velocity;
            return 1;
        }
        //cout << "bye" << endl;
        this->keyPointVelocity.at(startIndex + 1) = velocity;
    }
    return 0;
}

void Trajectory::calculate(double iterateSize, double findSize)
{
    int returnVal = 0;
    this->findKeyPoints(iterateSize, findSize, 0, this->kinematics.vMax);
    int counter = 0;
    for (int i = 0; i < this->keyPoints.size() - 1; i++)
    {
        returnVal = profileBetweenPoints(i, .001, 100);
        if (returnVal == -1)
        {
            if (i == 0)
            {
                cerr << "too fast first speed" << endl;
                return;
            }
            i -= 2;
        }
    }
    double time = 0.0;
    double startTime = 0.0;
    for (int i = 0; i < this->storedProfiles.size(); i++)
    {
        startTime = this->storedProfiles.at(i).at(0).time - time;
        for (int j = 0; j < this->storedProfiles.at(i).size(); j++)
        {
            this->profile.push_back(MotionState(this->storedProfiles.at(i).at(j)));
            this->profile.at(this->profile.size() - 1).time = this->profile.at(this->profile.size() - 1).time - startTime;
        }
        i++;
        startTime = this->storedProfiles.at(i).at(0).time - this->profile.at(this->profile.size() - 1).time;
        for (int j = 0; j < this->storedProfiles.at(i).size() - 1 || (j < this->storedProfiles.at(i).size() && i == this->storedProfiles.size() - 1); j++)
        {
            this->profile.push_back(MotionState(this->storedProfiles.at(i).at(j)));
            this->profile.at(this->profile.size() - 1).time = this->profile.at(this->profile.size() - 1).time - startTime;
        }
        time = this->profile.at(this->profile.size() - 1).time;
    }
}

vector<double> Trajectory::getKeyPoints()
{
    return this->keyPoints;
}

vector<MotionState> *Trajectory::getProfile()
{
    return &(this->profile);
}

Trajectory2::Trajectory2() {
    this->kinematics = Kinematics2();
    this->spline = Spline();
    this->rotaryPath = RotaryPath();
    this->keyPoints = vector<double>();
    this->profile = vector<MotionState>();
    this->storedProfiles = vector<vector<MotionState> >();
    this->keyPointVelocity = vector<double>();
    this->keyPointDisplacement = vector<double>();
}

Trajectory2::Trajectory2(Spline spline, RotaryPath rotaryPath, Kinematics2 kinematics2)
{
    this->kinematics = kinematics2;
    this->spline = spline;
    this->rotaryPath = rotaryPath;
    this->keyPoints = vector<double>();
    this->profile = vector<MotionState>();
    this->storedProfiles = vector<vector<MotionState> >();
    this->keyPointVelocity = vector<double>();
    this->keyPointDisplacement = vector<double>();
}

Trajectory2::Trajectory2(vector<WayPoint> wayPoints, RotaryPath rotaryPath, Kinematics2 kinematics2)
{
    this->kinematics = kinematics2;
    this->spline = Spline(wayPoints);
    this->rotaryPath = rotaryPath;
    this->keyPoints = vector<double>();
    this->profile = vector<MotionState>();
    this->storedProfiles = vector<vector<MotionState> >();
    this->keyPointVelocity = vector<double>();
    this->keyPointDisplacement = vector<double>();
}

double Trajectory2::findPointDCurvature(double val, double start, double end, double findSize)
{
    bool pos = sgn(this->spline.getDRadius(start) - val);
    for (double u = start; u < end; u += findSize)
    {
        if (pos == sgn(this->spline.getDRadius(u) - val))
            continue;
        if (pos == sgn(this->spline.getDDRadius(u)))
            return -1;
        return u - findSize / 2.0; //this code is to linear estimate instead of middle u-((this->spline.getDCurvature(u)-this->spline.getDCurvature(u-findSize))
    }
    return end;
}

void Trajectory2::findKeyPoints(double iterateSize, double findSize, double initialVelocity, double finalVelocity)
{
    bool pos = sgn(this->spline.getDRadius(0));
    this->keyPoints.clear();
    this->keyPointVelocity.clear();
    this->keyPoints.push_back(0);
    this->keyPointVelocity.push_back(initialVelocity);
    this->keyPointDisplacement.push_back(0);
    this->storedProfiles.push_back(vector<MotionState>());
    for (double u = 0; u < this->spline.getLength(); u += iterateSize)
    {
        double val;
        /*if((u > 1 && u-iterateSize <= 1) || (u > 2 && u-iterateSize <= 2) || (u > 3 && u-iterateSize <= 3)) {
            val = (double)(int)u;
            this->keyPoints.push_back(val);
            this->keyPointVelocity.push_back(1.0);
            this->keyPointDisplacement.push_back(this->keyPointDisplacement.at(this->keyPointDisplacement.size() - 1) + this->spline.getDisplacement(this->keyPoints.at(this->keyPoints.size() - 2), val, (val - this->keyPoints.at(this->keyPoints.size() - 2)) / iterateSize));
            this->storedProfiles.push_back(vector<MotionState>());
            this->storedProfiles.push_back(vector<MotionState>());
            continue;
        }*/
        if (pos == sgn(this->spline.getDRadius(u))) {
            continue;
        }
        pos = !pos;
        val = findPointDCurvature(0, u - iterateSize, u, findSize);
        if (val == -1) {
            continue;
        }
        this->keyPoints.push_back(val);
        this->keyPointVelocity.push_back(min(getVelocityOnCurve(val), (this->kinematics.voltageMax - this->kinematics.kS) / this->kinematics.kV));
        this->keyPointDisplacement.push_back(this->keyPointDisplacement.at(this->keyPointDisplacement.size() - 1) + this->spline.getDisplacement(this->keyPoints.at(this->keyPoints.size() - 2), val, (val - this->keyPoints.at(this->keyPoints.size() - 2)) / iterateSize));
        this->storedProfiles.push_back(vector<MotionState>());
        this->storedProfiles.push_back(vector<MotionState>());
    }
    this->keyPoints.push_back(this->spline.getLength());
    this->keyPointVelocity.push_back(finalVelocity);
    this->keyPointDisplacement.push_back(this->keyPointDisplacement.at(this->keyPointDisplacement.size() - 1) + this->spline.getDisplacement(this->keyPoints.at(this->keyPoints.size() - 1), this->spline.getLength(), (this->spline.getLength() - this->keyPoints.at(this->keyPoints.size() - 1) / iterateSize)));
    this->storedProfiles.push_back(vector<MotionState>());
}

double Trajectory2::getRotationPercentage(double startAngle, double endAngle, double startDisp, double endDisp) {
    double deltaAngle = endAngle-startAngle;
    double deltaDisp = endDisp-startDisp;
    if(deltaAngle > M_PI) {
        deltaAngle = -2*M_PI+deltaAngle;
    } else if(deltaAngle < -M_PI) {
        deltaAngle = 2*M_PI+deltaAngle;
    }
    //cout << "start:" << startAngle << " end:" << endAngle << endl;
    //cout << "angle:" << deltaAngle << endl;
    double rotationDistanceTraveled = hypot(this->kinematics.robotLength/2, this->kinematics.robotWidth/2)*deltaAngle;
    //cout << "rdist:" << rotationDistanceTraveled << " disp:" << deltaDisp << endl;
    //cout << "returnVal:" << rotationDistanceTraveled/(deltaDisp+abs(rotationDistanceTraveled)) << endl;
    return rotationDistanceTraveled/(deltaDisp+abs(rotationDistanceTraveled));
}

void Trajectory2::findRotationPercentage(double columns) {
    vector<double> wayPointDisp = vector<double>();
    vector<double> vRotationPercentage = vector<double>();//vector<double>((int)max(this->rotaryPath.angles.size()-1,(unsigned long)0),0.0);
    for(int i = 0; i < this->rotaryPath.angles.size(); i++) {
        vRotationPercentage.push_back(0.0);
    }
    vector<double> angles = vector<double>();
    wayPointDisp.push_back(0.0);
    for(int i = 1; i <= this->spline.getLength(); i++) {
        wayPointDisp.push_back(wayPointDisp.at(i-1)+this->spline.getDisplacement(i-1,i,columns));
    }
    double prevAngle = 0.0;
    double tempDisp = 0.0;
    bool firstAngle = true;
    int removed = 0;
    for(int i = 0; i < this->rotaryPath.angles.size(); i++) {
        if(this->rotaryPath.angles.at(i).used) {
            angles.push_back(this->rotaryPath.angles.at(i).theta);
            if(firstAngle) {
                firstAngle = false; 
            } else {
                vRotationPercentage.at(i-1) = getRotationPercentage(prevAngle, this->rotaryPath.angles.at(i).theta, tempDisp, wayPointDisp.at(i-removed));
            }
            tempDisp = wayPointDisp.at(i-removed);
            prevAngle = this->rotaryPath.angles.at(i).theta;
        } else {
            wayPointDisp.erase(wayPointDisp.begin()+i-removed);
            removed++;
        }
    }
    this->rotationPercentage = PieceWise();
    if(vRotationPercentage.size() == 0) return;
    vector<double> points = vector<double>();
    vector<double> values = vector<double>();
    double prevVal = vRotationPercentage.at(0);
    points.push_back(0);
    values.push_back(prevVal);
    for(int i = 1; i < vRotationPercentage.size(); i++) {
        if(vRotationPercentage.at(i) != prevVal) {
            points.push_back(i);
            values.push_back(vRotationPercentage.at(i));
            prevVal = vRotationPercentage.at(i);
        }
    }
    points.push_back(this->spline.getLength());
    this->rotationPercentage = PieceWise(points,values,wayPointDisp,angles);
}

double Trajectory2::getVelocityOnCurve(double u)
{
    double aCentr = this->kinematics.aCentrMax;
    return sqrt(this->spline.getRadius(u) * aCentr);
}

double Trajectory2::getTangentialAccelLeft(double u, double speed)
{
    double radius = this->spline.getRadius(u);
    if (radius == INFINITY || radius == -INFINITY) {
        return this->kinematics.aMax;
    }
    double aCentr = this->kinematics.aCentrMax;
    double aMax = this->kinematics.aMax;
    return sqrt(max(1 - pow(pow(speed, 2) / radius / aCentr, 2), 0.0)) * aMax;
}

void Trajectory2::iterate(vector<MotionState> &p, double &u1, double &u2, bool reversed, double iterationTime, double integralColumns, double &prevDisp)
{
    MotionState prev = p.at(p.size() - 1);
    u1 = u2;
    int posCoef = 1;
    if (reversed)
        posCoef = -1;
    double velocity = prev.getSpeed() + prev.getAccel() * (posCoef * iterationTime);
    double accel;
    double ratio = (1.0-abs(this->rotationPercentage.getValue(u1)));
    u2 = this->spline.getUOfDisplacement(u1, velocity * ratio * posCoef * iterationTime, (1 / (this->spline.getSpeed(u1))) * velocity * posCoef * iterationTime / integralColumns);
    accel = min(min(getTangentialAccelLeft(u1, velocity), getAccelKin(velocity)), (getVelocityOnCurve(u2) - velocity) / iterationTime) * posCoef;
    //cout << "percentage:" << (1.0-abs(this->rotationPercentage.getValue(u1))) << endl;
    //velocity *= 0.999;//(1.0-abs(this->rotationPercentage.getValue(u1)));
    //accel *= 0.999;//(1.0-abs(this->rotationPercentage.getValue(u1)));
    prevDisp += velocity * ratio * posCoef * iterationTime;
    double angle = this->rotationPercentage.getAngle(prevDisp);
    //cout << "angle:" << angle << endl;
    double rotationPercent = this->rotationPercentage.getValue(u1);
    if(prev.disp > 66.37) {
        //cout << "vel:" << velocity << " accel:" << accel << endl;
        //cout << "test1:" << getAccelKin(velocity) << endl;
        //cout << reversed << endl;
    }
    p.push_back(MotionState(prev.time + posCoef * iterationTime, spline.getValueX(u1), spline.getValueY(u1), angle,
                            spline.getXVelocityComponent(u1) * velocity, spline.getYVelocityComponent(u1) * velocity, rotationPercent,
                            accel, prevDisp));
}

double Trajectory2::getAccelKin(double velocity)
{
    return (this->kinematics.voltageMax - (this->kinematics.kS + velocity * this->kinematics.kV)) / this->kinematics.kA; //make sure veolcity is never negative
}

int counter = 0;

int Trajectory2::profileBetweenPoints(int startIndex, double iterationTime, double integralColumns)
{
    vector<MotionState> p1 = vector<MotionState>();
    vector<MotionState> p2 = vector<MotionState>();
    MotionState prev;
    double u1 = this->keyPoints.at(startIndex);
    double u2 = this->keyPoints.at(startIndex + 1);
    double disp1 = this->keyPointDisplacement.at(startIndex);
    double disp2 = this->keyPointDisplacement.at(startIndex + 1);
    double u12, u22;
    double velocity, accel;
    velocity = this->keyPointVelocity.at(startIndex);
    accel = min(getTangentialAccelLeft(u1, velocity), min(this->kinematics.aMax, getAccelKin(velocity)));
    p1.push_back(MotionState(0, spline.getValueX(u1), spline.getValueY(u1), this->rotationPercentage.getAngle(disp1),
                             spline.getXVelocityComponent(u1) * velocity, spline.getYVelocityComponent(u1) * velocity, this->rotationPercentage.getValue(u1),
                             accel, disp1));
    velocity = this->keyPointVelocity.at(startIndex + 1);
    accel = -min(getTangentialAccelLeft(u2, velocity), min(this->kinematics.aMax, getAccelKin(velocity)));
    p2.push_back(MotionState(0, spline.getValueX(u2), spline.getValueY(u2), this->rotationPercentage.getAngle(disp2),
                             spline.getXVelocityComponent(u2) * velocity, spline.getYVelocityComponent(u2) * velocity, this->rotationPercentage.getValue(u2),
                             accel, disp2));
    prev = p1.at(0);
    u12 = this->spline.getUOfDisplacement(u1, prev.getSpeed() * iterationTime, (1 / (this->spline.getSpeed(u1))) * prev.getSpeed() * iterationTime / integralColumns);
    prev = p2.at(0);
    u22 = this->spline.getUOfDisplacement(u2, prev.getSpeed() * -iterationTime, -(1 / (this->spline.getSpeed(u2))) * prev.getSpeed() * iterationTime / integralColumns);
    while (((u2 - u1) * min(this->spline.getSpeed(u2), this->spline.getSpeed(u1))) / max(p1.at(p1.size() - 1).getSpeed(), p2.at(p2.size() - 1).getSpeed()) > iterationTime)
    { //when the points are too further than the iteration time
        //cout << "u1:" + to_string(u1) + " u2:" + to_string(u2) << endl;
        //cout << counter++ << endl;
        if (p1.at(p1.size() - 1).getSpeed() < p2.at(p2.size() - 1).getSpeed())
        {
            iterate(p1, u1, u12, false, iterationTime, integralColumns, disp1);
        } else
        {
            iterate(p2, u2, u22, true, iterationTime, integralColumns, disp2);
        }
    }
    this->storedProfiles.at(startIndex * 2).clear();
    if (p1.size() > 1 || (p2.size()<=1 && p1.at(0).getSpeed() < p2.at(0).getSpeed()))
    {
        for (int i = 0; i < p1.size(); i++)
        { //add to total profile
            this->storedProfiles.at(startIndex * 2).push_back(p1.at(i));
        }
    }
    else
    {
        u1 = this->keyPoints.at(startIndex);
        double timeBetween = this->spline.getDisplacement(u2, u1, integralColumns) / p2.at(p2.size() - 1).getSpeed();
        prev = p2.at(p2.size() - 1);
        velocity = (prev.getSpeed() + prev.getAccel() * timeBetween);
        accel = min(getTangentialAccelLeft(u1, velocity), getAccelKin(velocity));
        this->storedProfiles.at(startIndex * 2).push_back(MotionState(0, this->spline.getValueX(u1), this->spline.getValueY(u1), this->rotationPercentage.getAngle(disp1), velocity * this->spline.getXVelocityComponent(u1), velocity * this->spline.getYVelocityComponent(u1), this->rotationPercentage.getValue(u1), accel, this->keyPointDisplacement.at(startIndex)));
        if (!((prev.getSpeed() * .99 <= this->keyPointVelocity.at(startIndex) && this->keyPointVelocity.at(startIndex) <= velocity * 1.01) || (prev.getSpeed() * 1.01 >= this->keyPointVelocity.at(startIndex) && this->keyPointVelocity.at(startIndex) >= velocity * 0.99)))
        {
            this->keyPointVelocity.at(startIndex) = velocity;
            return -1;
        }
        this->keyPointVelocity.at(startIndex) = velocity;
    }
    this->storedProfiles.at(startIndex * 2 + 1).clear();
    if (p2.size() > 1)
    {
        for (int i = p2.size() - 1; i >= 0; i--)
        {
            this->storedProfiles.at(startIndex * 2 + 1).push_back(p2.at(i));
        }
        //cout << "bye" << endl;
    }
    else
    {
        u2 = this->keyPoints.at(startIndex + 1);
        double timeBetween = this->spline.getDisplacement(u1, u2, integralColumns) / p1.at(p1.size() - 1).getSpeed();
        prev = p1.at(p1.size() - 1);
        velocity = (prev.getSpeed() + prev.getAccel() * timeBetween);
        accel = min(getTangentialAccelLeft(u2, velocity), getAccelKin(velocity));
        this->storedProfiles.at(startIndex * 2 + 1).push_back(MotionState(0, this->spline.getValueX(u2), this->spline.getValueY(u2), this->rotationPercentage.getAngle(disp2), velocity * this->spline.getXVelocityComponent(u2), velocity * this->spline.getYVelocityComponent(u2), this->rotationPercentage.getValue(u2), accel, this->keyPointDisplacement.at(startIndex + 1)));
        if (!((prev.getSpeed() * .99 <= this->keyPointVelocity.at(startIndex + 1) && this->keyPointVelocity.at(startIndex + 1) <= velocity * 1.01) || (prev.getSpeed() * 1.01 >= this->keyPointVelocity.at(startIndex + 1) && this->keyPointVelocity.at(startIndex + 1) >= velocity * 0.99)))
        {
            this->keyPointVelocity.at(startIndex + 1) = velocity;
            return 1;
        }
        //cout << "bye" << endl;
        this->keyPointVelocity.at(startIndex + 1) = velocity;
    }
    return 0;
}

void Trajectory2::calculate(double iterateSize, double findSize)
{
    int returnVal = 0;
    this->findKeyPoints(iterateSize, findSize, 0, (this->kinematics.voltageMax - this->kinematics.kS) / this->kinematics.kV);
    this->findRotationPercentage(1/iterateSize);
    int counter = 0;
    for (int i = 0; i < this->keyPoints.size() - 1; i++)
    {
        if(counter++ > this->keyPoints.size()*30) {
            MotionState temp = MotionState();
            temp.time = -1;
            this->profile.push_back(temp);
            return;
        }
        returnVal = profileBetweenPoints(i, .001, 100);
        if (returnVal == -1)
        {
            if (i == 0)
            {
                cerr << "too fast first speed" << endl;
                return;
            }
            i -= 2;
        }
    }
    double time = 0.0;
    double startTime = 0.0;
    for (int i = 0; i < this->storedProfiles.size(); i++)
    {
        startTime = this->storedProfiles.at(i).at(0).time - time;
        for (int j = 0; j < this->storedProfiles.at(i).size(); j++)
        {
            this->profile.push_back(MotionState(this->storedProfiles.at(i).at(j)));
            this->profile.at(this->profile.size() - 1).time = this->profile.at(this->profile.size() - 1).time - startTime;
        }
        i++;
        startTime = this->storedProfiles.at(i).at(0).time - this->profile.at(this->profile.size() - 1).time;
        for (int j = 0; j < this->storedProfiles.at(i).size() - 1 || (j < this->storedProfiles.at(i).size() && i == this->storedProfiles.size() - 1); j++)
        {
            this->profile.push_back(MotionState(this->storedProfiles.at(i).at(j)));
            this->profile.at(this->profile.size() - 1).time = this->profile.at(this->profile.size() - 1).time - startTime;
        }
        time = this->profile.at(this->profile.size() - 1).time;
    }
}

vector<double> Trajectory2::getKeyPoints()
{
    return this->keyPoints;
}

vector<MotionState> *Trajectory2::getProfile()
{
    return &(this->profile);
}