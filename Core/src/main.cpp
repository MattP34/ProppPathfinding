#include "trajectory.hpp"
#include "io_helpers.hpp"
#include <math.h>
#include <cstdio>
#include <ctime>

double run(char **argv, vector<vector<double> > allPoints, RotaryPath rotaryPath, bool save=true, double iterationSize = 0.01, double findSize = 0.00001) {
    string configFile = argv[1];
    string loadFile = argv[3];
    string saveFile = argv[4];
    vector<WayPoint> wayPoints = vector<WayPoint>();
    for(int i = 0; i < allPoints.size(); i++) {
        wayPoints.push_back(WayPoint(allPoints.at(i).at(0), allPoints.at(i).at(1), allPoints.at(i).at(2), allPoints.at(i).at(3), allPoints.at(i).at(4), allPoints.at(i).at(5)));
    }
    Spline spline = Spline(wayPoints);
    if (argv[2][0] == 'f')
    {
        Trajectory2 traj = Trajectory2();
        Kinematics2 kinematics2 = csvToKinematics2(configFile); //might add other configuratins
        traj = Trajectory2(spline, rotaryPath, kinematics2);
        traj.calculate(iterationSize, findSize);
        if(save) {
            saveTrajectory(traj.getProfile(), saveFile);
        }
        if(traj.getProfile()->empty()) return 0;
        return traj.getProfile()->at(traj.getProfile()->size()-1).time;
    }
    else
    {
        Trajectory traj = Trajectory();
        Kinematics kinematics = csvToKinematics(configFile); //might add other configurations
        traj = Trajectory(spline, rotaryPath, kinematics);
        traj.calculate(iterationSize, findSize);
        if(save) {
            saveTrajectory(traj.getProfile(), saveFile);
        }
        if(traj.getProfile()->size() == 0) return 0;
        return traj.getProfile()->at(traj.getProfile()->size()-1).time;
    }
}

void tune(char **argv, vector<vector<double> > allPoints, RotaryPath rotaryPath) {
    string loadFile = argv[3];
    double searchSize = 0.01;
    double learningRate = 0.03;
    vector<double*> values = vector<double*>();
    vector<double> gradient = vector<double>();
    for(int i = 0; i < allPoints.size(); i++) {
        vector <double>::iterator it = allPoints.at(i).begin();
        values.push_back(&(*(it+2)));
        values.push_back(&(*(it+3)));
        values.push_back(&(*(it+4)));
        values.push_back(&(*(it+5)));
        gradient.push_back(0);
        gradient.push_back(0);
        gradient.push_back(0);
        gradient.push_back(0);
    }
    for(int i = 0; i < 100; i++) {
        for(int j = 0; j < values.size(); j++) {
            if(j%1 == 0) cout << "run:" << j << endl;
            double iVal = run(argv, allPoints, rotaryPath, false, 0.005, 0.001);
            if(iVal < 0) {
                cout << "timeout1" << endl;
                gradient.at(j) = 0;
                continue;
            }
            *values.at(j) += searchSize;
            double fVal = run(argv, allPoints, rotaryPath, false, 0.005, 0.001);
            if(fVal < 0) {
                cout << "timeout2" << endl;
                gradient.at(j) = 0;
                continue;
            }
            *values.at(j) -= searchSize;
            gradient.at(j) = (fVal-iVal)/searchSize;
        }
        for(int j = 0; j < values.size(); j++) {
            *values.at(j) += gradient.at(j)*-1*learningRate;
        }
        cout << "Test8" << endl;
        saveWayPoints(allPoints, loadFile);
        cout << "iteration:" << i << " time:" << run(argv, allPoints, rotaryPath, false) << endl;
    }
}

int main(int argc, char **argv)
{
    //COMMAND LINE FORMAT: arg1: config file, arg2: f, arg3: load file, arg4: save file, arg5: anything but f to tune, otherwise not needed
    //tuning takes the path from trajectory visualization, optimizes it - can save seconds
    //CSV FORMAT for loadfile: each line is a waypoint
    //Waypoints: x, y, dr value, dtheta value, ddr, ddtheta, angle, if you use that angle (>1 use, <1 dont)
    //CSV for config file (always use Kinematics2): max voltage of robot, ks, kv, ka, max acceleration, robot width, robot length, maximum centripetal acceleration
    //max acceleration = max acceleration before the wheels skid, max centripetal = max before robot slides
    //Single line csv
    if (argc <= 4)
        return 0;
    string configFile = argv[1];
    string loadFile = argv[3];
    string saveFile = argv[4];
    vector<vector<double> > spline = csvToSpline(loadFile);
    RotaryPath rotaryPath = csvToRotaryPath(loadFile);
    if(argc <= 5 || argv[5][0] == 'f') {
        clock_t start;
        double duration;
        start = clock();
        run(argv, spline, rotaryPath);
        duration = (clock() - start ) / (double) CLOCKS_PER_SEC;
        cout << "Seconds: "<< duration <<'\n';
    } else {
        tune(argv, spline, rotaryPath);
    }
}