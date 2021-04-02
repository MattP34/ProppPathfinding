#include "trajectory.hpp"
#include "math.h"
#include "io_helpers.hpp"

int main(int argc, char **argv) {
    if(argc <= 3) return 0;
    string configFile = argv[1];
    string loadFile = argv[2];
    string saveFile = argv[3];
    Spline spline = csvToSpline(loadFile);
    Kinematics kinematics = csvToKinematics(configFile); //might add other configurationns
    cout << kinematics.vMax << endl;
    Trajectory traj = Trajectory(spline, kinematics);
    traj.calculate(.01,.0001);
    saveTrajectory(traj, saveFile);
}