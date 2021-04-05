#include "trajectory.hpp"
#include "math.h"
#include "io_helpers.hpp"

int main(int argc, char **argv)
{
    if (argc <= 4)
        return 0;
    string configFile = argv[1];
    bool kin1 = true;
    string loadFile = argv[3];
    string saveFile = argv[4];
    Spline spline = csvToSpline(loadFile);
    Trajectory traj = Trajectory();
    if (argv[2][0] == 'f')
    {
        kin1 = false;
        Kinematics2 kinematics2 = csvToKinematics2(configFile); //might add other configurationns
        traj = Trajectory(spline, kinematics2);
    }
    else
    {
        Kinematics kinematics = csvToKinematics(configFile); //might add other configurationns
        traj = Trajectory(spline, kinematics);
    }
    traj.calculate(.01, .0001);
    saveTrajectory(traj, saveFile);
}