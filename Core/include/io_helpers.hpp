#include <iostream>
#include <fstream>
#include <string>
#include "trajectory.hpp"

using namespace std;

Spline csvToSpline(string filename);

Kinematics csvToKinematics(string filename);

void saveTrajectory(Trajectory &trajectory, string filename);