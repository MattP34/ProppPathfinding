/**
 * @file io_helpers.hpp
 * @author Matthew Propp34 (https://github.com/MattP34)
 * @brief Defines csv io functionality for the program.
 * @version 0.1
 */

#include <iostream>
#include <fstream>
#include <string>
#include "trajectory.hpp"

using namespace std;

vector<vector<double> > csvToSpline(string filename);

Kinematics csvToKinematics(string filename);

Kinematics2 csvToKinematics2(string filename);

RotaryPath csvToRotaryPath(string filename);

void saveTrajectory(vector<MotionState>* profile, string filename);
void saveWayPoints(vector<vector<double> > wayPoints, string filename);