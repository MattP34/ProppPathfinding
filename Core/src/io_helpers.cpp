#include "io_helpers.hpp"

string getLine(ifstream infile) {
    string str = "";
    char ch = ' ';
    while(infile.good()) {
        infile.get(ch);
        if(ch == '\n') break;
        str += ch;
    }
    return str;
}

Spline csvToSpline(string filename) { //x,y,dr,dtheta,drr,ddtheta
    ifstream infile;
    infile.open (filename, ifstream::in);
    string line = "";
    vector<double> lineValues;
    vector<WayPoint> wayPoints = vector<WayPoint>();
    while (infile.good()){
        lineValues = vector<double>();
        line = "";
        char ch = ' ';
        while(infile.good()) {
            infile.get(ch);
            if(ch == '\n') break;
            line += ch;
        }
        int index;
        while(true) {
            index = line.find_first_of(',');
            if(index == -1) {
                lineValues.push_back(stod(line));
                break;
            }
            if(index >= line.size()) {
                break;
            }
            lineValues.push_back(stod(line.substr(0,index)));
            line = line.substr(index+1,line.length());
        }
        if(lineValues.size() < 6) {
            cerr << "invalid spline csv (not enought values)" << endl;
        }
        wayPoints.push_back(WayPoint(lineValues.at(0),lineValues.at(1),lineValues.at(2),lineValues.at(3),lineValues.at(4),lineValues.at(5)));
    }
    infile.close();
    return Spline(wayPoints);
}

Kinematics csvToKinematics(string filename) {
    ifstream infile;
    infile.open (filename, ifstream::in);
    string line = "";
    if(!infile.good()) {
        cerr << "can't read kinematics config" << endl;
        return Kinematics();
    }
    line = "";
    char ch = ' ';
    while(infile.good()) {
        infile.get(ch);
        if(ch == '\n') break;
        line += ch;
    }
    int index;
    vector<double> lineValues;
    while(true) {
        index = line.find_first_of(',');
        if(index == -1) {
            lineValues.push_back(stod(line));
            break;
        }
        if(index >= line.size()) {
            break;
        }
        lineValues.push_back(stod(line.substr(0,index)));
        line = line.substr(index+1,line.length());
    }
    return Kinematics(lineValues.at(0),lineValues.at(1),lineValues.at(2),lineValues.at(3),lineValues.at(4));
}

void saveTrajectory(Trajectory &trajectory, string filename) {
    ofstream myFile(filename);
    vector<MotionState> *profile = trajectory.getProfile();
    if(myFile.is_open()) {
        for(int i = 0; i < profile->size() && myFile.is_open(); i++) {
            myFile << profile->at(i).toString() + "\n";
        }
    } else cerr << "Unable to open file" << endl;
}