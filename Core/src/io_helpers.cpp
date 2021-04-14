#include "io_helpers.hpp"

string getLine(ifstream infile)
{
    string str = "";
    char ch = ' ';
    while (infile.good())
    {
        infile.get(ch);
        if (ch == '\n')
            break;
        str += ch;
    }
    return str;
}

vector<vector<double> > csvToSpline(string filename)
{ //x,y,dr,dtheta,drr,ddtheta
    ifstream infile;
    infile.open(filename, ifstream::in);
    string line = "";
    vector<double> lineValues;
    vector<vector<double> > allValues = vector<vector<double> >();
    while (infile.good())
    {
        lineValues = vector<double>();
        line = "";
        char ch = ' ';
        while (infile.good())
        {
            infile.get(ch);
            if (ch == '\n')
                break;
            line += ch;
        }
        int index;
        while (true)
        {
            index = line.find_first_of(',');
            if (index == -1)
            {
                lineValues.push_back(stod(line));
                break;
            }
            if (index >= line.size())
            {
                break;
            }
            lineValues.push_back(stod(line.substr(0, index)));
            line = line.substr(index + 1, line.length());
        }
        if (lineValues.size() < 6)
        {
            cerr << "invalid spline csv (not enough values)" << endl;
        }
        allValues.push_back(lineValues);
        //wayPoints.push_back(WayPoint(lineValues.at(0), lineValues.at(1), lineValues.at(2), lineValues.at(3), lineValues.at(4), lineValues.at(5)));
    }
    infile.close();
    return allValues;
}

RotaryPath csvToRotaryPath(string filename)
{ //x,y,dr,dtheta,drr,ddtheta
    ifstream infile;
    infile.open(filename, ifstream::in);
    string line = "";
    vector<double> lineValues;
    vector<RotaryWayPoint> wayPoints = vector<RotaryWayPoint>();
    while (infile.good())
    {
        lineValues = vector<double>();
        line = "";
        char ch = ' ';
        while (infile.good())
        {
            infile.get(ch);
            if (ch == '\n')
                break;
            line += ch;
        }
        int index;
        while (true)
        {
            index = line.find_first_of(',');
            if (index == -1)
            {
                lineValues.push_back(stod(line));
                break;
            }
            if (index >= line.size())
            {
                break;
            }
            lineValues.push_back(stod(line.substr(0, index)));
            line = line.substr(index + 1, line.length());
        }
        if (lineValues.size() < 8)
        {
            cerr << "invalid rotaryPath csv (not enough values)" << endl;
        }
        bool used = false;
        if(lineValues.at(7) > 0) used = true;
        double angle = lineValues.at(6);
        if(angle > M_PI) {
            angle = -2*M_PI+angle;
        } else if(angle < -M_PI) {
            angle = 2*M_PI+angle;
        }
        wayPoints.push_back(RotaryWayPoint(angle, used));
    }
    infile.close();
    return RotaryPath(wayPoints);
}

Kinematics csvToKinematics(string filename)
{
    ifstream infile;
    infile.open(filename, ifstream::in);
    string line = "";
    if (!infile.good())
    {
        cerr << "can't read kinematics config" << endl;
        return Kinematics();
    }
    line = "";
    char ch = ' ';
    while (infile.good())
    {
        infile.get(ch);
        if (ch == '\n')
            break;
        line += ch;
    }
    int index;
    vector<double> lineValues;
    while (true)
    {
        index = line.find_first_of(',');
        if (index == -1)
        {
            lineValues.push_back(stod(line));
            break;
        }
        if (index >= line.size())
        {
            break;
        }
        lineValues.push_back(stod(line.substr(0, index)));
        line = line.substr(index + 1, line.length());
    }
    return Kinematics(lineValues.at(0), lineValues.at(1), lineValues.at(2), lineValues.at(3), lineValues.at(4));
}

Kinematics2 csvToKinematics2(string filename)
{
    ifstream infile;
    infile.open(filename, ifstream::in);
    string line = "";
    if (!infile.good())
    {
        cerr << "can't read kinematics config" << endl;
        return Kinematics2();
    }
    line = "";
    char ch = ' ';
    while (infile.good())
    {
        infile.get(ch);
        if (ch == '\n')
            break;
        line += ch;
    }
    int index;
    vector<double> lineValues;
    while (true)
    {
        index = line.find_first_of(',');
        if (index == -1)
        {
            lineValues.push_back(stod(line));
            break;
        }
        if (index >= line.size())
        {
            break;
        }
        lineValues.push_back(stod(line.substr(0, index)));
        line = line.substr(index + 1, line.length());
    }
    return Kinematics2(lineValues.at(0), lineValues.at(1), lineValues.at(2), lineValues.at(3), lineValues.at(4), lineValues.at(5), lineValues.at(6), lineValues.at(7));
}

void saveTrajectory(vector<MotionState> *profile, string filename)
{
    ofstream myFile(filename);
    if (myFile.is_open())
    {
        for (int i = 0; i < profile->size() && myFile.is_open(); i++)
        {
            myFile << profile->at(i).toString() + "\n";
        }
    }
    else
        cerr << "Unable to open file" << endl;
}

void saveWayPoints(vector<vector<double> > wayPoints, string filename) {
    ofstream myFile(filename);
    if (myFile.is_open())
    {
        for (int i = 0; i < wayPoints.size() && myFile.is_open(); i++)
        {
            string line = "";
            for(int j = 0; j < wayPoints.at(i).size(); j++) {
                line += to_string(wayPoints.at(i).at(j));
                if(j < wayPoints.at(i).size()-1) {
                    line += ",";
                }
            }
            myFile << line + "\n";
        }
    }
    else
        cerr << "Unable to open file" << endl;
}