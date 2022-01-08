/**
 * @file io_helpers.cpp
 * @author Matthew Propp34 (https://github.com/MattP34)
 * @brief Defines csv io functionality for the program.
 * @version 0.1
 */
#include "io_helpers.hpp"
/**
 * Returns the next line of the given file stream.
 * 
 * stream must be closed seperatly
 * 
 * @param infile the file stream reference to read
 * @return the next line of the stream
 */
string getLine(ifstream &infile)
{
    string str = "";      //string to hold the current line
    char ch = ' ';        //temp variable to store the current character to check for new lines
    while (infile.good()) //while the stream still has remaining characters
    {
        infile.get(ch); //get the next character and set ch to its value
        if (ch == '\n') //if the next character is a new line, the line is over so break the loop
            break;
        str += ch; //append the line string with the current character
    }
    return str;
}

/**
 * Converts a line of a csv into a vector of doubles
 * 
 * @param line the line of the csv
 * @return the doubles values of the csv
 */
vector<double> convertCSVLineToDoubles(string line)
{
    //instantiating varaibles
    vector<double> lineValues;
    int index = 0;
    while (true) //loop is ended with break statements from within
    {
        index = line.find_first_of(','); //get the element of the next comma
        if (index == -1)                 //if there are no more commas, add the last value and end the loop
        {
            lineValues.push_back(stod(line));
            break;
        }
        //don't really know if this happens, TODO check if this if statement below can be removed
        if (index >= line.size()) //if the index would be too large, end the loop
        {
            break;
        }
        //adds the double between 0 and the index of the comma to the values of the line
        lineValues.push_back(stod(line.substr(0, index)));
        //deletes the previously added value and the first comma from the string
        line = line.substr(index + 1, line.length());
    }
    return lineValues;
}

/**
 * Interprets a CSV value of waypoints values into a 2D array of values that correspond to the spline.
 * 
 * The format of the csv file should be each line has 6 floating point numbers seperated by commas
 * with no trailing comma. Each line represents one waypoint. See format and example below.
 * x,y,dr,dtheta,ddr,ddtheta,{yaw,used(postive=true,negative=false)} {} are optional
 * 2.2052402,9.563319,5.0,1.5707963,5.0,1.5707963,-0.0,-1.0
 * 1.3755455,19.606987,5.0,1.5707963,5.0,1.5707963,-1.2565647,1.0
 * -0.5895195,24.323145,5.0,1.5707963,5.0,1.5707963,-1.2753555,1.0
 * 
 * @param filename the names of the csv file
 * @return A 2D array of doubles where each inner array is in the form {x,y,dr,dtheta,ddr,ddtheta}
 */
vector<vector<double> > csvToSpline(string filename)
{ //x,y,dr,dtheta,ddr,ddtheta,{yaw,used} {} are optional
    //open a file stream of the given filename
    ifstream infile;
    infile.open(filename, ifstream::in);
    //instantiating varaibles
    string line = "";
    vector<double> lineValues;
    vector<vector<double> > allValues = vector<vector<double> >();
    while (infile.good()) //while the file has characters left
    {
        lineValues = vector<double>(); //values for this line of the csv
        line = getLine(infile);        //get the next line
        //sometimes csv's get saved with an empty last line so this prevents it from breaking the code
        //just ignores the last line instead of throwing an error
        if (line.size() == 0)
        {
            break;
        }
        lineValues = convertCSVLineToDoubles(line); //interpret the csv line
        //if there are less than the 6 required values for a spline waypoint, print an error message
        if (lineValues.size() < 6)
        {
            cerr << "invalid spline csv (not enough values)" << endl;
        }
        //add this lines values to the 2D array
        allValues.push_back(lineValues);
    }
    infile.close(); //close the file stream
    return allValues;
}

/**
 * Interprets a CSV value of waypoints values into a rotary path
 * 
 * The format of the csv file should be each line has 6 floating point numbers seperated by commas
 * with no trailing comma. Each line represents one waypoint. Yaw is the directional facing of robot
 * and used is if the waypoint has a defined yaw (positve double is True, negative double is false)
 * All Angles should be in radians.
 * See format and example below.
 * x,y,dr,dtheta,ddr,ddtheta,{yaw,used(postive=true,negative=false)} {} are optional
 * 2.2052402,9.563319,5.0,1.5707963,5.0,1.5707963,-0.0,-1.0
 * 1.3755455,19.606987,5.0,1.5707963,5.0,1.5707963,-1.2565647,1.0
 * -0.5895195,24.323145,5.0,1.5707963,5.0,1.5707963,-1.2753555,1.0
 * 
 * @param filename the names of the csv file with radian values for angles
 * @return a rotary path to do the turns for the given set of waypoints
 */
RotaryPath csvToRotaryPath(string filename)
{ //x,y,dr,dtheta,drr,ddtheta
    //open the file stream with the given filename
    ifstream infile;
    infile.open(filename, ifstream::in);
    //instantiating varaibles
    string line = "";
    vector<double> lineValues;
    vector<RotaryWayPoint> wayPoints = vector<RotaryWayPoint>();
    while (infile.good()) //while the file stream has more characters
    {

        lineValues = vector<double>(); //values for this line of the csv
        line = getLine(infile);        //get the line from the file stream
        int index;
        //sometimes csv's get saved with an empty last line so this prevents it from breaking the code
        //just ignores the last line instead of throwing an error
        if (line.size() == 0)
        {
            break;
        }
        lineValues = convertCSVLineToDoubles(line); //interpret the csv line
        //if there are less than the 8 required values for a spline waypoint with a rotary path, print an error message
        if (lineValues.size() < 8)
        {
            cerr << "invalid rotaryPath csv (not enough values)" << endl;
        }
        bool used = false;
        if (lineValues.at(7) > 0)
        {
            used = true;
        }
        double yaw = lineValues.at(6);
        if (yaw > M_PI)
        {
            yaw = -2 * M_PI + yaw;
        }
        else if (yaw < -M_PI)
        {
            yaw = 2 * M_PI + yaw;
        }
        wayPoints.push_back(RotaryWayPoint(yaw, used));
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

void saveWayPoints(vector<vector<double> > wayPoints, string filename)
{
    ofstream myFile(filename);
    if (myFile.is_open())
    {
        for (int i = 0; i < wayPoints.size() && myFile.is_open(); i++)
        {
            string line = "";
            for (int j = 0; j < wayPoints.at(i).size(); j++)
            {
                line += to_string(wayPoints.at(i).at(j));
                if (j < wayPoints.at(i).size() - 1)
                {
                    line += ",";
                }
            }
            myFile << line + "\n";
        }
    }
    else
    {
        cerr << "Unable to open file" << endl;
    }
}