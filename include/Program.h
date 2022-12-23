#ifndef INC_3DOF_THING_PROGRAM_H
#define INC_3DOF_THING_PROGRAM_H

#include "Robot.h"
#include <string>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <cmath>
#include <cctype>
#include <sstream>


class Program {

    const int angle_step = 5;
    const double crd_step = 0.1;
    const double radius = 0.6;

    Robot robot;

    bool readDataInput(const std::string&, std::vector<double>&) const;
    void writeFiles() const;
    void prompt(const std::string& = "", const std::string& = ">") const;
    void printQuaternion(std::ostream& = std::cout) const;
    void printIKHelp() const;
    void runIKSession() const;
    void printFKHelp() const;
    void runFKSession() const;
    void printTopLevelHelp() const;
    void runTopLevelSession() const;

public:

    void run() const;
};


#endif //INC_3DOF_THING_PROGRAM_H
