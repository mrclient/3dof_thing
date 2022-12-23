#ifndef INC_3DOF_THING_PROGRAM_H
#define INC_3DOF_THING_PROGRAM_H

#include "Robot.h"
#include <string>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <cmath>


class Program {

    Robot robot;

    std::string readInput() const;
    void writeFiles() const;
    void prompt(std::string = "") const;
    void printQuaternion(std::ostream& = std::cout) const;
    void runIKSession() const;
    void runFKSession() const;
    void printTopLevelHelp() const;
    void runTopLevelSession() const;

public:

    void run() const;
};


#endif //INC_3DOF_THING_PROGRAM_H
