#ifndef INC_3DOF_THING_PROGRAM_H
#define INC_3DOF_THING_PROGRAM_H

#include <string>
#include <iostream>
#include "Robot.h"

class Program {
    std::string readInput();
    void writeFiles();
    void prompt();

public:

    void run();
};


#endif //INC_3DOF_THING_PROGRAM_H
