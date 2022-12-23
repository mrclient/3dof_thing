#include "Program.h"


void Program::run() const{
    std::cout << std::endl << "Welcome. Read carefully information below to proceed." << std::endl;
    runTopLevelSession();
    std::cout << "The program is about to close.." << std::endl;
}


void Program::printTopLevelHelp() const{
    std::cout << "Input:" << std::endl;
    std::cout << "\tw - to output files with a solution of the task;" << std::endl;
    std::cout << "\tf - for an interactive FK session;" << std::endl;
    std::cout << "\ti - for an interactive IK session;" << std::endl;
    std::cout << "\tr - to output the quaternion;" << std::endl;
    std::cout << "\th - to repeat this information;" << std::endl;
    std::cout << "\tq - to quit." << std::endl;
}


void Program::runTopLevelSession() const{
    std::string c;
    printTopLevelHelp();
    while(true) {
        prompt();
        c = readInput();
        if(c == "")
            { }
        else if(c == "w")
            writeFiles();
        else if(c == "f") {
            runFKSession();
            printTopLevelHelp();
        }
        else if(c == "i"){
            runIKSession();
            printTopLevelHelp();
        }
        else if(c == "r")
            printQuaternion();
        else if(c == "h")
            printTopLevelHelp();
        else if(c == "q")
            return;
        else
            std::cout << "Incorrect input. Please repeat." << std::endl;
    }
}


void Program::prompt(std::string prefix) const{
    std::cout << prefix << ">";
}


std::string Program::readInput() const {
    // TODO add simple preprocessing
    std::string input;
    std::getline(std::cin, input);
    return input;
}


void Program::runFKSession() const {

}


void Program::runIKSession() const {

}


void Program::printQuaternion() const {
    using std::sin;
    using std::cos;
    using std::acos;

    double r30 = 30 * M_PI / 180;
    double r40 = 40 * M_PI / 180;
    double r50 = 50 * M_PI / 180;

    double r11 = cos(r40) * cos(r50);
    double r12 = -cos(r40) * sin(r50);
    double r13 = sin(r40);
    double r21 = cos(r30) * sin(r50) + sin(r30) * sin(r40) * cos(r50);
    double r22 = cos(r30) * cos(r50) - sin(r30) * sin(r40) * sin(r50);
    double r23 = -sin(r30) * cos(r40);
    double r31 = sin(r30) * sin(r50) - cos(r30) * sin(r40) * cos(r50);
    double r32 = sin(r30) * cos(r50) + cos(r30) * sin(r40) * sin(r50);
    double r33 = cos(r30) * cos(r40);

    double theta = acos(0.5 * (r11 + r22 + r33 - 1));

    double w = cos(theta / 2);
    double i = 0.5 * sin(theta / 2) / sin(theta) * (r32 - r23);
    double j = 0.5 * sin(theta / 2) / sin(theta) * (r13 - r31);
    double k = 0.5 * sin(theta / 2) / sin(theta) * (r21 - r12);

    std::cout << "The quaternion describing the rotation of the detail in a format of [w i j k]:" << std::endl;
    printf("[%4.3f %4.3f %4.3f %4.3f]\n", w, i, j, k);
}



void Program::writeFiles() const{

}
