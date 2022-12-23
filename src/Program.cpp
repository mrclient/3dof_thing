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

}



void Program::writeFiles() const{

}
