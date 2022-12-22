#include "Program.h"

void Program::run() {
    std::cout << "Input:" << std::endl;
    std::cout << "\ti - for an interactive session;" << std::endl;
    std::cout << "\tw - to output files with a solution of the task;" << std::endl;
    std::cout << "\tq - to quit." << std::endl;
    std::cout << std::endl;
    prompt();
}

void Program::prompt() {
    std::cout << ">";
}

std::string Program::readInput() {

}


void Program::writeFiles() {

}