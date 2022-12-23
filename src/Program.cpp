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

    printTopLevelHelp();

    std::string c;
    while(true) {
        prompt();
        std::getline(std::cin, c);

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


void Program::prompt(const std::string& prefix, const std::string& sign) const{
    std::cout << prefix << sign;
}


bool Program::readDataInput(const std::string& str, std::vector<double>& data) const {
    std::istringstream istr(str);

    for(auto &d: data) {
        char ch = 0;
        istr >> ch;
        if (std::isdigit(ch)) {
            istr.putback(ch);
            istr >> d;
        }
        else if (ch == '+' || ch == '-'){
            char ch2 = 0;
            istr >> ch2;
            if (std::isdigit(ch2)) {
                istr.putback(ch2);
                istr.putback(ch);
                istr >> d;
            } else
                return false;
        } else
            return false;
    }
    char ch = 0;
    istr >> ch;
    if (ch != 0)
        return false;
    return true;
}


void Program::printFKHelp() const{
    std::cout << "Input:" << std::endl;
    std::cout << "\ttheta1 theta2 theta3 - values in degrees to perform calculations;" << std::endl;
    std::cout << "\ts - to switch off " << angle_step << " degrees step (super mode);" << std::endl;
    std::cout << "\th - to repeat this information;" << std::endl;
    std::cout << "\tq - to quit from the super mode (if activated) or to the main menu." << std::endl;
}


void Program::runFKSession() const {

    printFKHelp();

    bool su = false;
    std::string c;
    joints_angles_t joints_angles(3);
    while(true) {
        prompt("fk", (su ? "#" : ">"));
        std::getline(std::cin, c);

        if(c == "")
        { }
        else if(c == "s")
            su = true;
        else if(c == "h")
            printFKHelp();
        else if(c == "q") {
            if(su)
                su = false;
            else
                return;
        }
        else if(readDataInput(c, joints_angles)){

            bool flag = false;
            for(auto &ja: joints_angles) {
                if (!su && (std::abs(ja / angle_step - std::round(ja / angle_step)) > 1e-5)) { // 1e-5-"tolerance"
                    std::cout << "Incorrect input. Each value have to be equal to " << angle_step
                              << "*d, where d is an integer" << std::endl;
                    flag = true;
                    break;
                }
                ja *= M_PI / 180;
            }
            if(flag) continue;

            try {
                position_t point_C = robot.solveFK(joints_angles);
                std::cout << "Coordinates of point C in meters: " << point_C.x << " " << point_C.y << " " << point_C.z << " " << std::endl;
            }
            catch(Robot::JointAngleOutOfLimits){
                std::cout << "Incorrect input. Angles values are out of limits." << std::endl;
                continue;
            }
        }
        else
            std::cout << "Incorrect input. Please repeat." << std::endl;
    }
}


void Program::printIKHelp() const{
    std::cout << "Input:" << std::endl;
    std::cout << "\tx y z - values in meters to perform calculations;" << std::endl;
    std::cout << "\ts - to switch off " << crd_step << " m step and limiting sphere (super mode);" << std::endl;
    std::cout << "\th - to repeat this information;" << std::endl;
    std::cout << "\tq - to quit from the super mode (if activated) or to the main menu." << std::endl;
}


void Program::runIKSession() const {
    printIKHelp();

    bool su = false;
    std::string c;
    std::vector<double> point(3);
    while(true) {
        prompt("ik", (su ? "#" : ">"));
        std::getline(std::cin, c);

        if(c == "")
        { }
        else if(c == "s")
            su = true;
        else if(c == "h")
            printIKHelp();
        else if(c == "q") {
            if(su)
                su = false;
            else
                return;
        }
        else if(readDataInput(c, point)){

            bool flag = false;
            for(auto &p: point) {
                if (!su && (std::abs(p / crd_step - std::round(p / crd_step)) > 1e-5)) { // 1e-5 - "tolerance"
                    std::cout << "Incorrect input. Each value have to be equal to " << crd_step
                              << "*d, where d is an integer" << std::endl;
                    flag = true;
                    break;
                }
            }

            if(!su && (point[0] * point[0] + point[1] * point[1] + point[2] * point[2] > radius * radius)) {
                std::cout << "The point is out of the " << radius << " m sphere." << std::endl;
                flag = true;
            }
            if(flag) continue;

            try {
                std::vector<joints_angles_t> joints_angles_sets = robot.solveIK(Point{point[0], point[1], point[2]});
                std::cout << "Solutions in degrees are ";
                if(!robot.checkForSingularity(joints_angles_sets)) {
                    for (auto &jas: joints_angles_sets) {
                        std::cout << "{" << jas[0] * 180 / M_PI << " " << jas[1] * 180 / M_PI << " "
                                  << jas[2] * 180 / M_PI << "} ";
                    }
                }
                else {
                    std::cout << "{" << "any" << " " << 0.0 << " " << 0.0 << "}"
                            << " - position is singular or close to be so!";
                }
                std::cout << std::endl;
            }
            catch(Robot::UnreachablePosition) {
                std::cout <<  "The point is unreachable!" << std::endl;
            }
        }
        else
            std::cout << "Incorrect input. Please repeat." << std::endl;
    }
}


void Program::printQuaternion(std::ostream& out_stream) const {
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

    out_stream << "The quaternion describing the rotation of the detail in a format of [w i j k]:" << std::endl;
    auto old_flags = out_stream.flags();
    auto old_precision = out_stream.precision();
    out_stream << "[" << std::showpoint << std::setprecision(3) << std::setw(5) << w << " "
               << std::setw(5) << i << " " << std::setw(5) << j << " "
               << std::setw(5) << k << "]" << std::endl;
    out_stream << std::setprecision(old_precision);
    out_stream.flags(old_flags);
}


void Program::writeFiles() const{

    std::ofstream fk_file("task1.txt");
    auto old_flags = fk_file.flags();
    auto old_precision = fk_file.precision();
    fk_file << std::showpoint << std::setprecision(3) << std::fixed;

    fk_file << std::setw(5) << "th1 " << std::setw(5) << "th2 " << std::setw(5)
            << "th3 " << std::setw(7) << "Xc " << std::setw(7) << "Yc "
            << std::setw(7) << "Zc " << std::endl;

    for(int theta_1 = round(robot.joints_limits[0].min * 180 / M_PI);
        theta_1 <= round(robot.joints_limits[0].max * 180 / M_PI); theta_1 += angle_step) {

        for (int theta_2 = round(robot.joints_limits[1].min * 180 / M_PI);
             theta_2 <= round(robot.joints_limits[1].max * 180 / M_PI); theta_2 += angle_step) {

            for (int theta_3 = round(robot.joints_limits[2].min * 180 / M_PI);
                 theta_3 <= round(robot.joints_limits[2].max * 180 / M_PI); theta_3 += angle_step) {

                joints_angles_t joints_angles = {theta_1 * M_PI / 180, theta_2 * M_PI / 180, theta_3 * M_PI / 180};
                position_t point_C = robot.solveFK(joints_angles);
                fk_file << std::setw(4) << theta_1 << " " << std::setw(4) << theta_2 << " "
                        << std::setw(4) << theta_3 << " " << std::setw(6) << point_C.x << " "
                        << std::setw(6) << point_C.y << " " << std::setw(6) << point_C.z << " " << std::endl;
            }
        }
    }
    fk_file << std::setprecision(old_precision);
    fk_file.flags(old_flags);
    fk_file.close();

    std::ofstream ik_file("task2.txt");
    old_flags = ik_file.flags();
    old_precision = ik_file.precision();
    ik_file << std::showpoint << std::setprecision(3) << std::fixed;

    for(double x = -radius; x <= radius; x += crd_step){
        for(double y = -radius; y <= radius; y += crd_step){
            for(double z = -0.1; z <= radius; z += crd_step){ // -0.1 - is just an unreachable z plane

                if(x * x + y * y + z * z > radius * radius) {
//                    ik_file << "Point [" << std::setprecision(1) << std::setw(4) << x << " " << std::setw(4)
//                            << y << " " << std::setw(4) << z << "] m is out of the 0.6m sphere!" << std::endl;
                    continue;
                }

                try {
                    std::vector<joints_angles_t> joints_angles_sets = robot.solveIK(Point{x, y, z});

                    ik_file << "Point [" << std::setprecision(1) << std::setw(4) << x << " " << std::setw(4)
                            << y << " " << std::setw(4) << z << "] m is achievable with {t1, th2, th3} degrees: ";
                    if(!robot.checkForSingularity(joints_angles_sets)) {
                        for (auto &jas: joints_angles_sets) {
                            ik_file << std::setprecision(3) << "{" << std::setw(4) << jas[0] * 180 / M_PI << " "
                                    << std::setw(4) << jas[1] * 180 / M_PI << " "
                                    << std::setw(4) << jas[2] * 180 / M_PI << "} ";
                        }
                    }
                    else {
                        ik_file << std::setprecision(3) << "{" << std::setw(4) << "any" << " "
                                << std::setw(4) << 0.0 << " " << std::setw(4) << 0.0 << "} "
                                << " - position is singular or close to be so!";

                    }
                    ik_file << std::endl;
                }
                catch(Robot::UnreachablePosition) {
                    ik_file << std::setprecision(1) << "Point [" << std::setw(4) << x << " " << std::setw(4)
                            << y << " " << std::setw(4) << z << "] m is unreachable!" << std::endl;
                }
            }
        }
    }

    ik_file << std::setprecision(old_precision);
    ik_file.flags(old_flags);
    ik_file.close();

    std::ofstream qt_file("task3.txt");
    printQuaternion(qt_file);
}
