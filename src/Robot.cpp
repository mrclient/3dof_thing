#include "Robot.h"

using std::sin;
using std::cos;
using std::sqrt;
using std::abs;
using std::atan2;


angle_t normalizeAngle(angle_t angle){
    while(angle > M_PI)
        angle -= 2 * M_PI;
    while(angle < -M_PI)
        angle += 2 * M_PI;
    return angle; // but now it's good!
}


double findAB(const position_t& c){
    return sqrt(c.x * c.x + c.y * c.y);
}


double findPhi(const position_t& c, double d1){
    return atan2(c.z - d1, findAB(c));
}


double findAC(const position_t& c, double d1){
    double AB = findAB(c);
    double BC = abs(c.z - d1);
    return sqrt(AB * AB + BC * BC);
}


Robot::Robot() :
    rw(a2 * a2 + a3 * a3 - 2 * a2 * a3 * cos(130 * M_PI / 180)),
    joints_limits(N)
{
    joints_limits[0].min = -M_PI;
    joints_limits[0].max = M_PI;
    joints_limits[1].min = -M_PI / 2;
    joints_limits[1].max = M_PI / 2;
    joints_limits[2].min = -50 * M_PI / 180;
    joints_limits[2].max = 50 * M_PI / 180;
}


bool Robot::jointsAnglesOutOfLimits(const joints_angles_t& joints_angles) const{
    // Be carefull: the function doesn't find any other problem with its input argument
    for(int i = 0; i < N; i++){
        if(joints_angles[i] > joints_limits[i].max || joints_angles[i] < joints_limits[i].min)
            return true;
    }
    return false;
}


position_t Robot::solveFK(const joints_angles_t& joints_angles) const{

    if(joints_angles.size() != N)
        throw IncorrectNumOfLinks();

    if(jointsAnglesOutOfLimits(joints_angles))
        throw JointAngleOutOfLimits();

    position_t tool_position;
    double p = a3 * sin(joints_angles[1] + joints_angles[2]) + a2 * sin(joints_angles[1]);
    tool_position.x = -p * cos(joints_angles[0]);
    tool_position.y = -p * sin(joints_angles[0]);
    tool_position.z = a3 * cos(joints_angles[1] + joints_angles[2]) + a2 * cos(joints_angles[1]) + d1;

    return tool_position;
}


bool Robot::positionIsReachable(const position_t& c) const{

    if(c.z < d1 - a3 * sin(50 * M_PI / 180))
        return false;

    double AC = findAC(c, d1);
    if(AC < rw)
        return false;

    double phi = findPhi(c, d1);
    if(phi >= 0.0) {
        if (AC > Rw1)
            return false;
        else
            return true;
    }

    phi = -phi; // getting phi_star (not necessary actually but for a clear story)
    double Rw2 = a2 * cos(phi) + sqrt(a3 * a3 - a2 * a2 * sin(phi) * sin(phi));

    return (AC <= Rw2);
}


std::vector<joints_angles_t> Robot::solveIK(const position_t& tool_position) const{

    const int num_of_sets = 4;

    if(!positionIsReachable(tool_position))
        throw UnreachablePosition();

    std::vector<joints_angles_t> joints_angles_sets(num_of_sets);
    for(auto &x: joints_angles_sets) {
        x.resize(N);
        x[0] = 0.0;
    }

    if (tool_position.x != 0.0 || tool_position.y != 0.0){ // comparison of doubles with zeros may be vulnerable
        double psi = atan2(tool_position.y, tool_position.x);
        joints_angles_sets[0][0] = joints_angles_sets[1][0] = normalizeAngle(psi + M_PI);
        joints_angles_sets[2][0] = joints_angles_sets[3][0] = psi;
    }
    // else isn't required for the robot to mark singular solution for theta_1. It can be checked by comparing
    // its different values were found

    double AC = findAC(tool_position, d1);
    double cos_th_3 = (AC * AC - a2 * a2 - a3 * a3) / (2 * a2 * a3);
    joints_angles_sets[0][2] = joints_angles_sets[3][2] = atan2(sqrt(1 - cos_th_3 * cos_th_3), cos_th_3);
    joints_angles_sets[1][2] = joints_angles_sets[2][2] = -joints_angles_sets[0][2];

    double beta = atan2(a3 * sin(abs(joints_angles_sets[0][2])), a2 + a3 * cos(abs(joints_angles_sets[0][2])));
    double phi = findPhi(tool_position, d1);
    // the normalizing below may be unnecessary; but it doesn't break anything
    joints_angles_sets[2][1] = -(joints_angles_sets[0][1] = normalizeAngle(M_PI / 2 - phi - beta));
    joints_angles_sets[3][1] = -(joints_angles_sets[1][1] = normalizeAngle(M_PI / 2 - phi + beta));

    std::remove_if(joints_angles_sets.begin(), joints_angles_sets.end(),
                   [this](const joints_angles_t& joints_angles){return jointsAnglesOutOfLimits(joints_angles);});

    return joints_angles_sets;
}
