#include "Robot.h"

// TODO add angle normalizer function

Robot::Robot() :
    rw(a2 * a2 + a3 * a3 - 2 * a2 * a3 * std::cos(130 * M_PI / 180)),
    joints_limits(N)
{
    joints_limits[0].min = -M_PI;
    joints_limits[0].max = M_PI;
    joints_limits[1].min = -M_PI / 2;
    joints_limits[1].max = M_PI / 2;
    joints_limits[2].min = -50 * M_PI / 180;
    joints_limits[2].max = 50 * M_PI / 180;
}

position_t Robot::fk(joints_angles_t joints_angles) const{

    if(joints_angles.size() != N)
        throw IncorrectNumOfLinks();

    using std::sin;
    using std::cos;

    position_t tool_position;
    double p = a3 * sin(joints_angles[1] + joints_angles[2]) + a2 * sin(joints_angles[1]);
    tool_position.x = -p * cos(joints_angles[0]);
    tool_position.y = -p * sin(joints_angles[0]);
    tool_position.z = a3 * cos(joints_angles[1] + joints_angles[2]) + a2 * cos(joints_angles[1]) + d1;

    return tool_position;
}


bool Robot::positionIsReachable(position_t c) const{
    using std::sin;
    using std::cos;
    using std::sqrt;

    if(c.z < d1 - a3 * sin(50 * M_PI / 180))
        return false;

    double AB = sqrt(c.x * c.x + c.y * c.y);
    double BC = std::abs(c.z - d1);
    double AC = sqrt(AB * AB + BC * BC);

    if(AC < rw)
        return false;

    double phi = std::atan2(c.z - d1, AB);

    if(phi >= 0.0) {
        if (AC > Rw1)
            return false;
        else
            return true;
    }

    phi = -phi; // getting phi_star (not necessary actually but for a clear story)
    double Rw2 = a2 * cos(phi) + sqrt(a3 * a3 - a2 * a2 * sin(phi)* sin(phi));

    return (AC <= Rw2);
}


std::vector<joints_angles_t> Robot::ik(position_t tool_position) const{
    using std::sin;
    using std::cos;
    using std::sqrt;

    if(!positionIsReachable(tool_position))
        throw UnreachablePosition();

    joints_angles_t joints_angles(4);

    if (tool_position.x == 0.0 && tool_position.y == 0.0){ // comparison of doubles with zeros may be vulnerable
        // TODO mark that theta_1 can be any
    }
    else{
        angle_t psi = std::atan2(tool_position.y, tool_position.x);
        joints_angles[0][0] = joints_angles[1][0] = psi + M_PI;
        joints_angles[2][0] = joints_angles[3][0] = psi;
    }



}