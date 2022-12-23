#ifndef INC_3DOF_THING_ROBOT_H
#define INC_3DOF_THING_ROBOT_H

#include <vector>
#include <cmath>
#include <algorithm>

typedef double angle_t;
typedef double coord_t;

struct Point {
    coord_t x, y, z;
};

struct JointLimits {
    angle_t min, max;
};

typedef Point position_t;
typedef std::vector<angle_t> joints_angles_t;


class Robot {

    const int N = 3; // number of links
    const double d1 = 0.1;
    const double a2 = 0.3;
    const double a3 = 0.2;

    const double rw;
    const double Rw1 = a2 + a3;
    std::vector<JointLimits> joints_limits; // a "const"ness is desired

    bool positionIsReachable(const position_t&) const;
    bool jointsAnglesOutOfLimits(const joints_angles_t&) const;

public:

    class IncorrectNumOfLinks{};
    class UnreachablePosition{};
    class JointAngleOutOfLimits{};

    Robot();
    position_t solveFK(const joints_angles_t&) const;
    std::vector<joints_angles_t> solveIK(const position_t&) const;
};


#endif //INC_3DOF_THING_ROBOT_H
