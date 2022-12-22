#ifndef INC_3DOF_THING_ROBOT_H
#define INC_3DOF_THING_ROBOT_H

#include <vector>
#include <cmath>


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

    // TODO add std::cos etc definitions for all methods if possible

    const int N = 3; // number of links
    const double d1 = 0.1;
    const double a2 = 0.3;
    const double a3 = 0.2;

    const double rw;
    const double Rw1 = a2 + a3;
    std::vector<JointLimits> joints_limits; // a "const"ness is desired

    bool positionIsReachable(position_t) const;

public:

    class IncorrectNumOfLinks{};
    class UnreachablePosition{};

    Robot();
    position_t fk(joints_angles_t) const;
    std::vector<joints_angles_t> ik(position_t) const;
};


#endif //INC_3DOF_THING_ROBOT_H
