#include "quadrupedal_kinematics.h"
#include <math.h>

#ifndef PI
#define PI 3.14159265
#endif

QuadrupedalKinematics::
QuadrupedalKinematics(double l1, double l2, double l3, double offset_x, double offset_y):
    l1_(l1), l2_(l2), l3_(l3), ox_(offset_x), oy_(offset_y),
    A12(Eigen::Matrix4d::Zero()), A23(Eigen::Matrix4d::Zero()) // necessary to init eigen matrice
{
    P3e(3) = 1;
}

/**
 * FK: Frames are not rotating with corresponding joints
 *
 *                 A12 = | Cq1 0 -Sq1 0 |
 *                       | Sq1 0  Cq1 0 |
 *                       | 0  -1   0  0 |
 *                       | 0   0   0  1 |
 *
 * A23 =  | -Sq2 -Cq2 0 -l2Sq2 |  A23  =  | -Sq2 -Cq2 0 -l2Sq2 |
 * (left) |  Cq2 -Sq2 0  l2Cq2 |  (right) |  Cq2 -Sq2 0  l2Cq2 |
 *        |   0    0  1    l1  |          |   0    0  1   -l1  |
 *        |   0    0  0     1  |          |   0    0  0     1  |
 *
 *                      P3e = | l3Cq3 |
 *                            | l3Sq3 |
 *                            |    0  |
 *                            |    1  |
 */
Eigen::Vector3d QuadrupedalKinematics::
forward_kinematics(int location, double q1, double q2, double q3){
    A12 << cos(q1),  0, -sin(q1), 0,
           sin(q1),  0,  cos(q1), 0,
                 0, -1,        0, 0,
                 0,  0,        0, 1;

    A23 << -sin(q2), -cos(q2), 0, -l2_*sin(q2),
            cos(q2), -sin(q2), 0,  l2_*cos(q2),
                  0,        0, 1,          l1_, // l1 need to be corrected later
                  0,        0, 0,            1;

    if (location == RF || location == RH){
        A23(2,3) = -l1_;
    }

    P3e << l3_ * cos(q3),
           l3_ * sin(q3),
                       0,
                       1;

    P0e = A12 * A23 * P3e;

    switch (location){
        case LF:
            P0e(0) +=  ox_;
            P0e(1) +=  oy_;
            break;
        case LH:
            P0e(0) += -ox_;
            P0e(1) +=  oy_;
            break;
        case RF:
            P0e(0) +=  ox_;
            P0e(1) += -oy_;
            break;
        case RH:
            P0e(0) += -ox_;
            P0e(1) += -oy_;
            break;
    }

    return P0e.topRows(3);
}

/**
 * Assumptions:
 * - endeffecotrs always move in the lower half sphere
 * - lower joints pointing forward as much as possible
 *
 * Kinematic Decoupling:
 * - l1 is always penpendicular to the line difined by ee and joint 1 (second joint)
 * - distance between ee and joint 1 only depends on joint 2
 * - with assumptions above can solve for joint 2
 * - since the z position for joint 0 and joint 1 is fixed, with z of ee, joint 1 is determined
 * - once joint 1 is determined, we know the projection of l2_&l3_ on xy plane. we also know the
 *     projection of ee to origin. solve for joint on xy plane
 */

Eigen::Vector3d QuadrupedalKinematics::
inverse_kinematics(int location, double x, double y, double z){
    // shift the baselink of manipulator to origin
    switch (location){
        case LF:
            x -=  ox_;
            y -=  oy_;
            break;
        case LH:
            x -= -ox_;
            y -=  oy_;
            break;
        case RF:
            x -=  ox_;
            y -= -oy_;
            break;
        case RH:
            x -= -ox_;
            y -= -oy_;
            break;
    }

    Eigen::Vector3d q;
    // out of reachable space
    // return a configuration pointing to that direction while enforcing z
    if (x*x + y*y + z*z > l1_*l1_ + (l2_+l3_)*(l2_+l3_)){
        q(0) = -atan2(y,x); // atan2 return in [-pi, pi]
        q(1) = -acos((l3_*l3_ + z*z) / (-2 * l3_ * z)); // acos returns value in [0, pi]
        q(2) = PI - acos((l2_*l2_ + l3_*l3_ - z*z) / (2 * l2_ * l3_));
    }
    else{
        q(0) = -(atan2(x, y) + atan2(sqrt(x*x + y*y - l1_*l1_), l1_));
        q(1) = -acos(-z / sqrt(x*x + y*y + z*z - l1_*l1_));
        q(2) = PI - acos((l1_*l1_ + l2_*l2_ + l3_*l3_ - x*x - y*y - z*z) / (2 * l2_ * l3_));
    }
    return q;
}
