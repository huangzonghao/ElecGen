#ifndef QUADRUPEDAL_KINEMATICS_H_
#define QUADRUPEDAL_KINEMATICS_H_

#include <Eigen/Core>

// TODO: Whole thing is hard coded, ugly, use rbdl for this part
// TODO: read FK IK information from URDF directly
class QuadrupedalKinematics {
  public:
    enum QuadrupedIDs { LF, RF, LH, RH }; ///< Consistent with def in towr/models/endeffector_mappings.h

    QuadrupedalKinematics(double l1, double l2, double l3, double offset_x, double offset_y);

    Eigen::Vector3d forward_kinematics(int location, double q1, double q2, double q3);
    Eigen::Vector3d inverse_kinematics(int location, double x, double y, double z);
    Eigen::Vector3d FK(int location, double q1, double q2, double q3){ return forward_kinematics(location, q1, q2, q3); }
    Eigen::Vector3d IK(int location, double x, double y, double z){ return inverse_kinematics(location, x, y, z); }

  private:
    double l1_; ///< link between joint 0 and joint 1
    double l2_; ///< link between joint 1 and joint 2
    double l3_; ///< link between joint 2 and endeffector
    double ox_; ///< endeffector offset to CoM
    double oy_; ///< endeffector offset to CoM

    Eigen::Matrix4d A12;
    Eigen::Matrix4d A23;
    Eigen::Vector4d P3e;
    Eigen::Vector4d P0e;
};


#endif /* end of QUADRUPEDAL_KINEMATICS_H_ */
