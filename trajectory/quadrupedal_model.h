#ifndef QUADRUPEDAL_MODEL_H_
#define QUADRUPEDAL_MODEL_H_

#include <towr/models/kinematic_model.h>
#include <towr/models/single_rigid_body_dynamics.h>
#include <towr/models/endeffector_mappings.h>

namespace towr {

class QuadrupedalKinematicModel : public KinematicModel {
public:
  // note the optimizer's left and right is flipped from our chrono setup
  // offset_x : offset of endeffector from base com in x direction
  // offset_y : offset of endeffector from base com in y direction
  // offset_z : offset of endeffector from base com in z direction
  // dev_x    : max deviation of endeffector from its norminal position in x direction
  // dev_y    : max deviation of endeffector from its norminal position in y direction
  // dev_z    : max deviation of endeffector from its norminal position in z direction
  // the six variables above together creates a bounding box for the reachable space
  // for the endeffector
  QuadrupedalKinematicModel (double offset_x, double offset_y, double offset_z,
                             double dev_x, double dev_y, double dev_z) :
      KinematicModel(4)
  {

    nominal_stance_.at(LF) <<  offset_x,   offset_y, offset_z;
    nominal_stance_.at(RF) <<  offset_x,  -offset_y, offset_z;
    nominal_stance_.at(LH) << -offset_x,   offset_y, offset_z;
    nominal_stance_.at(RH) << -offset_x,  -offset_y, offset_z;

    max_dev_from_nominal_ << dev_x, dev_y, dev_z;
  }
  QuadrupedalKinematicModel () : KinematicModel(4)
  {
    const double x_nominal_b = 0.34;
    const double y_nominal_b = 0.19;
    const double z_nominal_b = -0.42;

    nominal_stance_.at(LF) <<  x_nominal_b,   y_nominal_b, z_nominal_b;
    nominal_stance_.at(RF) <<  x_nominal_b,  -y_nominal_b, z_nominal_b;
    nominal_stance_.at(LH) << -x_nominal_b,   y_nominal_b, z_nominal_b;
    nominal_stance_.at(RH) << -x_nominal_b,  -y_nominal_b, z_nominal_b;

    max_dev_from_nominal_ << 0.15, 0.1, 0.10;
  }

};

class QuadrupedalDynamicModel : public SingleRigidBodyDynamics {
public:
  QuadrupedalDynamicModel (double mass,
                           double Ixx, double Iyy, double Izz,
                           double Ixy, double Ixz, double Iyz,
                           int ee_count):
        SingleRigidBodyDynamics (mass, Ixx, Iyy, Izz, Ixy, Ixz, Iyz, ee_count) {}

  QuadrupedalDynamicModel() : SingleRigidBodyDynamics(29.5, 0.946438, 1.94478, 2.01835, 0.000938112, -0.00595386, -0.00146328, 4) {}
};

} /* namespace towr */

#endif /* end of QUADRUPEDAL_MODEL_H_ */
