#ifndef ROBOTCONTROLLER_H_WU5YVSBW
#define ROBOTCONTROLLER_H_WU5YVSBW

#include <chrono/core/ChMathematics.h>
#include <chrono/physics/ChBody.h>

#include "quadrupedal_kinematics.h"
#include "SimMotor.h"

class RobotController {
  public:
    enum ControllerType {MANIPULATOR, WHEEL, LEGGED} type;

    RobotController(std::vector<std::shared_ptr<SimMotor> > *motors, ControllerType type);

    RobotController(std::vector<std::shared_ptr<SimMotor> > *motors,
                    std::vector<chrono::ChVector<> > *waypoints,
                    ControllerType type);
    virtual ~RobotController() = default;

    virtual bool Update() = 0;

  protected:
    bool gait_lock = false;
    int waypoint_idx = 0;
    std::vector<std::shared_ptr<SimMotor> > *motors_;
    std::vector<chrono::ChVector<> > *waypoints_;

};

class ManipulatorController : public RobotController {
  public:
    ManipulatorController(std::vector<std::shared_ptr<SimMotor> > *motors);

    ~ManipulatorController(){};

    void SetJointPos(const std::shared_ptr<std::vector<double> >& start_joint_pos,
                     const std::shared_ptr<std::vector<double> >& goal_joint_pos);

    // TODO:need to modify simmotor controller
    void SetJointMaxVel(double new_max);

    bool Update() override;
  private:
    // TODO: should be the size of waypoint vector
    int num_waypoints = 2;
    double joint_max_vel_ = 0.2;
    std::shared_ptr<std::vector<double> > start_joint_pos_;
    std::shared_ptr<std::vector<double> > goal_joint_pos_;
};

class WheelController : public RobotController {
  public:

    WheelController(std::vector<std::shared_ptr<SimMotor> > *motors,
                    std::vector<chrono::ChVector<> > *waypoints,
                    const std::shared_ptr<chrono::ChBody>& ch_body):
        RobotController(motors, waypoints, WHEEL), robot_body(ch_body.get()){}

    ~WheelController(){};

    chrono::ChBody *robot_body;

    bool Update() override;

  private:
    enum GAITS {FORWARD, BACKWARD, LEFT1, RIGHT1, LEFT2, RIGHT2} gait;
    void exe_gait();

};

class LeggedController : public RobotController {
  public:
    enum Model {M1, M2, M3} model;
    LeggedController(std::vector<std::shared_ptr<SimMotor> > *motors,
                     std::vector<chrono::ChVector<> > *waypoints,
                     const std::shared_ptr<chrono::ChBody>& ch_body):
        RobotController(motors, waypoints, LEGGED), robot_body(ch_body.get()){}

    ~LeggedController(){};

    chrono::ChBody *robot_body;

    bool Update() override;
    void AddTrajectory(int traj_ID=1);
    void SetKinematics(double l1, double l2, double l3, double offset_x, double offset_y);
    Eigen::Vector3d GetFK(int location, double q1, double q2, double q3);
    Eigen::Vector3d GetFK_W(int location, double q1, double q2, double q3);
    // xyz in CoM frame
    Eigen::Vector3d GetIK(int location, double x, double y, double z);
    Eigen::Vector3d GetIK_W(int location, double x, double y, double z);

  private:
    enum GAITS {FORWARD, BACKWARD, LEFT1, RIGHT1, LEFT2, RIGHT2, STAND} gait;
    void exe_gait();
    void exe_gait2();
    void exe_gait3();
    // remaining steps to executate in the gait
    int gait_steps = 0;
    int update_counter = 0;
    int trajectory_;
    std::shared_ptr<QuadrupedalKinematics> kinematics_;
};


#endif /* end of include guard: ROBOTCONTROLLER_H_WU5YVSBW */
