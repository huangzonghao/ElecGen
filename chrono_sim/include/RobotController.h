#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include "chrono/core/ChMathematics.h"
#include "chrono/physics/ChBody.h"

#include "SimMotor.h"

class RobotController {
  public:
    enum ControllerType {MANIPULATOR, WHEEL, LEGGED} type;

    RobotController(std::vector<std::shared_ptr<SimMotor> > *motors,
                    std::vector<chrono::ChVector<> > *waypoints,
                    ControllerType type);
    virtual ~RobotController() = 0;

    virtual bool Update() = 0;

  protected:
    int waypoint_idx = 0;
    std::vector<std::shared_ptr<SimMotor> > *motors;
    std::vector<chrono::ChVector<> > *waypoints;

};

class ManipulatorController : public RobotController {
  public:
    double speed = 5;

    ManipulatorController(std::vector<std::shared_ptr<SimMotor> > *motors,
                          std::vector<chrono::ChVector<> > *waypoints):
        RobotController(motors, waypoints, MANIPULATOR){}

    ~ManipulatorController(){};

    bool Update() override;
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
    LeggedController(std::vector<std::shared_ptr<SimMotor> > *motors,
                     std::vector<chrono::ChVector<> > *waypoints,
                     const std::shared_ptr<chrono::ChBody>& ch_body):
        RobotController(motors, waypoints, LEGGED), robot_body(ch_body.get()){}

    ~LeggedController(){};

    chrono::ChBody *robot_body;

    bool Update() override;

  private:
    enum GAITS {FORWARD, BACKWARD, LEFT1, RIGHT1, LEFT2, RIGHT2} gait;
    void exe_gait();
    bool command_hold = false;
};


#endif /* end of include guard: ROBOT_CONTROLLER_H */
