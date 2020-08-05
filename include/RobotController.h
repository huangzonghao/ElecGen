#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include "chrono/core/ChMathematics.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChController.h"

#include "SimMotor.h"
#include "SimulationManager.h"

class SimulationManager; // forward declaration

class RobotController {
  public:
    enum ControllerType {MANIPULATOR, WHEEL, LEGGED} type;
    RobotController(SimulationManager *sm):sim_magr(sm){}
    RobotController(SimulationManager *sm, ControllerType type):sim_magr(sm), type(type){}
    virtual ~RobotController() = 0;

    SimulationManager *sim_magr;

    virtual bool Update() = 0;
};

class ManipulatorController : public RobotController {
  public:
    double speed = 5;

    ManipulatorController(SimulationManager *sm): RobotController(sm, MANIPULATOR){}
    ~ManipulatorController(){};

    bool Update() override;
};

class WheelController : public RobotController {
  public:

    WheelController(SimulationManager *sm,
                    const std::shared_ptr<chrono::ChBody>& ch_body);
    ~WheelController(){};
    chrono::ChBody *robot_body;

    chrono::ChControllerPID pid;

    double speed[2];
    double init_speed = 5;
    double max_speed = 15;
    double speed_diff = 0;

    int waypoint_idx = 0;

    bool Update() override;

};

class LeggedController : public RobotController {
  public:
    LeggedController(SimulationManager *sm,
                     const std::shared_ptr<chrono::ChBody>& ch_body);
    ~LeggedController(){};

    chrono::ChBody *robot_body;

    chrono::ChControllerPID pid;

    double speed[4];
    double init_speed = 5;
    double max_speed = 15;
    double speed_diff = 0;

    int waypoint_idx = 0;

    bool Update() override;
};


#endif /* end of include guard: ROBOT_CONTROLLER_H */
