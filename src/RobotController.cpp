#include "RobotController.h"

RobotController::~RobotController(){}

bool ManipulatorController::Update(){
    return true;
}

WheelController::WheelController(SimulationManager *sm,
                                 const std::shared_ptr<chrono::ChBody>& ch_body):
    RobotController(sm, WHEEL),
    robot_body(ch_body.get())
{
    speed[0] = init_speed;
    speed[1] = init_speed;
    pid.P = 0.5;
    pid.I = 0.0;
}

bool WheelController::Update(){
    auto& waypoints = sim_magr->waypoints;

    if ((robot_body->GetPos() - waypoints[waypoint_idx]).Length() < 2){
        if (waypoint_idx < waypoints.size() - 1){
            ++waypoint_idx;
            pid.Reset();
        }
        else {
            return true;
        }
    }

    speed_diff = pid.Get_Out((dynamic_cast<chrono::ChFrame<>*>(robot_body)->TransformParentToLocal(chrono::ChVector<>())).x(), robot_body->GetChTime());
    speed[0] = chrono::ChClamp(speed[0] + speed_diff, -max_speed, max_speed);
    speed[1] = chrono::ChClamp(speed[1] - speed_diff, -max_speed, max_speed);

    auto& motors = sim_magr->motors;

    sim_magr->motors[0]->Set(speed[0]);
    sim_magr->motors[1]->Set(speed[1]);

    return false;
}

LeggedController::LeggedController(SimulationManager *sm,
                                   const std::shared_ptr<chrono::ChBody>& ch_body):
    RobotController(sm, LEGGED),
    robot_body(ch_body.get())
{}

bool LeggedController::Update(){
    auto& waypoints = sim_magr->waypoints;

    if ((robot_body->GetPos() - waypoints[waypoint_idx]).Length() < 2){
        if (waypoint_idx < waypoints.size() - 1){
            ++waypoint_idx;
            pid.Reset();
        }
        else {
            return true;
        }
    }

    speed_diff = pid.Get_Out((dynamic_cast<chrono::ChFrame<>*>(robot_body)->TransformParentToLocal(chrono::ChVector<>())).x(), robot_body->GetChTime());
    speed[0] = chrono::ChClamp(speed[0] + speed_diff, -max_speed, max_speed);
    speed[1] = chrono::ChClamp(speed[1] - speed_diff, -max_speed, max_speed);
    speed[2] = chrono::ChClamp(speed[2] + speed_diff, -max_speed, max_speed);
    speed[3] = chrono::ChClamp(speed[3] - speed_diff, -max_speed, max_speed);

    auto& motors = sim_magr->motors;

    sim_magr->motors[0]->Set(speed[0]);
    sim_magr->motors[1]->Set(speed[1]);
    sim_magr->motors[2]->Set(speed[2]);
    sim_magr->motors[3]->Set(speed[3]);

    return false;
}
