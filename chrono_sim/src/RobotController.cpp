#include "RobotController.h"

RobotController::RobotController(std::vector<std::shared_ptr<SimMotor> > *motors,
                                 std::vector<chrono::ChVector<> > *waypoints,
                                 ControllerType type):
    motors(motors), waypoints(waypoints), type(type){}

RobotController::~RobotController(){}

bool ManipulatorController::Update(){
    return true;
}

bool WheelController::Update(){
    return true;
}

bool LeggedController::Update(){
    // make sure all the motors have moved to the right place
    if (command_hold){
        command_hold = false;
        for (auto motor : *motors){
            motor->UpdateTorque();
            command_hold |= !motor->CheckStatus();
        }

        if (command_hold)
            return false;
    }

    if ((robot_body->GetPos() - waypoints->at(waypoint_idx)).Length() < 2){
        if (waypoint_idx < waypoints->size() - 1){
            ++waypoint_idx;
        }
        else {
            return true;
        }
    }

    chrono::ChVector<> goal_local =
        robot_body->TransformPointParentToLocal(waypoints->at(waypoint_idx));

    double yx_ratio = goal_local.y() / (goal_local.x() + 1e-8); // in case x is zero

    // the head of robot is pointing +x
    if (yx_ratio > 0.33){
        gait = RIGHT2;
    }
    else if (yx_ratio < -0.33){
        gait = LEFT2;
    }
    else{
        // after turning, sync legs to the same phase
        if (gait != FORWARD && gait != BACKWARD){
            for (auto motor : *motors){
                motor->SetPhase(1.5);
            }
            // wait until all the legs are in the right phase
            command_hold = true;
            gait = FORWARD;
            return false;
        }
        if (goal_local.x() > 0){
            gait = FORWARD;
        }
        else {
            gait = BACKWARD;
        }
    }

    exe_gait();

    for (auto motor : *motors){
        motor->UpdateTorque();
    }

    return false;
}

void LeggedController::exe_gait(){
    // motor 0 1 right, 2 3 left
    // positive vel moves the robot forward
    switch(gait){
        case FORWARD:
            motors->at(0)->SetVel(6);
            motors->at(1)->SetVel(6);
            motors->at(2)->SetVel(6);
            motors->at(3)->SetVel(6);
            break;
        case BACKWARD:
            motors->at(0)->SetVel(-6);
            motors->at(1)->SetVel(-6);
            motors->at(2)->SetVel(-6);
            motors->at(3)->SetVel(-6);
            break;
        case LEFT1:
            motors->at(0)->SetVel(6);
            motors->at(1)->SetVel(6);
            motors->at(2)->SetVel(-6);
            motors->at(3)->SetVel(-6);
            break;
        case RIGHT1:
            motors->at(0)->SetVel(-6);
            motors->at(1)->SetVel(-6);
            motors->at(2)->SetVel(6);
            motors->at(3)->SetVel(6);
            break;
        case LEFT2:
            motors->at(0)->SetVel(6);
            motors->at(1)->SetVel(6);
            motors->at(2)->SetVel(-6);
            motors->at(3)->SetVel(-6);
            break;
        case RIGHT2:
            motors->at(0)->SetVel(-6);
            motors->at(1)->SetVel(-6);
            motors->at(2)->SetVel(6);
            motors->at(3)->SetVel(6);
            break;
    }
}
