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
        gait = LEFT2;
    }
    else if (yx_ratio < -0.33){
        gait = RIGHT2;
    }
    else{
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

bool LeggedController::Update(){
    // make sure all the motors have moved to the right place
    if (gait_lock){
        gait_lock = false;
        for (auto motor : *motors){
            motor->UpdateTorque();
            // check whether every motor arrived at target pos
            gait_lock |= !motor->CheckStatus();
        }

        if (gait_lock){
            // TODO:why do i need this.
            if (update_counter++ < 30)
                return false;
            update_counter = 0;
        }
    }

    if (gait_lock == 0){
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
        else if (goal_local.x() > 0){
            gait = FORWARD;
        }
        else {
            gait = BACKWARD;
        }
    }

    switch(model){
        case M1:
            exe_gait();
            break;
        case M2:
            if (gait != BACKWARD){
                // 2 seg leg can only move the robot forward or backward.
                gait = FORWARD;
            }
            exe_gait2();
            break;
        case M3:
            exe_gait3();
            break;
        default:
            std::cerr << "Error: legged robot model not specified" << std::endl;
    }

    for (auto motor : *motors){
        motor->UpdateTorque();
    }

    return false;
}

void WheelController::exe_gait(){
    // motor 0 1 left, 2 3 right
    // negative vel moves the robot forward
    switch(gait){
        case FORWARD:
            motors->at(0)->SetVel(-6);
            motors->at(1)->SetVel(-6);
            motors->at(2)->SetVel(-6);
            motors->at(3)->SetVel(-6);
            break;
        case BACKWARD:
            motors->at(0)->SetVel(6);
            motors->at(1)->SetVel(6);
            motors->at(2)->SetVel(6);
            motors->at(3)->SetVel(6);
            break;
        case RIGHT1:
            motors->at(0)->SetVel(-6);
            motors->at(1)->SetVel(-6);
            motors->at(2)->SetVel(6);
            motors->at(3)->SetVel(6);
            break;
        case LEFT1:
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
        case LEFT2:
            motors->at(0)->SetVel(6);
            motors->at(1)->SetVel(6);
            motors->at(2)->SetVel(-6);
            motors->at(3)->SetVel(-6);
            break;
    }
}

void LeggedController::exe_gait(){
    // motor 0 1 left, 2 3 right
    // positive vel moves the robot forward
    // // after turning, sync legs to the same phase
    // if (gait != FORWARD && gait != BACKWARD){
        // for (auto motor : *motors){
            // motor->SetPhase(1.5);
        // }
        // // wait until all the legs are in the right phase
        // gait_lock = true;
        // gait = FORWARD;
        // return false;
    // }
    switch(gait){
        case STAND:
            motors->at(0)->SetVel(1.5);
            motors->at(1)->SetVel(1.5);
            motors->at(2)->SetVel(1.5);
            motors->at(3)->SetVel(1.5);
            break;
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

void LeggedController::exe_gait2(){
    // motor pair sequence: fl fr bl br
    // positive vel moves the robot forward
    switch(gait){
        case STAND:
            motors->at(0)->SetPhase(-0.3);
            motors->at(1)->SetPhase(0.7);
            motors->at(2)->SetPhase(-0.3);
            motors->at(3)->SetPhase(0.7);
            motors->at(4)->SetPhase(-0.3);
            motors->at(5)->SetPhase(0.7);
            motors->at(6)->SetPhase(-0.3);
            motors->at(7)->SetPhase(0.7);
            break;
        case FORWARD:
            switch(gait_steps){
                case 0:
                    motors->at(0)->SetPhase(-0.3);
                    motors->at(1)->SetPhase(0.7);
                    motors->at(2)->SetPhase(-0.3);
                    motors->at(3)->SetPhase(0.7);
                    motors->at(4)->SetPhase(-0.3);
                    motors->at(5)->SetPhase(0.7);
                    motors->at(6)->SetPhase(-0.3);
                    motors->at(7)->SetPhase(0.7);
                    gait_steps = 3;
                    gait_lock = true;
                    break;
                case 1:
                    motors->at(0)->SetPhase(-0.3);
                    motors->at(1)->SetPhase(0.7);
                    motors->at(2)->SetPhase(-0.3);
                    motors->at(3)->SetPhase(0.7);
                    motors->at(4)->SetPhase(-0.3);
                    motors->at(5)->SetPhase(0.7);
                    motors->at(6)->SetPhase(-0.3);
                    motors->at(7)->SetPhase(0.7);
                    gait_steps = 0;
                    gait_lock = true;
                    break;
                case 2:
                    motors->at(0)->SetPhase(-0.3);
                    motors->at(1)->SetPhase(0.7);
                    motors->at(2)->SetPhase(-0.7);
                    motors->at(3)->SetPhase(1);
                    motors->at(4)->SetPhase(-0.7);
                    motors->at(5)->SetPhase(1);
                    motors->at(6)->SetPhase(-0.3);
                    motors->at(7)->SetPhase(0.7);
                    gait_steps = 0;
                    gait_lock = true;
                    break;
                case 3:
                    motors->at(0)->SetPhase(-0.7);
                    motors->at(1)->SetPhase(1);
                    motors->at(2)->SetPhase(-0.3);
                    motors->at(3)->SetPhase(0.7);
                    motors->at(4)->SetPhase(-0.3);
                    motors->at(5)->SetPhase(0.7);
                    motors->at(6)->SetPhase(-0.7);
                    motors->at(7)->SetPhase(1);
                    gait_steps = 2;
                    gait_lock = true;
                    break;
            }
            break;
        case BACKWARD:
            switch(gait_steps){
                case 0:
                    motors->at(0)->SetPhase(0.3);
                    motors->at(1)->SetPhase(-0.7);
                    motors->at(2)->SetPhase(0.3);
                    motors->at(3)->SetPhase(-0.7);
                    motors->at(4)->SetPhase(0.3);
                    motors->at(5)->SetPhase(-0.7);
                    motors->at(6)->SetPhase(0.3);
                    motors->at(7)->SetPhase(-0.7);
                    gait_steps = 3;
                    gait_lock = true;
                    break;
                case 1:
                    motors->at(0)->SetPhase(0.3);
                    motors->at(1)->SetPhase(-0.7);
                    motors->at(2)->SetPhase(0.3);
                    motors->at(3)->SetPhase(-0.7);
                    motors->at(4)->SetPhase(0.3);
                    motors->at(5)->SetPhase(-0.7);
                    motors->at(6)->SetPhase(0.3);
                    motors->at(7)->SetPhase(-0.7);
                    gait_steps = 0;
                    gait_lock = true;
                    break;
                case 2:
                    motors->at(0)->SetPhase(0.3);
                    motors->at(1)->SetPhase(-0.7);
                    motors->at(2)->SetPhase(0.7);
                    motors->at(3)->SetPhase(-1);
                    motors->at(4)->SetPhase(0.7);
                    motors->at(5)->SetPhase(-1);
                    motors->at(6)->SetPhase(0.3);
                    motors->at(7)->SetPhase(-0.7);
                    gait_steps = 0;
                    gait_lock = true;
                    break;
                case 3:
                    motors->at(0)->SetPhase(0.7);
                    motors->at(1)->SetPhase(-1);
                    motors->at(2)->SetPhase(0.3);
                    motors->at(3)->SetPhase(-0.7);
                    motors->at(4)->SetPhase(0.3);
                    motors->at(5)->SetPhase(-0.7);
                    motors->at(6)->SetPhase(0.7);
                    motors->at(7)->SetPhase(-1);
                    gait_steps = 2;
                    gait_lock = true;
                    break;
            }
            break;
    }
}

void LeggedController::exe_gait3(){
    // fl bl fr br
    switch(gait){
        case STAND:
            motors->at(0)->SetPhase(0);
            motors->at(1)->SetPhase(-0.3);
            motors->at(2)->SetPhase(0.7);
            motors->at(3)->SetPhase(0);
            motors->at(4)->SetPhase(-0.3);
            motors->at(5)->SetPhase(0.7);
            motors->at(6)->SetPhase(0);
            motors->at(7)->SetPhase(-0.3);
            motors->at(8)->SetPhase(0.7);
            motors->at(9)->SetPhase(0);
            motors->at(10)->SetPhase(-0.3);
            motors->at(11)->SetPhase(0.7);
            break;
        case FORWARD:
            switch(gait_steps){
                case 0:
                    motors->at(0)->SetPhase(0);
                    motors->at(1)->SetPhase(-0.3);
                    motors->at(2)->SetPhase(0.7);
                    motors->at(3)->SetPhase(0);
                    motors->at(4)->SetPhase(-0.3);
                    motors->at(5)->SetPhase(0.7);
                    motors->at(6)->SetPhase(0);
                    motors->at(7)->SetPhase(-0.3);
                    motors->at(8)->SetPhase(0.7);
                    motors->at(9)->SetPhase(0);
                    motors->at(10)->SetPhase(-0.3);
                    motors->at(11)->SetPhase(0.7);
                    gait_steps = 3;
                    gait_lock = true;
                    break;
                case 1:
                    motors->at(0)->SetPhase(0);
                    motors->at(1)->SetPhase(-0.3);
                    motors->at(2)->SetPhase(0.7);
                    motors->at(3)->SetPhase(0);
                    motors->at(4)->SetPhase(-0.3);
                    motors->at(5)->SetPhase(0.7);
                    motors->at(6)->SetPhase(0);
                    motors->at(7)->SetPhase(-0.3);
                    motors->at(8)->SetPhase(0.7);
                    motors->at(9)->SetPhase(0);
                    motors->at(10)->SetPhase(-0.3);
                    motors->at(11)->SetPhase(0.7);
                    gait_steps = 0;
                    gait_lock = true;
                    break;
                case 2:
                    motors->at(0)->SetPhase(0);
                    motors->at(1)->SetPhase(-0.3);
                    motors->at(2)->SetPhase(0.7);
                    motors->at(3)->SetPhase(0);
                    motors->at(4)->SetPhase(-0.7);
                    motors->at(5)->SetPhase(1);
                    motors->at(6)->SetPhase(0);
                    motors->at(7)->SetPhase(-0.7);
                    motors->at(8)->SetPhase(1);
                    motors->at(9)->SetPhase(0);
                    motors->at(10)->SetPhase(-0.3);
                    motors->at(11)->SetPhase(0.7);
                    gait_steps = 0;
                    gait_lock = true;
                    break;
                case 3:
                    motors->at(0)->SetPhase(0);
                    motors->at(1)->SetPhase(-0.7);
                    motors->at(2)->SetPhase(1);
                    motors->at(3)->SetPhase(0);
                    motors->at(4)->SetPhase(-0.3);
                    motors->at(5)->SetPhase(0.7);
                    motors->at(6)->SetPhase(0);
                    motors->at(7)->SetPhase(-0.3);
                    motors->at(8)->SetPhase(0.7);
                    motors->at(9)->SetPhase(0);
                    motors->at(10)->SetPhase(-0.7);
                    motors->at(11)->SetPhase(1);
                    gait_steps = 2;
                    gait_lock = true;
                    break;
            }
            break;
        case BACKWARD:
            switch(gait_steps){
                case 0:
                    motors->at(0)->SetPhase(0);
                    motors->at(1)->SetPhase(0.3);
                    motors->at(2)->SetPhase(-0.7);
                    motors->at(3)->SetPhase(0);
                    motors->at(4)->SetPhase(0.3);
                    motors->at(5)->SetPhase(-0.7);
                    motors->at(6)->SetPhase(0);
                    motors->at(7)->SetPhase(0.3);
                    motors->at(8)->SetPhase(-0.7);
                    motors->at(9)->SetPhase(0);
                    motors->at(10)->SetPhase(0.3);
                    motors->at(11)->SetPhase(-0.7);
                    gait_steps = 3;
                    gait_lock = true;
                    break;
                case 1:
                    motors->at(0)->SetPhase(0);
                    motors->at(1)->SetPhase(0.3);
                    motors->at(2)->SetPhase(-0.7);
                    motors->at(3)->SetPhase(0);
                    motors->at(4)->SetPhase(0.3);
                    motors->at(5)->SetPhase(-0.7);
                    motors->at(6)->SetPhase(0);
                    motors->at(7)->SetPhase(0.3);
                    motors->at(8)->SetPhase(-0.7);
                    motors->at(9)->SetPhase(0);
                    motors->at(10)->SetPhase(0.3);
                    motors->at(11)->SetPhase(-0.7);
                    gait_steps = 0;
                    gait_lock = true;
                    break;
                case 2:
                    motors->at(0)->SetPhase(0);
                    motors->at(1)->SetPhase(0.3);
                    motors->at(2)->SetPhase(-0.7);
                    motors->at(3)->SetPhase(0);
                    motors->at(4)->SetPhase(0.7);
                    motors->at(5)->SetPhase(-1);
                    motors->at(6)->SetPhase(0);
                    motors->at(7)->SetPhase(0.7);
                    motors->at(8)->SetPhase(-1);
                    motors->at(9)->SetPhase(0);
                    motors->at(10)->SetPhase(0.3);
                    motors->at(11)->SetPhase(-0.7);
                    gait_steps = 0;
                    gait_lock = true;
                    break;
                case 3:
                    motors->at(0)->SetPhase(0);
                    motors->at(1)->SetPhase(0.7);
                    motors->at(2)->SetPhase(-1);
                    motors->at(3)->SetPhase(0);
                    motors->at(4)->SetPhase(0.3);
                    motors->at(5)->SetPhase(-0.7);
                    motors->at(6)->SetPhase(0);
                    motors->at(7)->SetPhase(0.3);
                    motors->at(8)->SetPhase(-0.7);
                    motors->at(9)->SetPhase(0);
                    motors->at(10)->SetPhase(0.7);
                    motors->at(11)->SetPhase(-1);
                    gait_steps = 2;
                    gait_lock = true;
                    break;
            }
            break;
        case LEFT1:
            switch(gait_steps){
                case 0:
                    motors->at(0)->SetPhase(0);
                    motors->at(1)->SetPhase(-0.3);
                    motors->at(2)->SetPhase(0.7);
                    motors->at(3)->SetPhase(0);
                    motors->at(4)->SetPhase(-0.3);
                    motors->at(5)->SetPhase(0.7);
                    motors->at(6)->SetPhase(0);
                    motors->at(7)->SetPhase(-0.3);
                    motors->at(8)->SetPhase(0.7);
                    motors->at(9)->SetPhase(0);
                    motors->at(10)->SetPhase(-0.3);
                    motors->at(11)->SetPhase(0.7);
                    gait_steps = 4;
                    gait_lock = true;
                    break;
                case 1:
                    motors->at(0)->SetPhase(0.8);
                    motors->at(1)->SetPhase(-0.3);
                    motors->at(2)->SetPhase(0.7);
                    motors->at(3)->SetPhase(0.8);
                    motors->at(4)->SetPhase(-0.3);
                    motors->at(5)->SetPhase(0.7);
                    motors->at(6)->SetPhase(0.8);
                    motors->at(7)->SetPhase(-0.3);
                    motors->at(8)->SetPhase(0.7);
                    motors->at(9)->SetPhase(0.8);
                    motors->at(10)->SetPhase(-0.7);
                    motors->at(11)->SetPhase(0.7);
                    gait_steps = 0;
                    gait_lock = true;
                    break;
                case 2:
                    motors->at(0)->SetPhase(0.8);
                    motors->at(1)->SetPhase(-0.3);
                    motors->at(2)->SetPhase(0.7);
                    motors->at(3)->SetPhase(0.8);
                    motors->at(4)->SetPhase(-0.3);
                    motors->at(5)->SetPhase(0.7);
                    motors->at(6)->SetPhase(0.8);
                    motors->at(7)->SetPhase(-0.7);
                    motors->at(8)->SetPhase(0.7);
                    motors->at(9)->SetPhase(0);
                    motors->at(10)->SetPhase(-0.3);
                    motors->at(11)->SetPhase(0.7);
                    gait_steps = 1;
                    gait_lock = true;
                    break;
                case 3:
                    motors->at(0)->SetPhase(0.8);
                    motors->at(1)->SetPhase(-0.3);
                    motors->at(2)->SetPhase(0.7);
                    motors->at(3)->SetPhase(0.8);
                    motors->at(4)->SetPhase(-0.7);
                    motors->at(5)->SetPhase(0.7);
                    motors->at(6)->SetPhase(0);
                    motors->at(7)->SetPhase(-0.3);
                    motors->at(8)->SetPhase(0.7);
                    motors->at(9)->SetPhase(0);
                    motors->at(10)->SetPhase(-0.3);
                    motors->at(11)->SetPhase(0.7);
                    gait_steps = 2;
                    gait_lock = true;
                    break;
                case 4:
                    motors->at(0)->SetPhase(0.8);
                    motors->at(1)->SetPhase(-0.7);
                    motors->at(2)->SetPhase(0.7);
                    motors->at(3)->SetPhase(0);
                    motors->at(4)->SetPhase(-0.3);
                    motors->at(5)->SetPhase(0.7);
                    motors->at(6)->SetPhase(0);
                    motors->at(7)->SetPhase(-0.3);
                    motors->at(8)->SetPhase(0.7);
                    motors->at(9)->SetPhase(0);
                    motors->at(10)->SetPhase(-0.3);
                    motors->at(11)->SetPhase(0.7);
                    gait_steps = 3;
                    gait_lock = true;
                    break;
            }
            break;
        case RIGHT1:
            switch(gait_steps){
                case 0:
                    motors->at(0)->SetPhase(0);
                    motors->at(1)->SetPhase(-0.3);
                    motors->at(2)->SetPhase(0.7);
                    motors->at(3)->SetPhase(0);
                    motors->at(4)->SetPhase(-0.3);
                    motors->at(5)->SetPhase(0.7);
                    motors->at(6)->SetPhase(0);
                    motors->at(7)->SetPhase(-0.3);
                    motors->at(8)->SetPhase(0.7);
                    motors->at(9)->SetPhase(0);
                    motors->at(10)->SetPhase(-0.3);
                    motors->at(11)->SetPhase(0.7);
                    gait_steps = 4;
                    gait_lock = true;
                    break;
                case 1:
                    motors->at(6)->SetPhase(-0.8);
                    motors->at(7)->SetPhase(-0.3);
                    motors->at(8)->SetPhase(0.7);
                    motors->at(9)->SetPhase(-0.8);
                    motors->at(10)->SetPhase(-0.3);
                    motors->at(11)->SetPhase(0.7);
                    motors->at(0)->SetPhase(-0.8);
                    motors->at(1)->SetPhase(-0.3);
                    motors->at(2)->SetPhase(0.7);
                    motors->at(3)->SetPhase(-0.8);
                    motors->at(4)->SetPhase(-0.7);
                    motors->at(5)->SetPhase(0.7);
                    gait_steps = 0;
                    gait_lock = true;
                    break;
                case 2:
                    motors->at(6)->SetPhase(-0.8);
                    motors->at(7)->SetPhase(-0.3);
                    motors->at(8)->SetPhase(0.7);
                    motors->at(9)->SetPhase(-0.8);
                    motors->at(10)->SetPhase(-0.3);
                    motors->at(11)->SetPhase(0.7);
                    motors->at(0)->SetPhase(-0.8);
                    motors->at(1)->SetPhase(-0.7);
                    motors->at(2)->SetPhase(0.7);
                    motors->at(3)->SetPhase(0);
                    motors->at(4)->SetPhase(-0.3);
                    motors->at(5)->SetPhase(0.7);
                    gait_steps = 1;
                    gait_lock = true;
                    break;
                case 3:
                    motors->at(6)->SetPhase(-0.8);
                    motors->at(7)->SetPhase(-0.3);
                    motors->at(8)->SetPhase(0.7);
                    motors->at(9)->SetPhase(-0.8);
                    motors->at(10)->SetPhase(-0.7);
                    motors->at(11)->SetPhase(0.7);
                    motors->at(0)->SetPhase(0);
                    motors->at(1)->SetPhase(-0.3);
                    motors->at(2)->SetPhase(0.7);
                    motors->at(3)->SetPhase(0);
                    motors->at(4)->SetPhase(-0.3);
                    motors->at(5)->SetPhase(0.7);
                    gait_steps = 2;
                    gait_lock = true;
                    break;
                case 4:
                    motors->at(6)->SetPhase(-0.8);
                    motors->at(7)->SetPhase(-0.7);
                    motors->at(8)->SetPhase(0.7);
                    motors->at(9)->SetPhase(0);
                    motors->at(10)->SetPhase(-0.3);
                    motors->at(11)->SetPhase(0.7);
                    motors->at(0)->SetPhase(0);
                    motors->at(1)->SetPhase(-0.3);
                    motors->at(2)->SetPhase(0.7);
                    motors->at(3)->SetPhase(0);
                    motors->at(4)->SetPhase(-0.3);
                    motors->at(5)->SetPhase(0.7);
                    gait_steps = 3;
                    gait_lock = true;
                    break;
            }
            break;
        case LEFT2:
            switch(gait_steps){
                case 0:
                    motors->at(0)->SetPhase(0);
                    motors->at(1)->SetPhase(-0.3);
                    motors->at(2)->SetPhase(0.7);
                    motors->at(3)->SetPhase(0);
                    motors->at(4)->SetPhase(-0.3);
                    motors->at(5)->SetPhase(0.7);
                    motors->at(6)->SetPhase(0);
                    motors->at(7)->SetPhase(-0.3);
                    motors->at(8)->SetPhase(0.7);
                    motors->at(9)->SetPhase(0);
                    motors->at(10)->SetPhase(-0.3);
                    motors->at(11)->SetPhase(0.7);
                    gait_steps = 4;
                    gait_lock = true;
                    break;
                case 1:
                    motors->at(0)->SetPhase(0.8);
                    motors->at(1)->SetPhase(-0.3);
                    motors->at(2)->SetPhase(0.7);
                    motors->at(3)->SetPhase(0.8);
                    motors->at(4)->SetPhase(-0.3);
                    motors->at(5)->SetPhase(0.7);
                    motors->at(6)->SetPhase(0.8);
                    motors->at(7)->SetPhase(-0.3);
                    motors->at(8)->SetPhase(0.7);
                    motors->at(9)->SetPhase(0.8);
                    motors->at(10)->SetPhase(-0.7);
                    motors->at(11)->SetPhase(0.7);
                    gait_steps = 0;
                    gait_lock = true;
                    break;
                case 2:
                    motors->at(0)->SetPhase(0.8);
                    motors->at(1)->SetPhase(-0.3);
                    motors->at(2)->SetPhase(0.7);
                    motors->at(3)->SetPhase(0.8);
                    motors->at(4)->SetPhase(-0.3);
                    motors->at(5)->SetPhase(0.7);
                    motors->at(6)->SetPhase(0.8);
                    motors->at(7)->SetPhase(-0.7);
                    motors->at(8)->SetPhase(0.7);
                    motors->at(9)->SetPhase(0);
                    motors->at(10)->SetPhase(-0.3);
                    motors->at(11)->SetPhase(0.7);
                    gait_steps = 1;
                    gait_lock = true;
                    break;
                case 3:
                    motors->at(0)->SetPhase(0.8);
                    motors->at(1)->SetPhase(-0.3);
                    motors->at(2)->SetPhase(0.7);
                    motors->at(3)->SetPhase(0.8);
                    motors->at(4)->SetPhase(-0.7);
                    motors->at(5)->SetPhase(0.7);
                    motors->at(6)->SetPhase(0);
                    motors->at(7)->SetPhase(-0.3);
                    motors->at(8)->SetPhase(0.7);
                    motors->at(9)->SetPhase(0);
                    motors->at(10)->SetPhase(-0.3);
                    motors->at(11)->SetPhase(0.7);
                    gait_steps = 2;
                    gait_lock = true;
                    break;
                case 4:
                    motors->at(0)->SetPhase(0.8);
                    motors->at(1)->SetPhase(-0.7);
                    motors->at(2)->SetPhase(0.7);
                    motors->at(3)->SetPhase(0);
                    motors->at(4)->SetPhase(-0.3);
                    motors->at(5)->SetPhase(0.7);
                    motors->at(6)->SetPhase(0);
                    motors->at(7)->SetPhase(-0.3);
                    motors->at(8)->SetPhase(0.7);
                    motors->at(9)->SetPhase(0);
                    motors->at(10)->SetPhase(-0.3);
                    motors->at(11)->SetPhase(0.7);
                    gait_steps = 3;
                    gait_lock = true;
                    break;
            }
            break;
        case RIGHT2:
            switch(gait_steps){
                case 0:
                    motors->at(0)->SetPhase(0);
                    motors->at(1)->SetPhase(-0.3);
                    motors->at(2)->SetPhase(0.7);
                    motors->at(3)->SetPhase(0);
                    motors->at(4)->SetPhase(-0.3);
                    motors->at(5)->SetPhase(0.7);
                    motors->at(6)->SetPhase(0);
                    motors->at(7)->SetPhase(-0.3);
                    motors->at(8)->SetPhase(0.7);
                    motors->at(9)->SetPhase(0);
                    motors->at(10)->SetPhase(-0.3);
                    motors->at(11)->SetPhase(0.7);
                    gait_steps = 4;
                    gait_lock = true;
                    break;
                case 1:
                    motors->at(6)->SetPhase(-0.8);
                    motors->at(7)->SetPhase(-0.3);
                    motors->at(8)->SetPhase(0.7);
                    motors->at(9)->SetPhase(-0.8);
                    motors->at(10)->SetPhase(-0.3);
                    motors->at(11)->SetPhase(0.7);
                    motors->at(0)->SetPhase(-0.8);
                    motors->at(1)->SetPhase(-0.3);
                    motors->at(2)->SetPhase(0.7);
                    motors->at(3)->SetPhase(-0.8);
                    motors->at(4)->SetPhase(-0.7);
                    motors->at(5)->SetPhase(0.7);
                    gait_steps = 0;
                    gait_lock = true;
                    break;
                case 2:
                    motors->at(6)->SetPhase(-0.8);
                    motors->at(7)->SetPhase(-0.3);
                    motors->at(8)->SetPhase(0.7);
                    motors->at(9)->SetPhase(-0.8);
                    motors->at(10)->SetPhase(-0.3);
                    motors->at(11)->SetPhase(0.7);
                    motors->at(0)->SetPhase(-0.8);
                    motors->at(1)->SetPhase(-0.7);
                    motors->at(2)->SetPhase(0.7);
                    motors->at(3)->SetPhase(0);
                    motors->at(4)->SetPhase(-0.3);
                    motors->at(5)->SetPhase(0.7);
                    gait_steps = 1;
                    gait_lock = true;
                    break;
                case 3:
                    motors->at(6)->SetPhase(-0.8);
                    motors->at(7)->SetPhase(-0.3);
                    motors->at(8)->SetPhase(0.7);
                    motors->at(9)->SetPhase(-0.8);
                    motors->at(10)->SetPhase(-0.7);
                    motors->at(11)->SetPhase(0.7);
                    motors->at(0)->SetPhase(0);
                    motors->at(1)->SetPhase(-0.3);
                    motors->at(2)->SetPhase(0.7);
                    motors->at(3)->SetPhase(0);
                    motors->at(4)->SetPhase(-0.3);
                    motors->at(5)->SetPhase(0.7);
                    gait_steps = 2;
                    gait_lock = true;
                    break;
                case 4:
                    motors->at(6)->SetPhase(-0.8);
                    motors->at(7)->SetPhase(-0.7);
                    motors->at(8)->SetPhase(0.7);
                    motors->at(9)->SetPhase(0);
                    motors->at(10)->SetPhase(-0.3);
                    motors->at(11)->SetPhase(0.7);
                    motors->at(0)->SetPhase(0);
                    motors->at(1)->SetPhase(-0.3);
                    motors->at(2)->SetPhase(0.7);
                    motors->at(3)->SetPhase(0);
                    motors->at(4)->SetPhase(-0.3);
                    motors->at(5)->SetPhase(0.7);
                    gait_steps = 3;
                    gait_lock = true;
                    break;
            }
            break;
    }
}
