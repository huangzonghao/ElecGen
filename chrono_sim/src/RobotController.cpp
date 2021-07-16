#include "RobotController.h"
#include "LeggedGaits.h"

RobotController::
RobotController(std::vector<std::shared_ptr<SimMotor>> *motors, ControllerType type):
    motors_(motors), type(type){}

RobotController::RobotController(std::vector<std::shared_ptr<SimMotor>> *motors,
                                 std::vector<chrono::ChVector<>> *waypoints,
                                 ControllerType type):
    motors_(motors), waypoints_(waypoints), type(type){}

ManipulatorController::ManipulatorController(std::vector<std::shared_ptr<SimMotor>> *motors):
    RobotController(motors, MANIPULATOR){
        for (const auto& motor : *motors_){
            motor->SetMaxVel(joint_max_vel_);
        }
}

void ManipulatorController::
SetJointPos(const std::shared_ptr<std::vector<double>>& start_joint_pos,
            const std::shared_ptr<std::vector<double>>& goal_joint_pos){

    if (motors_->size() != start_joint_pos->size() || motors_->size() != goal_joint_pos->size()){
        std::cout << "Error: cannot initialize manipulator controller, number of motors not equal to number of joint vaules" << std::endl;
        exit(EXIT_FAILURE);
    }
    start_joint_pos_ = start_joint_pos;
    goal_joint_pos_ = goal_joint_pos;
}

void ManipulatorController::SetJointMaxVel(double new_max){
    if (joint_max_vel_ != new_max){
        joint_max_vel_ = new_max;
        for (const auto& motor : *motors_){
            motor->SetMaxVel(joint_max_vel_);
        }
    }
}

bool ManipulatorController::Update(){
    // make sure all the motors have moved to the right place
    if (gait_lock){
        gait_lock = false;
        for (auto motor : *motors_){
            motor->UpdateTorque();
            // check whether every motor arrived at target pos
            gait_lock |= !motor->CheckStatus();
        }

        if (gait_lock){
            return false;
        }
    }

    // now motors arrive at the current joint position. time to target at the next waypoint
    // check if this is the last one first
    ++waypoint_idx;
    if (waypoint_idx == num_waypoints){
        return true;
    }
    // otherwise we have next waypoint to go
    // TODO: hard code to goal_joint_pos for now
    for (int i = 0; i < motors_->size(); ++i){
        motors_->at(i)->SetPhase(goal_joint_pos_->at(i));
    }
    gait_lock = true;

    return false;
}

bool WheelController::Update(){
    if ((robot_body->GetPos() - waypoints_->at(waypoint_idx)).Length() < 2){
        if (waypoint_idx < waypoints_->size() - 1){
            ++waypoint_idx;
        }
        else {
            return true;
        }
    }

    chrono::ChVector<> goal_local =
        robot_body->TransformPointParentToLocal(waypoints_->at(waypoint_idx));

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

    for (auto motor : *motors_){
        motor->UpdateTorque();
    }

    return false;
}

bool LeggedController::Update(){
    // make sure all the motors have moved to the right place
    if (gait_lock){
        gait_lock = false;
        for (auto motor : *motors_){
            motor->UpdateTorque();
            // check whether every motor arrived at target pos
            gait_lock |= !motor->CheckStatus();
        }

        if (gait_lock){
            // TODO:why do i need this.
            if (update_counter++ < 3000)
                return false;
            update_counter = 0;
        }
    }

    if (gait_steps == 0){
        if ((robot_body->GetPos() - waypoints_->at(waypoint_idx)).Length() < 2){
            if (waypoint_idx < waypoints_->size() - 1){
                ++waypoint_idx;
            }
            else {
                return true;
            }
        }

        chrono::ChVector<> goal_local =
            robot_body->TransformPointParentToLocal(waypoints_->at(waypoint_idx));

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
            gait = FORWARD;
            exe_gait3();
            break;
        default:
            std::cerr << "Error: legged robot model not specified" << std::endl;
    }

    for (auto motor : *motors_){
        motor->UpdateTorque();
    }

    return false;
}

void LeggedController::SetKinematics(double l1, double l2, double l3,
                                     double offset_x, double offset_y){
    kinematics_ = std::make_shared<QuadrupedalKinematics>(l1, l2, l3, offset_x, offset_y);
    return;
}

Eigen::Vector3d LeggedController::
GetFK(int location, double q1, double q2, double q3){
    return kinematics_->FK(location, q1, q2, q3);
}

Eigen::Vector3d LeggedController::
GetIK(int location, double x, double y, double z){
    return kinematics_->IK(location, x, y, z);
}

Eigen::Vector3d LeggedController::
GetFK_W(int location, double q1, double q2, double q3){

    Eigen::Vector3d P_CoM_e = kinematics_->FK(location, q1, q2, q3);
    chrono::ChVector P_CoM = robot_body->GetPos();
    P_CoM_e(0) += P_CoM.x();
    P_CoM_e(1) += P_CoM.y();
    P_CoM_e(2) += P_CoM.z();
    return P_CoM_e;
}

Eigen::Vector3d LeggedController::
GetIK_W(int location, double x, double y, double z){
    chrono::ChVector P_CoM = robot_body->GetPos();
    x -= P_CoM.x();
    y -= P_CoM.y();
    z -= P_CoM.z();
    return kinematics_->IK(location, x, y, z);
}

void LeggedController::AddTrajectory(int traj_ID){
    trajectory_ = traj_ID;
}
