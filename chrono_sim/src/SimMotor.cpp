#include "SimMotor.h"

#include "chrono/assets/ChBoxShape.h"
// #include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChLinkMotorRotationTorque.h"
#include "chrono/utils/ChCompositeInertia.h"

int SimMotorController::counter = 0;

SimMotorController::SimMotorController(const std::shared_ptr<chrono::ChLinkMotorRotationTorque>& target_motor){
    ch_motor = target_motor.get();
    vel_pid = chrono_types::make_shared<chrono::ChControllerPID>();
    vel_pid->P = vel_P;
    vel_pid->I = vel_I;
    vel_pid->D = vel_D;

    pos_pid = chrono_types::make_shared<chrono::ChControllerPID>();
    pos_pid->P = pos_P;
    pos_pid->I = pos_I;
    pos_pid->D = pos_D;
}

void SimMotorController::SetVel(double new_vel){
    if (mode == VELOCITY){
        if (target_vel == new_vel)
            return;
    }
    else {
        mode = VELOCITY;
        vel_pid->I = vel_I;
        vel_pid->D = vel_D;
    }
    target_vel = new_vel;
    vel_pid->Reset();
}

// accumulation behavior
void SimMotorController::SetPos(double new_pos){
    if (mode == VELOCITY){
        vel_pid->I = 0;
        vel_pid->D = 0;
        vel_pid->Reset();
    }
    target_pos = new_pos + ch_motor->GetMotorRot();
    mode = POSITION;
    pos_pid->Reset();
}

void SimMotorController::SetPhase(double new_phase){
    if (mode == PHASE){
        if (target_pos == new_phase)
            return;
    }
    else{
        if (mode == VELOCITY){
            vel_pid->I = 0;
            vel_pid->D = 0;
            vel_pid->Reset();
        }
        mode = PHASE;
    }
    target_pos = new_phase;
    pos_pid->Reset();
}

bool SimMotorController::check_status(){
    if (mode == POSITION && std::abs(target_pos - ch_motor->GetMotorRot()) > pos_thresh){
        return false;
    }
    else if (mode == PHASE && std::abs(target_pos - ch_motor->GetMotorRotPeriodic()) > pos_thresh){
        return false;
    }
    return true;
}

double SimMotorController::GetTorque(){
    switch(mode){
        case POSITION:
            target_vel = std::clamp(pos_pid->Get_Out(target_pos - ch_motor->GetMotorRot(), ch_motor->GetChTime()),
                                    -max_pos_control_vel,
                                    max_pos_control_vel);
            break;
        case PHASE:
            target_vel = std::clamp(pos_pid->Get_Out(target_pos - ch_motor->GetMotorRotPeriodic(), ch_motor->GetChTime()),
                                    -max_pos_control_vel,
                                    max_pos_control_vel);
            break;
    }

    return vel_pid->Get_Out(target_vel - ch_motor->GetMotorRot_dt(), ch_motor->GetChTime());
}

SimPayload::SimPayload()
    :visible(false), check_collision(false), mass(0), size{0,0,0}
{}

SimPayload::SimPayload(double mass,
                       double size_x, double size_y, double size_z,
                       double pos_x, double pos_y, double pos_z)
    :visible(false), check_collision(false), mass(mass),
     size{size_x, size_y, size_z}, pos(pos_x, pos_y, pos_z)
{}

SimPayload::SimPayload(const std::string& body_name, double mass,
                       double size_x, double size_y, double size_z,
                       double pos_x, double pos_y, double pos_z)
    :body_name(body_name), visible(false), check_collision(false), mass(mass),
     size{size_x, size_y, size_z}, pos(pos_x, pos_y, pos_z)
{}

void SimPayload::AddtoSystem(const std::shared_ptr<chrono::ChSystem>& sys){
    const std::shared_ptr<chrono::ChBody>& parent_body = sys->SearchBody(body_name.c_str());
    if (!parent_body){
        std::cerr << "Error: Cannot add payload, " << body_name << " doesn't exist in robot" << std::endl;
        exit(EXIT_FAILURE);
    }
    AddtoSystem(sys, parent_body);
}

void SimPayload::AddtoSystem(const std::shared_ptr<chrono::ChSystem>& sys,
                             const std::shared_ptr<chrono::ChBody>& parent_body){
    // enabling visualization and collision detection
    // may introduce extra complexity in simulation
    if (visible){
        auto viasset = chrono_types::make_shared<chrono::ChBoxShape>();
        viasset->GetBoxGeometry().SetLengths(chrono::ChVector<>(size[0], size[1], size[2]));
        viasset->Pos = pos;
        parent_body->AddAsset(viasset);
    }
    if (check_collision){
        chrono_types::make_shared<chrono::ChMaterialSurfaceNSC>();
        parent_body->GetCollisionModel()->AddBox(chrono_types::make_shared<chrono::ChMaterialSurfaceNSC>(),
                                                size[0] / 2, size[1] / 2, size[2] / 2, pos);
        parent_body->GetCollisionModel()->BuildModel();
    }

    // setup mass and inertia
    if (mass != 0){
        chrono::utils::CompositeInertia comp;
        comp.AddComponent(chrono::ChFrame<>(), parent_body->GetMass(), parent_body->GetInertia());
        comp.AddComponent(chrono::ChFrame<>(pos), mass, chrono::ChMatrix33<>(1));
        parent_body->SetMass(comp.GetMass());
        parent_body->SetInertia(comp.GetInertia());
        std::dynamic_pointer_cast<chrono::ChBodyAuxRef>(parent_body)->SetFrame_COG_to_REF(chrono::ChFrame<>(comp.GetCOM()));
    }
}

SimMotor::SimMotor(const std::string& link_name,
                   double mass, double size_x, double size_y, double size_z,
                   double pos_x, double pos_y, double pos_z)
    :SimPayload(mass, size_x, size_y, size_z, pos_x, pos_y, pos_z),
     link_name(link_name)
{}

SimMotor::SimMotor(const std::string& body_name, const std::string& link_name,
                   double mass, double size_x, double size_y, double size_z,
                   double pos_x, double pos_y, double pos_z)
    :SimPayload(body_name, mass, size_x, size_y, size_z, pos_x, pos_y, pos_z),
     link_name(link_name)
{}

void SimMotor::AddtoSystem(const chrono::ChUrdfDoc& urdf_doc){
    auto& sys = urdf_doc.GetSystem();
    chlinkbody = &(urdf_doc.GetLinkBodies(link_name));

    if (!body_name.empty()){
        SimPayload::AddtoSystem(sys);
    }
    else {
        SimPayload::AddtoSystem(sys, chlinkbody->body2);
    }

    ch_motor = chrono_types::make_shared<chrono::ChLinkMotorRotationTorque>();

    // flip z axis of motor frame, so that a positive speed would make robot go forward
    chrono::ChFrame<> motor_frame(chlinkbody->link->GetLinkAbsoluteCoords());
    // motor_frame.ConcatenatePostTransformation(chrono::ChFrame<>(chrono::ChVector<>(), chrono::Q_FLIP_AROUND_X));
    ch_motor->Initialize(chlinkbody->body1, chlinkbody->body2, motor_frame);
    ch_func = chrono_types::make_shared<chrono::ChFunction_Setpoint>();
    ch_motor->SetMotorFunction(ch_func);
    sys->AddLink(ch_motor);

    ch_link = chlinkbody->link.get();
    motor_controller = std::make_shared<SimMotorController>(ch_motor);
}
void SimMotor::AddtoSystem(const std::shared_ptr<chrono::ChSystem>& sys){
    std::cerr << "Error: Need to pass on the ChUrdfDoc when adding SimMotor to system" << std::endl;
}

void SimMotor::SetVel(double new_vel){
    motor_controller->SetVel(new_vel);
}

void SimMotor::SetPos(double new_pos){
    motor_controller->SetPos(new_pos);
}

// in Chrono, phase is between [0, 2pi], and [0, -2pi] when rotating negatively
void SimMotor::SetPhase(double new_phase){
    if (std::abs(new_phase) > chrono::CH_C_2PI){
        std::cout << "Error: SimMotor::SetPhase cannot set phase to " << new_phase << std::endl;
        return;
    }

    double current_phase = ch_motor->GetMotorRotPeriodic();
    if (current_phase < 0){
        current_phase += chrono::CH_C_2PI;
    }
    new_phase = new_phase - current_phase;
    if (new_phase < -chrono::CH_C_PI){
        new_phase += chrono::CH_C_2PI;
    }
    else if (new_phase > chrono::CH_C_PI){
        new_phase -= chrono::CH_C_2PI;
    }

    motor_controller->SetPos(new_phase);
}

void SimMotor::UpdateTorque(){
    ch_func->SetSetpoint(motor_controller->GetTorque(), ch_motor->GetChTime());
    max_torque = std::max(max_torque, motor_controller->GetTorque());
}
