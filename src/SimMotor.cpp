#include "SimMotor.h"

#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChLinkMotorRotationTorque.h"

SimPayload::SimPayload()
    :check_collision(false), mass(1), size{1,1,1}
{}

SimPayload::SimPayload(double mass, double size_x, double size_y, double size_z,
                       double coord_x, double coord_y, double coord_z)
    :check_collision(false), mass(mass), size{size_x, size_y, size_z},
     coord(chrono::ChCoordsys<>(chrono::ChVector<>(coord_x, coord_y, coord_z), chrono::QUNIT))
{}

void SimPayload::AddtoSystem(const std::shared_ptr<chrono::ChSystem>& sys){
    const std::shared_ptr<chrono::ChBody>& parent_body = sys->SearchBody(body_name.c_str());
    AddtoSystem(sys, parent_body);
}

void SimPayload::AddtoSystem(const std::shared_ptr<chrono::ChSystem>& sys,
                             const std::shared_ptr<chrono::ChBody>& parent_body){
    auto tmp_payload = chrono_types::make_shared<chrono::ChBodyEasyBox>(size[0], size[1], size[2], mass / (size[0] * size[1] * size[2], check_collision));
    tmp_payload->SetCoord(coord >> parent_body->GetCoord());
    sys->AddBody(tmp_payload);

    auto ch_link = chrono_types::make_shared<chrono::ChLinkLockLock>();
    ch_link->Initialize(tmp_payload,
                        parent_body,
                        chrono::ChCoordsys<>(tmp_payload->GetPos(), chrono::QUNIT));
    sys->AddLink(ch_link);
}

SimMotorBase::SimMotorBase(const std::string& link_name, double mass,
                           double size_x, double size_y, double size_z,
                           double coord_x, double coord_y, double coord_z)
    :SimPayload(mass, size_x,size_y, size_z, coord_x, coord_y, coord_z),
     link_name(link_name)
{}

void SimMotorBase::AddtoSystem(const std::shared_ptr<chrono::ChSystem>& sys,
                               chrono::ChUrdfDoc& urdf_doc){
    chlinkbody = &(urdf_doc.GetLinkBodies(link_name));
    SimPayload::AddtoSystem(sys, chlinkbody->body2);
}

SimServo::SimServo(const std::string& link_name, double mass,
                   double size_x, double size_y, double size_z,
                   double coord_x, double coord_y, double coord_z,
                   double upper_limit, double lower_limit)
    :SimMotorBase(link_name, mass, size_x, size_y, size_z, coord_x, coord_y, coord_z),
     upper_limit(upper_limit), lower_limit(lower_limit)
{}

void SimServo::AddtoSystem(const std::shared_ptr<chrono::ChSystem>& sys,
                           chrono::ChUrdfDoc& urdf_doc){
    SimMotorBase::AddtoSystem(sys, urdf_doc);
    ch_motor = chrono_types::make_shared<chrono::ChLinkMotorRotationAngle>();
    ch_func = chrono_types::make_shared<chrono::ChFunction_Const>(0);
    ch_motor->SetMotorFunction(ch_func);
    ch_motor->Initialize(chlinkbody->body1, chlinkbody->body2,
                         chrono::ChFrame<>(chlinkbody->link->GetLinkAbsoluteCoords()));
    sys->AddLink(ch_motor);
    std::dynamic_pointer_cast<chrono::ChLinkLock>(chlinkbody->link)->GetLimit_Z().SetMax(upper_limit);
    std::dynamic_pointer_cast<chrono::ChLinkLock>(chlinkbody->link)->GetLimit_Z().SetMin(lower_limit);
}

void SimServo::Set(double value){
    SimMotorBase::Set(value);
}

SimMotor::SimMotor(const std::string& link_name, double mass,
                   double size_x, double size_y, double size_z,
                   double coord_x, double coord_y, double coord_z, MotorType type)
    :SimMotorBase(link_name, mass, size_x, size_y, size_z, coord_x, coord_y, coord_z),
     type(type)
{}

void SimMotor::AddtoSystem(const std::shared_ptr<chrono::ChSystem>& sys,
                           chrono::ChUrdfDoc& urdf_doc){
    SimMotorBase::AddtoSystem(sys, urdf_doc);
    switch (type){
        case ANGLE:
            ch_motor = chrono_types::make_shared<chrono::ChLinkMotorRotationAngle>();
            break;
        case SPEED:
            ch_motor = chrono_types::make_shared<chrono::ChLinkMotorRotationSpeed>();
            break;
        case TORQUE:
            ch_motor = chrono_types::make_shared<chrono::ChLinkMotorRotationTorque>();
            break;
    }
    ch_motor->Initialize(chlinkbody->body1, chlinkbody->body2,
                         chrono::ChFrame<>(chlinkbody->link->GetLinkAbsoluteCoords()));
    ch_func = chrono_types::make_shared<chrono::ChFunction_Const>(0);
    ch_motor->SetMotorFunction(ch_func);
    sys->AddLink(ch_motor);
}

void SimMotor::Set(double value){
    SimMotorBase::Set(value);
}
