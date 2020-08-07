#ifndef MOTOR_H
#define MOTOR_H

#include "chrono/core/ChCoordsys.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChLinkMotorRotation.h"
#include "chrono/core/ChMathematics.h"

#include "ChUrdfDoc.h"

class SimPayload {
  public:
    std::string body_name;
    // with respect  to the parent body frame
    chrono::ChCoordsys<> coord;
    double size[3];
    double mass;
    bool check_collision;
    SimPayload();
    SimPayload(double mass, double size_x, double size_y, double size_z,
               double coord_x, double coord_y, double coord_z);
    ~SimPayload(){};
    virtual void AddtoSystem(const std::shared_ptr<chrono::ChSystem>& sys);
    void AddtoSystem(const std::shared_ptr<chrono::ChSystem>& sys,
                     const std::shared_ptr<chrono::ChBody>& parent_body);
};

class SimMotorBase : public SimPayload {
  public:
    std::string link_name;
    double max_torque = 0;

    SimMotorBase(){};
    SimMotorBase(const std::string& link_name, double mass,
                 double size_x, double size_y, double size_z,
                 double coord_x, double coord_y, double coord_z);
    virtual ~SimMotorBase() = 0;
    virtual void AddtoSystem(const std::shared_ptr<chrono::ChSystem>& sys, chrono::ChUrdfDoc& urdf_doc);
    // Set motor values
    virtual void Set(double value){ ch_func->Set_yconst(value); };
    virtual void UpdateMaxTorque(){ max_torque = chrono::ChMax(max_torque, ch_motor->GetMotorTorque()); };

    const chrono::ChLinkBodies *chlinkbody;
    std::shared_ptr<chrono::ChLinkMotorRotation> ch_motor;
    std::shared_ptr<chrono::ChFunction_Const> ch_func;
};

class SimServo : public SimMotorBase {
  public:
    double upper_limit;
    double lower_limit;

    SimServo(){};
    SimServo(const std::string& link_name, double mass, double size_x, double size_y,
             double size_z, double coord_x, double coord_y, double coord_z,
             double upper_limit, double lower_limit);
    ~SimServo(){};

    void AddtoSystem(const std::shared_ptr<chrono::ChSystem>& sys, chrono::ChUrdfDoc& urdf_doc) override;
    void Set(double value) override;
};


class SimMotor : public SimMotorBase {
  public:
    enum MotorType {ANGLE, SPEED, TORQUE} type;

    SimMotor():type(SPEED){};
    SimMotor(const std::string& link_name, double mass,
             double size_x, double size_y, double size_z,
             double coord_x, double coord_y, double coord_z, MotorType type);
    ~SimMotor(){};

    void AddtoSystem(const std::shared_ptr<chrono::ChSystem>& sys, chrono::ChUrdfDoc& urdf_doc) override;
    void Set(double value) override;
};




#endif /* end of include guard: MOTOR_H */
