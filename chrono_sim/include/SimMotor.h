#ifndef MOTOR_H
#define MOTOR_H

#include "chrono/core/ChCoordsys.h"
#include "chrono/core/ChMathematics.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChLinkMotorRotation.h"
#include "chrono/physics/ChLinkMotorRotationTorque.h"
#include "chrono/physics/ChController.h"

#include "ChUrdfDoc.h"

class SimMotorController {
  public:
    static int counter;
    static constexpr double pos_thresh = 1e-2;
    static constexpr double vel_P = 80;
    static constexpr double vel_I = 0;
    static constexpr double vel_D = 0;
    static constexpr double pos_P = 10;
    static constexpr double pos_I = 0;
    static constexpr double pos_D = 0;

    enum Mode {POSITION, VELOCITY, PHASE} mode = VELOCITY;

    bool locked = false;
    double max_pos_control_vel = 6;

    std::shared_ptr<chrono::ChControllerPID> vel_pid;
    std::shared_ptr<chrono::ChControllerPID> pos_pid;
    chrono::ChLinkMotorRotationTorque *ch_motor;

    SimMotorController(const std::shared_ptr<chrono::ChLinkMotorRotationTorque>& target_motor);
    ~SimMotorController(){};

    void SetVel(double new_vel);
    void SetPos(double new_pos);
    void SetPhase(double new_phase);
    double GetTorque();
    bool check_status();
  private:
    double target_pos = 0;
    double target_vel = 0;
};


class SimPayload {
  public:
    std::string body_name;
    // with respect  to the parent body frame
    chrono::ChVector<> pos;
    chrono::ChMatrix33<> inertia;
    double size[3];
    double mass;
    bool visible;
    bool check_collision;
    SimPayload();
    SimPayload(const std::string& body_name, double mass,
               double size_x, double size_y, double size_z,
               double pos_x, double pos_y, double pos_z);
    ~SimPayload(){};
    virtual void AddtoSystem(const std::shared_ptr<chrono::ChSystem>& sys);
    void AddtoSystem(const std::shared_ptr<chrono::ChSystem>& sys,
                     const std::shared_ptr<chrono::ChBody>& parent_body);
};

class SimMotor : public SimPayload {
  public:

    std::string link_name;
    double max_torque = 0;

    std::shared_ptr<SimMotorController> motor_controller;

    SimMotor(const std::string& body_name, const std::string& link_name,
             double mass, double size_x, double size_y, double size_z,
             double pos_x, double pos_y, double pos_z);
    ~SimMotor(){};

    void AddtoSystem(const std::shared_ptr<chrono::ChSystem>& sys, const chrono::ChUrdfDoc& urdf_doc);
    void SetVel(double new_vel);
    void SetPos(double new_pos);
    void SetPhase(double new_phase);
    bool CheckStatus(){ return motor_controller->check_status(); }
    void UpdateTorque();
    void printrot(){
        std::cout << ch_motor->GetMotorRotPeriodic() << std::endl;
    }

  private:
    const chrono::ChLinkBodies *chlinkbody;
    std::shared_ptr<chrono::ChLinkMotorRotationTorque> ch_motor;
    std::shared_ptr<chrono::ChFunction_Setpoint> ch_func;
    chrono::ChLink *ch_link;
};

#endif /* end of include guard: MOTOR_H */
