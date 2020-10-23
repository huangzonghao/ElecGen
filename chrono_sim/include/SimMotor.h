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
    static constexpr double pos_thresh = 1e-3;
    static constexpr double vel_P = 1;
    static constexpr double vel_I = 0;
    static constexpr double vel_D = 0;
    static constexpr double pos_P = 1;
    static constexpr double pos_I = 0;
    static constexpr double pos_D = 0;

    enum Mode {POSITION, VELOCITY, PHASE} mode = VELOCITY;

    bool locked = false;

    std::shared_ptr<chrono::ChControllerPID> vel_pid;
    std::shared_ptr<chrono::ChControllerPID> pos_pid;
    chrono::ChLinkMotorRotationTorque *ch_motor;

    SimMotorController(const std::shared_ptr<chrono::ChLinkMotorRotationTorque>& target_motor);
    ~SimMotorController(){};

    void set_vel(double new_vel);
    void set_pos(double new_pos);
    void set_phase(double new_phase);
    double get_torque();
    bool check_status();
    void set_max_pos_control_vel(double pos_ctrl_vel){ max_pos_control_vel_ = pos_ctrl_vel; }
    double get_max_torque(){ return max_torque_; }
    double get_max_vel(){ return max_vel_; }
  private:
    double max_pos_control_vel_ = 0.2;
    double target_pos_ = 0;
    double target_vel_ = 0;
    double target_torque_ = 0;
    double max_torque_ = 0;
    double max_vel_ = 0;

};


class SimPayload {
  public:
    std::string body_name;
    // with respect to the parent body frame
    chrono::ChVector<> pos;
    chrono::ChMatrix33<> inertia;
    double size[3];
    double mass;
    bool visible;
    bool check_collision;
    void SetMass(double new_mass) { mass = new_mass; }
    const std::string& GetTypeName() {return type_name_;}
    SimPayload();
    SimPayload(const std::string& type_name, double mass,
               double size_x, double size_y, double size_z,
               double pos_x, double pos_y, double pos_z);
    SimPayload(const std::string& type_name, const std::string& body_name, double mass,
               double size_x, double size_y, double size_z,
               double pos_x, double pos_y, double pos_z);
    ~SimPayload(){};
    virtual void AddtoSystem(const std::shared_ptr<chrono::ChSystem>& sys);
  protected:
    void AddtoSystem(const std::shared_ptr<chrono::ChSystem>& sys,
                     const std::shared_ptr<chrono::ChBody>& parent_body);
  private:
    std::string type_name_; ///< type name that will be sent to electronic generator
};

class SimMotor : public SimPayload {
  public:

    std::string link_name;

    std::shared_ptr<SimMotorController> motor_controller;

    // the motor will actuate the link specified by link_name, and the mass
    // of the motor will be added to the body specified by body_name
    // i.e. the motor actuating the joint between body B and C could be residing
    // on body A.
    SimMotor(const std::string& type_name, const std::string& body_name,
             const std::string& link_name, double mass,
             double size_x, double size_y, double size_z,
             double pos_x, double pos_y, double pos_z);
    // if body name is not specified, the mass will be added to the parent body
    // of the link (body 2)
    SimMotor(const std::string& type_name, const std::string& link_name,
             double mass, double size_x, double size_y, double size_z,
             double pos_x, double pos_y, double pos_z);
    ~SimMotor(){};

    void AddtoSystem(const std::shared_ptr<chrono::ChSystem>& sys) override;
    void AddtoSystem(const chrono::ChUrdfDoc& urdf_doc);
    void SetVel(double new_vel);
    void SetPos(double new_pos);
    void SetPhase(double new_phase);
    bool CheckStatus(){ return motor_controller->check_status(); }
    void UpdateTorque();
    void printrot(){
        std::cout << ch_motor->GetMotorRotPeriodic() << std::endl;
    }

    double GetMaxTorque(){ return motor_controller->get_max_torque(); }
    double GetMaxVel(){ return motor_controller->get_max_vel(); }

  protected:
    const chrono::ChLinkBodies *chlinkbody;
    std::shared_ptr<chrono::ChLinkMotorRotationTorque> ch_motor;
    std::shared_ptr<chrono::ChFunction_Setpoint> ch_func;
    chrono::ChLink *ch_link;
};

#endif /* end of include guard: MOTOR_H */
