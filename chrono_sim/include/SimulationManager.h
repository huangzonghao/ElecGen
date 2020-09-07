#ifndef SIMULATION_MANAGER_H
#define SIMULATION_MANAGER_H

#include "chrono/physics/ChSystem.h"
#include "SimMotor.h"
#include "RobotController.h"

class RobotController;

class  SimulationManager {
  public:

    // input motors and waypoints, output control command for each motor for the next step
    // first for servo
    enum SystemType {NSC, SMC} system_type;

    double system_friction_k;
    double system_friction_s;

    std::vector<std::shared_ptr<SimPayload> > payloads;
    std::vector<std::shared_ptr<SimMotor> > motors;

    std::vector<chrono::ChVector<> > waypoints;

    // for manipulator
    std::shared_ptr<std::vector<double> > start_joint_pos;
    std::shared_ptr<std::vector<double> > goal_joint_pos;

    std::string urdf_file;
    std::string env_file;
    chrono::ChVector<> env_size;

    std::shared_ptr<chrono::ChSystem> sim_system;
    std::shared_ptr<RobotController> controller;

    bool task_done = false;

    double step_size;
    double timeout;

    SimulationManager(double step_size=0.005,
                      double timeout=50,
                      double system_friction_k=1.9,
                      double system_friction_s=2.0,
                      SystemType system_type=NSC);

    ~SimulationManager(){
        payloads.clear();
        motors.clear();
        waypoints.clear();
    }

    void SetSystemType(SystemType new_type){ system_type = new_type; }
    void SetUrdfFile(std::string filename){ urdf_file = filename; };
    void SetEnvFile(std::string filename){ env_file = filename; };
    void SetFrictionS(double fs) {system_friction_s = fs;};
    void SetFrictionK(double fk) {system_friction_k = fk;};

    void AddPayload(double mass, double size_x, double size_y, double size_z,
                    double coord_x=0, double coord_y=0, double coord_z=0);
    void AddMotor(const std::string& link_name, double mass,
                  double size_x, double size_y, double size_z,
                  double coord_x=0, double coord_y=0, double coord_z=0);
    void AddWaypoint(double x, double y, double z){ waypoints.push_back(chrono::ChVector<>(x,y,z)); };
    void SetStartJointPos(const std::shared_ptr<std::vector<double> >& jointpos) {start_joint_pos = jointpos;};
    void SetGoalJointPos(const std::shared_ptr<std::vector<double> >& jointpos) {goal_joint_pos = jointpos;};

    bool RunSimulation(bool do_viz=true);

};

#endif /* end of SIMULATION_MANAGER_H */
