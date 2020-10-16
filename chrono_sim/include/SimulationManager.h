#ifndef SIMULATION_MANAGER_H
#define SIMULATION_MANAGER_H

#include "chrono/physics/ChSystem.h"
#include "SimMotor.h"
#include "RobotController.h"
#include "ChUrdfDoc.h"
#include <Eigen/Core>

class  SimulationManager {
  public:

    // input motors and waypoints, output control command for each motor for the next step
    // first for servo
    enum SystemType {NSC, SMC};

    SimulationManager(double step_size=0.005,
                      double timeout=50,
                      double system_friction_k=1.9,
                      double system_friction_s=2.0,
                      SystemType system_type=NSC);

    ~SimulationManager(){
        payloads_.clear();
        motors_.clear();
        ch_waypoints_.clear();
    }

    void SetSystemType(SystemType new_type){ system_type_ = new_type; }
    void SetUrdfFile(std::string filename);
    void SetEnv(std::string filename, double env_x=1, double env_y=1, double env_z=0.08);
    // TODO: should be done within SetEnv, but currently having difficulty readin bitmap in c++
    void SetEigenHeightmap(const std::shared_ptr<const Eigen::MatrixXd>& heightmap);
    void SetFrictionS(double fs) {s_friction_ = fs;};
    void SetFrictionK(double fk) {k_friction_ = fk;};

    void AddPayload(const std::string& body_name, double mass,
                    double size_x, double size_y, double size_z,
                    double pos_x=0, double pos_y=0, double pos_z=0);
    void AddMotor(const std::string& link_name,
                  double mass, double size_x, double size_y, double size_z,
                  double pos_x=0, double pos_y=0, double pos_z=0);
    void AddMotor(const std::string& body_name, const std::string& link_name,
                  double mass, double size_x, double size_y, double size_z,
                  double pos_x=0, double pos_y=0, double pos_z=0);
    void AddWaypoint(double x, double y, double z);
    void AddWaypoints(const std::shared_ptr<const Eigen::MatrixXd>& waypoints_ptr);
    void SetStartJointPos(const std::shared_ptr<std::vector<double> >& jointpos) {start_joint_pos_ = jointpos;};
    void SetGoalJointPos(const std::shared_ptr<std::vector<double> >& jointpos) {goal_joint_pos_ = jointpos;};

    const std::shared_ptr<SimMotor> GetMotor(int motor_idx) const { return motors_[motor_idx];}

    bool RunSimulation(bool do_viz=true);
    const std::string& GetUrdfFileName();

    /** Interface functions **/
    void GetComponentTypes(std::vector<std::string> *types_container) const;
    void GetActuatorVels(std::vector<std::pair<double, double> > *vels_container) const;
    void GetActuatorTorques(std::vector<std::pair<double, double> > *torqs_container) const;
    void UpdateMassInfo(const std::vector<double>& mass_vec);
    int GetMotorNumber() { return motors_.size(); }
    int GetComponentNumber() { return motors_.size() + payloads_.size(); }
  private:
    SystemType  system_type_;
    double k_friction_;
    double s_friction_;

    // for manipulator
    // TODO: automatically switch waypoints to be pose of ee?
    std::shared_ptr<std::vector<double> > start_joint_pos_;
    std::shared_ptr<std::vector<double> > goal_joint_pos_;

    std::vector<std::shared_ptr<SimPayload> > payloads_;
    std::vector<std::shared_ptr<SimMotor> > motors_;
    std::vector<chrono::ChVector<> > ch_waypoints_;
    std::shared_ptr<chrono::ChUrdfDoc> urdf_doc_;
    std::shared_ptr<const Eigen::MatrixXd> eigen_waypoints_;
    std::shared_ptr<chrono::ChSystem> ch_system_;
    std::shared_ptr<RobotController> controller_;
    std::shared_ptr<const Eigen::MatrixXd> heightmap_;

    bool task_done_ = false;
    double step_size_;
    double timeout_;

    std::string env_file_;
    // unit: m
    double env_x_ = 1;
    double env_y_ = 1;
    double env_z_ = 0.08;

    // names of bodies that would use ChBodyAuxRef
    std::unordered_set<std::string> auxrefs_;

};

#endif /* end of SIMULATION_MANAGER_H */
