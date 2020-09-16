#ifndef TRAJ_OPTIMIZER_H_
#define TRAJ_OPTIMIZER_H_

#include "towr/nlp_formulation.h"
#include "eigen_heightmap.h"
#include "quadrupedal_model.h"


class TrajectoryOptimizer {
  public:
    TrajectoryOptimizer(int num_ee, double mass, int num_steps,
                        const std::shared_ptr<const Eigen::MatrixXd>& heightmap,
                        double env_x, double env_y,
                        const Eigen::Vector3d& init_pos, const Eigen::Vector3d& init_ang,
                        const Eigen::Vector3d& goal_pos, const Eigen::Vector3d& goal_ang);

    void set_heightmap(const std::shared_ptr<const Eigen::MatrixXd>& heightmap,
                       double env_x, double env_y);
    void set_init_pose(const Eigen::Vector3d& init_pos, const Eigen::Vector3d& init_ang);
    void set_goal_pose(const Eigen::Vector3d& goal_pos, const Eigen::Vector3d& goal_ang);
    void set_ee_pos(const Eigen::Vector3d& ee_LF,   ///< left front
                    const Eigen::Vector3d& ee_LH,   ///< left hind
                    const Eigen::Vector3d& ee_RF,   ///< right front
                    const Eigen::Vector3d& ee_RH);  ///< right hind
    void set_kinematic_model(double offset_x, double offset_y, double offset_z,
                             double dev_x, double dev_y, double dev_z);
    void set_dynamic_model(double Ixx=1, double Iyy=1, double Izz=1,
                           double Ixy=0, double Ixz=0, double Iyz=0);
    void optimize();

    const towr::SplineHolder& get_solution() const;

  private:
    towr::SplineHolder solution_;
    std::shared_ptr<towr::EigenHeightMap> heightmap_;
    Eigen::Vector3d init_pos_;
    Eigen::Vector3d init_ang_;
    Eigen::Vector3d goal_pos_;
    Eigen::Vector3d goal_ang_;
    std::vector<Eigen::Vector3d> ee_pos_;
    std::shared_ptr<towr::QuadrupedalKinematicModel> k_model_;
    std::shared_ptr<towr::QuadrupedalDynamicModel> d_model_;
    int num_ee_; // number of endeffectors
    double mass_;
    int num_steps_; // number of steps of each foot
};


#endif /* end of TRAJ_OPTIMIZER_H_ */
