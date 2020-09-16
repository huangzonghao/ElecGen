#include "traj_optimizer.h"

#include <cmath>
#include <iostream>

#include "ifopt/ipopt_solver.h"


using namespace towr;

TrajectoryOptimizer::
TrajectoryOptimizer(int num_ee, double mass, int num_steps,
                    const std::shared_ptr<const Eigen::MatrixXd>& heightmap,
                    double env_x, double env_y,
                    const Eigen::Vector3d& init_pos, const Eigen::Vector3d& init_ang,
                    const Eigen::Vector3d& goal_pos, const Eigen::Vector3d& goal_ang):
    num_ee_(num_ee), mass_(mass), num_steps_(num_steps), ee_pos_(num_ee, Eigen::Vector3d::Zero()),
    heightmap_(std::make_shared<towr::EigenHeightMap>(heightmap, env_x, env_y)),
    init_pos_(init_pos), init_ang_(init_ang),goal_pos_(goal_pos), goal_ang_(goal_ang)
{}

void TrajectoryOptimizer::set_heightmap(const std::shared_ptr<const Eigen::MatrixXd>& heightmap,
                                        double env_x, double env_y) {
    heightmap_ = std::make_shared<towr::EigenHeightMap>(heightmap, env_x, env_y);
}

void TrajectoryOptimizer::set_init_pose(const Eigen::Vector3d& init_pos,
                                        const Eigen::Vector3d& init_ang){
    init_pos_ = init_pos;
    init_ang_ = init_ang;
}

void TrajectoryOptimizer::set_goal_pose(const Eigen::Vector3d& goal_pos,
                                        const Eigen::Vector3d& goal_ang){
    goal_pos_ = goal_pos;
    goal_ang_ = goal_ang;
}

void TrajectoryOptimizer::set_ee_pos(const Eigen::Vector3d& ee_LF,
                                     const Eigen::Vector3d& ee_LH,
                                     const Eigen::Vector3d& ee_RF,
                                     const Eigen::Vector3d& ee_RH){
    ee_pos_[towr::LF] = ee_LF;
    ee_pos_[towr::LH] = ee_LH;
    ee_pos_[towr::RF] = ee_RF;
    ee_pos_[towr::RH] = ee_RH;
}


void TrajectoryOptimizer::
set_kinematic_model(double offset_x, double offset_y, double offset_z,
                    double dev_x, double dev_y, double dev_z){

    k_model_ = std::make_shared<QuadrupedalKinematicModel>(offset_x, offset_y, offset_z,
                                                         dev_x, dev_y, dev_z);
}
void TrajectoryOptimizer::
set_dynamic_model(double Ixx, double Iyy, double Izz,
                  double Ixy, double Ixz, double Iyz){

    d_model_ = std::make_shared<QuadrupedalDynamicModel>(mass_, Ixx, Iyy, Izz,
                                                       Ixy, Ixz, Iyz, num_ee_);
}

void TrajectoryOptimizer::optimize() {

    NlpFormulation formulation;

    // terrain
    formulation.terrain_ = heightmap_;

    RobotModel robot_model;
    robot_model.kinematic_model_ = k_model_;
    robot_model.dynamic_model_ = d_model_;
    formulation.model_ = robot_model;

    // set the initial position
    formulation.initial_base_.lin.at(kPos) = init_pos_;
    formulation.initial_base_.ang.at(kPos) = init_ang_;
    formulation.initial_ee_W_ = ee_pos_;

    // define the desired goal state
    formulation.final_base_.lin.at(kPos) = goal_pos_;
    formulation.final_base_.ang.at(kPos) = goal_ang_;

    std::vector<double> tmp_sequence(num_steps_, 0.4);
    formulation.params_.ee_phase_durations_ = std::vector<std::vector<double> >(num_ee_, tmp_sequence);

    // TODO: harded coded for quadrupedal
    formulation.params_.ee_in_contact_at_start_ = std::vector<bool>(num_ee_, true);

    ifopt::Problem nlp;
    for (auto c : formulation.GetVariableSets(solution_))
        nlp.AddVariableSet(c);
    for (auto c : formulation.GetConstraints(solution_))
        nlp.AddConstraintSet(c);
    for (auto c : formulation.GetCosts())
        nlp.AddCostSet(c);

    auto solver = std::make_shared<ifopt::IpoptSolver>();
    solver->SetOption("jacobian_approximation", "exact"); // "finite difference-values"
    solver->SetOption("max_cpu_time", 20.0);
    solver->Solve(nlp);

}

const SplineHolder& TrajectoryOptimizer::get_solution() const {
/*
 *     // Can directly view the optimization variables through:
 *     // Eigen::VectorXd x = nlp.GetVariableValues()
 *     // However, it's more convenient to access the splines constructed from these
 *     // variables and query their values at specific times:
 *     using namespace std;
 *     cout.precision(2);
 *     nlp.PrintCurrent(); // view variable-set, constraint violations, indices,...
 *     cout << fixed;
 *     cout << "\n====================\nMonoped trajectory:\n====================\n";
 *
 *     double t = 0.0;
 *     while (t<=solution_.base_linear_->GetTotalTime() + 1e-5) {
 *         cout << "t=" << t << "\n";
 *         cout << "Base linear position x,y,z:   \t";
 *         cout << solution_.base_linear_->GetPoint(t).p().transpose() << "\t[m]" << endl;
 *
 *         cout << "Base Euler roll, pitch, yaw:  \t";
 *         Eigen::Vector3d rad = solution_.base_angular_->GetPoint(t).p();
 *         cout << (rad/M_PI*180).transpose() << "\t[deg]" << endl;
 *
 *         cout << "Foot position x,y,z:          \t";
 *         cout << solution_.ee_motion_.at(0)->GetPoint(t).p().transpose() << "\t[m]" << endl;
 *
 *         cout << "Contact force x,y,z:          \t";
 *         cout << solution_.ee_force_.at(0)->GetPoint(t).p().transpose() << "\t[N]" << endl;
 *
 *         bool contact = solution_.phase_durations_.at(0)->IsContactPhase(t);
 *         std::string foot_in_contact = contact? "yes" : "no";
 *         cout << "Foot in contact:              \t" + foot_in_contact << endl;
 *
 *         cout << endl;
 *
 *         t += 0.2;
 *     }
 */
    return solution_;
}
