#include <iostream>
#include <Eigen/Core>
#include "SimulationManager.h"

const double s_friction = 2.0;
const double k_friction = 1.9;

void test_fourwheel(const std::shared_ptr<const Eigen::MatrixXd>& waypoints,
                    const std::string& env_file){
    const std::string urdf_file = "../data/robots/fourwheels.urdf";

    SimulationManager sm;
    sm.SetUrdfFile(urdf_file);
    sm.SetEnv(env_file);
    sm.SetFrictionK(k_friction);
    sm.SetFrictionS(s_friction);
    // sm.timeout = 10;
    sm.AddMotor("chassis_wheel_fl", 1,0.1,0.1,0.1);
    sm.AddMotor("chassis_wheel_rl", 1,0.1,0.1,0.1);
    sm.AddMotor("chassis_wheel_fr", 1,0.1,0.1,0.1);
    sm.AddMotor("chassis_wheel_rr", 1,0.1,0.1,0.1);

    sm.AddWaypoints(waypoints);

    // use this loop for iterations
    bool sim_done = false;
    while (!sim_done){
        bool task_done = sm.RunSimulation();

        // now the torques are ready to read
        std::cout << "motor 1 torque " << sm.GetMotor(0)->max_torque << std::endl;
        std::cout << "motor 2 torque " << sm.GetMotor(1)->max_torque << std::endl;
        std::cout << "motor 3 torque " << sm.GetMotor(2)->max_torque << std::endl;
        std::cout << "motor 4 torque " << sm.GetMotor(3)->max_torque << std::endl;

        sim_done = true;
    }

}

void test_fourleg(const std::shared_ptr<const Eigen::MatrixXd>& waypoints, const std::string& env_file){
    const std::string urdf_file = "../data/robots/fourleg.urdf";
    // first init simulation manager
    SimulationManager sm;
    sm.SetUrdfFile(urdf_file);
    sm.SetEnv(env_file);
    sm.SetFrictionK(k_friction);
    sm.SetFrictionS(s_friction);
    // sm.timeout = 10;
    // try to use a wrong body name and see what happened
    // this part will be done by UI
    // front left
    sm.AddMotor("base_link_link1", 1,0.1,0.1,0.1);
    // rear left
    sm.AddMotor("base_link_link2", 1,0.1,0.1,0.1);
    // front right
    sm.AddMotor("base_link_link4", 1,0.1,0.1,0.1);
    // rear right
    sm.AddMotor("base_link_link3", 1,0.1,0.1,0.1);

    sm.AddWaypoints(waypoints);

    // use this loop for iterations
    bool sim_done = false;
    while (!sim_done){
        bool task_done = sm.RunSimulation();

        // now the torques are ready to read
        std::cout << "motor 1 torque " << sm.GetMotor(0)->max_torque << std::endl;
        std::cout << "motor 2 torque " << sm.GetMotor(1)->max_torque << std::endl;
        std::cout << "motor 3 torque " << sm.GetMotor(2)->max_torque << std::endl;
        std::cout << "motor 4 torque " << sm.GetMotor(3)->max_torque << std::endl;

        sim_done = true;
    }

}

void test_fourleg2(const std::shared_ptr<const Eigen::MatrixXd>& waypoints, const std::string& env_file){
    const std::string urdf_file = "../data/robots/fourleg2.urdf";
    // first init simulation manager
    SimulationManager sm;
    sm.SetUrdfFile(urdf_file);
    sm.SetEnv(env_file);
    sm.SetFrictionK(k_friction);
    sm.SetFrictionS(s_friction);
    // sm.timeout = 10;
    // try to use a wrong body name and see what happened
    sm.AddMotor("chassis", "chassis-fl_upper", 1,0.1,0.1,0.1);
    sm.AddMotor("chassis", "fl_upper-fl_lower", 1,0.1,0.1,0.1);
    sm.AddMotor("chassis", "chassis-bl_upper", 1,0.1,0.1,0.1);
    sm.AddMotor("chassis", "bl_upper-bl_lower", 1,0.1,0.1,0.1);
    sm.AddMotor("chassis", "chassis-fr_upper", 1,0.1,0.1,0.1);
    sm.AddMotor("chassis", "fr_upper-fr_lower", 1,0.1,0.1,0.1);
    sm.AddMotor("chassis", "chassis-br_upper", 1,0.1,0.1,0.1);
    sm.AddMotor("chassis", "br_upper-br_lower", 1,0.1,0.1,0.1);

    sm.AddWaypoints(waypoints);

    // use this loop for iterations
    bool sim_done = false;
    while (!sim_done){
        bool task_done = sm.RunSimulation(true);
        // bool task_done = sm.RunSimulation(false);

        // now the torques are ready to read
        std::cout << "motor 1 torque " << sm.GetMotor(0)->max_torque << std::endl;
        std::cout << "motor 2 torque " << sm.GetMotor(1)->max_torque << std::endl;
        std::cout << "motor 3 torque " << sm.GetMotor(2)->max_torque << std::endl;
        std::cout << "motor 4 torque " << sm.GetMotor(3)->max_torque << std::endl;

        sim_done = true;
    }

}

void test_fourleg3(const std::shared_ptr<const Eigen::MatrixXd>& waypoints,
                   const std::string& env_file,
                   const std::shared_ptr<const Eigen::MatrixXd>& heightmap){
    const std::string urdf_file = "../data/robots/fourleg3.urdf";
    // first init simulation manager
    SimulationManager sm;
    sm.SetUrdfFile(urdf_file);
    sm.SetEnv(env_file, 50, 50, 5);
    sm.SetFrictionK(k_friction);
    sm.SetFrictionS(s_friction);
    // sm.timeout = 10;
    // try to use a wrong body name and see what happened
    // this part will be done by UI
    sm.AddMotor("chassis", "chassis-fl_cyl", 1,0.1,0.1,0.1);
    sm.AddMotor("chassis", "fl_cyl-fl_upper", 1,0.1,0.1,0.1);
    sm.AddMotor("chassis", "fl_upper-fl_lower", 1,0.1,0.1,0.1);
    sm.AddMotor("chassis", "chassis-bl_cyl", 1,0.1,0.1,0.1);
    sm.AddMotor("chassis", "bl_cyl-bl_upper", 1,0.1,0.1,0.1);
    sm.AddMotor("chassis", "bl_upper-bl_lower", 1,0.1,0.1,0.1);
    sm.AddMotor("chassis", "chassis-fr_cyl", 1,0.1,0.1,0.1);
    sm.AddMotor("chassis", "fr_cyl-fr_upper", 1,0.1,0.1,0.1);
    sm.AddMotor("chassis", "fr_upper-fr_lower", 1,0.1,0.1,0.1);
    sm.AddMotor("chassis", "chassis-br_cyl", 1,0.1,0.1,0.1);
    sm.AddMotor("chassis", "br_cyl-br_upper", 1,0.1,0.1,0.1);
    sm.AddMotor("chassis", "br_upper-br_lower", 1,0.1,0.1,0.1);

    sm.AddWaypoints(waypoints);
    sm.SetEigenHeightmap(heightmap);

    // use this loop for iterations
    bool sim_done = false;
    while (!sim_done){
        bool task_done = sm.RunSimulation(true);
        // bool task_done = sm.RunSimulation(false);

        // now the torques are ready to read
        std::cout << "motor 1 torque " << sm.GetMotor(0)->max_torque << std::endl;
        std::cout << "motor 2 torque " << sm.GetMotor(1)->max_torque << std::endl;
        std::cout << "motor 3 torque " << sm.GetMotor(2)->max_torque << std::endl;
        std::cout << "motor 4 torque " << sm.GetMotor(3)->max_torque << std::endl;

        sim_done = true;
    }

}
void launch_simulation(const std::string& urdf_file,
                       const std::string& env_file,
                       const std::shared_ptr<const Eigen::MatrixXd>& heightmap,
                       const std::shared_ptr<const Eigen::MatrixXd>& waypoints) {

    if      (urdf_file.find("fourwheel") != std::string::npos) test_fourwheel(waypoints, env_file);
    else if (urdf_file.find("fourleg2")  != std::string::npos) test_fourleg2(waypoints, env_file);
    else if (urdf_file.find("fourleg3")  != std::string::npos) test_fourleg3(waypoints, env_file, heightmap);
    else if (urdf_file.find("fourleg")   != std::string::npos) test_fourleg(waypoints, env_file);
    else std::cout << "Error: unknown robot " << urdf_file << std::endl;
    return;
}
