#include <iostream>
#include <Eigen/Core>
#include "SimulationManager.h"
#include "Inference.h"

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
        std::cout << "motor 1 torque " << sm.GetMotor(0)->GetMaxTorque() << std::endl;
        std::cout << "motor 2 torque " << sm.GetMotor(1)->GetMaxTorque() << std::endl;
        std::cout << "motor 3 torque " << sm.GetMotor(2)->GetMaxTorque() << std::endl;
        std::cout << "motor 4 torque " << sm.GetMotor(3)->GetMaxTorque() << std::endl;

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
        std::cout << "motor 1 torque " << sm.GetMotor(0)->GetMaxTorque() << std::endl;
        std::cout << "motor 2 torque " << sm.GetMotor(1)->GetMaxTorque() << std::endl;
        std::cout << "motor 3 torque " << sm.GetMotor(2)->GetMaxTorque() << std::endl;
        std::cout << "motor 4 torque " << sm.GetMotor(3)->GetMaxTorque() << std::endl;

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
        std::cout << "motor 1 torque " << sm.GetMotor(0)->GetMaxTorque() << std::endl;
        std::cout << "motor 2 torque " << sm.GetMotor(1)->GetMaxTorque() << std::endl;
        std::cout << "motor 3 torque " << sm.GetMotor(2)->GetMaxTorque() << std::endl;
        std::cout << "motor 4 torque " << sm.GetMotor(3)->GetMaxTorque() << std::endl;

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
    sm.SetEnv(env_file, 1, 1, 0.08);
    sm.SetFrictionK(k_friction);
    sm.SetFrictionS(s_friction);
    // sm.timeout = 10;
    // try to use a wrong body name and see what happened
    // this part will be done by UI
    // TODO: finalize mass of components
    sm.AddMotor("chassis", "chassis-fl_cyl",    1, 0.01, 0.01, 0.01);
    sm.AddMotor("chassis", "fl_cyl-fl_upper",   1, 0.01, 0.01, 0.01);
    sm.AddMotor("chassis", "fl_upper-fl_lower", 1, 0.01, 0.01, 0.01);
    sm.AddMotor("chassis", "chassis-bl_cyl",    1, 0.01, 0.01, 0.01);
    sm.AddMotor("chassis", "bl_cyl-bl_upper",   1, 0.01, 0.01, 0.01);
    sm.AddMotor("chassis", "bl_upper-bl_lower", 1, 0.01, 0.01, 0.01);
    sm.AddMotor("chassis", "chassis-fr_cyl",    1, 0.01, 0.01, 0.01);
    sm.AddMotor("chassis", "fr_cyl-fr_upper",   1, 0.01, 0.01, 0.01);
    sm.AddMotor("chassis", "fr_upper-fr_lower", 1, 0.01, 0.01, 0.01);
    sm.AddMotor("chassis", "chassis-br_cyl",    1, 0.01, 0.01, 0.01);
    sm.AddMotor("chassis", "br_cyl-br_upper",   1, 0.01, 0.01, 0.01);
    sm.AddMotor("chassis", "br_upper-br_lower", 1, 0.01, 0.01, 0.01);
    // encoder * 8 camera * 1
    // sm.AddPayload();
    // ...
    // n + 1 vector: n for input components masses,
    // 1 for sum of all other components mass
    stringvec input_types(sm.GetComponentNumber());
    doublepairs input_vels(sm.GetMotorNumber(), std::pair<double, double>(0.0, 0.0));
    doublepairs input_torqs(sm.GetMotorNumber(), std::pair<double, double>(0.0, 0.0));
    std::shared_ptr<BBNode> best_node;

    sm.AddWaypoints(waypoints);
    sm.SetEigenHeightmap(heightmap);

    // use this loop for iterations
    bool sim_done = false;
    int cnt = 0;
    while (!sim_done){
        bool task_done = sm.RunSimulation(true);

        if (task_done && !cnt) {
            sm.GetComponentTypes(&input_types);
            sm.GetActuatorVels(&input_vels);
            sm.GetActuatorTorques(&input_torqs);

            stringvec2d component_versions = preprocess(input_types, input_torqs, input_vels);
            infernodevec2d infer_nodes_vec = initialize(component_versions, input_torqs, input_vels);
            bbnodevec bbnodes = initialize(infer_nodes_vec);
            best_node = branchNBound(bbnodes);
            writeDesign(*best_node);
        }
        else if (task_done) {
             sm.GetActuatorVels(&input_vels);
             sm.GetActuatorTorques(&input_torqs);

            // verify design
            if (doubleCheck(*best_node, input_torqs, input_vels)) {
                sim_done = true;
            }
            else {
                stringvec2d component_versions = preprocess(input_types, input_torqs, input_vels);
                infernodevec2d infer_nodes_vec = initialize(component_versions, input_torqs, input_vels);
                bbnodevec bbnodes = initialize(infer_nodes_vec);
                best_node = branchNBound(bbnodes);
                writeDesign(*best_node);
            }
        }
        sm.UpdateMassInfo(getMassVec(*best_node, input_torqs.size()));
        cnt++;

        // now the torques are ready to read
        // std::cout << "motor 1 torque " << sm.GetMotor(0)->GetMaxTorque() << std::endl;
        // std::cout << "motor 2 torque " << sm.GetMotor(1)->GetMaxTorque() << std::endl;
        // std::cout << "motor 3 torque " << sm.GetMotor(2)->GetMaxTorque() << std::endl;
        // std::cout << "motor 4 torque " << sm.GetMotor(3)->GetMaxTorque() << std::endl;

        // TODO: exit condition?
        // sim_done = true;
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
