#include <iostream>
#include "SimulationManager.h"

// const std::string env_filename = "../data/maps/env1.bmp";
std::string env_filename = "";

const double s_friction = 2.0;
const double k_friction = 1.9;

void test_fourwheel(){
    const std::string urdf_filename = "../data/robots/fourwheels.urdf";

    SimulationManager sm;
    sm.SetUrdfFile(urdf_filename);
    sm.SetEnv(env_filename);
    sm.SetFrictionK(k_friction);
    sm.SetFrictionS(s_friction);
    // sm.timeout = 10;
    sm.AddMotor("chassis_wheel_fl", 1,0.1,0.1,0.1);
    sm.AddMotor("chassis_wheel_rl", 1,0.1,0.1,0.1);
    sm.AddMotor("chassis_wheel_fr", 1,0.1,0.1,0.1);
    sm.AddMotor("chassis_wheel_rr", 1,0.1,0.1,0.1);

    sm.AddWaypoint(0, 0, 1.5);
    sm.AddWaypoint(8, 8, 1);
    sm.AddWaypoint(12, 0, 1);

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

void test_fourleg(){
    const std::string urdf_filename = "../data/robots/fourleg.urdf";
    // first init simulation manager
    SimulationManager sm;
    sm.SetUrdfFile(urdf_filename);
    sm.SetEnv(env_filename);
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

    sm.AddWaypoint(0, 0, 1.5);
    sm.AddWaypoint(8, 0, 1);
    sm.AddWaypoint(10, 3, 1);
    // sm.AddWaypoint(8, 3, 1);

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

void test_fourleg2(){
    const std::string urdf_filename = "../data/robots/fourleg2.urdf";
    // first init simulation manager
    SimulationManager sm;
    sm.SetUrdfFile(urdf_filename);
    sm.SetEnv(env_filename);
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

    sm.AddWaypoint(0, 0, 3.5);
    sm.AddWaypoint(10, 0, 1);

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

void test_fourleg3(){
    const std::string urdf_filename = "../data/robots/fourleg3.urdf";
    // first init simulation manager
    SimulationManager sm;
    sm.SetUrdfFile(urdf_filename);
    sm.SetEnv(env_filename, 50, 50, 5);
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

    sm.AddWaypoint(0, 0, 2.5);
    sm.AddWaypoint(8, 8, 1);
    sm.AddWaypoint(12, 0, 1);

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

int main(int argc, char* argv[]){
    // for (int i = 1; i < argc; ++i){
        // if      (std::string(argv[i]) == "0") test_fourwheel();
        // else if (std::string(argv[i]) == "1") test_fourleg();
        // else if (std::string(argv[i]) == "2") test_fourleg2();
        // else if (std::string(argv[i]) == "3") test_fourleg3();
        // else std::cerr << "Error: unknown command " << argv[i] << std::endl;
    // }
    if (argc == 1) {
        test_fourleg();
    }
    else {
        if (argc == 3){
            env_filename = argv[2];
        }
        if      (std::string(argv[1]) == "0") test_fourwheel();
        else if (std::string(argv[1]) == "1") test_fourleg();
        else if (std::string(argv[1]) == "2") test_fourleg2();
        else if (std::string(argv[1]) == "3") test_fourleg3();
        else std::cerr << "Error: unknown command " << argv[1] << std::endl;
    }


    return EXIT_SUCCESS;
}
