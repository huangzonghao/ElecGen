#include <iostream>
#include "SimulationManager.h"

const std::string urdf_filename = "../robots/fourwheels.urdf";
const std::string env_filename = "";

const double s_friction = 2.0;
const double k_friction = 1.9;

int main(){
    // first init simulation manager
    SimulationManager sm;
    sm.SetUrdfFile(urdf_filename);
    sm.SetEnvFile(env_filename);
    sm.SetFrictionK(k_friction);
    sm.SetFrictionS(s_friction);

    // this part will be done by UI
    sm.AddMotor("chasis_wheel_rl", 1,1,1,1);
    sm.AddMotor("chasis_wheel_rr", 1,1,1,1);
    sm.AddWaypoint(0, 0, 1);
    sm.AddWaypoint(5, 5, 1);

    // use this loop for iterations
    bool sim_done = false;
    while (!sim_done){
        bool task_done = sm.RunSimulation(true);

        // now the torques are ready to read
        std::cout << sm.motors[0]->max_torque << std::endl;
        std::cout << sm.motors[1]->max_torque << std::endl;

        sim_done = true;
    }


    return 0;
}
