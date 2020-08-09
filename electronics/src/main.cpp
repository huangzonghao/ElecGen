#include "stdafx.h"
#include "testfile.h"

#include <iostream>
#include "SimulationManager.h"

const std::string urdf_filename = "../data/robots/fourwheels.urdf";
// const std::string urdf_filename = "../data/robots/fourleg.urdf";
const std::string env_filename = "";

const double s_friction = 2.0;
const double k_friction = 1.9;

// #include <limits>

using namespace Eigen;

int main() {

	// READ TEST
	motor_read();
	h_bridge_read();
	battery_read();
	micro_controller_read();
	force_sensor_read();
	voltage_regulator_read();
	servo_read();
	camera_read();
	bluetooth_read();
	encoder_read();

    /*
    // first init simulation manager
    SimulationManager sm;
    sm.SetUrdfFile(urdf_filename);
    sm.SetEnvFile(env_filename);
    sm.SetFrictionK(k_friction);
    sm.SetFrictionS(s_friction);

    // this part will be done by UI
    sm.AddMotor("chassis_wheel_rl", 1,0.1,0.1,0.1);
    sm.AddMotor("chassis_wheel_rr", 1,0.1,0.1,0.1);
    // sm.AddMotor("base_link_link1", 1,0.1,0.1,0.1);
    // sm.AddMotor("base_link_link2", 1,0.1,0.1,0.1);
    // sm.AddMotor("base_link_link3", 1,0.1,0.1,0.1);
    // sm.AddMotor("base_link_link4", 1,0.1,0.1,0.1);
    sm.AddWaypoint(0, 0, 1);
    sm.AddWaypoint(5, 0, 1);
    sm.AddWaypoint(8, 0, 1);

    // use this loop for iterations
    bool sim_done = false;
    while (!sim_done){
        bool task_done = sm.RunSimulation(true);
        // bool task_done = sm.RunSimulation(false);

        // now the torques are ready to read
        std::cout << "motor 1 torque " << sm.motors[0]->max_torque << std::endl;
        std::cout << "motor 2 torque " << sm.motors[1]->max_torque << std::endl;

        sim_done = true;
    }
    */

	// SOLVE TEST
//	motor_solve();
//	motor_hbridge_solve();
//	motor_hbridge_battery_solve();
//	motor_hbridge_battery_micro_controller_solve();


	// CONNECT TEST
//	motor_hbridge_connect();
//	motor_hbridge_battery_connect();
//	motor_hbridge_battery_micro_controller_connect();
//	motor_hbridge_voltage_regulator_micro_controller_connect();
//	motor_hbridge_voltage_regulator_micro_controller_battery_connect();
//	servo_micro_controller_voltage_regulator_battery_connect();
//	force_sensor_battery_micro_controller_connect();
//	camera_voltage_regulator_micro_controller_battery_connect();
//	bluetooth_battery_micro_controller_connect();
//	encoder_battery_micro_controller_connect();


	// SYSTEM TEST
	system_test();
	system("pause");
}
