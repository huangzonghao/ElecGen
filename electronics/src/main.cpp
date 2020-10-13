#include "stdafx.h"
#include "testfile.h"

// #include <limits>

using namespace std;
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
//	motor_encoder_hbridge_battery_connect();

	// SYSTEM TEST
	system_test();
	system("pause");
}