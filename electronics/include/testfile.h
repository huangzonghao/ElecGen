#ifndef TESTFILE_H_7WN4LDYW
#define TESTFILE_H_7WN4LDYW

#include "Circuit.h"
#include "Matching.h"
#include "Inference.h"

// READING TEST
void motor_read();
void h_bridge_read();
void battery_read();
void micro_controller_read();
void force_sensor_read();
void voltage_regulator_read();
void servo_read();
void camera_read();
void bluetooth_read();
void encoder_read();


// SOLVING TEST
void motor_solve();
void motor_hbridge_solve();
void motor_hbridge_battery_solve();
void motor_hbridge_battery_micro_controller_solve();


// CONNECTION TEST
void motor_hbridge_connect();
void motor_hbridge_battery_connect();
void motor_hbridge_battery_micro_controller_connect();
void motor_hbridge_voltage_regulator_micro_controller_connect();
void motor_hbridge_voltage_regulator_micro_controller_battery_connect();
void force_sensor_battery_micro_controller_connect();
void servo_micro_controller_voltage_regulator_battery_connect();
void camera_voltage_regulator_micro_controller_battery_connect();
void bluetooth_battery_micro_controller_connect();
void encoder_battery_micro_controller_connect();
void motor_encoder_hbridge_battery_connect();

// SYSTEM TEST
void system_test();

#endif /* end of include guard: TESTFILE_H_7WN4LDYW */
