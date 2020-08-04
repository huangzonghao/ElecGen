#include "testfile.h"
using std::vector;
using std::make_pair;

void motor_read()
{
	initializeAllMotors(dc_motor_path);
	Motor dc_motor(dc_motor_path + "Pololu_4797.txt");
//	dc_motor.parameters();
}

void h_bridge_read()
{
	initializeAllHBridges(h_bridge_path);
	H_Bridge h_bridge(h_bridge_path + "DRV8838.txt");
//	h_bridge.parameters();
}

void battery_read()
{
	initializeAllBatteries(battery_path);
	Battery battery(battery_path + "D1604-1F.txt");
//	battery.parameters();
}

void micro_controller_read()
{
	initializeAllMicroController(micro_controller_path);
	Micro_Controller micro_controller(micro_controller_path + "Arduino_Mini.txt");
//	micro_controller.parameters();
}

void force_sensor_read()
{
	initializeAllForceSensors(forcesensor_path);
	Force_Sensor force_sensor(forcesensor_path + "Pololu_1645.txt");
//	force_sensor.parameters();
}

void voltage_regulator_read()
{
	initializeAllVoltageRegulators(voltage_regulator_path);
	Voltage_Regulator voltage_regulator(voltage_regulator_path + "DC-DC_Convertor.txt");
//	voltage_regulator.parameters();
}

void servo_read()
{
	initializeAllServos(servo_path);
	Motor servo_motor(servo_path + "LS-0009AF.txt");
//	servo_motor.parameters();
}

void camera_read()
{
	initializeAllServos(camera_path);
	Camera camera(camera_path + "MU_Vision_Sensor.txt");
//	camera.parameters();
}

void bluetooth_read()
{
	initializeAllBluetooths(bluetooth_path);
	Bluetooth bluetooth(bluetooth_path + "Bluefruit_SPI.txt");
//	bluetooth.parameters();
}

void motor_solve()
{
	Motor dc_motor(dc_motor_path + "Pololu_4797.txt");
	dc_motor.setWorkPoint(1e-3, 6.0);

	// SOLVE TEST
	GRBEnv env = GRBEnv();
	GRBModel model = GRBModel(env);
	vector<Electrical_Component*> components;
	components.push_back(&dc_motor);
	components[0]->getUsedVarsName(stringvec());
	Circuit test_circuit(components);
	test_circuit.verify(&model);
}

void motor_hbridge_solve()
{
	Motor dc_motor(dc_motor_path + "Pololu_4797.txt");
	dc_motor.setWorkPoint(1e-3, 6.0);

	GRBEnv env = GRBEnv();
	GRBModel model = GRBModel(env);
	vector<Electrical_Component*> components;
	components.push_back(&dc_motor);
	components[0]->getUsedVarsName(stringvec());
	Circuit test_circuit(components);
	test_circuit.verify(&model);
	test_circuit.maxSolve(&model);

	H_Bridge h_bridge(h_bridge_path + "DRV8835.txt");
	//"TA7291P.txt" *
	//"MTD-01.txt"
	//"DRV8838.txt"
	//"DRV8835.txt"

	// CONNECT TEST
	Electrical_Component *left_component = &h_bridge,
		*right_component = components[0];
	Component_Pair component_pair = make_pair(left_component,
		right_component);
	Pin_Connections pin_connection_list = groupMatch(
		vector<Component_Pair>{component_pair});
	printPinConnections(pin_connection_list);
	
	stringvec2d pins_vec(2);
	for (auto &beg = pin_connection_list.begin(); beg != 
		pin_connection_list.end(); beg++)
	{
		pins_vec[0].push_back(separateNames(beg->second).second);
		pins_vec[1].push_back(separateNames(beg->first).second);
	}

	// SOLVE TEST
	components.push_back(&h_bridge);
	components[1]->getUsedNonLinVarNames(pins_vec[1]);
	components[1]->getUsedVarsName(pins_vec[1]);

	test_circuit.updateComponents(vector<Electrical_Component*>{components[1]});
	test_circuit.updateConnections(pin_connection_list);
	test_circuit.updateVerify(&model);
}

void motor_hbridge_battery_solve()
{
	Motor dc_motor(dc_motor_path + "Pololu_4797.txt");
	dc_motor.setWorkPoint(1e-3, 6.0);

	GRBEnv env = GRBEnv();
	GRBModel model = GRBModel(env);
	vector<Electrical_Component*> components;
	components.push_back(&dc_motor);
	components[0]->getUsedVarsName(stringvec());
	Circuit test_circuit(components);
	test_circuit.verify(&model);
	test_circuit.maxSolve(&model);

	H_Bridge h_bridge(h_bridge_path + "DRV8835.txt");

	// CONNECT TEST
	Electrical_Component *left_component = &h_bridge,
		*right_component = components[0];
	Component_Pair component_pair = make_pair(left_component,
		right_component);
	Pin_Connections pin_connection_list = groupMatch(
		vector<Component_Pair>{component_pair});
	printPinConnections(pin_connection_list);

	stringvec2d pins_vec(2);
	for (auto &beg = pin_connection_list.begin(); beg !=
		pin_connection_list.end(); beg++)
	{
		pins_vec[0].push_back(separateNames(beg->second).second);
		pins_vec[1].push_back(separateNames(beg->first).second);
	}

	// SOLVE TEST
	components.push_back(&h_bridge);
	components[1]->getUsedNonLinVarNames(pins_vec[1]);
	components[1]->getUsedVarsName(pins_vec[1]);

	test_circuit.updateComponents(vector<Electrical_Component*>{components[1]});
	test_circuit.updateConnections(pin_connection_list);
	test_circuit.updateVerify(&model);
	test_circuit.maxSolve(&model);

	Battery battery(battery_path + "PR-CU-R198.txt");
	// "PR-CU-R198.txt"
	// "6LR61.txt"
	// "Turnigy_20C.txt"
	// "LP503562.txt"
	// "85AAAHCB.txt"
	// "D1604-1F.txt"

	// CONNECT TEST
	vector<Component_Pair> component_pairs;
	left_component = &battery;
	right_component = components[1];
	component_pair = make_pair(left_component,
		right_component);
	component_pairs.push_back(component_pair);

	pin_connection_list = groupMatch(component_pairs);
	printPinConnections(pin_connection_list);

	pins_vec[0].clear();
	pins_vec[1].clear();
	for (auto &beg = pin_connection_list.begin(); beg !=
		pin_connection_list.end(); beg++)
	{
		pins_vec[0].push_back(separateNames(beg->second).second);
		pins_vec[1].push_back(separateNames(beg->first).second);
	}

	// SOLVE TEST
	components.push_back(&battery);
	components[2]->getUsedNonLinVarNames(pins_vec[1]);
	components[2]->getUsedVarsName(pins_vec[1]);

	test_circuit.updateComponents(vector<Electrical_Component*>{components[2]});
	test_circuit.updateConnections(pin_connection_list);
	test_circuit.updateVerify(&model);
}

void motor_hbridge_battery_micro_controller_solve()
{
	Motor dc_motor(dc_motor_path + "Pololu_4797.txt");
	dc_motor.setWorkPoint(1e-3, 6.0);

	GRBEnv env = GRBEnv();
	GRBModel model = GRBModel(env);
	vector<Electrical_Component*> components;
	components.push_back(&dc_motor);
	components[0]->getUsedVarsName(stringvec());
	Circuit test_circuit(components);
	test_circuit.verify(&model);
	test_circuit.maxSolve(&model);

	H_Bridge h_bridge(h_bridge_path + "DRV8835.txt");

	// CONNECT TEST
	Electrical_Component *left_component = &h_bridge,
		*right_component = components[0];
	Component_Pair component_pair = make_pair(left_component,
		right_component);
	Pin_Connections pin_connection_list = groupMatch(
		vector<Component_Pair>{component_pair});
	printPinConnections(pin_connection_list);

	stringvec2d pins_vec(2);
	for (auto &beg = pin_connection_list.begin(); beg !=
		pin_connection_list.end(); beg++)
	{
		pins_vec[0].push_back(separateNames(beg->second).second);
		pins_vec[1].push_back(separateNames(beg->first).second);
	}

	// SOLVE TEST
	components.push_back(&h_bridge);
	components[1]->getUsedNonLinVarNames(pins_vec[1]);
	components[1]->getUsedVarsName(pins_vec[1]);

	test_circuit.updateComponents(vector<Electrical_Component*>{components[1]});
	test_circuit.updateConnections(pin_connection_list);
	test_circuit.updateVerify(&model);
	test_circuit.maxSolve(&model);

	Battery battery(battery_path + "Turnigy_20C.txt");
	// "PR-CU-R198.txt"
	// "6LR61.txt"
	// "Turnigy_20C.txt"
	// "LP503562.txt"
	// "85AAAHCB.txt"
	// "D1604-1F.txt"

	Micro_Controller micro_controller(micro_controller_path + "Arduino_Mega.txt");
	// "Arduino_Mini.txt"
	// "Arduino_Uno.txt"
	// "Arduino_Mega.txt"

	// CONNECT TEST
	components.push_back(&battery);
	components.push_back(&micro_controller);
	vector<Component_Pair> component_pairs;
	left_component = &battery;
	right_component = components[1];
	component_pair = make_pair(left_component,
		right_component);
	component_pairs.push_back(component_pair);

	left_component = &micro_controller;
	right_component = components[1];
	component_pair = make_pair(left_component,
		right_component);
	component_pairs.push_back(component_pair);

	pin_connection_list = groupMatch(component_pairs);
//	printPinConnections(pin_connection_list);

	pins_vec.clear();
	pins_vec.resize(2);
	// process connections
	for (auto &beg = pin_connection_list.begin(); beg != pin_connection_list.end();
		beg++)
	{
		stringpair left_pair = separateNames(beg->first),
			right_pair = separateNames(beg->second);
		for (size_t i = 2; i < components.size(); i++)
		{
			if (components[i]->getComponentName() == left_pair.first)
			{
				pins_vec[i-2].push_back(left_pair.second);
			}

			if (components[i]->getComponentName() == right_pair.first)
			{
				pins_vec[i-2].push_back(right_pair.second);
			}
		}
	}

	for (size_t i = 2; i < components.size(); i++)
	{
		components[i]->getUsedNonLinVarNames(pins_vec[i-2]);
		components[i]->getUsedVarsName(pins_vec[i-2]);
	}


	// CONNECT TEST
	component_pairs.clear();
	left_component = components[2];
	right_component = components[3];
	component_pair = make_pair(left_component,
		right_component);
	component_pairs.push_back(component_pair);

	Pin_Connections same_level_connection = groupMatch(component_pairs);
	pin_connection_list.insert(same_level_connection.begin(), same_level_connection.end());
	printPinConnections(pin_connection_list);

	test_circuit.updateComponents(vector<Electrical_Component*>{components[2], components[3]});
	test_circuit.updateConnections(pin_connection_list);
	test_circuit.updateVerify(&model);
	test_circuit.maxSolve(&model);
}

void motor_hbridge_voltage_regulator_micro_controller_solve()
{
}

void motor_hbridge_voltage_regulator_micro_controller_battery_solve()
{
}


void motor_hbridge_connect()
{
	Motor dc_motor(dc_motor_path + "Pololu_4797.txt");
	dc_motor.setWorkPoint(1e-3, 6.0);

	GRBEnv env = GRBEnv();
	GRBModel model = GRBModel(env);
	vector<Electrical_Component*> components;
	components.push_back(&dc_motor);
	components[0]->getUsedVarsName(stringvec());
	Circuit test_circuit(components);
	test_circuit.verify(&model);
	test_circuit.maxSolve(&model);

	H_Bridge h_bridge(h_bridge_path + "DRV8835.txt");
	//"TA7291P.txt" *
	//"MTD-01.txt"
	//"DRV8838.txt"
	//"DRV8835.txt"

	// CONNECT TEST
	Electrical_Component *left_component = &h_bridge,
		*right_component = components[0];
	Component_Pair component_pair = make_pair(left_component,
		right_component);
	Pin_Connections pin_connection_list = groupMatch(
		vector<Component_Pair>{component_pair});
	printPinConnections(pin_connection_list);
}

void motor_hbridge_battery_connect()
{
	Motor dc_motor(dc_motor_path + "Pololu_4797.txt");
	dc_motor.setWorkPoint(1e-3, 6.0);

	GRBEnv env = GRBEnv();
	GRBModel model = GRBModel(env);
	vector<Electrical_Component*> components;
	components.push_back(&dc_motor);
	components[0]->getUsedVarsName(stringvec());
	Circuit test_circuit(components);
	test_circuit.verify(&model);
	test_circuit.maxSolve(&model);

	H_Bridge h_bridge(h_bridge_path + "DRV8835.txt");

	// CONNECT TEST
	Electrical_Component *left_component = &h_bridge,
		*right_component = components[0];
	Component_Pair component_pair = make_pair(left_component,
		right_component);
	Pin_Connections pin_connection_list = groupMatch(
		vector<Component_Pair>{component_pair});
	printPinConnections(pin_connection_list);

	stringvec2d pins_vec(2);
	for (auto &beg = pin_connection_list.begin(); beg !=
		pin_connection_list.end(); beg++)
	{
		pins_vec[0].push_back(separateNames(beg->second).second);
		pins_vec[1].push_back(separateNames(beg->first).second);
	}

	// SOLVE TEST
	components.push_back(&h_bridge);
	components[1]->getUsedNonLinVarNames(pins_vec[1]);
	components[1]->getUsedVarsName(pins_vec[1]);

	test_circuit.updateComponents(vector<Electrical_Component*>{components[1]});
	test_circuit.updateConnections(pin_connection_list);
	test_circuit.updateVerify(&model);
	test_circuit.maxSolve(&model);

	Battery battery(battery_path + "D1604-1F.txt");
	// "PR-CU-R198.txt"
	// "6LR61.txt"
	// "Turnigy_20C.txt"
	// "LP503562.txt"
	// "85AAAHCB.txt"
	// "D1604-1F.txt"

	// CONNECT TEST
	vector<Component_Pair> component_pairs;
	left_component = &battery;
	right_component = components[1];
	component_pair = make_pair(left_component,
		right_component);
	component_pairs.push_back(component_pair);

	pin_connection_list = groupMatch(component_pairs);
	printPinConnections(pin_connection_list);
}

void motor_hbridge_battery_micro_controller_connect()
{
	Motor dc_motor(dc_motor_path + "Pololu_4797.txt");
	dc_motor.setWorkPoint(1e-3, 6.0);

	GRBEnv env = GRBEnv();
	GRBModel model = GRBModel(env);
	vector<Electrical_Component*> components;
	components.push_back(&dc_motor);
	components[0]->getUsedVarsName(stringvec());
	Circuit test_circuit(components);
	test_circuit.verify(&model);
	test_circuit.maxSolve(&model);

	H_Bridge h_bridge(h_bridge_path + "DRV8835.txt");

	// CONNECT TEST
	Electrical_Component *left_component = &h_bridge,
		*right_component = components[0];
	Component_Pair component_pair = make_pair(left_component,
		right_component);
	Pin_Connections pin_connection_list = groupMatch(
		vector<Component_Pair>{component_pair});
	printPinConnections(pin_connection_list);

	stringvec2d pins_vec(2);
	for (auto &beg = pin_connection_list.begin(); beg !=
		pin_connection_list.end(); beg++)
	{
		pins_vec[0].push_back(separateNames(beg->second).second);
		pins_vec[1].push_back(separateNames(beg->first).second);
	}

	// SOLVE TEST
	components.push_back(&h_bridge);
	components[1]->getUsedNonLinVarNames(pins_vec[1]);
	components[1]->getUsedVarsName(pins_vec[1]);

	test_circuit.updateComponents(vector<Electrical_Component*>{components[1]});
	test_circuit.updateConnections(pin_connection_list);
	test_circuit.updateVerify(&model);
	test_circuit.maxSolve(&model);

	Battery battery(battery_path + "LP503562.txt");
	// "PR-CU-R198.txt"
	// "6LR61.txt"
	// "Turnigy_20C.txt"
	// "LP503562.txt"
	// "85AAAHCB.txt"
	// "D1604-1F.txt"

	Micro_Controller micro_controller(micro_controller_path + "Arduino_Mini.txt");
	// "Arduino_Mini.txt"
	// "Arduino_Uno.txt"
	// "Arduino_Mega.txt"

	// CONNECT TEST
	components.push_back(&battery);
	components.push_back(&micro_controller);
	vector<Component_Pair> component_pairs;
	left_component = &battery;
	right_component = components[1];
	component_pair = make_pair(left_component,
		right_component);
	component_pairs.push_back(component_pair);

	left_component = &micro_controller;
	right_component = components[1];
	component_pair = make_pair(left_component,
		right_component);
	component_pairs.push_back(component_pair);

	pin_connection_list = groupMatch(component_pairs);
	//	printPinConnections(pin_connection_list);

	pins_vec.clear();
	pins_vec.resize(2);
	// process connections
	for (auto &beg = pin_connection_list.begin(); beg != pin_connection_list.end();
		beg++)
	{
		stringpair left_pair = separateNames(beg->first),
			right_pair = separateNames(beg->second);
		for (size_t i = 2; i < components.size(); i++)
		{
			if (components[i]->getComponentName() == left_pair.first)
			{
				pins_vec[i - 2].push_back(left_pair.second);
			}

			if (components[i]->getComponentName() == right_pair.first)
			{
				pins_vec[i - 2].push_back(right_pair.second);
			}
		}
	}

	for (size_t i = 2; i < components.size(); i++)
	{
		components[i]->getUsedNonLinVarNames(pins_vec[i - 2]);
		components[i]->getUsedVarsName(pins_vec[i - 2]);
	}


	// CONNECT TEST
	component_pairs.clear();
	left_component = components[2];
	right_component = components[3];
	component_pair = make_pair(left_component,
		right_component);
	component_pairs.push_back(component_pair);

	Pin_Connections same_level_connection = groupMatch(component_pairs);
	pin_connection_list.insert(same_level_connection.begin(), same_level_connection.end());
	printPinConnections(pin_connection_list);

}

void motor_hbridge_voltage_regulator_micro_controller_connect()
{
	Motor dc_motor(dc_motor_path + "Pololu_4797.txt");
	dc_motor.setWorkPoint(1e-3, 6.0);

	GRBEnv env = GRBEnv();
	GRBModel model = GRBModel(env);
	vector<Electrical_Component*> components;
	components.push_back(&dc_motor);
	components[0]->getUsedVarsName(stringvec());
	Circuit test_circuit(components);
	test_circuit.verify(&model);
	test_circuit.maxSolve(&model);

	H_Bridge h_bridge(h_bridge_path + "DRV8835.txt");

	// CONNECT TEST
	Electrical_Component *left_component = &h_bridge,
		*right_component = components[0];
	Component_Pair component_pair = make_pair(left_component,
		right_component);
	Pin_Connections pin_connection_list = groupMatch(
		vector<Component_Pair>{component_pair});
	printPinConnections(pin_connection_list);

	stringvec2d pins_vec(2);
	for (auto &beg = pin_connection_list.begin(); beg !=
		pin_connection_list.end(); beg++)
	{
		pins_vec[0].push_back(separateNames(beg->second).second);
		pins_vec[1].push_back(separateNames(beg->first).second);
	}

	// SOLVE TEST
	components.push_back(&h_bridge);
	components[1]->getUsedNonLinVarNames(pins_vec[1]);
	components[1]->getUsedVarsName(pins_vec[1]);

	test_circuit.updateComponents(vector<Electrical_Component*>{components[1]});
	test_circuit.updateConnections(pin_connection_list);
	test_circuit.updateVerify(&model);
	test_circuit.maxSolve(&model);

	Voltage_Regulator voltage_regulator(voltage_regulator_path + "DC-DC_Convertor.txt");
	// "DC-DC_Convertor.txt"
	// "L4931-33.txt"


	Micro_Controller micro_controller(micro_controller_path + "Arduino_Mega.txt");
	// "Arduino_Mini.txt"
	// "Arduino_Uno.txt"
	// "Arduino_Mega.txt"

	// CONNECT TEST
	components.push_back(&voltage_regulator);
	components.push_back(&micro_controller);
	vector<Component_Pair> component_pairs;
	left_component = &voltage_regulator;
	right_component = components[1];
	component_pair = make_pair(left_component,
		right_component);
	component_pairs.push_back(component_pair);

	left_component = &micro_controller;
	right_component = components[1];
	component_pair = make_pair(left_component,
		right_component);
	component_pairs.push_back(component_pair);

	pin_connection_list = groupMatch(component_pairs);
//	printPinConnections(pin_connection_list);

	pins_vec.clear();
	pins_vec.resize(2);
	// process connections
	for (auto &beg = pin_connection_list.begin(); beg != pin_connection_list.end();
		beg++)
	{
		stringpair left_pair = separateNames(beg->first),
			right_pair = separateNames(beg->second);
		for (size_t i = 2; i < components.size(); i++)
		{
			if (components[i]->getComponentName() == left_pair.first)
			{
				pins_vec[i - 2].push_back(left_pair.second);
			}

			if (components[i]->getComponentName() == right_pair.first)
			{
				pins_vec[i - 2].push_back(right_pair.second);
			}
		}
	}

	for (size_t i = 2; i < components.size(); i++)
	{
		components[i]->getUsedNonLinVarNames(pins_vec[i - 2]);
		components[i]->getUsedVarsName(pins_vec[i - 2]);
	}


	// CONNECT TEST
	component_pairs.clear();
	left_component = components[2];
	right_component = components[3];
	component_pair = make_pair(left_component,
		right_component);
	component_pairs.push_back(component_pair);

	Pin_Connections same_level_connection = groupMatch(component_pairs);
	pin_connection_list.insert(same_level_connection.begin(), same_level_connection.end());
	printPinConnections(pin_connection_list);

	test_circuit.updateComponents(vector<Electrical_Component*>{components[2], components[3]});
	test_circuit.updateConnections(pin_connection_list);
	test_circuit.updateVerify(&model);
	test_circuit.maxSolve(&model);
}

void motor_hbridge_voltage_regulator_micro_controller_battery_connect()
{
	Motor dc_motor(dc_motor_path + "Pololu_4797.txt");
	dc_motor.setWorkPoint(1e-3, 6.0);

	GRBEnv env = GRBEnv();
	GRBModel model = GRBModel(env);
	vector<Electrical_Component*> components;
	components.push_back(&dc_motor);
	components[0]->getUsedVarsName(stringvec());
	Circuit test_circuit(components);
	test_circuit.verify(&model);
	test_circuit.maxSolve(&model);

	H_Bridge h_bridge(h_bridge_path + "DRV8835.txt");

	// CONNECT TEST
	Electrical_Component *left_component = &h_bridge,
		*right_component = components[0];
	Component_Pair component_pair = make_pair(left_component,
		right_component);
	Pin_Connections pin_connection_list = groupMatch(
		vector<Component_Pair>{component_pair});
	printPinConnections(pin_connection_list);

	stringvec2d pins_vec(2);
	for (auto &beg = pin_connection_list.begin(); beg !=
		pin_connection_list.end(); beg++)
	{
		pins_vec[0].push_back(separateNames(beg->second).second);
		pins_vec[1].push_back(separateNames(beg->first).second);
	}

	// SOLVE TEST
	components.push_back(&h_bridge);
	components[1]->getUsedNonLinVarNames(pins_vec[1]);
	components[1]->getUsedVarsName(pins_vec[1]);

	test_circuit.updateComponents(vector<Electrical_Component*>{components[1]});
	test_circuit.updateConnections(pin_connection_list);
	test_circuit.updateVerify(&model);
	test_circuit.maxSolve(&model);

	Voltage_Regulator voltage_regulator(voltage_regulator_path + "DC-DC_Convertor.txt");
	// "DC-DC_Convertor.txt"
	// "L4931-33.txt"


	Micro_Controller micro_controller(micro_controller_path + "Arduino_Mega.txt");
	// "Arduino_Mini.txt"
	// "Arduino_Uno.txt"
	// "Arduino_Mega.txt"

	// CONNECT TEST
	components.push_back(&voltage_regulator);
	components.push_back(&micro_controller);
	vector<Component_Pair> component_pairs;
	left_component = &voltage_regulator;
	right_component = components[1];
	component_pair = make_pair(left_component,
		right_component);
	component_pairs.push_back(component_pair);

	left_component = &micro_controller;
	right_component = components[1];
	component_pair = make_pair(left_component,
		right_component);
	component_pairs.push_back(component_pair);

	pin_connection_list = groupMatch(component_pairs);
	//	printPinConnections(pin_connection_list);

	pins_vec.clear();
	pins_vec.resize(2);
	// process connections
	for (auto &beg = pin_connection_list.begin(); beg != pin_connection_list.end();
		beg++)
	{
		stringpair left_pair = separateNames(beg->first),
			right_pair = separateNames(beg->second);
		for (size_t i = 2; i < components.size(); i++)
		{
			if (components[i]->getComponentName() == left_pair.first)
			{
				pins_vec[i - 2].push_back(left_pair.second);
			}

			if (components[i]->getComponentName() == right_pair.first)
			{
				pins_vec[i - 2].push_back(right_pair.second);
			}
		}
	}

	for (size_t i = 2; i < components.size(); i++)
	{
		components[i]->getUsedNonLinVarNames(pins_vec[i - 2]);
		components[i]->getUsedVarsName(pins_vec[i - 2]);
	}


	// CONNECT TEST
	component_pairs.clear();
	left_component = components[2];
	right_component = components[3];
	component_pair = make_pair(left_component,
		right_component);
	component_pairs.push_back(component_pair);

	Pin_Connections same_level_connection = groupMatch(component_pairs);
	pin_connection_list.insert(same_level_connection.begin(), same_level_connection.end());
	printPinConnections(pin_connection_list);

	test_circuit.updateComponents(vector<Electrical_Component*>{components[2], components[3]});
	test_circuit.updateConnections(pin_connection_list);
	test_circuit.updateVerify(&model);
	test_circuit.maxSolve(&model);

	Battery battery(battery_path + "D1604-1F.txt");

	// CONNECT TEST
	components.push_back(&battery);
	component_pairs.clear();
	left_component = &battery;
	right_component = components[2];
	component_pair = make_pair(left_component,
		right_component);
	component_pairs.push_back(component_pair);

	left_component = &battery;
	right_component = components[3];
	component_pair = make_pair(left_component,
		right_component);
	component_pairs.push_back(component_pair);

	pin_connection_list = groupMatch(component_pairs);
	printPinConnections(pin_connection_list);

	pins_vec.clear();
	pins_vec.resize(1);
	// process connections
	for (auto &beg = pin_connection_list.begin(); beg != pin_connection_list.end();
		beg++)
	{
		stringpair left_pair = separateNames(beg->first),
			right_pair = separateNames(beg->second);
		for (size_t i = 4; i < components.size(); i++)
		{
			if (components[i]->getComponentName() == left_pair.first)
			{
				pins_vec[i - 4].push_back(left_pair.second);
			}

			if (components[i]->getComponentName() == right_pair.first)
			{
				pins_vec[i - 4].push_back(right_pair.second);
			}
		}
	}

	for (size_t i = 4; i < components.size(); i++)
	{
		components[i]->getUsedNonLinVarNames(pins_vec[i - 4]);
		components[i]->getUsedVarsName(pins_vec[i - 4]);
	}

	test_circuit.updateComponents(vector<Electrical_Component*>{components[4]});
	test_circuit.updateConnections(pin_connection_list);
	test_circuit.updateVerify(&model);
	test_circuit.maxSolve(&model);
}


void force_sensor_battery_micro_controller_connect()
{
	Force_Sensor force_sensor(forcesensor_path + "Pololu_1645.txt");

	GRBEnv env = GRBEnv();
	GRBModel model = GRBModel(env);
	vector<Electrical_Component*> components;
	components.push_back(&force_sensor);
	components[0]->getUsedVarsName(stringvec());
	Circuit test_circuit(components);
	test_circuit.verify(&model);
	test_circuit.maxSolve(&model);

	Battery battery(battery_path + "Turnigy_20C.txt");
	components.push_back(&battery);
	// "PR-CU-R198.txt"
	// "6LR61.txt"
	// "Turnigy_20C.txt"
	// "LP503562.txt"
	// "85AAAHCB.txt"
	// "D1604-1F.txt"

	Micro_Controller micro_controller(micro_controller_path + "Arduino_Mini.txt");
	components.push_back(&micro_controller);
	// "Arduino_Mini.txt"
	// "Arduino_Uno.txt"
	// "Arduino_Mega.txt"

	// CONNECT TEST
	vector<Component_Pair> component_pairs;
	Electrical_Component *left_component = &battery,
		*right_component = components[0];
	Component_Pair component_pair = make_pair(left_component,
		right_component);
	component_pairs.push_back(component_pair);

	left_component = components[0];
	right_component = &micro_controller;
	component_pair = make_pair(left_component,
		right_component);
	component_pairs.push_back(component_pair);

	Pin_Connections pin_connection_list = groupMatch(component_pairs);
//	printPinConnections(pin_connection_list);

	stringvec2d pins_vec(2);
	// process connections
	for (auto &beg = pin_connection_list.begin(); beg != pin_connection_list.end();
		beg++)
	{
		stringpair left_pair = separateNames(beg->first),
			right_pair = separateNames(beg->second);
		for (size_t i = 1; i < components.size(); i++)
		{
			if (components[i]->getComponentName() == left_pair.first)
			{
				pins_vec[i - 1].push_back(left_pair.second);
			}

			if (components[i]->getComponentName() == right_pair.first)
			{
				pins_vec[i - 1].push_back(right_pair.second);
			}
		}
	}

	for (size_t i = 1; i < components.size(); i++)
	{
		components[i]->getUsedNonLinVarNames(pins_vec[i - 1]);
		components[i]->getUsedVarsName(pins_vec[i - 1]);
	}

	component_pairs.clear();
	left_component = components[1];
	right_component = components[2];
	component_pair = make_pair(left_component, right_component);
	component_pairs.push_back(component_pair);

	Pin_Connections same_level_connection = groupMatch(component_pairs);
	pin_connection_list.insert(same_level_connection.begin(), same_level_connection.end());
	printPinConnections(pin_connection_list);

	test_circuit.updateComponents(vector<Electrical_Component*>{components[1], components[2]});
	test_circuit.updateConnections(pin_connection_list);
	test_circuit.updateVerify(&model);
	test_circuit.maxSolve(&model);
}

void servo_micro_controller_voltage_regulator_battery_connect()
{
	Motor servo(servo_path + "LS-0009AF.txt");
	servo.setWorkPoint(1e-3, 10);

	GRBEnv env = GRBEnv();
	GRBModel model = GRBModel(env);
	vector<Electrical_Component*> components;
	components.push_back(&servo);
	components[0]->getUsedVarsName(stringvec());
	Circuit test_circuit(components);
	test_circuit.verify(&model);
	test_circuit.maxSolve(&model);

	Voltage_Regulator voltage_regulator(voltage_regulator_path + "DC-DC_Convertor.txt");
	Micro_Controller micro_controller(micro_controller_path + "Arduino_Mini.txt");
	components.push_back(&voltage_regulator);
	components.push_back(&micro_controller);

	// CONNECT TEST
	vector<Component_Pair> component_pairs;
	Electrical_Component *left_component = &voltage_regulator ,
		*right_component = components[0];
	Component_Pair component_pair = make_pair(left_component,
		right_component);
	component_pairs.push_back(component_pair);

	left_component = components[0];
	right_component = &micro_controller;
	component_pair = make_pair(left_component, right_component);
	component_pairs.push_back(component_pair);

	Pin_Connections pin_connection_list = groupMatch(component_pairs);
//	printPinConnections(pin_connection_list);

	stringvec2d pins_vec(2);
	// process connections
	for (auto &beg = pin_connection_list.begin(); beg != pin_connection_list.end();
		beg++)
	{
		stringpair left_pair = separateNames(beg->first),
			right_pair = separateNames(beg->second);
		for (size_t i = 1; i < components.size(); i++)
		{
			if (components[i]->getComponentName() == left_pair.first)
			{
				pins_vec[i - 1].push_back(left_pair.second);
			}

			if (components[i]->getComponentName() == right_pair.first)
			{
				pins_vec[i - 1].push_back(right_pair.second);
			}
		}
	}

	for (size_t i = 1; i < components.size(); i++)
	{
		components[i]->getUsedNonLinVarNames(pins_vec[i - 1]);
		components[i]->getUsedVarsName(pins_vec[i - 1]);
	}

	component_pairs.clear();
	left_component = components[1];
	right_component = components[2];
	component_pair = make_pair(left_component, right_component);
	component_pairs.push_back(component_pair);

	Pin_Connections same_level_connection = groupMatch(component_pairs);
	pin_connection_list.insert(same_level_connection.begin(), same_level_connection.end());
	printPinConnections(pin_connection_list);

	test_circuit.updateComponents(vector<Electrical_Component*>{components[1], components[2]});
	test_circuit.updateConnections(pin_connection_list);
	test_circuit.updateVerify(&model);
	test_circuit.maxSolve(&model);

	Battery battery(battery_path + "D1604-1F.txt");
	components.push_back(&battery);

	// CONNECT TEST
	component_pairs.clear();
	left_component = &battery;
	right_component = components[1];
	component_pair = make_pair(left_component,
		right_component);
	component_pairs.push_back(component_pair);

	left_component = &battery;
	right_component = components[2];
	component_pair = make_pair(left_component,
		right_component);
	component_pairs.push_back(component_pair);

	pin_connection_list = groupMatch(component_pairs);
	printPinConnections(pin_connection_list);

	pins_vec.clear();
	pins_vec.resize(1);
	// process connections
	for (auto &beg = pin_connection_list.begin(); beg != pin_connection_list.end();
		beg++)
	{
		stringpair left_pair = separateNames(beg->first),
			right_pair = separateNames(beg->second);
		for (size_t i = 3; i < components.size(); i++)
		{
			if (components[i]->getComponentName() == left_pair.first)
			{
				pins_vec[i - 3].push_back(left_pair.second);
			}

			if (components[i]->getComponentName() == right_pair.first)
			{
				pins_vec[i - 3].push_back(right_pair.second);
			}
		}
	}

	for (size_t i = 3; i < components.size(); i++)
	{
		components[i]->getUsedNonLinVarNames(pins_vec[i - 3]);
		components[i]->getUsedVarsName(pins_vec[i - 3]);
	}

	test_circuit.updateComponents(vector<Electrical_Component*>{components[3]});
	test_circuit.updateConnections(pin_connection_list);
	test_circuit.updateVerify(&model);
	test_circuit.maxSolve(&model);
}

void camera_voltage_regulator_micro_controller_battery_connect()
{
	Camera camera(camera_path + "TTL_Serial_Camera.txt");
	// TTL_Serial_Camera

	GRBEnv env = GRBEnv();
	GRBModel model = GRBModel(env);
	vector<Electrical_Component*> components;
	components.push_back(&camera);
	components[0]->getUsedVarsName(stringvec());
	Circuit test_circuit(components);
	test_circuit.verify(&model);
	test_circuit.maxSolve(&model);

	Voltage_Regulator voltage_regulator(voltage_regulator_path + "DC-DC_Convertor.txt");
	Micro_Controller micro_controller(micro_controller_path + "Arduino_Mega.txt");
	components.push_back(&voltage_regulator);
	components.push_back(&micro_controller);

	// CONNECT TEST
	vector<Component_Pair> component_pairs;
	Electrical_Component *left_component = &voltage_regulator,
		*right_component = components[0];
	Component_Pair component_pair = make_pair(left_component,
		right_component);
	component_pairs.push_back(component_pair);

	left_component = components[0];
	right_component = &micro_controller;
	component_pair = make_pair(left_component, right_component);
	component_pairs.push_back(component_pair);

	Pin_Connections pin_connection_list = groupMatch(component_pairs);
//	printPinConnections(pin_connection_list);

	stringvec2d pins_vec(2);
	// process connections
	for (auto &beg = pin_connection_list.begin(); beg != pin_connection_list.end();
		beg++)
	{
		stringpair left_pair = separateNames(beg->first),
			right_pair = separateNames(beg->second);
		for (size_t i = 1; i < components.size(); i++)
		{
			if (components[i]->getComponentName() == left_pair.first)
			{
				pins_vec[i - 1].push_back(left_pair.second);
			}

			if (components[i]->getComponentName() == right_pair.first)
			{
				pins_vec[i - 1].push_back(right_pair.second);
			}
		}
	}

	for (size_t i = 1; i < components.size(); i++)
	{
		components[i]->getUsedNonLinVarNames(pins_vec[i - 1]);
		components[i]->getUsedVarsName(pins_vec[i - 1]);
	}

	component_pairs.clear();
	left_component = components[1];
	right_component = components[2];
	component_pair = make_pair(left_component, right_component);
	component_pairs.push_back(component_pair);

	Pin_Connections same_level_connection = groupMatch(component_pairs);
	pin_connection_list.insert(same_level_connection.begin(), same_level_connection.end());
	printPinConnections(pin_connection_list);

	test_circuit.updateComponents(vector<Electrical_Component*>{components[1], components[2]});
	test_circuit.updateConnections(pin_connection_list);
	test_circuit.updateVerify(&model);
	test_circuit.maxSolve(&model);

	Battery battery(battery_path + "D1604-1F.txt");
	components.push_back(&battery);

	// CONNECT TEST
	component_pairs.clear();
	left_component = &battery;
	right_component = components[1];
	component_pair = make_pair(left_component,
		right_component);
	component_pairs.push_back(component_pair);

	left_component = &battery;
	right_component = components[2];
	component_pair = make_pair(left_component,
		right_component);
	component_pairs.push_back(component_pair);

	pin_connection_list = groupMatch(component_pairs);
	printPinConnections(pin_connection_list);

	pins_vec.clear();
	pins_vec.resize(1);
	// process connections
	for (auto &beg = pin_connection_list.begin(); beg != pin_connection_list.end();
		beg++)
	{
		stringpair left_pair = separateNames(beg->first),
			right_pair = separateNames(beg->second);
		for (size_t i = 3; i < components.size(); i++)
		{
			if (components[i]->getComponentName() == left_pair.first)
			{
				pins_vec[i - 3].push_back(left_pair.second);
			}

			if (components[i]->getComponentName() == right_pair.first)
			{
				pins_vec[i - 3].push_back(right_pair.second);
			}
		}
	}

	for (size_t i = 3; i < components.size(); i++)
	{
		components[i]->getUsedNonLinVarNames(pins_vec[i - 3]);
		components[i]->getUsedVarsName(pins_vec[i - 3]);
	}

	test_circuit.updateComponents(vector<Electrical_Component*>{components[3]});
	test_circuit.updateConnections(pin_connection_list);
	test_circuit.updateVerify(&model);
	test_circuit.maxSolve(&model);
}

void bluetooth_battery_micro_controller_connect()
{
	Bluetooth bluetooth(bluetooth_path + "Bluefruit_UART.txt");
	// Bluefruit_UART.txt

	GRBEnv env = GRBEnv();
	GRBModel model = GRBModel(env);
	vector<Electrical_Component*> components;
	components.push_back(&bluetooth);
	components[0]->getUsedVarsName(stringvec());
	Circuit test_circuit(components);
	test_circuit.verify(&model);
	test_circuit.maxSolve(&model);

	Battery battery(battery_path + "D1604-1F.txt");
	Micro_Controller micro_controller(micro_controller_path + "Arduino_Mega.txt");
	components.push_back(&battery);
	components.push_back(&micro_controller);

	// CONNECT TEST
	vector<Component_Pair> component_pairs;
	Electrical_Component *left_component = &battery,
		*right_component = components[0];	
	Component_Pair component_pair = make_pair(left_component,
		right_component);
	component_pairs.push_back(component_pair);

	left_component = components[0];
	right_component = &micro_controller;
	component_pair = make_pair(left_component, right_component);
	component_pairs.push_back(component_pair);

	Pin_Connections pin_connection_list = groupMatch(component_pairs);
//	printPinConnections(pin_connection_list);

	stringvec2d pins_vec(2);
	// process connections
	for (auto &beg = pin_connection_list.begin(); beg != pin_connection_list.end();
		beg++)
	{
		stringpair left_pair = separateNames(beg->first),
			right_pair = separateNames(beg->second);
		for (size_t i = 1; i < components.size(); i++)
		{
			if (components[i]->getComponentName() == left_pair.first)
			{
				pins_vec[i - 1].push_back(left_pair.second);
			}

			if (components[i]->getComponentName() == right_pair.first)
			{
				pins_vec[i - 1].push_back(right_pair.second);
			}
		}
	}

	for (size_t i = 1; i < components.size(); i++)
	{
		components[i]->getUsedNonLinVarNames(pins_vec[i - 1]);
		components[i]->getUsedVarsName(pins_vec[i - 1]);
	}

	component_pairs.clear();
	left_component = components[1];
	right_component = components[2];
	component_pair = make_pair(left_component, right_component);
	component_pairs.push_back(component_pair);

	Pin_Connections same_level_connection = groupMatch(component_pairs);
	pin_connection_list.insert(same_level_connection.begin(), same_level_connection.end());
	printPinConnections(pin_connection_list);

	test_circuit.updateComponents(vector<Electrical_Component*>{components[1], components[2]});
	test_circuit.updateConnections(pin_connection_list);
	test_circuit.updateVerify(&model);
	test_circuit.maxSolve(&model);
}

void system_test()
{
	Motor dc_motor(dc_motor_path + "Pololu_4797.txt");
	dc_motor.setWorkPoint(1e-3, 6.0);

	Infer_Node infer_node(&dc_motor);
	BBNode root(infernodevec{ infer_node });
	branchNBound(bbnodevec{ root });

}
