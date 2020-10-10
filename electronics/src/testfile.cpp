#include "testfile.h"
using std::vector;
using std::make_pair;
using std::shared_ptr;
using std::make_shared;

void motor_read()
{
	initializeAllMotors(dc_motor_path);
//	Motor dc_motor(dc_motor_path + "Pololu_4797.txt");
//	dc_motor.parameters();
}

void h_bridge_read()
{
	initializeAllHBridges(h_bridge_path);
//	H_Bridge h_bridge(h_bridge_path + "MTD-01.txt");
//	h_bridge.parameters();
}

void battery_read()
{
	initializeAllBatteries(battery_path);
//	Battery battery(battery_path + "85AAAHCB.txt");
//	battery.parameters();
}

void micro_controller_read()
{
	initializeAllMicroController(micro_controller_path);
//	Micro_Controller micro_controller(micro_controller_path + "Arduino_Mini.txt");
//	micro_controller.parameters();
}

void force_sensor_read()
{
	initializeAllForceSensors(forcesensor_path);
//	Force_Sensor force_sensor(forcesensor_path + "Pololu_1645.txt");
//	force_sensor.parameters();
}

void voltage_regulator_read()
{
	initializeAllVoltageRegulators(voltage_regulator_path);
//	Voltage_Regulator voltage_regulator(voltage_regulator_path + "DC-DC_Convertor.txt");
//	voltage_regulator.parameters();
}

void servo_read()
{
	initializeAllServos(servo_path);
//	Motor servo_motor(servo_path + "LS-0009AF.txt");
//	servo_motor.parameters();
}

void camera_read()
{
	initializeAllCameras(camera_path);
//	Camera camera(camera_path + "TTL_Serial_Camera.txt");
//	camera.parameters();
}

void bluetooth_read()
{
	initializeAllBluetooths(bluetooth_path);
//	Bluetooth bluetooth(bluetooth_path + "Bluefruit_SPI.txt");
//	bluetooth.parameters();
}

void encoder_read()
{
	initializeAllEncoders(encoder_path);
//	Encoder encoder(encoder_path + "Pololu_4761.txt");
//	encoder.parameters();
}

void motor_solve()
{
	shared_ptr<Motor> dc_motor = make_shared<Motor>(dc_motor_path + "Pololu_4797.txt");
	dc_motor->setWorkPoint(1e-3, 6.0);

	// SOLVE TEST
	GRBEnv env = GRBEnv();
	GRBModel model = GRBModel(env);
	vector<shared_ptr<Electrical_Component>> components;
	components.push_back(dc_motor);
	components[0]->getUsedVarsName(stringvec());
	Circuit test_circuit(components);
	test_circuit.verify(&model);
}


void motor_hbridge_solve()
{
	shared_ptr<Motor> dc_motor = make_shared<Motor>(dc_motor_path + "Pololu_4797.txt");
	dc_motor->setWorkPoint(1e-3, 6.0);

	GRBEnv env = GRBEnv();
	GRBModel model = GRBModel(env);
	vector<shared_ptr<Electrical_Component>> components;
	components.push_back(dc_motor);
	components[0]->getUsedVarsName(stringvec());
	Circuit test_circuit(components);
	test_circuit.verify(&model);
	test_circuit.maxSolve(&model);

	shared_ptr<H_Bridge> h_bridge = make_shared<H_Bridge>(h_bridge_path + "DRV8835.txt");
	components.push_back(h_bridge);
	//"TA7291P.txt" *
	//"MTD-01.txt"
	//"DRV8838.txt"
	//"DRV8835.txt"

	// CONNECT TEST
	shared_ptr<Electrical_Component> left_component = components[1],
		right_component = components[0];
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
	components[1]->getUsedNonLinVarNames(pins_vec[1]);
	components[1]->getUsedVarsName(pins_vec[1]);

	test_circuit.updateComponents(vector<shared_ptr<Electrical_Component>>{components[1]});
	test_circuit.updateConnections(pin_connection_list);
	test_circuit.updateVerify(&model);
}

void motor_hbridge_battery_solve()
{
	shared_ptr<Motor> dc_motor = make_shared<Motor>(dc_motor_path + "Pololu_4797.txt");
	dc_motor->setWorkPoint(1e-3, 6.0);

	GRBEnv env = GRBEnv();
	GRBModel model = GRBModel(env);
	vector<shared_ptr<Electrical_Component>> components;
	components.push_back(dc_motor);
	components[0]->getUsedVarsName(stringvec());
	Circuit test_circuit(components);
	test_circuit.verify(&model);
	test_circuit.maxSolve(&model);

	shared_ptr<H_Bridge> h_bridge = make_shared<H_Bridge>(h_bridge_path + "DRV8835.txt");
	components.push_back(h_bridge);
	// CONNECT TEST
	shared_ptr<Electrical_Component> left_component = components[1],
		right_component = components[0];
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
	components[1]->getUsedNonLinVarNames(pins_vec[1]);
	components[1]->getUsedVarsName(pins_vec[1]);

	test_circuit.updateComponents(vector<shared_ptr<Electrical_Component>>{components[1]});
	test_circuit.updateConnections(pin_connection_list);
	test_circuit.updateVerify(&model);
	test_circuit.maxSolve(&model);

	shared_ptr<Battery> battery = make_shared<Battery>(battery_path + "PR-CU-R198.txt");
	components.push_back(battery);
	// "PR-CU-R198.txt"
	// "6LR61.txt"
	// "Turnigy_20C.txt"
	// "LP503562.txt"
	// "85AAAHCB.txt"
	// "D1604-1F.txt"

	// CONNECT TEST
	vector<Component_Pair> component_pairs;
	left_component = components[2];
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
	components[2]->getUsedNonLinVarNames(pins_vec[1]);
	components[2]->getUsedVarsName(pins_vec[1]);

	test_circuit.updateComponents(vector<shared_ptr<Electrical_Component>>{components[2]});
	test_circuit.updateConnections(pin_connection_list);
	test_circuit.updateVerify(&model);
}

void motor_hbridge_battery_micro_controller_solve()
{
	shared_ptr<Motor> dc_motor = make_shared<Motor>(dc_motor_path + "Pololu_4797.txt");
	dc_motor->setWorkPoint(1e-3, 6.0);

	GRBEnv env = GRBEnv();
	GRBModel model = GRBModel(env);
	vector<shared_ptr<Electrical_Component>> components;
	components.push_back(dc_motor);
	components[0]->getUsedVarsName(stringvec());
	Circuit test_circuit(components);
	test_circuit.verify(&model);
	test_circuit.maxSolve(&model);

	shared_ptr<H_Bridge> h_bridge = make_shared<H_Bridge>(h_bridge_path + "DRV8835.txt");
	components.push_back(h_bridge);

	// CONNECT TEST
	shared_ptr<Electrical_Component> left_component = components[1],
		right_component = components[0];
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
	components[1]->getUsedNonLinVarNames(pins_vec[1]);
	components[1]->getUsedVarsName(pins_vec[1]);

	test_circuit.updateComponents(vector<shared_ptr<Electrical_Component>>{components[1]});
	test_circuit.updateConnections(pin_connection_list);
	test_circuit.updateVerify(&model);
	test_circuit.maxSolve(&model);

	shared_ptr<Battery> battery = make_shared<Battery>(battery_path + "Turnigy_20C.txt");
	components.push_back(battery);
	// "PR-CU-R198.txt"
	// "6LR61.txt"
	// "Turnigy_20C.txt"
	// "LP503562.txt"
	// "85AAAHCB.txt"
	// "D1604-1F.txt"

	shared_ptr<Micro_Controller> micro_controller = 
		make_shared<Micro_Controller>(micro_controller_path + "Arduino_Mega.txt");
	components.push_back(micro_controller);
	// "Arduino_Mini.txt"
	// "Arduino_Uno.txt"
	// "Arduino_Mega.txt"

	// CONNECT TEST
	vector<Component_Pair> component_pairs;
	left_component = components[2];
	right_component = components[1];
	component_pair = make_pair(left_component,
		right_component);
	component_pairs.push_back(component_pair);

	left_component = components[3];
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

	test_circuit.updateComponents(vector<shared_ptr<Electrical_Component>>{components[2], components[3]});
	test_circuit.updateConnections(pin_connection_list);
	test_circuit.updateVerify(&model);
	test_circuit.maxSolve(&model);
}

void motor_hbridge_connect()
{
	shared_ptr<Motor> dc_motor = make_shared<Motor>(dc_motor_path + "Pololu_4797.txt");
	dc_motor->setWorkPoint(1e-3, 6.0);

	GRBEnv env = GRBEnv();
	GRBModel model = GRBModel(env);
	vector<shared_ptr<Electrical_Component>> components;
	components.push_back(dc_motor);
	components[0]->getUsedVarsName(stringvec());
	Circuit test_circuit(components);
	test_circuit.verify(&model);
	test_circuit.maxSolve(&model);

	shared_ptr<H_Bridge> h_bridge = make_shared<H_Bridge>(h_bridge_path + "DRV8835.txt");
	components.push_back(h_bridge);
	//"TA7291P.txt" *
	//"MTD-01.txt"
	//"DRV8838.txt"
	//"DRV8835.txt"

	// CONNECT TEST
	shared_ptr<Electrical_Component> left_component = components[1],
		right_component = components[0];
	Component_Pair component_pair = make_pair(left_component,
		right_component);
	Pin_Connections pin_connection_list = groupMatch(
		vector<Component_Pair>{component_pair});
	printPinConnections(pin_connection_list);
}

void motor_hbridge_battery_connect()
{
	shared_ptr<Motor> dc_motor = make_shared<Motor>(dc_motor_path + "Pololu_4797.txt");
	dc_motor->setWorkPoint(1e-3, 6.0);

	GRBEnv env = GRBEnv();
	GRBModel model = GRBModel(env);
	vector<shared_ptr<Electrical_Component>> components;
	components.push_back(dc_motor);
	components[0]->getUsedVarsName(stringvec());
	Circuit test_circuit(components);
	test_circuit.verify(&model);
	test_circuit.maxSolve(&model);

	shared_ptr<H_Bridge> h_bridge = make_shared<H_Bridge>(h_bridge_path + "DRV8835.txt");
	components.push_back(h_bridge);

	// CONNECT TEST
	shared_ptr<Electrical_Component> left_component = components[1],
		right_component = components[0];
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
	components[1]->getUsedNonLinVarNames(pins_vec[1]);
	components[1]->getUsedVarsName(pins_vec[1]);

	test_circuit.updateComponents(vector<shared_ptr<Electrical_Component>>{components[1]});
	test_circuit.updateConnections(pin_connection_list);
	test_circuit.updateVerify(&model);
	test_circuit.maxSolve(&model);

	shared_ptr<Battery> battery = make_shared<Battery>(battery_path + "D1604-1F.txt");
	components.push_back(battery);
	// "PR-CU-R198.txt"
	// "6LR61.txt"
	// "Turnigy_20C.txt"
	// "LP503562.txt"
	// "85AAAHCB.txt"
	// "D1604-1F.txt"

	// CONNECT TEST
	vector<Component_Pair> component_pairs;
	left_component = components[2];
	right_component = components[1];
	component_pair = make_pair(left_component,
		right_component);
	component_pairs.push_back(component_pair);

	pin_connection_list = groupMatch(component_pairs);
	printPinConnections(pin_connection_list);
}

void motor_hbridge_battery_micro_controller_connect()
{
	shared_ptr<Motor> dc_motor1 = make_shared<Motor>(dc_motor_path + "Pololu_4797.txt");
	dc_motor1->setWorkPoint(1e-3, 6.0);


	GRBEnv env = GRBEnv();
	GRBModel model = GRBModel(env);
	vector<shared_ptr<Electrical_Component>> components;
	components.push_back(dc_motor1);
	components[0]->getUsedVarsName(stringvec());
	Circuit test_circuit(components);
	test_circuit.verify(&model);
	test_circuit.maxSolve(&model);

	shared_ptr<H_Bridge> h_bridge1 = make_shared<H_Bridge>(h_bridge_path + "DRV8835.txt");

	// CONNECT TEST
	shared_ptr<Electrical_Component> left_component = components[4],
		right_component = components[0];
	Component_Pair component_pair = make_pair(left_component,
		right_component);
	vector<Component_Pair> component_pairs;
	component_pairs.push_back(component_pair);

	left_component = components[4], right_component = components[1];
	component_pairs.push_back(make_pair(left_component, right_component));

	left_component = components[5], right_component = components[2];
	component_pairs.push_back(make_pair(left_component, right_component));

	left_component = components[5], right_component = components[3];
	component_pairs.push_back(make_pair(left_component, right_component));

	Pin_Connections pin_connection_list = groupMatch(
		vector<Component_Pair>{component_pair});
	printPinConnections(pin_connection_list);

	stringvec2d pins_vec(2);
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

	test_circuit.updateComponents(vector<shared_ptr<Electrical_Component>>{components[4], components[5]});
	test_circuit.updateConnections(pin_connection_list);
	test_circuit.updateVerify(&model);
	test_circuit.maxSolve(&model);

	shared_ptr<Battery> battery = make_shared<Battery>(battery_path + "85AAAHCB.txt");
	components.push_back(battery);
	// "PR-CU-R198.txt"
	// "6LR61.txt"
	// "Turnigy_20C.txt"
	// "LP503562.txt"
	// "85AAAHCB.txt"
	// "D1604-1F.txt"

	shared_ptr<Micro_Controller> micro_controller = 
		make_shared<Micro_Controller>(micro_controller_path + "Arduino_Mini.txt");
	components.push_back(micro_controller);
	// "Arduino_Mini.txt"
	// "Arduino_Uno.txt"
	// "Arduino_Mega.txt"

	// CONNECT TEST
	component_pairs.clear();
	left_component = components[6];
	right_component = components[4];
	component_pair = make_pair(left_component,
		right_component);
	component_pairs.push_back(component_pair);

	left_component = components[6];
	right_component = components[5];
	component_pair = make_pair(left_component,
		right_component);
	component_pairs.push_back(component_pair);

	left_component = components[7];
	right_component = components[4];
	component_pair = make_pair(left_component,
		right_component);
	component_pairs.push_back(component_pair);

	left_component = components[7];
	right_component = components[5];
	component_pair = make_pair(left_component,
		right_component);
	component_pairs.push_back(component_pair);

	pin_connection_list = groupMatch(component_pairs);
	printPinConnections(pin_connection_list);

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
	shared_ptr<Motor> dc_motor = make_shared<Motor>(dc_motor_path + "Pololu_4797.txt");
	dc_motor->setWorkPoint(1e-3, 6.0);

	GRBEnv env = GRBEnv();
	GRBModel model = GRBModel(env);
	vector<shared_ptr<Electrical_Component>> components;
	components.push_back(dc_motor);
	components[0]->getUsedVarsName(stringvec());
	Circuit test_circuit(components);
	test_circuit.verify(&model);
	test_circuit.maxSolve(&model);

	shared_ptr<H_Bridge> h_bridge = make_shared<H_Bridge>(h_bridge_path + "DRV8835.txt");
	components.push_back(h_bridge);

	// CONNECT TEST
	shared_ptr<Electrical_Component> left_component = components[1],
		right_component = components[0];
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
	components[1]->getUsedNonLinVarNames(pins_vec[1]);
	components[1]->getUsedVarsName(pins_vec[1]);

	test_circuit.updateComponents(vector<shared_ptr<Electrical_Component>>{components[1]});
	test_circuit.updateConnections(pin_connection_list);
	test_circuit.updateVerify(&model);
	test_circuit.maxSolve(&model);

	shared_ptr<Voltage_Regulator> voltage_regulator = 
		make_shared<Voltage_Regulator>(voltage_regulator_path + "DC-DC_Convertor.txt");
	components.push_back(voltage_regulator);
	// "DC-DC_Convertor.txt"
	// "L4931-33.txt"


	shared_ptr<Micro_Controller> micro_controller = 
		make_shared<Micro_Controller>(micro_controller_path + "Arduino_Mega.txt");
	components.push_back(micro_controller);
	// "Arduino_Mini.txt"
	// "Arduino_Uno.txt"
	// "Arduino_Mega.txt"

	// CONNECT TEST
	vector<Component_Pair> component_pairs;
	left_component = components[2];
	right_component = components[1];
	component_pair = make_pair(left_component,
		right_component);
	component_pairs.push_back(component_pair);

	left_component = components[3];
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

	test_circuit.updateComponents(vector<shared_ptr<Electrical_Component>>{components[2], components[3]});
	test_circuit.updateConnections(pin_connection_list);
	test_circuit.updateVerify(&model);
	test_circuit.maxSolve(&model);
}

void motor_hbridge_voltage_regulator_micro_controller_battery_connect()
{
	shared_ptr<Motor> dc_motor = make_shared<Motor>(dc_motor_path + "Pololu_4797.txt");
	dc_motor->setWorkPoint(1e-3, 6.0);

	GRBEnv env = GRBEnv();
	GRBModel model = GRBModel(env);
	vector<shared_ptr<Electrical_Component>> components;
	components.push_back(dc_motor);
	components[0]->getUsedVarsName(stringvec());
	Circuit test_circuit(components);
	test_circuit.verify(&model);
	test_circuit.maxSolve(&model);

	shared_ptr<H_Bridge> h_bridge = make_shared<H_Bridge>(h_bridge_path + "DRV8835.txt");
	components.push_back(h_bridge);

	// CONNECT TEST
	shared_ptr<Electrical_Component> left_component = components[1],
		right_component = components[0];
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
	components[1]->getUsedNonLinVarNames(pins_vec[1]);
	components[1]->getUsedVarsName(pins_vec[1]);

	test_circuit.updateComponents(vector<shared_ptr<Electrical_Component>>{components[1]});
	test_circuit.updateConnections(pin_connection_list);
	test_circuit.updateVerify(&model);
	test_circuit.maxSolve(&model);

	shared_ptr<Voltage_Regulator> voltage_regulator = 
		make_shared<Voltage_Regulator>(voltage_regulator_path + "DC-DC_Convertor.txt");
	components.push_back(voltage_regulator);
	// "DC-DC_Convertor.txt"
	// "L4931-33.txt"


	shared_ptr<Micro_Controller> micro_controller = 
		make_shared<Micro_Controller>(micro_controller_path + "Arduino_Mega.txt");
	components.push_back(micro_controller);
	// "Arduino_Mini.txt"
	// "Arduino_Uno.txt"
	// "Arduino_Mega.txt"

	// CONNECT TEST
	vector<Component_Pair> component_pairs;
	left_component = components[2];
	right_component = components[1];
	component_pair = make_pair(left_component,
		right_component);
	component_pairs.push_back(component_pair);

	left_component = components[3];
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

	test_circuit.updateComponents(vector<shared_ptr<Electrical_Component>>{components[2], components[3]});
	test_circuit.updateConnections(pin_connection_list);
	test_circuit.updateVerify(&model);
	test_circuit.maxSolve(&model);

	shared_ptr<Battery> battery = make_shared<Battery>(battery_path + "D1604-1F.txt");
	components.push_back(battery);

	// CONNECT TEST
	component_pairs.clear();
	left_component = components[4];
	right_component = components[2];
	component_pair = make_pair(left_component,
		right_component);
	component_pairs.push_back(component_pair);

	left_component = components[4];
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

	test_circuit.updateComponents(vector<shared_ptr<Electrical_Component>>{components[4]});
	test_circuit.updateConnections(pin_connection_list);
	test_circuit.updateVerify(&model);
	test_circuit.maxSolve(&model);
}


void force_sensor_battery_micro_controller_connect()
{
	shared_ptr<Force_Sensor> force_sensor = 
		make_shared<Force_Sensor>(forcesensor_path + "Pololu_1645.txt");

	GRBEnv env = GRBEnv();
	GRBModel model = GRBModel(env);
	vector<shared_ptr<Electrical_Component>> components;
	components.push_back(force_sensor);
	components[0]->getUsedVarsName(stringvec());
	Circuit test_circuit(components);
	test_circuit.verify(&model);
	test_circuit.maxSolve(&model);

	shared_ptr<Battery> battery = make_shared<Battery>(battery_path + "Turnigy_20C.txt");
	components.push_back(battery);
	// "PR-CU-R198.txt"
	// "6LR61.txt"
	// "Turnigy_20C.txt"
	// "LP503562.txt"
	// "85AAAHCB.txt"
	// "D1604-1F.txt"

	shared_ptr<Micro_Controller> micro_controller = 
		make_shared<Micro_Controller>(micro_controller_path + "Arduino_Mini.txt");
	components.push_back(micro_controller);
	// "Arduino_Mini.txt"
	// "Arduino_Uno.txt"
	// "Arduino_Mega.txt"

	// CONNECT TEST
	vector<Component_Pair> component_pairs;
	shared_ptr<Electrical_Component> left_component = components[1],
		right_component = components[0];
	Component_Pair component_pair = make_pair(left_component,
		right_component);
	component_pairs.push_back(component_pair);

	left_component = components[2];
	right_component = components[0];
	component_pair = make_pair(left_component,
		right_component);
	component_pairs.push_back(component_pair);

	Pin_Connections pin_connection_list = groupMatch(component_pairs);
	printPinConnections(pin_connection_list);

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

	test_circuit.updateComponents(vector<shared_ptr<Electrical_Component>>{components[1], components[2]});
	test_circuit.updateConnections(pin_connection_list);
	test_circuit.updateVerify(&model);
	test_circuit.maxSolve(&model);
}

void servo_micro_controller_voltage_regulator_battery_connect()
{
	shared_ptr<Motor> servo = make_shared<Motor>(servo_path + "LS-0009AF.txt");
//	servo.setWorkPoint(1e-3, 10);
	servo->setWorkPoint(0.14, 1e-3);

	GRBEnv env = GRBEnv();
	GRBModel model = GRBModel(env);
	vector<shared_ptr<Electrical_Component>> components;
	components.push_back(servo);
	components[0]->getUsedVarsName(stringvec());
	Circuit test_circuit(components);
	test_circuit.verify(&model);
	test_circuit.maxSolve(&model);

	shared_ptr<Voltage_Regulator> voltage_regulator = 
		make_shared<Voltage_Regulator>(voltage_regulator_path + "DC-DC_Convertor.txt");
	shared_ptr<Micro_Controller> micro_controller = 
		make_shared<Micro_Controller>(micro_controller_path + "Arduino_Mini.txt");
	components.push_back(voltage_regulator);
	components.push_back(micro_controller);

	// CONNECT TEST
	vector<Component_Pair> component_pairs;
	shared_ptr<Electrical_Component> left_component = components[1],
		right_component = components[0];
	Component_Pair component_pair = make_pair(left_component,
		right_component);
	component_pairs.push_back(component_pair);

	left_component = components[2];
	right_component = components[0];
	component_pair = make_pair(left_component, right_component);
	component_pairs.push_back(component_pair);

	Pin_Connections pin_connection_list = groupMatch(component_pairs);
	printPinConnections(pin_connection_list);

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

	test_circuit.updateComponents(vector<shared_ptr<Electrical_Component>>{components[1], components[2]});
	test_circuit.updateConnections(pin_connection_list);
	test_circuit.updateVerify(&model);
	test_circuit.maxSolve(&model);

	shared_ptr<Battery> battery = make_shared<Battery>(battery_path + "D1604-1F.txt");
	components.push_back(battery);

	// CONNECT TEST
	component_pairs.clear();
	left_component = components[3];
	right_component = components[1];
	component_pair = make_pair(left_component,
		right_component);
	component_pairs.push_back(component_pair);

	left_component = components[3];
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

	test_circuit.updateComponents(vector<shared_ptr<Electrical_Component>>{components[3]});
	test_circuit.updateConnections(pin_connection_list);
	test_circuit.updateVerify(&model);
	test_circuit.maxSolve(&model);
}

void camera_voltage_regulator_micro_controller_battery_connect()
{
	shared_ptr<Camera> camera = make_shared<Camera>(camera_path + "MU_Vision_Sensor.txt");
	// TTL_Serial_Camera

	GRBEnv env = GRBEnv();
	GRBModel model = GRBModel(env);
	vector<shared_ptr<Electrical_Component>> components;
	components.push_back(camera);
	components[0]->getUsedVarsName(stringvec());
	Circuit test_circuit(components);
	test_circuit.verify(&model);
	test_circuit.maxSolve(&model);

	shared_ptr<Voltage_Regulator> voltage_regulator = 
		make_shared<Voltage_Regulator>(voltage_regulator_path + "DC-DC_Convertor.txt");
	shared_ptr<Micro_Controller> micro_controller = 
		make_shared<Micro_Controller>(micro_controller_path + "Arduino_Uno.txt");
	components.push_back(voltage_regulator);
	components.push_back(micro_controller);

	// CONNECT TEST
	vector<Component_Pair> component_pairs;
	shared_ptr<Electrical_Component> left_component = components[1],
		right_component = components[0];
	Component_Pair component_pair = make_pair(left_component,
		right_component);
	component_pairs.push_back(component_pair);

	left_component = components[2];
	right_component = components[0];
	component_pair = make_pair(left_component, right_component);
	component_pairs.push_back(component_pair);

	Pin_Connections pin_connection_list = groupMatch(component_pairs);
	printPinConnections(pin_connection_list);

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

	test_circuit.updateComponents(vector<shared_ptr<Electrical_Component>>{components[1], components[2]});
	test_circuit.updateConnections(pin_connection_list);
	test_circuit.updateVerify(&model);
	test_circuit.maxSolve(&model);

	shared_ptr<Battery> battery = make_shared<Battery>(battery_path + "D1604-1F.txt");
	components.push_back(battery);

	// CONNECT TEST
	component_pairs.clear();
	left_component = components[3];
	right_component = components[1];
	component_pair = make_pair(left_component,
		right_component);
	component_pairs.push_back(component_pair);

	left_component = components[3];
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

	test_circuit.updateComponents(vector<shared_ptr<Electrical_Component>>{components[3]});
	test_circuit.updateConnections(pin_connection_list);
	test_circuit.updateVerify(&model);
	test_circuit.maxSolve(&model);
}

void bluetooth_battery_micro_controller_connect()
{
	shared_ptr<Bluetooth> bluetooth = make_shared<Bluetooth>(bluetooth_path + "Bluefruit_SPI.txt");
	// Bluefruit_UART.txt

	GRBEnv env = GRBEnv();
	GRBModel model = GRBModel(env);
	vector<shared_ptr<Electrical_Component>> components;
	components.push_back(bluetooth);
	components[0]->getUsedVarsName(stringvec());
	Circuit test_circuit(components);
	test_circuit.verify(&model);
	test_circuit.maxSolve(&model);

	shared_ptr<Battery> battery = make_shared<Battery>(battery_path + "D1604-1F.txt");
	shared_ptr<Micro_Controller> micro_controller = 
		make_shared<Micro_Controller>(micro_controller_path + "Arduino_Mega.txt");
	components.push_back(battery);
	components.push_back(micro_controller);

	// CONNECT TEST
	vector<Component_Pair> component_pairs;
	shared_ptr<Electrical_Component> left_component = components[1],
		right_component = components[0];	
	Component_Pair component_pair = make_pair(left_component,
		right_component);
	component_pairs.push_back(component_pair);

	left_component = components[2];
	right_component = components[0];
	component_pair = make_pair(left_component, right_component);
	component_pairs.push_back(component_pair);

	Pin_Connections pin_connection_list = groupMatch(component_pairs);
	printPinConnections(pin_connection_list);

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

	test_circuit.updateComponents(vector<shared_ptr<Electrical_Component>>{components[1], components[2]});
	test_circuit.updateConnections(pin_connection_list);
	test_circuit.updateVerify(&model);
	test_circuit.maxSolve(&model);
}


void encoder_battery_micro_controller_connect()
{
	shared_ptr<Encoder> bluetooth = make_shared<Encoder>(encoder_path + "Pololu_4761.txt");
	// Bluefruit_UART.txt

	GRBEnv env = GRBEnv();
	GRBModel model = GRBModel(env);
	vector<shared_ptr<Electrical_Component>> components;
	components.push_back(bluetooth);
	components[0]->getUsedVarsName(stringvec());
	Circuit test_circuit(components);
	test_circuit.verify(&model);
	test_circuit.maxSolve(&model);

	shared_ptr<Battery> battery = make_shared<Battery>(battery_path + "D1604-1F.txt");
	shared_ptr<Micro_Controller> micro_controller = make_shared<Micro_Controller>(micro_controller_path + "Arduino_Mini.txt");
	components.push_back(battery);
	components.push_back(micro_controller);

	// CONNECT TEST
	vector<Component_Pair> component_pairs;
	shared_ptr<Electrical_Component> left_component = components[1],
		right_component = components[0];
	Component_Pair component_pair = make_pair(left_component,
		right_component);
	component_pairs.push_back(component_pair);

	left_component = components[2];
	right_component = components[0];
	component_pair = make_pair(left_component, right_component);
	component_pairs.push_back(component_pair);

	Pin_Connections pin_connection_list = groupMatch(component_pairs);
	printPinConnections(pin_connection_list);

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

	test_circuit.updateComponents(vector<shared_ptr<Electrical_Component>>{components[1], components[2]});
	test_circuit.updateConnections(pin_connection_list);
	test_circuit.updateVerify(&model);
	test_circuit.maxSolve(&model);
}

void motor_encoder_hbridge_battery_connect()
{
	shared_ptr<Motor> dc_motor = make_shared<Motor>(dc_motor_path + "Pololu_4797.txt");
	dc_motor->setWorkPoint(1e-3, 6.0);
	shared_ptr<Encoder> encoder = make_shared<Encoder>(encoder_path + "Pololu_3081.txt");

	GRBEnv env = GRBEnv();
	GRBModel model = GRBModel(env);
	vector<shared_ptr<Electrical_Component>> components;
	components.push_back(dc_motor);
	components.push_back(encoder);
	components[0]->getUsedVarsName(stringvec());
	components[1]->getUsedVarsName(stringvec());
	Circuit test_circuit(components);
	test_circuit.verify(&model);
	test_circuit.maxSolve(&model);

	shared_ptr<H_Bridge> h_bridge = make_shared<H_Bridge>(h_bridge_path + "DRV8835.txt");
	shared_ptr<Battery> battery = make_shared<Battery>(battery_path + "LP503562.txt");
	shared_ptr<Micro_Controller> micro_controller = make_shared<Micro_Controller>(micro_controller_path + "Arduino_Mega.txt");
	components.push_back(h_bridge);
	components.push_back(battery);
	components.push_back(micro_controller);
	

	// CONNECT TEST
	vector<Component_Pair> component_pairs;
	shared_ptr<Electrical_Component> left_component = components[2], // h_bridge - dc_motor
		right_component = components[0];
	Component_Pair component_pair = make_pair(left_component,
		right_component);
	component_pairs.push_back(component_pair);

	left_component = components[3]; // battery - encoder
	right_component = components[1];
	component_pair = make_pair(left_component, right_component);
	component_pairs.push_back(component_pair);

	left_component = components[4]; // micro-controller - encoder
	right_component = components[1];
	component_pair = make_pair(left_component, right_component);
	component_pairs.push_back(component_pair);

	Pin_Connections pin_connection_list = groupMatch(component_pairs);
//	printPinConnections(pin_connection_list);

	stringvec2d pins_vec(3);
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

	component_pairs.clear();
	left_component = components[3];
	right_component = components[2];
	component_pair = make_pair(left_component, right_component);
	component_pairs.push_back(component_pair);

	left_component = components[4];
	right_component = components[2];
	component_pair = make_pair(left_component, right_component);
	component_pairs.push_back(component_pair);

	Pin_Connections same_level_connection = groupMatch(component_pairs);
	pin_connection_list.insert(same_level_connection.begin(), same_level_connection.end());
	printPinConnections(pin_connection_list);

	test_circuit.updateComponents(vector<shared_ptr<Electrical_Component>>{components[1], components[2]});
	test_circuit.updateConnections(pin_connection_list);
	test_circuit.updateVerify(&model);
	test_circuit.maxSolve(&model);
}

void system_test()
{
	// pre-processing 
//	stringvec input_types{ Component_Type::Motor, Component_Type::Motor, 
//		Component_Type::Motor, Component_Type::Motor,
//		Component_Type::Encoder, Component_Type::Encoder, 
//		Component_Type::Encoder, Component_Type::Encoder };
//	doublepairs input_torques{make_pair<double>(1e-3, 0.4), 
//		make_pair<double>(1e-3, 0.4), make_pair<double>(1e-3, 0.4), 
//		make_pair<double>(1e-3, 0.4) };
//	doublepairs input_velocities{make_pair<double>(1e-3, 6.0), 
//		make_pair<double>(1e-3, 6.0), make_pair<double>(1e-3, 6.0), 
//		make_pair<double>(1e-3, 6.0)};

//	stringvec2d component_versions = preprocess(input_types, input_torques, input_velocities);
//	infernodevec2d infer_nodes_vec = initialize(component_versions, input_torques, input_velocities);
//	bbnodevec bbnode_vec = initialize(infer_nodes_vec);
	
	// SINGLE/MULTIPLE MOTOR TEST
//	shared_ptr<Motor> dc_motor1 = make_shared<Motor>(dc_motor_path + "Pololu_4797.txt");
//	shared_ptr<Motor> dc_motor2 = make_shared<Motor>(dc_motor_path + "Pololu_4797.txt");
//	shared_ptr<Motor> dc_motor3 = make_shared<Motor>(dc_motor_path + "Pololu_4797.txt");
//	shared_ptr<Motor> dc_motor4 = make_shared<Motor>(dc_motor_path + "Pololu_4797.txt");
//  HIGH SPEED - LOW TORQUE
//	dc_motor1->setWorkPoint(1e-3, 6.0);  
//	dc_motor2->setWorkPoint(1e-3, 6.0);
//	dc_motor3->setWorkPoint(1e-3, 6.0);
//	dc_motor4->setWorkPoint(1e-3, 6.0);
	// LOW SPEED - HIGH TORQUE
//	dc_motor1->setWorkPoint(0.4, 1e-3);
//	dc_motor2->setWorkPoint(0.4, 1e-3);
//	dc_motor3->setWorkPoint(0.4, 1e-3);
//	dc_motor4->setWorkPoint(0.4, 1e-3);

	// SINGLE/MULTIPLE SERVO TEST
//	shared_ptr<Motor> servo_motor1 = make_shared<Motor>(servo_path + "LS-0009AF.txt");
//	shared_ptr<Motor> servo_motor2 = make_shared<Motor>(servo_path + "LS-0009AF.txt");
//	shared_ptr<Motor> servo_motor3 = make_shared<Motor>(servo_path + "LS-0009AF.txt");
//	shared_ptr<Motor> servo_motor4 = make_shared<Motor>(servo_path + "LS-0009AF.txt");
	// HIGH SPEED - LOW TORQUE	
//	servo_motor1->setWorkPoint(1e-3, 10);
//	servo_motor2->setWorkPoint(1e-3, 10);
//	servo_motor3->setWorkPoint(1e-3, 10);
//	servo_motor4->setWorkPoint(1e-3, 10);
	// LOW SPEED - HIGH TORQUE
//	servo_motor1->setWorkPoint(0.14, 1e-3);
//	servo_motor2->setWorkPoint(0.14, 1e-3);
//	servo_motor3->setWorkPoint(0.14, 1e-3);
//	servo_motor4->setWorkPoint(0.14, 1e-3);

	// SINGLE CAMERA TEST
//	shared_ptr<Camera> camera = make_shared<Camera>(camera_path + "MU_Vision_Sensor.txt");

	// SINGLE/MULTIPLE FORCE SENSOR TEST
//	shared_ptr<Force_Sensor> force_sensor1 = make_shared<Force_Sensor>(forcesensor_path + "Pololu_1645.txt");
//	shared_ptr<Force_Sensor> force_sensor2 = make_shared<Force_Sensor>(forcesensor_path + "Pololu_1645.txt");
//	shared_ptr<Force_Sensor> force_sensor3 = make_shared<Force_Sensor>(forcesensor_path + "Pololu_1645.txt");
//	shared_ptr<Force_Sensor> force_sensor4 = make_shared<Force_Sensor>(forcesensor_path + "Pololu_1645.txt");

	// SINGLE BLUETOOTH TEST
//	shared_ptr<Bluetooth> bluetooth = make_shared<Bluetooth>(bluetooth_path + "Bluefruit_SPI.txt");

	// SINGLE ENCODER TEST
//	shared_ptr<Encoder> encoder1 = make_shared<Encoder>(encoder_path + "Pololu_3081.txt");
//	shared_ptr<Encoder> encoder2 = make_shared<Encoder>(encoder_path + "Pololu_3081.txt");
//	shared_ptr<Encoder> encoder3 = make_shared<Encoder>(encoder_path + "Pololu_3081.txt");
//	shared_ptr<Encoder> encoder4 = make_shared<Encoder>(encoder_path + "Pololu_3081.txt");

	// dc motor*4 + encoder*4 TEST
//	shared_ptr<Motor> dc_motor1 = make_shared<Motor>(dc_motor_path + "Pololu_4797.txt");
//	shared_ptr<Motor> dc_motor2 = make_shared<Motor>(dc_motor_path + "Pololu_4797.txt");
//	shared_ptr<Motor> dc_motor3 = make_shared<Motor>(dc_motor_path + "Pololu_4797.txt");
//	shared_ptr<Motor> dc_motor4 = make_shared<Motor>(dc_motor_path + "Pololu_4797.txt");
//	dc_motor1->setWorkPoint(1e-3, 6.0);  
//	dc_motor2->setWorkPoint(1e-3, 6.0);
//	dc_motor3->setWorkPoint(1e-3, 6.0);
//	dc_motor4->setWorkPoint(1e-3, 6.0);
//	shared_ptr<Encoder> encoder1 = make_shared<Encoder>(encoder_path + "Pololu_3081.txt");
//	shared_ptr<Encoder> encoder2 = make_shared<Encoder>(encoder_path + "Pololu_3081.txt");
//	shared_ptr<Encoder> encoder3 = make_shared<Encoder>(encoder_path + "Pololu_3081.txt");
//	shared_ptr<Encoder> encoder4 = make_shared<Encoder>(encoder_path + "Pololu_3081.txt");

	// servo motor*4 + force sensor*2
//	shared_ptr<Motor> servo_motor1 = make_shared<Motor>(servo_path + "LS-0009AF.txt");
//	shared_ptr<Motor> servo_motor2 = make_shared<Motor>(servo_path + "LS-0009AF.txt");
//	shared_ptr<Motor> servo_motor3 = make_shared<Motor>(servo_path + "LS-0009AF.txt");
//	shared_ptr<Motor> servo_motor4 = make_shared<Motor>(servo_path + "LS-0009AF.txt");
	// HIGH SPEED - LOW TORQUE	
//	servo_motor1->setWorkPoint(1e-3, 10);
//	servo_motor2->setWorkPoint(1e-3, 10);
//	servo_motor3->setWorkPoint(1e-3, 10);
//	servo_motor4->setWorkPoint(1e-3, 10); 

//	shared_ptr<Force_Sensor> force_sensor1 = make_shared<Force_Sensor>(forcesensor_path + "Pololu_1645.txt");
//	shared_ptr<Force_Sensor> force_sensor2 = make_shared<Force_Sensor>(forcesensor_path + "Pololu_1645.txt");

	// dc motor*2 + camera + bluetooth
//	shared_ptr<Motor> dc_motor1 = make_shared<Motor>(dc_motor_path + "Pololu_4797.txt");
//	shared_ptr<Motor> dc_motor2 = make_shared<Motor>(dc_motor_path + "Pololu_4797.txt");
//	dc_motor1->setWorkPoint(1e-3, 6.0);  
//	dc_motor2->setWorkPoint(1e-3, 6.0);

//	shared_ptr<Camera> camera = make_shared<Camera>(camera_path + "MU_Vision_Sensor.txt");
//	shared_ptr<Bluetooth> bluetooth = make_shared<Bluetooth>(bluetooth_path + "Bluefruit_SPI.txt");

	// dc motor*8 + encoder*8 + servo motor*4 + camera
	shared_ptr<Motor> dc_motor1 = make_shared<Motor>(dc_motor_path + "Pololu_4797.txt");
	shared_ptr<Motor> dc_motor2 = make_shared<Motor>(dc_motor_path + "Pololu_4797.txt");
	shared_ptr<Motor> dc_motor3 = make_shared<Motor>(dc_motor_path + "Pololu_4797.txt");
	shared_ptr<Motor> dc_motor4 = make_shared<Motor>(dc_motor_path + "Pololu_4797.txt");
	shared_ptr<Motor> dc_motor5 = make_shared<Motor>(dc_motor_path + "Pololu_4797.txt");
	shared_ptr<Motor> dc_motor6 = make_shared<Motor>(dc_motor_path + "Pololu_4797.txt");
	shared_ptr<Motor> dc_motor7 = make_shared<Motor>(dc_motor_path + "Pololu_4797.txt");
	shared_ptr<Motor> dc_motor8 = make_shared<Motor>(dc_motor_path + "Pololu_4797.txt");
	dc_motor1->setWorkPoint(1e-3, 6.0);  
	dc_motor2->setWorkPoint(1e-3, 6.0);
	dc_motor3->setWorkPoint(1e-3, 6.0);
	dc_motor4->setWorkPoint(1e-3, 6.0);
	dc_motor5->setWorkPoint(1e-3, 6.0);
	dc_motor6->setWorkPoint(1e-3, 6.0);
	dc_motor7->setWorkPoint(1e-3, 6.0);
	dc_motor8->setWorkPoint(1e-3, 6.0);
	shared_ptr<Encoder> encoder1 = make_shared<Encoder>(encoder_path + "Pololu_3081.txt");
	shared_ptr<Encoder> encoder2 = make_shared<Encoder>(encoder_path + "Pololu_3081.txt");
	shared_ptr<Encoder> encoder3 = make_shared<Encoder>(encoder_path + "Pololu_3081.txt");
	shared_ptr<Encoder> encoder4 = make_shared<Encoder>(encoder_path + "Pololu_3081.txt");
	shared_ptr<Encoder> encoder5 = make_shared<Encoder>(encoder_path + "Pololu_3081.txt");
	shared_ptr<Encoder> encoder6 = make_shared<Encoder>(encoder_path + "Pololu_3081.txt");
	shared_ptr<Encoder> encoder7 = make_shared<Encoder>(encoder_path + "Pololu_3081.txt");
	shared_ptr<Encoder> encoder8 = make_shared<Encoder>(encoder_path + "Pololu_3081.txt");

	shared_ptr<Motor> servo_motor1 = make_shared<Motor>(servo_path + "LS-0009AF.txt");
	shared_ptr<Motor> servo_motor2 = make_shared<Motor>(servo_path + "LS-0009AF.txt");
	shared_ptr<Motor> servo_motor3 = make_shared<Motor>(servo_path + "LS-0009AF.txt");
	shared_ptr<Motor> servo_motor4 = make_shared<Motor>(servo_path + "LS-0009AF.txt");
	// HIGH SPEED - LOW TORQUE	
	servo_motor1->setWorkPoint(1e-3, 10);
	servo_motor2->setWorkPoint(1e-3, 10);
	servo_motor3->setWorkPoint(1e-3, 10);
	servo_motor4->setWorkPoint(1e-3, 10); 

	shared_ptr<Camera> camera = make_shared<Camera>(camera_path + "MU_Vision_Sensor.txt");

	Infer_Node infer_node1(dc_motor1), infer_node2(dc_motor2), infer_node3(dc_motor3), infer_node4(dc_motor4),
		infer_node5(dc_motor5), infer_node6(dc_motor6), infer_node7(dc_motor7), infer_node8(dc_motor8),
		infer_node9(encoder1), infer_node10(encoder2), infer_node11(encoder3), infer_node12(encoder4),
		infer_node13(encoder5), infer_node14(encoder6), infer_node15(encoder7), infer_node16(encoder8),
		infer_node17(servo_motor1), infer_node18(servo_motor2), infer_node19(servo_motor3), infer_node20(servo_motor4),
		infer_node21(camera);
	infernodevec infer_nodes{ infer_node1,  infer_node2, infer_node3, infer_node4, infer_node5, infer_node6,
		infer_node7, infer_node8, infer_node9, infer_node10, infer_node11, infer_node12,  infer_node13, infer_node14,
		infer_node15, infer_node16, infer_node17, infer_node18,  infer_node19, infer_node20, infer_node21 };
	BBNode root(infer_nodes);
	bbnodevec bbnodes{ root };
	shared_ptr<BBNode> best_node = branchNBound(bbnodes);
	writeDesign(*best_node); 
 	int a = 1;
	
}

 