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

// using namespace std;
using namespace Eigen;

int main() {

	// READ TEST
	motor_read();
	h_bridge_read();
	battery_read();
	micro_controller_read();
	system_test();

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


//	force_sensor_read();
//	voltage_regulator_read();
//	servo_read();
//	camera_read();
//	bluetooth_read();

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


	// SYSTEM TEST
	

	int a = 1;
	// For generated voltage vector
	/*
	doublevec voltages, torques; 
	ifstream input(current_path + "\\files\\arm_m2.txt");
	if (input.is_open())
	{
		string str;
		while (getline(input, str))
		{
			torques.push_back(stod(str));
		}
	}
	input.close();

	ofstream output(current_path + "\\files\\arm_m2_results.txt");
	double speed = 3.15;
	for (auto &tbeg = torques.begin(); tbeg != torques.end(); tbeg++)
	{
		GRBEnv env = GRBEnv();
		GRBModel model = GRBModel(env);
		Motor motor(motor_path + "\\Pololu_2216.txt", speed, *tbeg);
		Circuit myCircuit(vector<Actu_components>{motor});
		myCircuit.verify(&model);
		output << model.getVar(0).get(GRB_DoubleAttr_X) << endl;

	}
	*/
	
	// INFERENCE TEST
	
//	try
//	{ 
		/*
		unsigned cnt = 0, num_of_actuations;
		doublepairs torqs, vels;
		doublevec coefficients;
		auto &tbeg = torqs.begin(), &vbeg = vels.begin();

		// read actuation requirements
		std::ifstream input(current_path + "\\files\\actuation_requirements.txt");
		if (input.is_open())
		{
			string str;
			while (std::getline(input, str))
			{
				if (cnt == 0)
				{
					num_of_actuations = stol(str);
					torqs.resize(num_of_actuations);
					vels.resize(num_of_actuations);
					tbeg = torqs.begin();
					vbeg = vels.begin();
				}
				else if (str == "")
				{
					getline(input, str);
					size_t pos1 = str.find(" "), pos2 = str.find(" ", pos1+1), pos3 = str.find(" ", pos2 + 1),
						pos4= str.find(" ", pos3 + 1), pos5 = str.find(" ", pos4 + 1);
					double c1 = stod(str.substr(0, pos1)), c2 = stod(str.substr(pos1 + 1, pos2 - pos1)),
						c3 = stod(str.substr(pos2 + 1, pos3 - pos2)), c4 = stod(str.substr(pos3, pos4 - pos3)),
						c5 = stod(str.substr(pos4, pos5 - pos4)), c6 = stod(str.substr(pos5, str.size() - pos5));
					coefficients = { c1, c2, c3, c4, c5, c6 };
					break;
				}
				else 
				{
					size_t pos = str.find(" ");
					double torq = stod(str.substr(0, pos)),
						vel = stod(str.substr(pos + 1, str.size() - pos));
					*tbeg++ = make_pair(torq, torq);
					*vbeg++ = make_pair(vel, vel);
				}
				cnt++;
			}
		}
		input.close();
		std::cout << "acutation requirements loaded" << std::endl;
		testinput actuation_input = make_pair(torqs, vels);

		if (getFileNumInDirectory(current_path + "\\files\\") > 1)
		{
			// if exit, double check it; if failed, generated again
			// reload components and connections
			string raw_data = current_path + "\\files\\circuit_specs1_raw.txt";
			stringvec component_names;
			connection_relation connections;
			bool components_indicator = false, connections_indicator = false;
			ifstream input(raw_data);
			if (input.is_open())
			{
				string str;
				while (getline(input, str))
				{
					if (str == "COMPONENTS: ")
					{
						components_indicator = true;
					}

					if (str == "CONNECTIONS: ")
					{
						connections_indicator = true;
					}

					if (components_indicator && !connections_indicator)
					{
						component_names.insert(component_names.end(), str);
					}
					else if (components_indicator && connections_indicator)
					{
						size_t pos1 = str.find(" "), pos2 = str.find(" ", pos1 + 1), pos3 = str.find(" ", pos2 + 1);
						string lhc = str.substr(0, pos1), rhc = str.substr(pos1 + 1, pos2 - pos1 - 1),
							lhp = str.substr(pos2 + 1, pos3-pos2-1), rhp = str.substr(pos3+1, str.size()-pos3);						
						connections.insert(connections.end(), make_pair(make_pair(lhc, rhc), make_pair(lhp, rhp)));
					}
				}
			}
			component_names.erase(component_names.begin());
			connections.erase(connections.begin());
			input.close();

			vector<Actu_components> components = generateComponents(component_names, actuation_input);

			GRBEnv env = GRBEnv();
			GRBModel model = GRBModel(env);
			Circuit circuit(components, connections);
			circuit.verify(&model);

			if (model.get(GRB_IntAttr_Status) == GRB_OPTIMAL)
			{
				std::cout << "Generated Circuit is Compatible with New Constraints" << std::endl;
				remove(raw_data.c_str());
				string generated_metrics = current_path + "\\files\\metric.txt";
				remove(generated_metrics.c_str());
			}
			else
			{
				remove(raw_data.c_str());
				string generated_circuit = current_path + "\\files\\circuit_specs_1_.txt";
				remove(generated_circuit.c_str());
				string generated_metrics = current_path + "\\files\\metric.txt";
				remove(generated_metrics.c_str());
				goto start_of_generation;
			}
		}
		else
		{
		start_of_generation:
			// circuit generation process
			bbglobalmap gmap;
			BBNode::coefficients = coefficients;
			auto start = std::chrono::high_resolution_clock::now();
			bbnodevec roots = allInitialization(motor_type, actuation_input.first, actuation_input.second);
			int optimal_index = brachNBound(gmap, roots, connection_relation_vec2d(), actuation_input.first, actuation_input.second);
			BBNode optimal_design = getOptimalDesign(gmap, optimal_index);
			writeDesign(optimal_design);
			writeRawDesign(optimal_design);
		}
		*/

//		system("pause");
//	}
//	catch (GRBException e)
//	{
//		std::cout << "Error code = " << e.getErrorCode() << std::endl;
//		std::cout << e.getMessage() << std::endl;
//	}
//	catch (std::exception& e) {
//		std::cout << "Exception:" << e.what() << std::endl;
//	}
}
