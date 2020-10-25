#include "Inference.h"
#include <algorithm>
#include <filesystem>

// connection_relation BBNode::final_connections = connection_relation();
GRBEnv env = GRBEnv();
unsigned BBNode::num_of_bbnodes = 0;
unsigned BBNode::num_of_pruned_bbnodes = 0;
doublevec BBNode::coefficients = { 1, 0, 0, 0, 0, 0 };
double BBNode::best_metric_val = 0.0;
std::vector<std::pair<unsigned, double>> BBNode::metric_vec =
	std::vector<std::pair<unsigned, double>>();
unsigned BBNode::last_level = 0;
bool BBNode::pruning_enable = true;
bool BBNode::solver_info = false;
unsigned Infer_Node::num_of_nodes = 0;

static std::unordered_map<unsigned, Infer_Node> global_infer_node_map;

using std::cout;
using std::endl;
using std::fstream;
using std::string;
using std::vector;
using std::filesystem::directory_iterator;
using std::unique_ptr;
using std::unordered_map;
using std::unordered_multimap;
using std::pair;
using std::make_pair;
using std::find;
using std::shared_ptr;
using std::make_shared;
using std::make_unique;

extern unordered_map<string, shared_ptr<Electrical_Component>> MOTOR_PART_MAP,
H_BRIDDE_PART_MAP, MICRO_CONTROLLER_PART_MAP, VOLTAGE_REGULATOR_PART_MAP,
BATTERY_PART_MAP, ENCODER_PART_MAP, CAMERA_PART_MAP, FORCE_SENSOR_PART_MAP,
BLUETOOTH_PART_MAP, SERVO_PART_MAP, POWER_SUPPLY_PART_MAP;

extern unordered_multimap<string, string> connection_map;

struct hash_pair {
	template <class T1, class T2>
	size_t operator()(const pair<T1, T2>& p) const
	{
		auto hash1 = std::hash<T1>{}(p.first);
		auto hash2 = std::hash<T2>{}(p.second);
		return hash1 ^ hash2;
	}
};

unordered_map<stringpair, Electronics::CLASS, hash_pair> in_vol_map{
	{{Component_Type::Motor, Component_Type::H_Bridge}, Electronics::POWER},
	{{Component_Type::H_Bridge, Component_Type::Micro_Controller},
	Electronics::CLASS::FUNCTION},
	{{Component_Type::H_Bridge, Component_Type::Voltage_Regulator},
	Electronics::POWER},
	{{Component_Type::H_Bridge, Component_Type::Battery}, Electronics::POWER},
	{{Component_Type::H_Bridge, Component_Type::Power_Supply}, Electronics::POWER},
	{{Component_Type::Micro_Controller, Component_Type::Voltage_Regulator },
	Electronics::CLASS::POWER},
	{{Component_Type::Micro_Controller, Component_Type::Battery },
	Electronics::POWER},
	{{Component_Type::Micro_Controller, Component_Type::Power_Supply },
	Electronics::POWER},
	{{Component_Type::Voltage_Regulator, Component_Type::Battery},
	Electronics::CLASS::POWER},
	{{Component_Type::Voltage_Regulator, Component_Type::Power_Supply},
	Electronics::CLASS::POWER},
	{{Component_Type::Encoder, Component_Type::Micro_Controller },
	Electronics::CLASS::FUNCTION},
	{{Component_Type::Encoder, Component_Type::Voltage_Regulator},
	Electronics::POWER },
	{{Component_Type::Encoder, Component_Type::Battery}, Electronics::POWER},
	{{Component_Type::Encoder, Component_Type::Power_Supply}, Electronics::POWER},
	{{Component_Type::Camera, Component_Type::Micro_Controller},
	Electronics::CLASS::FUNCTION},
	{{Component_Type::Camera, Component_Type::Voltage_Regulator },
	Electronics::POWER},
	{{Component_Type::Camera, Component_Type::Battery }, Electronics::POWER},
	{{Component_Type::Camera, Component_Type::Power_Supply }, Electronics::POWER},
	{{Component_Type::Bluetooth, Component_Type::Micro_Controller},
	Electronics::CLASS::FUNCTION},
	{{Component_Type::Bluetooth, Component_Type::Voltage_Regulator},
	Electronics::POWER},
	{{Component_Type::Bluetooth, Component_Type::Battery}, Electronics::POWER},
	{{Component_Type::Bluetooth, Component_Type::Power_Supply}, Electronics::POWER},
	{{Component_Type::Force_Sensor, Component_Type::Micro_Controller},
	Electronics::CLASS::FUNCTION},
	{{Component_Type::Force_Sensor, Component_Type::Battery},
	Electronics::CLASS::POWER},
	{{Component_Type::Force_Sensor, Component_Type::Voltage_Regulator},
	Electronics::CLASS::POWER},
	{{Component_Type::Force_Sensor, Component_Type::Power_Supply},
	Electronics::CLASS::POWER},
	{{Component_Type::Servo, Component_Type::Micro_Controller},
	Electronics::CLASS::FUNCTION},
	{{Component_Type::Servo, Component_Type::Voltage_Regulator}, Electronics::POWER },
	{{Component_Type::Servo, Component_Type::Battery}, Electronics::POWER },
	{{Component_Type::Servo, Component_Type::Power_Supply}, Electronics::POWER }
};

unordered_map<stringpair, Electronics::CLASS, hash_pair> in_metric_map{
	{{Component_Type::Motor, Component_Type::H_Bridge}, Electronics::FUNCTION},
	{{Component_Type::H_Bridge, Component_Type::Micro_Controller},
	Electronics::CLASS::FUNCTION},
	{{Component_Type::H_Bridge, Component_Type::Voltage_Regulator},
	Electronics::POWER},
	{{Component_Type::H_Bridge, Component_Type::Battery}, Electronics::POWER},
	{{Component_Type::H_Bridge, Component_Type::Power_Supply}, Electronics::POWER},
	{{Component_Type::Micro_Controller, Component_Type::Voltage_Regulator },
	Electronics::CLASS::POWER},
	{{Component_Type::Micro_Controller, Component_Type::Battery },
	Electronics::POWER},
	{{Component_Type::Micro_Controller, Component_Type::Power_Supply },
	Electronics::POWER},
	{{Component_Type::Voltage_Regulator, Component_Type::Battery},
	Electronics::CLASS::POWER},
	{{Component_Type::Encoder, Component_Type::Motor}, Electronics::POWER},
	{{Component_Type::Encoder, Component_Type::Micro_Controller },
	Electronics::CLASS::FUNCTION},
	{{Component_Type::Encoder, Component_Type::Voltage_Regulator},
	Electronics::POWER },
	{{Component_Type::Encoder, Component_Type::Battery}, Electronics::POWER},
	{{Component_Type::Encoder, Component_Type::Power_Supply}, Electronics::POWER},
	{{Component_Type::Camera, Component_Type::Micro_Controller},
	Electronics::CLASS::FUNCTION},
	{{Component_Type::Camera, Component_Type::Voltage_Regulator},
	Electronics::POWER},
	{{Component_Type::Camera, Component_Type::Battery }, Electronics::POWER},
	{{Component_Type::Camera, Component_Type::Power_Supply }, Electronics::POWER},
	{{Component_Type::Bluetooth, Component_Type::Micro_Controller},
	Electronics::CLASS::FUNCTION},
	{{Component_Type::Bluetooth, Component_Type::Voltage_Regulator},
	Electronics::POWER},
	{{Component_Type::Bluetooth, Component_Type::Battery}, Electronics::POWER},
	{{Component_Type::Bluetooth, Component_Type::Power_Supply}, Electronics::POWER},
	{{Component_Type::Force_Sensor, Component_Type::Micro_Controller},
	Electronics::CLASS::FUNCTION},
	{{Component_Type::Force_Sensor, Component_Type::Voltage_Regulator},
	Electronics::CLASS::POWER},
	{{Component_Type::Force_Sensor, Component_Type::Battery}, Electronics::POWER},
	{{Component_Type::Force_Sensor, Component_Type::Power_Supply},
	Electronics::CLASS::POWER},
	{{Component_Type::Servo, Component_Type::Micro_Controller},
	Electronics::CLASS::FUNCTION},
	{{Component_Type::Servo, Component_Type::Voltage_Regulator}, Electronics::POWER },
	{{Component_Type::Servo, Component_Type::Battery}, Electronics::POWER },
	{{Component_Type::Servo, Component_Type::Power_Supply}, Electronics::POWER }
};

unordered_map<string, Electronics::CLASS> out_vol_map{
	{Component_Type::Motor, Electronics::POWER},
	{Component_Type::H_Bridge, Electronics::FUNCTION},
	{Component_Type::Micro_Controller, Electronics::FUNCTION},
	{Component_Type::Voltage_Regulator, Electronics::POWER},
	{Component_Type::Battery, Electronics::POWER},
	{Component_Type::Power_Supply, Electronics::POWER},
};

unordered_map<string, double(*)(const doublevec &)> in_current_map{
	{Component_Type::H_Bridge, Current_Operation::max},
	{Component_Type::Micro_Controller, Current_Operation::max},
	{Component_Type::Voltage_Regulator, Current_Operation::add},
	{Component_Type::Battery, Current_Operation::add},
	{Component_Type::Power_Supply, Current_Operation::add},
	{Component_Type::Motor, Current_Operation::max}
};

str_strvec_map type_infer_map{
	{Component_Type::Motor, stringvec{Component_Type::H_Bridge}},
	{Component_Type::H_Bridge, stringvec{Component_Type::Power_Supply,
	Component_Type::Micro_Controller}},
	{Component_Type::Micro_Controller, stringvec{Component_Type::Power_Supply}},
	{Component_Type::Voltage_Regulator, stringvec{Component_Type::Power_Supply}},
	{Component_Type::Battery, stringvec{Component_Type::None}},
	{Component_Type::Encoder, stringvec{
	Component_Type::Power_Supply, Component_Type::Micro_Controller}},
	{Component_Type::Camera, stringvec{Component_Type::Power_Supply,
	Component_Type::Micro_Controller}},
	{Component_Type::Bluetooth, stringvec{Component_Type::Power_Supply,
	Component_Type::Micro_Controller}},
	{Component_Type::Force_Sensor, stringvec{Component_Type::Power_Supply,
	Component_Type::Micro_Controller}},
	{Component_Type::Servo, stringvec{Component_Type::Power_Supply,
	Component_Type::Micro_Controller}}
};

unordered_map<string, unsigned> precedence_map{
	{Component_Type::Motor, 0},
	{Component_Type::Servo, 0},
	{Component_Type::Encoder, 0},
	{Component_Type::Camera, 0},
	{Component_Type::Bluetooth, 0},
	{Component_Type::Force_Sensor, 0},
	{Component_Type::H_Bridge, 1},
	{Component_Type::Micro_Controller, 2},
	{Component_Type::Voltage_Regulator, 3},
	{Component_Type::Battery, 4}
};

unordered_map<string, string> sensor_path_map{
	{Component_Type::Encoder, encoder_path},
	{Component_Type::Camera, camera_path},
	{Component_Type::Bluetooth, bluetooth_path},
	{Component_Type::Force_Sensor, forcesensor_path}
};


stringvec2d getMotorVersions(const std::string &type,
	const vector<pair<doublepair, doublepair>> &torq_vels)
{
	stringvec2d versions(torq_vels.size());
	for (size_t i = 0; i < torq_vels.size(); i++)
	{
		versions[i] = getMotorVersions(type, torq_vels[i]);
	}
	return versions;
}

stringvec getMotorVersions(const string &type,
	const pair<doublepair, doublepair> &torq_vel)
{
	doublepair torq = torq_vel.first, vel = torq_vel.second;
	string obj_path;
	stringvec component_names;
	double max_torq, max_vel, vel_h, vel_l;
	type == Component_Type::Motor ? obj_path = dc_motor_path :
		obj_path = servo_path;
	for (auto &entry : directory_iterator(obj_path))
	{
		Motor obj(entry.path().string());
		max_torq = obj.getMaxTorq();
		max_vel = obj.getMaxVel();

		if (max_torq >= torq.second && max_vel >= vel.second)
		{
			vel_h = obj.getVel(torq.first);
			vel_l = obj.getVel(torq.second);
			if (vel_l >= vel.first && vel_h >= vel.second)
			{
				component_names.push_back(entry.path().string());
			}
		}
	}
	return component_names;
}

/*
compoundtype generateNeedVec(const infernodevec &nodes, const doublepairs &torq_range, const doublepairs &vel_range)
{
	std::vector<Actu_components> component_vec;
	unsignedpairs power_index_vec, func_index_vec, current_index_vec;
	std::string component_type, component_name;
	auto &tbeg = torq_range.begin(), &sbeg = vel_range.begin();
	for (auto &nbeg = nodes.begin(); nbeg != nodes.end(); nbeg++)
	{
		component_type = nbeg->type;
		component_name = nbeg->version;
		if (component_type == motor_type)
		{
			Motor obj(component_name, (sbeg++)->first, (tbeg++)->second);
			component_vec.push_back(obj);
			power_index_vec.push_back(obj.getPowerInputVarsIndex());
			func_index_vec.push_back(obj.getFuncInputVarsIndex());
			current_index_vec.push_back(obj.getCurrentVarsIndex());
		}
		else if (component_type == hbridge_type)
		{
			H_bridge obj(component_name);
			component_vec.push_back(obj);
			power_index_vec.push_back(obj.getPowerInputVarsIndex());
			func_index_vec.push_back(obj.getFuncInputVarsIndex());
		}
		else if (component_type == voltage_regulator_type)
		{
			V_regulator obj(component_name);
			component_vec.push_back(obj);
			power_index_vec.push_back(obj.getPowerInputVarsIndex());
			func_index_vec.push_back(obj.getFuncInputVarsIndex());
		}
		else if (component_type == micro_controller_type)
		{
			Micro_Controller obj(component_name);
			component_vec.push_back(obj);
			power_index_vec.push_back(obj.getPowerInputVarsIndex());
			func_index_vec.push_back(obj.getFuncInputVarsIndex());
		}
		else if (component_type == battery_type)
		{
			Battery obj(component_name);
			component_vec.push_back(obj);
			power_index_vec.push_back(obj.getPowerInputVarsIndex());
			func_index_vec.push_back(obj.getFuncInputVarsIndex());
		}
	}

	return std::make_tuple(component_vec, power_index_vec, func_index_vec, current_index_vec);
}

compoundsettype generateNeedSet(Circuit &circuit, GRBModel &model, compstructptrvec &component_connections,
	CopyCurrentTracker &ctracker, const std::vector<Actu_components> &component_vec, unsignedpairs &power_index_vec,
	unsignedpairs &func_index_vec, unsignedpairs &current_index_vec)
{
	//	std::cout << "MINIMIZATION OPTMIZATION STARTED " << std::endl;
	//	circuit.updateMinObjs(&model);

	doublevec2d lbs2d, ubs2d;
	Actu_components match_component;
	for (auto &ccbeg = component_connections.begin(); ccbeg != component_connections.end(); ccbeg++)
	{
		unsigned var_index = 0;
		auto &opt_components = circuit.getComponents();
		for (auto &ocbeg = opt_components.begin(); ocbeg != opt_components.end(); ocbeg++)
		{
			if (ocbeg->getName() == switchPath((*ccbeg)->getComponentName()))
			{
				match_component = *ocbeg;
				break;
			}
			var_index += ocbeg->getVarSize();
		}

		lbs2d.push_back(doublevec());
		auto &pins = (*ccbeg)->getUsablePins();
		for (auto &pbeg = pins.begin(); pbeg != pins.end(); pbeg++)
		{
			auto &var_names = match_component.getVarNames();
			unsigned _var_index = var_index + getPosInVec(pbeg->name, var_names);
			lbs2d.rbegin()->push_back(model.getVar(_var_index).get(GRB_DoubleAttr_X));
		}
	}

	if (model.get(GRB_IntAttr_Status) == GRB_OPTIMAL)
	{
		unsigned start_index = circuit.getComponentsNum() - component_vec.size(), end_index = circuit.getComponentsNum();
		doublepairs power_voltage_range_set, func_voltage_range_set;
		doublevec2d power_voltage_min_set, power_voltage_max_set, func_voltage_min_set, func_voltage_max_set;
		doublevec current_clique_set;
		auto &p_beg = power_index_vec.begin(), &f_beg = func_index_vec.begin();

		if (ctracker.current_set.empty())
		{
			doublevec temp_vec;
			auto &c_beg = current_index_vec.begin();
			for (unsigned i = start_index; i < end_index; i++, c_beg++)
			{
				temp_vec = circuit.getVals(i, c_beg->first, c_beg->second);
				ctracker.current_set.push_back(*std::max_element(temp_vec.begin(), temp_vec.end()));
			}
		}

		for (unsigned i = start_index; i < end_index; i++, p_beg++, f_beg++)
		{
			power_voltage_min_set.push_back(circuit.getVals(i, p_beg->first, p_beg->second));
			func_voltage_min_set.push_back(circuit.getVals(i, f_beg->first, f_beg->second));
		}

		//		std::cout << "MAXIMIZATION OPTIMIZATION STARTED" << std::endl;
		circuit.updateMaxObjs(&model);
		model.optimize();
		for (auto &ccbeg = component_connections.begin(); ccbeg != component_connections.end(); ccbeg++)
		{
			unsigned var_index = 0;
			auto &opt_components = circuit.getComponents();
			for (auto &ocbeg = opt_components.begin(); ocbeg != opt_components.end(); ocbeg++)
			{
				if (ocbeg->getName() == switchPath((*ccbeg)->getComponentName()))
				{
					match_component = *ocbeg;
					break;
				}
				var_index += ocbeg->getVarSize();
			}

			ubs2d.push_back(doublevec());
			auto &pins = (*ccbeg)->getUsablePins();
			for (auto &pbeg = pins.begin(); pbeg != pins.end(); pbeg++)
			{
				auto &var_names = match_component.getVarNames();
				unsigned _var_index = var_index + getPosInVec(pbeg->name, var_names);
				ubs2d.rbegin()->push_back(model.getVar(_var_index).get(GRB_DoubleAttr_X));
			}
		}
		// assign usable pins voltage range
		auto &lbbeg = lbs2d.begin(), &ubbeg = ubs2d.begin();
		for (auto &ccbeg = component_connections.begin(); ccbeg != component_connections.end(); ccbeg++)
		{

			(*ccbeg)->assignPinVoltageRange(*lbbeg++, *ubbeg++);
		}

		p_beg = power_index_vec.begin(), f_beg = func_index_vec.begin();
		for (unsigned i = start_index; i < end_index; i++, p_beg++, f_beg++)
		{
			power_voltage_max_set.push_back(circuit.getVals(i, p_beg->first, p_beg->second));
			func_voltage_max_set.push_back(circuit.getVals(i, f_beg->first, f_beg->second));
		}

		// return power/functional pin range
		auto &pvminbeg = power_voltage_min_set.begin(), &pvmaxbeg = power_voltage_max_set.begin(),
			&fvminbeg = func_voltage_min_set.begin(), &fvmaxbeg = func_voltage_max_set.begin();
		std::unordered_map<unsigned, unsigned> power_index_convert_vec, func_index_convert_vec;
		for (; pvminbeg != power_voltage_min_set.end(); pvminbeg++, pvmaxbeg++, fvminbeg++, fvmaxbeg++)
		{
			doublepairs temp_set = makePairs(*pvminbeg, *pvmaxbeg);
			auto &clique_results = minCliqueCover(temp_set);
			auto &clique_range = std::get<0>(clique_results);
			for (size_t i = 0; i < clique_range.size(); i++)
			{
				power_index_convert_vec.insert(std::make_pair(power_voltage_range_set.size() + i,
					pvminbeg - power_voltage_min_set.begin()));
			}
			power_voltage_range_set.insert(power_voltage_range_set.end(), clique_range.begin(), clique_range.end());

			temp_set = makePairs(*fvminbeg, *fvmaxbeg);
			clique_results = minCliqueCover(temp_set);
			clique_range = std::get<0>(clique_results);
			for (size_t i = 0; i < clique_range.size(); i++)
			{
				func_index_convert_vec.insert(std::make_pair(func_voltage_range_set.size() + i,
					fvminbeg - func_voltage_min_set.begin()));
			}
			func_voltage_range_set.insert(func_voltage_range_set.end(), clique_range.begin(), clique_range.end());
		}

		cliquetype power_clique, func_clique;
		doublepairs power_clique_range, func_clique_range;
		cliqueindex power_clique_index, func_clique_index;
		if (!power_voltage_range_set.empty())
		{
			power_clique = minCliqueCover(power_voltage_range_set);
			power_clique_range = std::get<0>(power_clique);
			power_clique_index = std::get<1>(power_clique);
		}

		if (!func_voltage_range_set.empty())
		{
			func_clique = minCliqueCover(func_voltage_range_set);
			func_clique_range = std::get<0>(func_clique);
			func_clique_index = std::get<1>(func_clique);
		}

		cliqueindex new_power_clique_index, new_func_clique_index;
		for (auto &pibeg = power_clique_index.begin(); pibeg != power_clique_index.end(); pibeg++)
		{
			new_power_clique_index.push_back(intvec());
			for (auto &piibeg = pibeg->begin(); piibeg != pibeg->end(); piibeg++)
			{
				new_power_clique_index.rbegin()->push_back(power_index_convert_vec[*piibeg]);
			}
		}

		for (auto &fibeg = func_clique_index.begin(); fibeg != func_clique_index.end(); fibeg++)
		{
			new_func_clique_index.push_back(intvec());
			for (auto &fiibeg = fibeg->begin(); fiibeg != fibeg->end(); fiibeg++)
			{
				new_func_clique_index.rbegin()->push_back(func_index_convert_vec[*fiibeg]);
			}
		}

		if (!power_clique_index.empty())
		{
			current_clique_set = getCurrentClique(new_power_clique_index, ctracker.current_set);
		}
		else
		{
			current_clique_set = getCurrentClique(new_func_clique_index, ctracker.current_set);
		}

		return std::make_tuple(power_clique_range, func_clique_range, new_power_clique_index, new_func_clique_index, current_clique_set,
			power_voltage_range_set, func_voltage_range_set);
	}
	else
	{
		throw "OPTIMIZATION FAILED";
	}
}
*/

bool doubleCheck2(Circuit &circuit, GRBModel &model,
	const doublepairs &torqs, const doublepairs &vels)
{
	bool success;
	circuit.syncVars(model);
	circuit.setMotorWorkPoint2(&model, torqs, vels);
	model.optimize();
	model.get(GRB_IntAttr_Status) == GRB_OPTIMAL ? success = true :
		success = false;
	return success;
}

bool doubleCheck(BBNode &node, const doublepairs &torqs, const doublepairs &vels)
{
	return doubleCheck2(node.circuit, node.model, torqs, vels);
}

bool doubleCheck1(Circuit& circuit, GRBModel& model,
	const doublepairs& torqs, const doublepairs& vels)
{
	bool success;
	circuit.syncVars(model);
	circuit.setMotorWorkPoint1(&model, torqs, vels);
	model.optimize();
	model.get(GRB_IntAttr_Status) == GRB_OPTIMAL ? success = true :
		success = false;
	return success;
}

bool reEvaluate(BBNode& node, const doublepairs& torqs, const doublepairs& vels)
{
	bool limit1 = doubleCheck1(node.circuit, node.model, torqs, vels),
		limit2 = doubleCheck2(node.circuit, node.model, torqs, vels);
	return limit1 && limit2;
}



stringvec2d preprocess(const stringvec &component_types, const doublepairs &torqs, const doublepairs &vels)
{
	stringvec actuator_types(torqs.size()),
		sensor_types(component_types.size() - torqs.size());
	for (size_t i = 0; i < component_types.size(); i++)
	{
		if (i < torqs.size())
		{
			actuator_types[i] = component_types[i];
		}
		else
		{
			sensor_types[i - torqs.size()] = component_types[i];
		}
	}

	stringvec2d actuator_versions, sensor_versions;
	actuator_versions = actuatorPreprocess(actuator_types, torqs, vels);
	sensor_versions = sensorPreprocess(sensor_types);

	stringvec2d all_component_versions(actuator_versions.size() * sensor_versions.size()); // combinations
	size_t k = 0;
	for (size_t i = 0; i < actuator_versions.size(); i++)
	{
		for (size_t j = 0; j < sensor_versions.size(); j++)
		{
			all_component_versions[k].insert(all_component_versions[k].end(),
				actuator_versions[i].begin(), actuator_versions[i].end());
			all_component_versions[k].insert(all_component_versions[k].end(),
				sensor_versions[j].begin(), sensor_versions[j].end());
			k++;
		}
	}
	return all_component_versions;
}

stringvec2d sensorPreprocess(const stringvec &sensor_types)
{
	stringvec uniq_types = unique_vec(sensor_types);
	unsignedvec occur_num(uniq_types.size());
	for (size_t i = 0; i < sensor_types.size(); i++)
	{
		for (size_t j = 0; j < uniq_types.size(); j++)
		{
			if (sensor_types[i] == uniq_types[j])
			{
				occur_num[j]++;
			}
		}
	}

	stringvec2d all_versions(uniq_types.size());
	for (size_t i = 0; i < uniq_types.size(); i++)
	{
		all_versions[i] = getAllSensorVersions(uniq_types[i]);
	}
	all_versions = vectorCombinations(all_versions);

	for (size_t i = 0; i < all_versions.size(); i++)
	{
		stringvec new_versions; // consider number
		for (size_t j = 0; j < occur_num.size(); j++)
		{
			for (size_t k = 0; k < occur_num[j]; k++)
			{
				new_versions.push_back(all_versions[i][j]);
			}
		}
		all_versions[i] = new_versions;
	}

	return all_versions;
}

stringvec2d actuatorPreprocess(const stringvec &actuator_types, const doublepairs &torqs,
	 const doublepairs &vels)
{
	vector<pair<doublepair, doublepair>> torq_vels(torqs.size()),
		dc_motor_torq_vels, servo_motor_torq_vels;
	stringvec uniq_types = unique_vec(actuator_types);
	unsignedvec occur_num(uniq_types.size());
	for (size_t i = 0; i < actuator_types.size(); i++)
	{
		torq_vels[i] = make_pair(torqs[i], vels[i]);
		for (size_t j = 0; j < uniq_types.size(); j++)
		{
			if (uniq_types[j] == actuator_types[i])
			{
				occur_num[j]++;
			}
		}
	}

	for (size_t i = 0; i < occur_num.size(); i++)
	{
		for (size_t j = 0; j < occur_num[i]; j++)
		{
			if (i == 0)
			{
				dc_motor_torq_vels.push_back(torq_vels[j]);
			}
			else
			{
				servo_motor_torq_vels.push_back(torq_vels[j + occur_num[0]]);
			}
		}
	}

	stringvec2d dc_motor_versions, servo_motor_versions;
	for (size_t i = 0; i < uniq_types.size(); i++)
	{
		if (i == 0)
		{
			dc_motor_versions = getAllActuatorVersions(uniq_types[0],
				dc_motor_torq_vels);
		}
		else
		{
			servo_motor_versions = getAllActuatorVersions(uniq_types[1], servo_motor_torq_vels);
		}
	}


	stringvec2d all_motor_versions;
	if (dc_motor_torq_vels.size() == 0)
	{
		all_motor_versions = servo_motor_versions;
	}
	else if (servo_motor_torq_vels.size() == 0)
	{
		all_motor_versions = dc_motor_versions;
	}
	else
	{
		all_motor_versions.resize(dc_motor_versions.size() * servo_motor_versions.size()); // combinations
		size_t k = 0;
		for (size_t i = 0; i < dc_motor_versions.size(); i++)
		{
			for (size_t j = 0; j < servo_motor_versions.size(); j++)
			{
				all_motor_versions[k].insert(all_motor_versions[k].end(),
					dc_motor_versions[i].begin(), dc_motor_versions[i].end());
				all_motor_versions[k].insert(all_motor_versions[k].end(),
					servo_motor_versions[j].begin(), servo_motor_versions[j].end());
				k++;
			}
		}
	}

	return all_motor_versions;
}

infernodevec2d initialize(const stringvec2d &component_versions,
	const doublepairs &torqs, const doublepairs &vels)
{
	infernodevec2d infer_nodes_vec(component_versions.size());
	for (size_t i = 0; i < component_versions.size(); i++)
	{
		for (size_t j = 0; j < component_versions[i].size(); j++)
		{
			Infer_Node node;
			if (j < torqs.size())
			{
				shared_ptr<Motor> motor = make_shared<Motor>(component_versions[i][j]);
				motor->setWorkPoint(torqs[j].first, vels[j].second);
				node = Infer_Node(motor);
			}
			else
			{
				node = Infer_Node(component_versions[i][j]);
			}
			infer_nodes_vec[i].push_back(node);
		}
	}
	return infer_nodes_vec;
}

bbnodevec initialize(const infernodevec2d &infer_nodes_vec)
{
	bbnodevec bbnodes;
	for (size_t i = 0; i < infer_nodes_vec.size(); i++)
	{
		bbnodes.emplace_back(BBNode(infer_nodes_vec[i]));
	}
	return bbnodes;
}

vector<stringpair> postprocessing(const Pin_Connections &connections,
	const stringvec &component_names, const unsignedvec &motor_index,
	const unsignedvec &encoder_index)
{
	vector<stringpair> processed_connections, final_connections;
	unsignedpairs connection_group_index(component_names.size());

	// reorder left side connections
	unsigned cnt = 0;
	for (unsigned i = 0; i < component_names.size(); i++)
	{
		connection_group_index[i].first = cnt;
		for (auto beg = connections.begin(); beg != connections.end(); beg++)
		{
			stringpair name_pair = separateNames(beg->first);
			if (component_names[i] == name_pair.first &&
				!isDutyCycle(name_pair.second))
			{
				processed_connections.push_back(*beg);
				cnt++;
			}
		}
		connection_group_index[i].second = cnt;
	}

	// reorder right side connections
	for (size_t i = 0; i < connection_group_index.size(); i++)
	{
		for (size_t j = 0; j < component_names.size(); j++)
		{
			for (size_t k = connection_group_index[i].first;
				k < connection_group_index[i].second; k++)
			{
				stringpair name_pair = separateNames(processed_connections[k].second);
				if (component_names[j] == name_pair.first)
				{
					final_connections.push_back(processed_connections[k]);
				}
			}
		}
	}

	// replace dc motor with encoder connections
	if (encoder_index.size())
	{
		for (size_t i = 0; i < connection_group_index.size(); i++)
		{
			for (size_t j = connection_group_index[i].first;
				j < connection_group_index[i].second; j++)
			{
				stringpair name_pair = separateNames(final_connections[j].second);
				unsigned right_name_index = getPosInVec(name_pair.first, component_names),
					matched_index = getPosInVec(right_name_index, motor_index);
				if (matched_index != -1)
				{
					string replace_name = replaceConnections(final_connections[j].second,
						component_names[encoder_index[matched_index]]);
					final_connections[j].second = replace_name;
				}
			}
		}
	}

	return final_connections;
}

std::string replaceConnections(const std::string &origin,
	const std::string &replacement)
{
	stringpair name_pair = separateNames(origin);
	if (name_pair.second == "+")
	{
		return replacement + ".M1";
	}
	else
	{
		return replacement + ".M2";
	}
}

doublevec getMassVec(const BBNode &node,
	const unsigned &input_size)
{
	doublevec mass_vec(input_size + 1);
	vector<shared_ptr<Electrical_Component>> components =
		node.circuit.getComponents();
	for (size_t i = 0; i < components.size(); i++)
	{
		if (i < input_size)
		{
			mass_vec[i] = components[i]->getWeight();
		}
		else
		{
			mass_vec[input_size] += components[i]->getWeight();
		}
	}
	return mass_vec;
}

bool backTrack(BBNode &bbnode)
{
	return false;
}

stringvec typeInfer(const Infer_Node &node)
{
	return typeInfer(node.getType());
}

stringvec typeInfer(const std::string &type)
{
	const auto &iter = type_infer_map.find(type);
	return type_infer_map.find(type)->second;
}

stringvec removeEmptyTypes(const stringvec &types)
{
	stringvec non_empty_types;
	for (size_t i = 0; i < types.size(); i++)
	{
		if (types[i] != "")
		{
			non_empty_types.push_back(types[i]);
		}
	}

	return non_empty_types;
}

boolvec hasMatchComponents(const stringvec &types, const infernodevec &next_nodes,
	const Infer_Node &node, const Pin_Connections &pin_connections)
{
	boolvec type_vec(types.size());
	doublepairs power_in_vol_ranges = node.getPowerInVolRange(),
		func_in_vol_ranges = node.getFuncInVolRange();
	for (size_t i = 0; i < types.size(); i++)
	{
		size_t next_component_size = 0;
		bool next_power_component = false;
		if (in_vol_map[make_pair(node.getType(), types[i])] ==
			Electronics::POWER)
		{
			next_component_size = node.getPowerInVolRange().size();
			next_power_component = true;
		}
		else
		{
			next_component_size = node.getFuncInVolRange().size();
		}

		for (size_t j = 0; j < next_nodes.size(); j++)
		{
			if (next_nodes[j].getType() == types[i] ||
				(next_nodes[j].getType() == Component_Type::Battery
					&& types[i] == Component_Type::Power_Supply) ||
				(next_nodes[j].getType() == Component_Type::Voltage_Regulator
					&& types[i] == Component_Type::Power_Supply))
			{
				stringvec out_pins = next_nodes[j].getOutPinNames(node.getType()),
					in_pins = node.getInPinNames(next_nodes[j].getType());

				for (size_t k = 0; k < out_pins.size(); k++)
				{
					for (size_t l = 0; l < in_pins.size(); l++)
					{
						string left = createConnectionName(next_nodes[j].getName(), out_pins[k]),
							right = createConnectionName(node.getName(), in_pins[l]);
						const auto &range = pin_connections.equal_range(left);
						for (auto beg = range.first; beg != range.second; beg++)
						{
							if (beg->second == right) {
								if (next_power_component)
								{
									doublepair power_out_vol_range = next_nodes[j].getPowerOutVolRange();
									for (auto beg = power_in_vol_ranges.begin();
										beg != power_in_vol_ranges.end(); beg++)
									{
										if (isIntersect(*beg, power_out_vol_range))
										{
											node.setPowerExistVec(beg - power_in_vol_ranges.begin());
										}
									}
								}
								else
								{
									doublepair func_out_vol_range = next_nodes[j].getFuncOutVolRange();
									for (auto beg = func_in_vol_ranges.begin();
										beg != func_in_vol_ranges.end(); beg++)
									{
										if (isIntersect(*beg, func_out_vol_range))
										{
											node.setFuncExistVec(beg - func_in_vol_ranges.begin());
										}
									}
								}
								next_component_size--;
								goto RESTART;
							}
						}
					}
				}
			}
		RESTART: int a = 1; // not used
		}

		if (next_component_size == 0)
		{
			type_vec[i] = true;
		}
	}

	return type_vec;
}
/*
stringvec2d versionInfer(const string &type,
	const doublepairs &pow_vol_ranges, const doublepairs &func_vol_ranges,
	const doublevec &current_set)
{
	unordered_map<string, pair<doublepairs, doublevec>> infer_code;
	infer_code[Component_Type::H_Bridge] = make_pair(pow_vol_ranges,
		current_set);
	infer_code[Component_Type::Micro_Controller] = make_pair(func_vol_ranges,
		current_set);
	infer_code[Component_Type::Voltage_Regulator] = make_pair(pow_vol_ranges,
		current_set);
	infer_code[Component_Type::Battery] = make_pair(pow_vol_ranges,
		current_set);

	return versionInfer(type, infer_code[type].first,
		infer_code[type].second);
}
*/
stringvec2d versionInfer(const string &type, const doublepairs &vol_ranges,
	const doublevec2d &current_set)
{
	stringvec2d versions(vol_ranges.size());
	for (size_t i = 0; i < vol_ranges.size(); i++)
	{
		versions[i] = versionInfer(type, vol_ranges[i], current_set[i]);
	}
	return versions;
}

stringvec versionInfer(const string &type, const doublepair &vol_range, const doublevec &currents)
{
	stringvec versions;
	doublepair vol_output_range;
	double current_limit;

	unordered_map<string, unordered_map<string, shared_ptr<Electrical_Component>>>
		all_component_map = { {Component_Type::Motor, MOTOR_PART_MAP},
		{Component_Type::H_Bridge, H_BRIDDE_PART_MAP},
		{Component_Type::Micro_Controller, MICRO_CONTROLLER_PART_MAP},
		{Component_Type::Voltage_Regulator, VOLTAGE_REGULATOR_PART_MAP},
		{Component_Type::Battery, BATTERY_PART_MAP},
		{Component_Type::Encoder, ENCODER_PART_MAP},
		{Component_Type::Camera, CAMERA_PART_MAP},
		{Component_Type::Force_Sensor, FORCE_SENSOR_PART_MAP},
		{Component_Type::Bluetooth, BLUETOOTH_PART_MAP},
		{Component_Type::Servo, SERVO_PART_MAP},
		{Component_Type::Power_Supply, POWER_SUPPLY_PART_MAP}
	};

	unordered_map<string, shared_ptr<Electrical_Component>> components =
		all_component_map[type];

	for (auto beg = components.begin(); beg != components.end(); beg++)
	{
		vol_output_range = beg->second->getOutVolRange(out_vol_map[type]);
		current_limit = beg->second->getOutCurrentLimit(out_vol_map[type]);

		if (isIntersect(vol_range, vol_output_range) &&
			in_current_map[type](currents) <= current_limit)
		{
			versions.push_back(beg->first);
		}
	}

	return versions;
}

// infernodevec numberInfer(const string &type, infernodevec &prev_nodes,
//	const stringvec &versions, const cliqueindex &power_clique_index,
//	const cliqueindex &func_clique_index)
// {

//	unordered_map<string, cliqueindex> index_code;
//	index_code[Component_Type::H_Bridge] = func_clique_index;
//	index_code[Component_Type::Micro_Controller] = func_clique_index;
//	index_code[Component_Type::Voltage_Regulator] = power_clique_index;
//	index_code[Component_Type::Battery] = power_clique_index;
//	return numberInfer(type, prev_nodes, versions, index_code[type]);
// }

infernodevec numberInfer(const string &type, infernodevec &prev_nodes,
	const stringvec &versions, const cliqueindex &nodes_index,
	const cliqueindex &vol_vol_index)
{
	infernodevec next_infer_nodes;
	double prev_val = 0.0, next_val = 0.0;
	for (size_t i = 0; i < nodes_index.size(); i++)
	{
		Infer_Node infer_node = Infer_Node(versions[i]);
		if (!BBNode::pruning_enable &&
			hasTwoVoltageRegulators(versions[i], prev_nodes[nodes_index[i][0]].getType()))
		{
			return infernodevec();
		}
		next_infer_nodes.push_back(infer_node);
		prev_nodes[nodes_index[i][0]].addNextNode(*next_infer_nodes.rbegin());

		// for actuators: input
		// for sensors: input, output: digital/uart/spi/i2c

		next_val = next_infer_nodes.rbegin()->getOutVal(out_vol_map[type]);
		for (size_t j = 0; j < nodes_index[i].size(); j++)
		{
			prev_val += prev_nodes[nodes_index[i][j]].getInVal(type)[vol_vol_index[i][j]];
			if (prev_val > next_val)
			{
				Infer_Node infer_node = Infer_Node(versions[i]);
				if (!BBNode::pruning_enable &&
					hasTwoVoltageRegulators(versions[i], prev_nodes[nodes_index[i][j]].getType()))
				{
					return infernodevec();
				}
				next_infer_nodes.push_back(infer_node);
				prev_nodes[nodes_index[i][j]].addNextNode(*next_infer_nodes.rbegin());
				next_val += next_infer_nodes.rbegin()->getOutVal(out_vol_map[type]);
			}
			else
			{
				if (j != 0)
				{
					if (!BBNode::pruning_enable &&
						hasTwoVoltageRegulators(versions[i], prev_nodes[nodes_index[i][j]].getType()))
					{
						return infernodevec();
					}
					prev_nodes[nodes_index[i][j]].addNextNode(*next_infer_nodes.rbegin());
				}
			}
		}
	}
	return next_infer_nodes;
}

cliquetype exactrRangeCover(const doublepairs &ranges)
{
	doublepairs cliquerange;
	cliqueindex cliqueindex;
	if (!ranges.empty())
	{
		intvec num_sequence(ranges.size());
		std::iota(num_sequence.begin(), num_sequence.end(), 0);

		for (size_t clique_size = ranges.size(); clique_size != 1; clique_size--)
		{
			const auto &X = discreture::combinations(num_sequence, clique_size);
			for (auto &&x : X)
			{
				doublepairs temppairs(x.size());
				auto tpbeg = temppairs.begin();
				for (auto beg = x.begin(); beg != x.end(); beg++)
				{
					*tpbeg++ = ranges[*beg];
				}

				if (std::adjacent_find(temppairs.begin(), temppairs.end(), std::not_equal_to<doublepair>()) == temppairs.end())
				{
					// clique exists
					cliquerange.insert(cliquerange.end(), temppairs[0]);
					cliqueindex.insert(cliqueindex.end(), intvec(x.begin(), x.end()));

					// remove from graph
					for (auto i = x.begin(); i != x.end(); i++)
					{
						num_sequence.erase(std::find(num_sequence.begin(), num_sequence.end(), *i));
					}

					if (num_sequence.size() == 0)
					{
						goto endloop;
					}

					clique_size = num_sequence.size() + 1;
					break;
				}
			}
		}

		for (auto beg = num_sequence.begin(); beg != num_sequence.end(); beg++)
		{
			cliquerange.insert(cliquerange.end(), ranges[*beg]);
			cliqueindex.insert(cliqueindex.end(), intvec{ *beg });
		}
	}
endloop:
	return std::make_tuple(cliquerange, cliqueindex);
}
/*
doublevec getCurrentClique(const cliqueindex &index, const doublevec &current_set)
{
	doublevec current_clique_set(index.size());
	auto &ccbeg = current_clique_set.begin();
	for (auto &beg = index.begin(); beg != index.end(); beg++)
	{
		doublevec temp_vec(beg->size());
		auto &tbeg = temp_vec.begin();
		for (auto &cbeg = beg->begin(); cbeg != beg->end(); cbeg++)
		{
			*tbeg = current_set[*cbeg];
		}
		*ccbeg++ = *std::max_element(temp_vec.begin(), temp_vec.end());
	}
	return current_clique_set;
}

doublevec changeCurrentSet(doublevec &current_set, const connect_relation &relation, const unsigned &num)
{
	doublevec new_current_set(num);
	unsigned refer_index = 0;
	double current_sum = 0;
	auto &ncbeg = new_current_set.begin();
	for (auto &rbeg = relation.begin(); rbeg != relation.end(); rbeg++)
	{
		if (rbeg->second != refer_index)
		{
			*ncbeg++ = current_sum;
			current_sum = current_set[rbeg->first];
			refer_index = rbeg->second;
		}
		else
		{
			current_sum += current_set[rbeg->first];
		}
	}
	*new_current_set.rbegin() = current_sum;
	return new_current_set;
}

void findMatchNodes(BBNode &bbnode, const std::string &type, bbnodevec &match_nodes)
{
	for (auto &nvbeg = bbnode.nodes_vec.begin(); nvbeg != bbnode.nodes_vec.end(); nvbeg++)
	{
		for (auto &nbeg = nvbeg->begin(); nbeg != nvbeg->end(); nbeg++)
		{
			if (nbeg->type == type ||
				type == power_supply_type && nbeg->type == voltage_regulator_type || nbeg->type == battery_type)
			{
				match_nodes.push_back(bbnode);
			}
		}
	}
	findMatchNodes(*bbnode.prev_bbnode, type, match_nodes);
}
*/
/*
void findMatchNodes(const nodeptrvec &roots, const std::string &type, nodeptrvec &match_nodes)
{
	for (auto &rnbeg = roots.begin(); rnbeg != roots.end(); rnbeg++)
	{
		if (*rnbeg && (*rnbeg)->type == type ||
			type == power_supply_type && (*rnbeg)->type == voltage_regulator_type || (*rnbeg)->type == battery_type)
		{
			match_nodes.push_back(*rnbeg);
		}
		findMatchNodes((*rnbeg)->nextnodes, type, match_nodes);
	}
}
*/
/*
bool findMatchBattery(const doublepair &power_range, const double &current)
{
	for (auto &entry : std::filesystem::directory_iterator(battery_path))
	{
		Battery obj(entry.path().string());
		if (intersect(power_range, obj.getPowerOutputSet()) &&
			current <= obj.getPowerOutputCurrentLimit())
		{
			return true;
		}
	}
	return false;
}

connection_relation sameLevelComponentsReduction(BBNode &bbnode)
{
	connection_relation relations;
	std::vector<std::pair<Connection_Structure &, Connection_Structure &>> component_relations;
	auto &node_vec = bbnode.nodes_vec;

	// find usable battery
	if (node_vec.size() > 1 && node_vec[0].begin()->type == battery_type && node_vec[1].begin()->type == micro_controller_type)
	{
		auto &pcrange = bbnode.power_clique_range[1];
		auto &pcindex = bbnode.power_clique_index[1];
		auto &pcibeg = pcindex.begin();
		for (auto &pcrbeg = pcrange.begin(); pcrbeg != pcrange.end(); pcrbeg++, pcibeg++)
		{
			double sum_of_current = 0;
			for (auto &ibeg = pcibeg->begin(); ibeg != pcibeg->end(); ibeg++)
			{
				sum_of_current += node_vec[1][*ibeg].component_used_for_connection.getUsedOutputPinNum()*0.01;
			}

			for (auto &bbeg = node_vec[0].begin(); bbeg != node_vec[0].end(); bbeg++)
			{
				if (inBatteryFolder(bbeg->version))
				{
					if (intersect(*pcrbeg, Battery(bbeg->version).getPowerOutputSet()) &&
						bbeg->extra_current >= sum_of_current)
					{
						for (auto &ibeg = pcibeg->begin(); ibeg != pcibeg->end(); ibeg++)
						{
							node_vec[1][*ibeg].type = degenerate_micro_controller_type;
							node_vec[1][*ibeg].nextnodes.push_back(*bbeg);
							bbeg->prevnodes.push_back(node_vec[1][*ibeg]);

							// generate connection
							component_relations.push_back(std::make_pair(
								std::reference_wrapper<Connection_Structure>(bbeg->component_used_for_connection),
								std::reference_wrapper<Connection_Structure>(node_vec[1][*ibeg].component_used_for_connection)));
						}
					}
				}
			}
		}

	}
	Component_Connection TestConnection(component_relations);
	TestConnection.components_matching();
	relations = TestConnection.get_netlist();
	return relations;
}

connection_relation_vec sameLevelComponentsReduction(bbnodevec &bbnode_vec)
{
	connection_relation_vec relations_vec;
	for (auto &bvbeg = bbnode_vec.begin(); bvbeg != bbnode_vec.end(); bvbeg++)
	{
		connection_relation relations = sameLevelComponentsReduction(*bvbeg);
		if (!relations.empty())
		{
			relations_vec.insert(relations_vec.end(), relations);
		}
	}
	return relations_vec;
}
*/
/*
nodeptr findAvailPowerComponent(nodeptrvec &match_nodes, const stringvec &versions, const double &input_current)
{
	for (auto &mbeg = match_nodes.begin(); mbeg != match_nodes.end(); mbeg++)
	{
		if (std::find(versions.begin(), versions.end(), (*mbeg)->version) != versions.end() &&
			(*mbeg)->extra_current >= input_current)
		{
			(*mbeg)->extra_current -= input_current;
			return (*mbeg);
		}
	}
	return nodeptr();
}
*/

/*
void treeVisualize(std::shared_ptr<InferredNode> root, unsigned cnt)
{
}
*/
/*
std::string indexComponent(std::string component)
{
	std::string post_fix = "";
	auto tpos = component.rfind("t") + 1, endpos = component.rfind(*component.rbegin()) + 2;
	*component.rbegin() == 't' ? post_fix : post_fix = ("_" + component.substr(tpos, endpos - tpos));

	return removeComponentPrePostfix(component) + post_fix;
}
*/
/*
void writeTree(const nodeptrvec &roots, std::ofstream &output, str_unsigned_uomap &components_map)
{
	for (auto &rbeg = roots.begin(); rbeg != roots.end(); rbeg++)
	{
		if (*rbeg && !(*rbeg)->visited)
		{
			(*rbeg)->visited = true;
			std::string simp_name = removeComponentPrePostfix((*rbeg)->version);
			auto &iter = components_map.find(simp_name);
			if (iter == components_map.end())
			{
				components_map.insert(std::make_pair(simp_name, 1));
			}
			else
			{
				iter->second++;
			}

			if (!(*rbeg)->nextnodes.empty())
			{
				writeTree((*rbeg)->nextnodes, output, components_map);
			}
		}
	}
}
*/
/*
void writeDesign(const nodeptrvec &roots, const Circuit &circuit, str_unsigned_uomap &components_map)
{
	std::ofstream design;
	design.open(design_path + design_pf + "_" + std::to_string(getFileNumInDirectory(design_path)) + text_pf);
	writeTree(roots, design, components_map);
	for (auto &cmbeg = components_map.begin(); cmbeg != components_map.end(); cmbeg++)
	{
		if (cmbeg->second != 1)
		{
			design << "COMMENTS: For multiple same components in the file, " <<
				"they are differentiated by index in the end" << std::endl;
			design << std::endl;
			break;
		}
	}

	design << "COMPONENTS: \n";
	design << std::setw(20) << std::left << "Version" << "|" << std::setw(20) << std::right << "Number \n";
	design << sline + "\n";
	for (auto &cmbeg = components_map.begin(); cmbeg != components_map.end(); cmbeg++)
	{
		design << std::setw(20) << std::left << cmbeg->first << "|" << std::setw(15) << std::right << cmbeg->second << std::endl;
	}

	design << "\n";
	design << "CONNECTIONS: \n";
	auto &relations = circuit.getComponentRelations();
	for (auto &relation = relations.begin(); relation != relations.end(); relation++)
	{
		if (relation->second.first.substr(0, 4) != duty && relation->second.second.substr(0, 4) != duty)
		{
			design << std::setw(25) << std::left << indexComponent(relation->first.first) + ": " + relation->second.first
				<< "--------"
				<< std::setw(25) << std::right << indexComponent(relation->first.second) + ": " + relation->second.second << "\n";
		}
	}
	design.close();
}
*/
void writeDesign(const BBNode &bbnode)
{
    // check existence of diectory
    if (!std::filesystem::exists(design_path)){
        std::filesystem::create_directory(design_path);
    }

	std::ofstream design;
	design.open(design_path + design_pf + "_" + std::to_string(getFileNumInDirectory(design_path)) + text_pf);

	// component list
	design << "COMPONENT LIST: " << endl;
	for (size_t i = 0; i < bbnode.component_names.size(); i++)
	{
		design << bbnode.component_names[i] << "\n";
	}
	design << endl;

	// connection list
	vector<shared_ptr<Electrical_Component>> components = bbnode.circuit.getComponents();
	unsignedvec motor_index, encoder_index;
	for (size_t i = 0; i < components.size(); i++)
	{
		if (components[i]->getComponentType() == Component_Type::Encoder)
		{
			encoder_index.push_back(i);
		}
		else if (components[i]->getComponentType() == Component_Type::Motor)
		{
			motor_index.push_back(i);
		}
	}
	Pin_Connections connections = bbnode.circuit.getComponentRelations();
	vector<stringpair> processed_connections = postprocessing(connections, bbnode.component_names,
		motor_index, encoder_index);
	for (auto beg = processed_connections.begin(); beg != processed_connections.end(); beg++)
	{
		design << beg->first << " -- "  << beg->second << "\n";
	}
	design << endl;

	design << "METRICS:" << endl;
	design << "NUM OF COMPONENTS: " << bbnode.getComponenetNum() << endl;
	design << "NUM OF COMPONENT PINS: " << bbnode.getPinNum() << endl;
	design << "NUM OF CONNECTIONS: " << processed_connections.size() << endl;
	design << "PRICE:  " << bbnode.getPrice() << endl;
	design << "WEIGHT: " << bbnode.getWeight() << endl;
	design << "POWER: " << bbnode.getPowerConsump() << endl;
	design << endl;

	design.close();

    std::cout << "Design file has been written to : " << design_path << std::endl;
}
/*
void writeRawDesign(const BBNode &bbnode)
{
	std::ofstream design, metrics;
	str_unsigned_uomap components_map;
	design.open(design_path + design_pf + std::to_string(getFileNumInDirectory(design_path)-1) + raw_text_pf);

	std::vector<Actu_components> components = bbnode.circuit.getComponents();

	design << "COMPONENTS: \n";
	for (auto &cbeg = components.begin(); cbeg != components.end(); cbeg++)
	{
		design << cbeg->getName() << std::endl;
	}

	design << "CONNECTIONS: \n";
	auto &relations = bbnode.final_connections;
	for (auto &relation = relations.begin(); relation != relations.end(); relation++)
	{
		design << relation->first.first + " " + relation->first.second << " " <<
		 relation->second.first + " "  + relation->second.second << std::endl;
	}
	design.close();

	metrics.open(design_path + "metric.txt");
	metrics << bbnode.returnComponenetNum() << std::endl;
	metrics << bbnode.returnComponentsPinNum() << std::endl;
	metrics << bbnode.returnConnectionsNum() << std::endl;
	metrics << bbnode.returnPrice() << std::endl;
	metrics << bbnode.returnWeight() << std::endl;
	metrics << bbnode.returnPower() << std::endl;

	metrics << std::endl;
	metrics << bbnode.motor_mass.size() << std::endl;
	for (auto &mmbeg = bbnode.motor_mass.begin(); mmbeg != bbnode.motor_mass.end(); mmbeg++)
	{
		metrics << *mmbeg << std::endl;
	}
	metrics.close();
}
*/
/*
std::tuple<nodeptrvec2d, unsignedvec2d, connection_relation_vec>
identifyTypes(const nodeptrvec &nodes, const connection_relation_vec &relations_vec)
{
	unsignedvec2d index_vec;
	nodeptrvec2d nodes_vec;
	connection_relation_vec new_relations_vec;
	stringvec type_vec;
	auto &rbeg = relations_vec.begin();
	for (auto& nbeg = nodes.begin(); nbeg < nodes.end(); nbeg++)
	{
		auto &iter = std::find(type_vec.begin(), type_vec.end(), (*nbeg)->type);
		if (iter == type_vec.end())
		{
			type_vec.push_back((*nbeg)->type);
			index_vec.push_back(unsignedvec());
			nodes_vec.push_back(nodeptrvec());
			index_vec.rbegin()->push_back(nbeg - nodes.begin());
			nodes_vec.rbegin()->push_back(nodes[nbeg - nodes.begin()]);
			if (!relations_vec.empty())
			{
				new_relations_vec.push_back(connection_relation());
				*new_relations_vec.rbegin() = *rbeg++;
			}
		}
		else
		{
			index_vec[iter - type_vec.begin()].push_back(nbeg - nodes.begin());
			nodes_vec[iter - type_vec.begin()].push_back(nodes[nbeg - nodes.begin()]);
			if (!relations_vec.empty())
			{
				new_relations_vec[iter - type_vec.begin()].insert(new_relations_vec[iter - type_vec.begin()].end(),
					rbeg->begin(), rbeg->end());
				rbeg++;
			}
		}
	}

	if (relations_vec.empty())
	{
		new_relations_vec.resize(nodes_vec.size());
	}
	return std::make_tuple(nodes_vec, index_vec, new_relations_vec);
}
*/
/*
std::tuple<infernodevec2d, unsignedvec2d> identifyTypes(const infernodevec &nodes)
{
	unsignedvec2d index_vec;
	infernodevec2d nodes_vec;
	stringvec type_vec;
	for (auto& nbeg = nodes.begin(); nbeg < nodes.end(); nbeg++)
	{
		auto &iter = std::find(type_vec.begin(), type_vec.end(), nbeg->type);
		if (iter == type_vec.end())
		{
			type_vec.push_back(nbeg->type);
			index_vec.push_back(unsignedvec());
			nodes_vec.push_back(infernodevec());
			index_vec.rbegin()->push_back(nbeg - nodes.begin());
			nodes_vec.rbegin()->push_back(nodes[nbeg - nodes.begin()]);
		}
		else
		{
			index_vec[iter - type_vec.begin()].push_back(nbeg - nodes.begin());
			nodes_vec[iter - type_vec.begin()].push_back(nodes[nbeg - nodes.begin()]);
		}
	}

	return std::make_tuple(nodes_vec, index_vec);
}
*/
/*
nodeptrvec getAproNodes(const nodeptrvec &nodes)
{
	nodeptrvec apronodes;
	for (auto &nbeg = nodes.begin(); nbeg != nodes.end(); nbeg++)
	{
		if (!typeInfer(*nbeg).empty())
		{
			apronodes.push_back(*nbeg);
		}
	}
	return apronodes;
}
*/
/*
infernodevec generateNodeptrVec(const std::string &type, const stringvec &versions)
{

	infernodevec nodes(versions.size());
	stringvec component_names, initial_pin_names{ "LEAD1", "LEAD2" };
	auto &nbeg = nodes.begin();
	for (auto &vbeg = versions.begin(); vbeg != versions.end(); vbeg++)
	{
		*nbeg++ = InferredNode(type, *vbeg);
	}

	nbeg = nodes.begin();
	for (auto &vbeg = versions.begin(); vbeg != versions.end(); vbeg++, nbeg++)
	{
		unsigned copy_pf = 0;
		std::string component_name = connection_path_convert(*vbeg);
		nbeg->component_used_for_connection = Connection_Structure(component_name, initial_pin_names);
		component_name = makeReplicate(component_names, component_name, copy_pf);
		component_names.push_back(component_name);
		nbeg->component_used_for_connection.setComponentName(component_name);
	}
	return nodes;
}
*/

/*
bool inputsValidityCheck(const doublepairs &torqs, const doublepairs &vels)
{
	if (torqs.empty())
	{
		throw EMPTY_TORQUE;
	}

	if (vels.empty())
	{
		throw EMPTY_VELOCITY;
	}

	if (torqs.size() != vels.size())
	{
		throw TORQUE_VELOCITY_SIZE_NOT_MATCH;
	}

	for (size_t i = 0; i < torqs.size(); i++)
	{
		if (torqs[i].first > torqs[i].second)
		{
			throw TORQUE_RANGE_INVALID;
		}

		if (torqs[i].first < 0 || torqs[i].second < 0)
		{
			throw NEGATIVE_TORQUE;
		}

		if (vels[i].first > vels[i].second)
		{
			throw VELOCITY_RANGE_INVALID;
		}

		if (vels[i].first < 0 || vels[i].second < 0)
		{
			throw NEGATIVE_VELOCITY;
		}
	}
	return true;
}
*/

shared_ptr<BBNode> branchNBound(bbnodevec *roots)
{
	static bool first_branch = true;
	static shared_ptr<BBNode> best_node;

	bbnodevec descendents;
	for (size_t i = 0; i < roots->size(); i++)
	{
		descendents.clear();
		if (!roots->at(i).empty())
		{
			roots->at(i).evaluate();
			if (!first_branch && BBNode::pruning_enable)
			{
				roots->at(i).bound();
			}

			if (roots->at(i).feasibility)
			{
				descendents = roots->at(i).branch();
			}

			if (!descendents.size())
			{
				if (first_branch)
				{
					roots->at(i).reevaluate();
					BBNode::best_metric_val = roots->at(i).getMetricVal();
					BBNode::metric_vec.push_back(make_pair(BBNode::num_of_bbnodes,
						BBNode::best_metric_val));

					// get best node
					best_node =  make_shared<BBNode>(roots->at(i).infer_nodes, roots->at(i).circuit,
						roots->at(i).model, roots->at(i).prev_bbnode);
					best_node->copyMetrics(roots->at(i));
					roots->at(i).prev_bbnode->next_bbnodes.pop_back();
					first_branch = false;
				}
				else if (roots->at(i).feasibility)
				{
					roots->at(i).reevaluate();
					if (BBNode::best_metric_val > roots->at(i).getMetricVal())
					{
						BBNode::best_metric_val = roots->at(i).getMetricVal();
						BBNode::metric_vec.push_back(make_pair(BBNode::num_of_bbnodes,
							BBNode::best_metric_val));

						best_node = make_shared<BBNode>(roots->at(i).infer_nodes, roots->at(i).circuit,
							roots->at(i).model, roots->at(i).prev_bbnode);
						best_node->copyMetrics(roots->at(i));
						roots->at(i).prev_bbnode->next_bbnodes.pop_back();
					}
				}
			}
			else
			{
				best_node = branchNBound(&descendents);
			}
		}
	}
	return best_node;
}

/*
void expandMap(const bbnodevec &bbnodes, bbglobalmap &gmap)
{
	for (auto &bnbeg = bbnodes.begin(); bnbeg != bbnodes.end(); bnbeg++)
	{
		if (!bnbeg->empty())
		{
			gmap.push_back(std::make_tuple(*bnbeg, bnbeg->level));
		}
	}
}

BBNode getOptimalDesign(const bbglobalmap &gmap, const int &index)
{
	for (auto &gbeg = gmap.begin(); gbeg != gmap.end(); gbeg++)
	{
		BBNode bbnode = std::get<0>(*gbeg);
		if (bbnode.id == index)
		{
			return bbnode;
		}
	}
}
*/

/*
compstructptrvec getComponentConnections(infernodevec &nodes)
{
	compstructptrvec connection_components(nodes.size());
	auto &cbeg = connection_components.begin();
	for (auto &nbeg = nodes.begin(); nbeg != nodes.end(); nbeg++, cbeg++)
	{
		*cbeg = &nbeg->component_used_for_connection;
	}
	return connection_components;
}
*/

Pin_Connections maxNodeMatch(BBNode &bbnode, infernodevec &infer_nodes)
{
	Pin_Connections pin_connections, prev_pin_connections;
	if (!bbnode.prev_bbnode->empty())
	{
		prev_pin_connections = maxNodeMatch(*bbnode.prev_bbnode, infer_nodes);
	}

	infernodevec all_infer_nodes = infer_nodes;
	all_infer_nodes.insert(all_infer_nodes.end(),
		bbnode.infer_nodes.begin(), bbnode.infer_nodes.end());
	unsignedpair prev_ranges = make_pair(infer_nodes.size(), all_infer_nodes.size())
		, ranges = make_pair(0, infer_nodes.size());


	// TO DO: break into 2 parts: 1. previous nodes - current nodes
	//                            2. cuurent nodes - current nodes

	pin_connections = nodeMatch(all_infer_nodes, prev_ranges, ranges);
	getDependentPins(all_infer_nodes, pin_connections);
	infer_nodes = infernodevec(all_infer_nodes.begin(),
		all_infer_nodes.begin() + infer_nodes.size());
	bbnode.infer_nodes = infernodevec(all_infer_nodes.begin() +
		infer_nodes.size(), all_infer_nodes.end());

	pin_connections.insert(prev_pin_connections.begin(), prev_pin_connections.end());
	return pin_connections;
}

vector<Component_Pair> extractComponentPairs(infernodevec &infer_nodes)
{
	vector<Component_Pair> component_pairs;
	stringvec component_types;
	for (size_t i = 0; i < infer_nodes.size(); i++)
	{
		component_types.push_back(infer_nodes[i].getType());
	}

	for (size_t i = 0; i < component_types.size(); i++)
	{
		const auto &range = connection_map.equal_range(component_types[i]);
		for (auto beg = range.first; beg != range.second; beg++)
		{
			for (size_t j = 0; j < component_types.size(); j++)
			{
				if (beg->second == component_types[j])
				{
					component_pairs.push_back(make_pair(infer_nodes[i].getComponent(),
						infer_nodes[j].getComponent()));
				}
			}
		}
	}

	std::sort(component_pairs.begin(), component_pairs.end(),
		[](Component_Pair &left, Component_Pair &right) {
		return precedence_map[left.first->getComponentType()] >
			precedence_map[right.first->getComponentType()]; });
	return component_pairs;
}

std::vector<Component_Pair> extractComponentPairs(infernodevec &infer_nodes,
	const unsignedpair &prev_ranges, const unsignedpair &ranges)
{
	vector<Component_Pair> component_pairs;
	stringvec component_types;
	for (size_t i = 0; i < infer_nodes.size(); i++)
	{
		component_types.push_back(infer_nodes[i].getType());
	}

	for (size_t i = 0; i < component_types.size(); i++)
	{
		const auto &range = connection_map.equal_range(component_types[i]);
		for (auto beg = range.first; beg != range.second; beg++)
		{
			for (size_t j = 0; j < component_types.size(); j++)
			{
				if ((!(prev_ranges.first <= i && i <= prev_ranges.second) ||
					!(prev_ranges.first <= j && j <= prev_ranges.second)) &&
					beg->second == component_types[j])
				{
					component_pairs.push_back(make_pair(infer_nodes[i].getComponent(),
						infer_nodes[j].getComponent()));
				}
			}
		}
	}

	std::sort(component_pairs.begin(), component_pairs.end(),
		[](Component_Pair &left, Component_Pair &right) {
		return precedence_map[left.first->getComponentType()] >
			precedence_map[right.first->getComponentType()]; });
	return component_pairs;
}

vector<Component_Pair> extractConsecutiveComponentPairs(infernodevec &prev_infer_nodes, infernodevec &infer_nodes)
{
	vector<Component_Pair> component_pairs;
	for (size_t i = 0; i < infer_nodes.size(); i++)
	{
		const infernodevec &ancestors = infer_nodes[i].getAncestors();
		for (size_t j = 0; j < ancestors.size(); j++)
		{
			size_t pos = getPosInVec(ancestors[j], prev_infer_nodes);
			if (pos != -1)
			{
				component_pairs.push_back(make_pair(infer_nodes[i].getComponent(),
					prev_infer_nodes[pos].getComponent()));
			}
		}
	}

	std::sort(component_pairs.begin(), component_pairs.end(),
		[](Component_Pair &left, Component_Pair &right) {
		return precedence_map[left.first->getComponentType()] >
			precedence_map[right.first->getComponentType()]; });
	return component_pairs;
}

Pin_Connections nodeMatch(infernodevec &infer_nodes)
{
	// add relations
	vector<Component_Pair> component_pairs =
		extractComponentPairs(infer_nodes);
	Pin_Connections pin_connections = groupMatch(component_pairs);
	createLinks(infer_nodes, pin_connections);
	return pin_connections;
}

Pin_Connections nodeMatch(infernodevec &infer_nodes,
	const unsignedpair &prev_ranges, const unsignedpair &ranges)
{
	vector<Component_Pair> component_pairs =
		extractComponentPairs(infer_nodes, prev_ranges, ranges);
	Pin_Connections pin_connections = groupMatch(component_pairs);
	createLinks(infer_nodes, pin_connections);
	return pin_connections;
}

Pin_Connections consecutiveNodeMatch(infernodevec &prev_infer_nodes,
	infernodevec &infer_nodes)
{
	vector<Component_Pair> component_pairs =
		extractConsecutiveComponentPairs(prev_infer_nodes, infer_nodes);
	Pin_Connections pin_connections = groupMatch(component_pairs);
	getDependentPins(infer_nodes, pin_connections);
	return pin_connections;
}

void createLinks(infernodevec &infer_nodes, Pin_Connections &pin_connections)
{
	stringvec component_names(infer_nodes.size());
	for (size_t i = 0; i < component_names.size(); i++)
	{
		component_names[i] = infer_nodes[i].getName();
	}

	for (auto beg = pin_connections.begin(); beg != pin_connections.end();
			beg++)
	{
		stringpair left_pair = separateNames(beg->first),
		right_pair = separateNames(beg->second);
		for (size_t i = 0; i < component_names.size(); i++)
		{
			if (component_names[i] == left_pair.first)
			{
				size_t component_pos = getPosInVec(right_pair.first,
					component_names);
				size_t ancestor_pos = getPosInVec(infer_nodes[component_pos],
					infer_nodes[i].getAncestors());
				if (ancestor_pos == -1)
				{
					infer_nodes[i].addPrevNode(infer_nodes[component_pos]);
					infer_nodes[component_pos].addNextNode(infer_nodes[i]);
				}
				break;
			}
		}
	}
}

void createLinks(infernodevec &prev_infer_nodes, infernodevec &infer_nodes,
	Pin_Connections &pin_connections)
{
	infernodevec all_infer_nodes = infer_nodes;
	all_infer_nodes.insert(all_infer_nodes.end(), prev_infer_nodes.begin(),
		prev_infer_nodes.end());

	createLinks(all_infer_nodes, pin_connections);
	infer_nodes = infernodevec(all_infer_nodes.begin(),
		all_infer_nodes.begin() + infer_nodes.size());
	prev_infer_nodes = infernodevec(all_infer_nodes.begin() + infer_nodes.size(),
		all_infer_nodes.end());
}

void getDependentPins(infernodevec &infer_nodes, Pin_Connections &pin_connections)
{
	stringvec2d pins_vec(infer_nodes.size());
	// process connections
	for (auto beg = pin_connections.begin(); beg != pin_connections.end();
		beg++)
	{
		stringpair left_pair = separateNames(beg->first),
			right_pair = separateNames(beg->second);
		for (size_t i = 0; i < infer_nodes.size(); i++)
		{
			if (infer_nodes[i].getName() == left_pair.first)
			{
				pins_vec[i].push_back(left_pair.second);
			}

			if (infer_nodes[i].getName() == right_pair.first)
			{
				pins_vec[i].push_back(right_pair.second);
			}
		}
	}

	for (size_t i = 0; i < infer_nodes.size(); i++)
	{
		infer_nodes[i].getUsedNonLinVarNames(pins_vec[i]);
		infer_nodes[i].getUsedVarsName(pins_vec[i]);
	}
}

infernodevec getAllAnscenstor(BBNode &bbnode)
{
	infernodevec anscentors, prev_anscenstors;
	if (!bbnode.prev_bbnode->empty())
	{
		anscentors = bbnode.prev_bbnode->infer_nodes;
		prev_anscenstors = getAllAnscenstor(*bbnode.prev_bbnode);
		anscentors.insert(anscentors.end(), prev_anscenstors.begin(),
			prev_anscenstors.end());
	}
	return anscentors;
}

void addInferNodeMap(const infernodevec &infer_nodes)
{
	for (size_t i = 0; i < infer_nodes.size(); i++)
	{
		global_infer_node_map.insert(make_pair(infer_nodes[i].getId(),
			infer_nodes[i]));
	}
}
// string makeReplicate(const string &name, const stringvec &references)
// {
	// int max_id = -1;
	// int max_tmp = 0;
	// size_t pos_tmp;
	// std::string basename;
	// // TODO: will throw error if string after @ doesn't contain a number
	// // first check if name contains an id
	// pos_tmp = name.find_last_of('@');
	// if (pos_tmp != std::string::npos){
		// max_id = std::stoi(name.substr(pos_tmp+1, std::string::npos));
		// basename = name.substr(0, pos_tmp);
	// }
	// else{
		// basename = name;
	// }

	// for (int i = 0; i < references.size(); ++i){
		// pos_tmp = references[i].find(basename);
		// if (pos_tmp != std::string::npos){
			// // decode the string in ref and get the number after @
			// pos_tmp = references[i].find_last_of('@');
			// if (pos_tmp == std::string::npos){
				// max_tmp = 0;
			// }
			// else{
				// max_tmp = std::stoi(references[i].substr(pos_tmp+1, std::string::npos));
			// }
			// max_id = std::max(max_id, max_tmp);
		// }
	// }
	// return basename + '@' + std::to_string(max_id);
// }
string makeReplicate(const string &name, const stringvec &references)
{
	string result;
	size_t pos = getPosInVec(name, references);
	if (pos != -1)
	{
		result = makeReplicate(makeReplicate(name), references);
	}
	else
	{
		result = name;
	}
	return result;
}

string makeReplicate(const string &name)
{
	string result;
	size_t pos = name.find_last_of("@");
	if (pos >= name.size())
	{
		result = name + "@1";
	}
	else
	{
		result = name.substr(0, pos) + "@" + std::to_string(name[pos + 1] - 47);
	}
	return result;
}

// string makeReplicate(const string &name, const stringvec &references)
// {
	// int max_id = -1;
	// int max_tmp = 0;
	// size_t pos_tmp;
	// std::string basename;
	// // TODO: will throw error if string after @ doesn't contain a number
	// // first check if name contains an id
	// pos_tmp = name.find_last_of('@');
	// if (pos_tmp != std::string::npos){
		// max_id = std::stoi(name.substr(pos_tmp+1, std::string::npos));
		// basename = name.substr(0, pos_tmp);
	// }
	// else{
		// basename = name;
	// }

	// for (int i = 0; i < references.size(); ++i){
		// pos_tmp = references[i].find(basename);
		// if (pos_tmp != std::string::npos){
			// // decode the string in ref and get the number after @
			// pos_tmp = references[i].find_last_of('@');
			// if (pos_tmp == std::string::npos){
				// max_tmp = 0;
			// }
			// else{
				// max_tmp = std::stoi(references[i].substr(pos_tmp+1, std::string::npos));
			// }
			// max_id = std::max(max_id, max_tmp);
		// }
	// }
	// return basename + '@' + std::to_string(max_id);
// }

/*
void printConnections(const BBNode &opt_node, const bbnodevec &descendents, const connection_relation_vec2d &next_relations_vec)
{
	std::cout << std::endl;
	for (auto &pbeg = opt_node.nodes_vec.begin(); pbeg != opt_node.nodes_vec.end(); pbeg++)
	{
		for (auto &pnbeg = pbeg->begin(); pnbeg != pbeg->end(); pnbeg++)
		{
			std::cout << "PARENT: " << removeComponentPrePostfix(pnbeg->version) << std::endl;
		}
	}

	std::cout << std::endl;
	for (auto &dbeg = descendents.begin(); dbeg != descendents.end(); dbeg++)
	{
		for (auto &dvbeg = dbeg->nodes_vec.begin(); dvbeg != dbeg->nodes_vec.end(); dvbeg++)
		{
			for (auto &dnbeg = dvbeg->begin(); dnbeg != dvbeg->end(); dnbeg++)
			{
				std::cout << "CHILD " << removeComponentPrePostfix(dnbeg->version) << std::endl;
			}
		}
	}

	std::cout << std::endl;
	std::cout << "CONNECTION RELATIONS: " << std::endl;
	for (auto &rvvbeg = next_relations_vec.begin(); rvvbeg != next_relations_vec.end(); rvvbeg++)
	{
		for (auto &rvbeg = rvvbeg->begin(); rvbeg != rvvbeg->end(); rvbeg++)
		{
			for (auto &rbeg = rvbeg->begin(); rbeg != rvbeg->end(); rbeg++)
			{
				std::cout << removeComponentPrePostfix(rbeg->first.first) << ": " << rbeg->second.first << " -- " <<
					removeComponentPrePostfix(rbeg->first.second) << ": " << rbeg->second.second << std::endl;
			}
		}
	}
}

std::vector<Actu_components> generateComponents(stringvec &component_names, testinput &actuaction_input)
{
	std::vector<Actu_components> components(component_names.size());
	auto &cbeg = components.begin();
	doublepairs &torq_range = actuaction_input.first, &vel_range = actuaction_input.second;
	auto &tbeg = torq_range.begin(), &vbeg = vel_range.begin();
	for (auto &cnbeg = component_names.begin(); cnbeg != component_names.end(); cnbeg++)
	{
		if (inBatteryFolder(*cnbeg))
		{
			*cbeg = Battery(removeComponentPostfix(*cnbeg));
			cbeg->setName(*cnbeg);
			cbeg++;
		}

		if (inVoltageRegulatorFolder(*cnbeg))
		{
			*cbeg = V_regulator(removeComponentPostfix(*cnbeg));
			cbeg->setName(*cnbeg);
			cbeg++;
		}

		if (inHbridgeFolder(*cnbeg))
		{
			*cbeg = H_bridge(removeComponentPostfix(*cnbeg));
			cbeg->setName(*cnbeg);
			cbeg++;
		}

		if (inMicroControllerFolder(*cnbeg))
		{
			*cbeg = Micro_Controller(removeComponentPostfix(*cnbeg));
			cbeg->setName(*cnbeg);
			cbeg++;
		}

		if (inMotorFolder(*cnbeg))
		{
			*cbeg = Motor(removeComponentPostfix(*cnbeg), vbeg->first, tbeg->first);
			vbeg++, tbeg++;
			cbeg->setName(*cnbeg);
			cbeg++;
		}
	}
	return components;
}
*/

shared_ptr<Electrical_Component> creatComponent(const string &file)
{
	std::bitset<10> component_code;
	isDCMotor(file) ? component_code[0] = 1 : component_code[0] = 0;
	isHbridge(file) ? component_code[1] = 1 : component_code[1] = 0;
	isMicroController(file) ? component_code[2] = 1 : component_code[2] = 0;
	isVoltageRegulator(file) ? component_code[3] = 1 : component_code[3] = 0;
	isBattery(file) ? component_code[4] = 1 : component_code[4] = 0;
	isEncoder(file) ? component_code[5] = 1 : component_code[5] = 0;
	isCamera(file) ? component_code[6] = 1 : component_code[6] = 0;
	isForceSensor(file) ? component_code[7] = 1 : component_code[7] = 0;
	isBluetooth(file) ? component_code[8] = 1 : component_code[8] = 0;
	isServo(file) ? component_code[9] = 1 : component_code[9] = 0;

	shared_ptr<Electrical_Component> component = nullptr;
	switch (component_code.to_ulong())
	{
	case DC_MOTOR_OPT: component =
		make_shared<Motor>(file);
		break;
	case HBRIDGE_OPT: component =
		make_shared<H_Bridge>(file);
		break;
	case MICRO_CONTROLLER_OPT: component =
		make_shared<Micro_Controller>(file);
		break;
	case VOLTAGE_REGULATOR_OPT: component =
		make_shared<Voltage_Regulator>(file);
		break;
	case BATTERY_OPT: component =
		make_shared<Battery>(file);
		break;
	case ENCODER_OPT: component =
		make_shared<Encoder>(file);
		break;
	case CAMERA_OPT: component =
		make_shared<Camera>(file);
		break;
	case FORCE_SENSOT_OPT: component =
		make_shared<Force_Sensor>(file);
		break;
	case BLUETOOTH_OPT: component =
		make_shared<Bluetooth>(file);
		break;
	case SERVO_OPT: component =
		make_shared<Motor>(file);
		break;
	default: throw COMPONENT_NOT_FOUND;
		break;
	}

	return component;
}

stringvec getAllSensorVersions(const string &type)
{
	return getAllVersions(sensor_path_map[type]);
}

stringvec getAllVersions(const string &dir)
{
	stringvec versions;
	for  (auto &entry : directory_iterator(dir))
	{
		versions.push_back(entry.path().string());
	}
	return versions;
}

stringvec2d getAllActuatorVersions(const string &type,
	const vector<pair<doublepair, doublepair>> &torq_vels)
{
	// unpack torqs and vels
	doublepairs torqs(torq_vels.size()), vels(torq_vels.size());
	for (size_t i = 0; i < torqs.size(); i++)
	{
		torqs[i] = torq_vels[i].first;
		vels[i] = torq_vels[i].second;
	}

	// group torqs and vels
	cliquetype torq_result = minCliqueCover(torqs),
		vel_result = minCliqueCover(vels);

	cliqueindex torq_index = std::get<1>(torq_result),
		vel_index = std::get<1>(vel_result),
		uniq_index(torq_index.size() * vel_index.size(), intvec(torqs.size()));
	int cnt = 0;
	unsignedvec empty_group_index;
	for (size_t i = 0; i < torq_index.size(); i++)
	{
		for (size_t j = 0; j < vel_index.size(); j++)
		{
			std::sort(torq_index[i].begin(), torq_index[i].end());
			std::sort(vel_index[j].begin(), vel_index[j].end());

			auto it = std::set_intersection(torq_index[i].begin(), torq_index[i].end(),
				vel_index[j].begin(), vel_index[j].end(), uniq_index[cnt].begin());
			uniq_index[cnt].resize(it - uniq_index[cnt].begin());
			if (!uniq_index[cnt].size())
			{
				empty_group_index.push_back(cnt);
			}
			cnt++;
		}
	}

	// post-processing
	for (size_t i = empty_group_index.size(); i > 0; i--)
	{
		uniq_index.erase(uniq_index.begin() + empty_group_index[i - 1]);
	}

	// compute intersected torqs and vels
	doublepairs uniq_torqs(uniq_index.size()), uniq_vels(uniq_index.size());
	for (size_t i = 0; i < uniq_index.size(); i++)
	{
		doublepairs temp_torqs, temp_vels;
		for (size_t j = 0; j < uniq_index[i].size(); j++)
		{
			temp_torqs.push_back(torqs[uniq_index[i][j]]);
			temp_vels.push_back(vels[uniq_index[i][j]]);
		}
		cliquetype temp_torq_result = minCliqueCover(temp_torqs),
			temp_vel_result = minCliqueCover(temp_vels);
		uniq_torqs[i] = std::get<0>(temp_torq_result)[0];
		uniq_vels[i] = std::get<0>(temp_vel_result)[0];
	}


	vector<pair<doublepair, doublepair>> uniq_torq_vels(uniq_torqs.size());
	for (size_t i = 0; i < uniq_torqs.size(); i++)
	{
		uniq_torq_vels[i] = make_pair(uniq_torqs[i], uniq_vels[i]);
	}

	stringvec2d motor_versions = getMotorVersions(type, uniq_torq_vels);
	motor_versions = vectorCombinations(motor_versions);
	for (size_t i = 0; i < motor_versions.size(); i++)
	{
		stringvec new_versions(torqs.size()); // consider number
		for (size_t j = 0; j < motor_versions[i].size(); j++)
		{
			for (size_t k = 0; k < uniq_index[j].size(); k++)
			{
				new_versions[uniq_index[j][k]] = motor_versions[i][j];
			}
		}
		motor_versions[i] = new_versions;
	}
	return motor_versions;
}

bool operator==(const std::pair<doublepair, doublepair> &left,
	const std::pair<doublepair, doublepair> &right)
{
	if (left.first.first == right.first.first &&
		left.first.second == right.first.second &&
		left.second.first == left.second.first &&
		left.second.second == left.second.second)
	{
		return true;
	}
	else
	{
		return false;
	}
}

/*
void evaluate(bbnodevec &bbnodes, connection_relation_vec2d &relations_vec,
	const doublepairs &torq_range, const doublepairs &vel_range)
{
	if (relations_vec.empty())
	{
		relations_vec.resize(bbnodes.size());
	}

	auto &rbeg = relations_vec.begin();
	for (auto &bnbeg = bbnodes.begin(); bnbeg != bbnodes.end(); bnbeg++, rbeg++)
	{
		evaluate(*bnbeg, *rbeg, torq_range, vel_range);
	}
}
*/
void BBNode::computeComponentNum()
{
	component_num += static_cast<unsigned>(infer_nodes.size());
}

void BBNode::computePinNum()
{
	for (size_t i = 0; i < this->infer_nodes.size(); i++)
	{
		pin_num += this->infer_nodes[i].getPinNum();
	}
}

void BBNode::computeConnectionNum()
{
	connection_num = this->circuit.getConnectionsSize();
}

void BBNode::computePrice()
{
	for (size_t i = 0; i < this->infer_nodes.size(); i++)
	{
		price += this->infer_nodes[i].getPrice();
	}
}

void BBNode::computeWeight()
{
	for (size_t i = 0; i < this->infer_nodes.size(); i++)
	{
		weight += this->infer_nodes[i].getWeight();
	}
}

void BBNode::computePowerConsump()
{
	if (this->level ==2)
	{
		infernodevec first_level_infer_nodes = this->prev_bbnode->infer_nodes;
		for (size_t i = 0; i < first_level_infer_nodes.size(); i++)
		{
			unsigned main_index = first_level_infer_nodes[i].getMainPowerIndex();
			double voltage = first_level_infer_nodes[i].getPowerInVolRange()[main_index].first,
				current = first_level_infer_nodes[i].getPowerInCurrent()[main_index];
			power_consumption += voltage * current;
		}
	}
}

void BBNode::computeMetricVal()
{
	eval_metric = 0;
	doublevec var_vals{ static_cast<double>(component_num),
		static_cast<double>(pin_num), static_cast<double>(connection_num),
		price, weight, power_consumption };
	for (size_t i = 0; i < coefficients.size(); i++)
	{
		eval_metric += coefficients[i]*var_vals[i];
	}
}

void BBNode::optimize()
{
	circuit.syncVars(model);
	circuit.updateComponents(getComponents());
	circuit.updateConnections(pin_connections);
	circuit.updateVerify(&model);
	circuit.maxSolve(&model);
}

void BBNode::updateConnections(const Pin_Connections &connections)
{
	circuit.updateConnections(connections);
	circuit.minUpdateConnectionsSolve(&model);
	circuit.maxSolve(&model);
}

bbnodevec BBNode::branch()
{
	this->clearPinConnections();
	// connection
	// TO DO: use lower current and check the limit of battery current
	this->pin_connections = consecutiveNodeMatch(this->prev_bbnode->infer_nodes,
		this->infer_nodes);
	this->optimize();
//	printPinConnections(this->pin_connections);
	Pin_Connections connections;
	if (this->level < 2)
	{
		connections = nodeMatch(this->infer_nodes);
		getDependentPins(this->infer_nodes, connections);
	}
	else
	{
		connections = maxNodeMatch(*this->prev_bbnode, this->infer_nodes);
	}
//	printPinConnections(connections);
	this->pin_connections.insert(connections.begin(), connections.end());
	this->updateConnections(connections);

	// get vals
	std::sort(infer_nodes.begin(), infer_nodes.end(),
		[](Infer_Node &left, Infer_Node &right) {
		return precedence_map[left.getType()] <
		precedence_map[right.getType()]; });

	for (size_t i = 0; i < infer_nodes.size(); i++)
	{
		infer_nodes[i].computeElectricProperties(pin_connections);
		for (size_t j = i + 1; j < infer_nodes.size(); j++)
		{
			infer_nodes[j].updatePrevNode(infer_nodes[i]);
		}
	}

	for (size_t i = 0; i < infer_nodes.size(); i++)
	{
		for (size_t j = i + 1; j < infer_nodes.size(); j++)
		{
			infer_nodes[i].updateNextNode(infer_nodes[j]);
		}
	}

	bbnodevec next_nodes;
	this->typeInfer();
	this->minCliqueCover();
	stringvec2d versions = this->versionInfer();
	infernodevec2d next_infer_nodes_vec = this->numberInfer(versions);

	for (size_t i = 0; i < next_infer_nodes_vec.size(); i++)
	{
		BBNode next_node(next_infer_nodes_vec[i], this->circuit, this->model, this);
//		next_node.circuit.syncVars(this->model);
//		next_node.copyAttributes(bbnode);
		next_nodes.push_back(next_node);
//		this->addDescendent(*next_nodes.rbegin());
//		next_nodes.rbegin()->addAncenstor(this);
	}
	this->updateInPrevNode();
	return next_nodes;
}

void BBNode::bound()
{
	if (this->getMetricVal() >= best_metric_val)
	{
		feasibility = false;
	}
}

stringvec BBNode::typeInfer()
{
	stringvec all_types;
	stringvec2d types_vec;
	for (size_t i = 0; i < infer_nodes.size(); i++)
	{
		stringvec types = ::typeInfer(infer_nodes[i]);
		boolvec has_types = hasMatchComponents(types,
			infer_nodes[i].getDescendants(), infer_nodes[i], pin_connections);
		for (size_t j = 0; j < has_types.size(); j++)
		{
			if (has_types[j])
			{
				types[j] = string();
			}
		}

		types_vec.push_back(types);
		all_types.insert(all_types.end(), types.begin(), types.end());
	}
	next_types = removeEmptyTypes(unique_vec(all_types));

	for (size_t i = 0; i < next_types.size(); i++)
	{
		for (size_t j = 0; j < infer_nodes.size(); j++)
		{
			if (find(types_vec[j].begin(), types_vec[j].end(),
				next_types[i]) != types_vec[j].end())
			{
				this->next_type_map[next_types[i]].push_back(j);
			}
		}
	}
	return next_types;
}

stringvec2d BBNode::versionInfer()
{
	unsigned cnt = 0;
	stringvec2d all_versions;
	for (size_t i = 0; i < next_types.size(); i++)
	{
		stringvec2d versions_vec = ::versionInfer(next_types[i],
			vol_ranges[i], current_set[i]);
		all_versions.insert(all_versions.end(), versions_vec.begin(),
			versions_vec.end());

		next_version_map.insert(make_pair(next_types[i], make_pair(cnt, cnt + versions_vec.size())));
		cnt += static_cast<unsigned>(versions_vec.size());
	}
	return vectorCombinations(all_versions);
}

infernodevec2d BBNode::numberInfer(const stringvec2d &versions_vec)
{
	infernodevec2d next_infer_nodes_vec;
	for (size_t i = 0; i < versions_vec.size(); i++)
	{
		infernodevec sub_infer_nodes;
		for (size_t j = 0; j < next_types.size(); j++)
		{
			stringvec sub_versions = getSubVector(versions_vec[i],
				next_version_map[next_types[j]].first,
				next_version_map[next_types[j]].second);
			infernodevec next_infer_nodes = ::numberInfer(next_types[j],
				infer_nodes, sub_versions, vol_index[j], vol_vol_index[j]);

			if (next_infer_nodes.empty())
			{
				sub_infer_nodes.clear();
				break;
			}
			sub_infer_nodes.insert(sub_infer_nodes.end(),
				next_infer_nodes.begin(), next_infer_nodes.end());
		}
		if (!sub_infer_nodes.empty())
		{
			next_infer_nodes_vec.push_back(sub_infer_nodes);
		}
	}
	// create relations between layers
	for (size_t k = 0; k < next_infer_nodes_vec.size(); k++)
	{
		for (size_t i = 0; i < infer_nodes.size(); i++)
		{
			infernodevec &descendenst = infer_nodes[i].getDescendants();
			for (size_t j = 0; j < descendenst.size(); j++)
			{
				size_t pos = getPosInVec(descendenst[j], next_infer_nodes_vec[k]);
				if (pos != -1)
				{
					next_infer_nodes_vec[k][pos].addPrevNode(infer_nodes[i]);
				}
			}
		}
	}
	return next_infer_nodes_vec;
}

bool hasTwoVoltageRegulators(const string &version, const string &type)
{
	return (isVoltageRegulator(version) && type == Component_Type::Voltage_Regulator);
}

void BBNode::minCliqueCover()
{
	for (auto beg = next_type_map.begin(); beg != next_type_map.end(); beg++)
	{
		unordered_map<unsigned, unsigned> component_vol_index_map, vol_vol_index_map;
		doublepairs all_vol_ranges;
		unsigned vol_cnt = 0;
		for (auto ibeg = beg->second.begin(); ibeg != beg->second.end();
			ibeg++)
		{
			doublepairs in_vol_ranges = infer_nodes[*ibeg].getInVolRange(beg->first);
			for (size_t i = 0; i < in_vol_ranges.size(); i++)
			{
				component_vol_index_map[vol_cnt + i] = *ibeg;
				vol_vol_index_map[vol_cnt + i] = i;
			}
			vol_cnt += in_vol_ranges.size();
			all_vol_ranges.insert(all_vol_ranges.end(), in_vol_ranges.begin(),
				in_vol_ranges.end());
		}
		// TO DO: figure out the best possible cover for two voltage regulators
		cliquetype vol_clique = ::minCliqueCover(all_vol_ranges);
		vol_ranges.push_back(std::get<0>(vol_clique));

		intvec2d clique_index_vec = std::get<1>(vol_clique),
			new_index_vec(clique_index_vec.size()),
			vol_vol_vec(clique_index_vec.size());
		for (size_t i = 0; i < clique_index_vec.size(); i++)
		{
			for (size_t j = 0; j < clique_index_vec[i].size(); j++)
			{
				new_index_vec[i].push_back(component_vol_index_map[clique_index_vec[i][j]]);
				vol_vol_vec[i].push_back(vol_vol_index_map[clique_index_vec[i][j]]);
			}
		}
		vol_index.push_back(new_index_vec);
		vol_vol_index.push_back(vol_vol_vec);

		// current set
		doublevec2d currents_per_type;
		intvec2d component_index_vec = *vol_index.rbegin();
		for (size_t i = 0; i < component_index_vec.size(); i++)
		{
			doublevec currents;
			for (size_t j = 0; j < component_index_vec[i].size(); j++)
			{
				double current = infer_nodes[component_index_vec[i][j]]
					.getInCurrentLimit(beg->first)[vol_vol_index_map[clique_index_vec[i][j]]];
				currents.push_back(current);
			}
			currents_per_type.push_back(currents);
		}
		current_set.push_back(currents_per_type);
	}
}

void BBNode::clearPinConnections()
{
	bbnodevec &same_level_nodes = this->prev_bbnode->next_bbnodes;
	int pos = getPosInVec(*this, same_level_nodes), pre_pos = 0;
	if (pos != 0)
	{
		for (size_t i = pos - 1; i >= 0; i--)
		{
			if (!same_level_nodes[i].pin_connections.empty())
			{
				pre_pos = i;
				break;
			}
		}
		Pin_Connections prev_pin_connections = same_level_nodes[pre_pos].
			getPinConections();

		infernodevec prev_infer_nodes = getAllAnscenstor(*this);
		vector<shared_ptr<Electrical_Component>> components(prev_infer_nodes.size());
		for (size_t i = 0; i < prev_infer_nodes.size(); i++)
		{
			components[i] = prev_infer_nodes[i].getComponent();
		}

		// clear previous component connection
		stringvec2d pins_vec(components.size());
		for (auto beg = prev_pin_connections.begin(); beg != prev_pin_connections.end();
			beg++)
		{
			stringpair right_pair = separateNames(beg->second);
			for (size_t i = 0; i < components.size(); i++)
			{
				if (right_pair.first == components[i]->getComponentName())
				{
					pins_vec[i].push_back(right_pair.second);
				}
			}
		}

		for (size_t i = 0; i < components.size(); i++)
		{
			components[i]->clearConnections(pins_vec[i]);
			for (size_t j = 0; j < pins_vec[i].size(); j++)
			{
				components[i]->restorePinVolBound(pins_vec[i][j]);
			}
		}
	}
}

void BBNode::makeReplicates()
{
	for (size_t i = 0; i < infer_nodes.size(); i++)
	{
		string new_name = makeReplicate(infer_nodes[i].getName(),
			component_names);
		infer_nodes[i].setName(new_name);
		component_names.push_back(new_name);
	}
}

vector<shared_ptr<Electrical_Component>> BBNode::getComponents()
{
	vector<shared_ptr<Electrical_Component>> components;
	for (size_t i = 0; i < infer_nodes.size(); i++)
	{
		components.push_back(infer_nodes[i].getComponent());
	}
	return components;
}

void BBNode::updateInPrevNode()
{
	bbnodevec &same_level_nodes = this->prev_bbnode->next_bbnodes;
	size_t pos = getPosInVec(*this, same_level_nodes);
	same_level_nodes[pos].pin_connections = this->pin_connections;
}

/*
void BBNode::getMotorMass()
{
	for (auto &mbeg = nodes_vec.begin()->begin(); mbeg != nodes_vec.begin()->end(); mbeg++)
	{
		motor_mass.push_back(mbeg->component_used_for_connection.getWeight());
	}
}
*/

double BBNode::getMetricVal() const
{
	return eval_metric;
}

bool BBNode::isBottom()
{
	return this->empty();
}

/*
nodeptrvec getNodesByIndices(const nodeptrvec &nodes, const unsignedvec &index_vec)
{
	nodeptrvec pnodes(index_vec.size());
	auto &pnbeg = pnodes.begin();
	for (auto &ibeg = index_vec.begin(); ibeg != index_vec.end(); ibeg++, pnbeg++)
	{
		*pnbeg = nodes[*ibeg];
	}
	return pnodes;
}
*/


void BBNode::copyMetrics(const BBNode &bbnode)
{
	this->component_num = bbnode.component_num;
	this->pin_num = bbnode.pin_num;
	this->connection_num = bbnode.connection_num;
	this->price = bbnode.price;
	this->weight = bbnode.weight;
	this->power_consumption = bbnode.power_consumption;
}
/*
void BBNode::getRangeNIndex(const doublepairs &_power_clique_range, const doublepairs &_func_clique_range,
	const cliqueindex &_power_clique_index, const cliqueindex &_func_clique_index, const doublevec &_current_clique_set)
{
	power_clique_range.push_back(_power_clique_range);
	power_clique_index.push_back(_power_clique_index);
	func_clique_range.push_back(_func_clique_range);
	func_clique_index.push_back(_func_clique_index);
	current_clique_set.push_back(_current_clique_set);
}
*/
void BBNode::evaluate()
{
	computeComponentNum();
	computePinNum();
	computeConnectionNum();
	computePrice();
	computeWeight();
	computePowerConsump();
	computeMetricVal();
}

void BBNode::reevaluate()
{
	computeConnectionNum();
	computeMetricVal();
}


std::string Infer_Node::getType() const
{
	return component->getComponentType();
}

std::string Infer_Node::getName() const
{
	return component->getComponentName();
}

double Infer_Node::getPrice() const
{
	return this->component->getPrice();
}

double Infer_Node::getWeight() const
{
	return this->component->getWeight();
}

unsigned Infer_Node::getPinNum() const
{
	return this->component->getPinNum();
}

void Infer_Node::getUsedVarsName(const stringvec &pin_names)
{
	component->getUsedVarsName(pin_names);
}

void Infer_Node::getUsedNonLinVarNames(const stringvec &pin_names)
{
	component->getUsedNonLinVarNames(pin_names);
}

Infer_Node::Infer_Node(const string &file)
{
	id = num_of_nodes++;
	component = creatComponent(file);
	Infer_Node* prev_node = new Infer_Node;
	if (!prev_node->empty())
	{
		prev_node_ids.push_back(prev_node->id);
	}
}

Infer_Node::Infer_Node(const string &file, Infer_Node &prev_node)
{
	id = num_of_nodes++;
	component = creatComponent(file);
	if (!prev_node.empty())
	{
		prev_node_ids.push_back(prev_node.id);
	}
}

Infer_Node::Infer_Node(shared_ptr<Electrical_Component> _component):
	component(_component)
{
	id = num_of_nodes++;
    Infer_Node* prev_node = new Infer_Node;
	if (!prev_node->empty())
	{
		prev_node_ids.push_back(prev_node->id);
	}
}

Infer_Node::Infer_Node(shared_ptr<Electrical_Component> _component, Infer_Node &prev_node):
	component(_component)
{
	id = num_of_nodes++;
	if (!prev_node.empty())
	{
		prev_node_ids.push_back(prev_node.id);
	}
}

bool Infer_Node::empty() const
{
	return component == nullptr;
}

void Infer_Node::addPrevNode(Infer_Node &prev_node)
{
	prev_node_ids.push_back(prev_node.id);
}

void Infer_Node::addNextNode(Infer_Node &next_node)
{
	next_nodes.push_back(next_node);
}

void Infer_Node::computeElectricProperties(Pin_Connections &pin_connections)
{
	this->computePowInPinNames();
	this->computePowOutPinNames();
	this->computeFuncInPinNames();
	this->computeFuncOutPinNames();
	this->pow_in_vol_range = component->getPowerInVolRange();
	this->func_in_vol_range = component->getFuncInVolRange();
	this->pow_in_exist_vec = boolvec(this->pow_in_vol_range.size(), false);
	this->func_in_exist_vec = boolvec(this->func_in_vol_range.size(), false);
	this->func_in_current = component->getFuncInCurrentLimit();
	this->computePowInCurrent(pin_connections);

	global_infer_node_map[this->id].pow_in_current = this->pow_in_current;
	global_infer_node_map[this->id].func_in_current = this->func_in_current;
	global_infer_node_map[this->id].pow_in_vol_range = this->pow_in_vol_range;
	global_infer_node_map[this->id].func_in_vol_range = this->func_in_vol_range;
	global_infer_node_map[this->id].pow_in_pin_names = this->pow_in_pin_names;
	global_infer_node_map[this->id].func_in_pin_names = this->func_in_pin_names;
	global_infer_node_map[this->id].pow_out_pin_names = this->pow_out_pin_names;
	global_infer_node_map[this->id].func_out_pin_names =this->func_out_pin_names;
}

doublevec Infer_Node::getInVal(const string &next_type)
{
	if (in_metric_map[make_pair(getType(), next_type)] ==
		Electronics::CLASS::POWER)
	{
		return getPowerInVal();
	}
	else
	{
		return doublevec{ getFuncInSize() };
	}
}

double Infer_Node::getOutVal(const Electronics::CLASS &pin_class)
{
	if (pin_class == Electronics::CLASS::POWER)
	{
		return getPowerOutLimit();
	}
	else
	{
		return getFuncOutSize();
	}
}

doublepairs Infer_Node::getInVolRange(const string &next_type)
{
	doublepairs valid_in_vol_ranges;
	if (in_vol_map[make_pair(getType(), next_type)] ==
		Electronics::CLASS::POWER)
	{
		for (size_t i = 0; i < pow_in_vol_range.size(); i++)
		{
			if (!pow_in_exist_vec[i])
			{
				valid_in_vol_ranges.push_back(pow_in_vol_range[i]);
			}
		}
	}
	else
	{
		for (size_t i = 0; i < func_in_vol_range.size(); i++)
		{
			if (!func_in_exist_vec[i])
			{
				valid_in_vol_ranges.push_back(func_in_vol_range[i]);
			}
		}
	}
	return valid_in_vol_ranges;
}

doublevec Infer_Node::getInCurrentLimit(const string &next_type)
{
	if (in_vol_map[make_pair(getType(), next_type)] ==
		Electronics::CLASS::POWER)
	{
		return pow_in_current;
	}
	else
	{
		return func_in_current;
	}
}

stringvec Infer_Node::getInPinNames(const string &next_type) const
{
	if (in_vol_map[make_pair(getType(), next_type)] ==
		Electronics::CLASS::POWER)
	{
		return pow_in_pin_names;
	}
	else
	{
		return func_in_pin_names;
	}
}

stringvec Infer_Node::getOutPinNames(const string &prev_type) const
{
	if (in_vol_map[make_pair(prev_type, getType())] ==
		Electronics::CLASS::POWER)
	{
		return pow_out_pin_names;
	}
	else
	{
		return func_out_pin_names;
	}
}

doublepair Infer_Node::getPowerOutVolRange() const
{
	return component->getPowerOutVolRange();
}

doublepair Infer_Node::getFuncOutVolRange() const
{
	return component->getFuncOutVolRange();
}

unsigned Infer_Node::getMainPowerIndex()
{
	return component->getMainPowerIndex();
}

void Infer_Node::computePowInCurrent(Pin_Connections &pin_connections)
{
	string type = getType();
	pow_in_current = component->getPowerInCurrentLimit();
	double current = 0;

	if (getPosInVec(type, add_components) != -1)
	{
		stringvec out_pins = getPowOutPinNames(), in_pins;
		for (size_t i = 0; i < this->prev_node_ids.size(); i++)
		{
			Infer_Node prev_node = global_infer_node_map[prev_node_ids[i]];
			in_pins = prev_node.getPowInPinNames();

			for (size_t j = 0;  j < out_pins.size();  j++)
			{
				for (size_t k = 0; k < in_pins.size(); k++)
				{
					string left = createConnectionName(this->getName(),
						out_pins[j]),
						right = createConnectionName(prev_node.getName(),
							in_pins[k]);
					const auto &range = pin_connections.equal_range(left);
					for (auto beg = range.first; beg != range.second; beg++)
					{
						if (beg->second == right)
						{
							current += prev_node.getPowerInCurrent()[k];
							break;
						}
					}
				}
			}
		}

		unsigned pow_inedx = this->getMainPowerIndex();
		pow_in_current[pow_inedx] = current;
	}
	else
	{
		// fill here
	}
}

void Infer_Node::computePowInPinNames()
{
	pow_in_pin_names = component->getPowerInPinNames();
}

void Infer_Node::computePowOutPinNames()
{
	pow_out_pin_names = component->getPowerOutPinNames();
}

void Infer_Node::computeFuncInPinNames()
{
	func_in_pin_names = component->getFuncInPinNames();
}

void Infer_Node::computeFuncOutPinNames()
{
	func_out_pin_names = component->getFuncOutPinNames();
}

void Infer_Node::setPowerExistVec(const size_t &pos) const
{
	pow_in_exist_vec[pos] = true;

}

void Infer_Node::setFuncExistVec(const size_t &pos) const
{
	func_in_exist_vec[pos] = true;
}

std::shared_ptr<Electrical_Component> Infer_Node::getComponent()
{
	return component;
}

infernodevec Infer_Node::getAncestors()
{
	infernodevec prev_nodes(prev_node_ids.size());
	for (size_t i = 0; i < prev_node_ids.size(); i++)
	{
		prev_nodes[i] = global_infer_node_map[prev_node_ids[i]];
	}
	return prev_nodes;
}

void Infer_Node::updatePrevNode(const Infer_Node &prev_infer_node)
{
	size_t pos = getPosInVec(prev_infer_node.id, this->prev_node_ids);
	if (pos != -1)
	{
		Infer_Node prev_node = global_infer_node_map[pos];
		prev_node.pow_in_current = prev_infer_node.getPowerInCurrent();
		prev_node.func_in_current = prev_infer_node.getFuncInCurrent();
		prev_node.pow_in_vol_range = prev_infer_node.getPowerInVolRange();
		prev_node.func_in_vol_range = prev_infer_node.getFuncInVolRange();
		prev_node.pow_in_pin_names = prev_infer_node.getPowInPinNames();
		prev_node.pow_out_pin_names = prev_infer_node.getPowOutPinNames();
	}
}

void Infer_Node::updateNextNode(const Infer_Node &next_node)
{
	size_t pos = getPosInVec(next_node, this->next_nodes);
	if (pos != -1)
	{
		this->next_nodes[pos].pow_in_pin_names = next_node.getPowInPinNames();
		this->next_nodes[pos].pow_out_pin_names = next_node.getPowOutPinNames();
		this->next_nodes[pos].func_in_pin_names = next_node.getFuncInPinNames();
		this->next_nodes[pos].func_out_pin_names = next_node.getFuncOutPinNames();
	}
}

double Infer_Node::getFuncOutSize()
{
	return component->getFuncOutSize();
}

double Infer_Node::getPowerOutLimit()
{
	return component->getPowerOutCurrentLimit();
}

double Infer_Node::getFuncInSize()
{
	return component->getFuncInSize();
}

doublevec Infer_Node::getPowerInVal()
{
	return pow_in_current;
}

BBNode::BBNode(const infernodevec &_infer_nodes, BBNode *_prev_bbnode)
{
    if (!_prev_bbnode){
        _prev_bbnode = new BBNode;
    }
    this->prev_bbnode = _prev_bbnode;
	if (!BBNode::solver_info)
	{
		this->model.set(GRB_IntParam_LogToConsole, 0);
	}
	this->id = num_of_bbnodes++;
	this->level = _prev_bbnode->level + 1;
	this->infer_nodes = _infer_nodes;
	this->component_names = _prev_bbnode->component_names;
	this->copyMetrics(*_prev_bbnode);
	this->makeReplicates();
	this->prev_bbnode->next_bbnodes.push_back(*this);
	::addInferNodeMap(infer_nodes);
}

BBNode::BBNode(const infernodevec &_infer_nodes, const Circuit &_circuit,
	GRBModel &_model, BBNode *_prev_bbnode) : circuit(_circuit), model(_model)
{
    if (!_prev_bbnode){
        _prev_bbnode = new BBNode;
    }
	if (!BBNode::solver_info)
	{
		this->model.set(GRB_IntParam_LogToConsole, 0);
	}
	this->id = num_of_bbnodes++;
	this->level = _prev_bbnode->level + 1;
	this->infer_nodes = _infer_nodes;
	this->component_names = _prev_bbnode->component_names;
	this->copyMetrics(*_prev_bbnode);
	this->makeReplicates();
	this->prev_bbnode = _prev_bbnode;
	this->prev_bbnode->next_bbnodes.push_back(*this);
	::addInferNodeMap(infer_nodes);
}

double Current_Operation::add(const doublevec &vec)
{
	return std::accumulate(vec.begin(), vec.end(), 0);
}

double Current_Operation::max(const doublevec &vec)
{
	return *std::max_element(vec.begin(), vec.end());
}
