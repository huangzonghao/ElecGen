#include "Inference.h"

// connection_relation BBNode::final_connections = connection_relation();
unsigned BBNode::num_of_bbnodes = 0;
doublevec BBNode::coefficients = { 1, 0, 0, 0, 0, 0 };
double BBNode::best_metric_val = 0.0;
unsigned BBNode::last_level = 0;
unsigned Infer_Node::num_of_nodes = 0;

using std::cout;
using std::endl;
using std::string;
using std::vector;
using std::filesystem::directory_iterator;
using std::unordered_map;
using std::unordered_multimap;
using std::pair;
using std::make_pair;
using std::find;

extern unordered_map<string, Electrical_Component*> MOTOR_PART_MAP,
H_BRIDDE_PART_MAP, MICRO_CONTROLLER_PART_MAP, VOLTAGE_REGULATOR_PART_MAP,
BATTERY_PART_MAP, ENCODER_PART_MAP, CAMERA_PART_MAP, FORCE_SENSOR_PART_MAP,
BLUETOOTH_PART_MAP, SERVO_PART_MAP;

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
	{{Component_Type::Micro_Controller, Component_Type::Voltage_Regulator },
	Electronics::CLASS::POWER},
	{{Component_Type::Micro_Controller, Component_Type::Battery },
	Electronics::POWER},
	{{Component_Type::Voltage_Regulator, Component_Type::Battery},
	Electronics::CLASS::POWER},
	{{Component_Type::Encoder, Component_Type::Motor}, Electronics::POWER},
	{{Component_Type::Encoder, Component_Type::Micro_Controller },
	Electronics::CLASS::FUNCTION},
	{{Component_Type::Encoder, Component_Type::Voltage_Regulator},
	Electronics::POWER },
	{{Component_Type::Encoder, Component_Type::Battery}, Electronics::POWER},
	{{Component_Type::Camera, Component_Type::Micro_Controller},
	Electronics::CLASS::FUNCTION},
	{{Component_Type::Camera, Component_Type::Voltage_Regulator },
	Electronics::POWER},
	{{Component_Type::Camera, Component_Type::Battery }, Electronics::POWER},
	{{Component_Type::Bluetooth, Component_Type::Micro_Controller},
	Electronics::CLASS::FUNCTION},
	{{Component_Type::Bluetooth, Component_Type::Voltage_Regulator}, 
	Electronics::POWER},
	{{Component_Type::Bluetooth, Component_Type::Battery}, Electronics::POWER},
	{{Component_Type::Force_Sensor, Component_Type::Micro_Controller}, 
	Electronics::CLASS::FUNCTION},
	{{Component_Type::Servo, Component_Type::Micro_Controller}, 
	Electronics::CLASS::FUNCTION},
	{{Component_Type::Servo, Component_Type::Voltage_Regulator}, Electronics::POWER },
	{{Component_Type::Servo, Component_Type::Battery}, Electronics::POWER }};

unordered_map<stringpair, Electronics::CLASS, hash_pair> in_metric_map{
	{{Component_Type::Motor, Component_Type::H_Bridge}, Electronics::FUNCTION},
	{{Component_Type::H_Bridge, Component_Type::Micro_Controller},
	Electronics::CLASS::FUNCTION},
	{{Component_Type::H_Bridge, Component_Type::Voltage_Regulator},
	Electronics::POWER},
	{{Component_Type::H_Bridge, Component_Type::Battery}, Electronics::POWER},
	{{Component_Type::Micro_Controller, Component_Type::Voltage_Regulator },
	Electronics::CLASS::POWER},
	{{Component_Type::Micro_Controller, Component_Type::Battery },
	Electronics::POWER},
	{{Component_Type::Voltage_Regulator, Component_Type::Battery},
	Electronics::CLASS::POWER},
	{{Component_Type::Encoder, Component_Type::Motor}, Electronics::POWER},
	{{Component_Type::Encoder, Component_Type::Micro_Controller },
	Electronics::CLASS::FUNCTION},
	{{Component_Type::Encoder, Component_Type::Voltage_Regulator},
	Electronics::POWER },
	{{Component_Type::Encoder, Component_Type::Battery}, Electronics::POWER},
	{{Component_Type::Camera, Component_Type::Micro_Controller},
	Electronics::CLASS::FUNCTION},
	{{Component_Type::Camera, Component_Type::Voltage_Regulator},
	Electronics::POWER},
	{{Component_Type::Camera, Component_Type::Battery }, Electronics::POWER},
	{{Component_Type::Bluetooth, Component_Type::Micro_Controller},
	Electronics::CLASS::FUNCTION},
	{{Component_Type::Bluetooth, Component_Type::Voltage_Regulator},
	Electronics::POWER},
	{{Component_Type::Bluetooth, Component_Type::Battery}, Electronics::POWER},
	{{Component_Type::Force_Sensor, Component_Type::Micro_Controller},
	Electronics::CLASS::FUNCTION},
	{{Component_Type::Servo, Component_Type::Micro_Controller},
	Electronics::CLASS::FUNCTION},
	{{Component_Type::Servo, Component_Type::Voltage_Regulator}, Electronics::POWER },
	{{Component_Type::Servo, Component_Type::Battery}, Electronics::POWER } };

unordered_map<string, Electronics::CLASS> out_vol_map{
	{Component_Type::Motor, Electronics::POWER},
	{Component_Type::H_Bridge, Electronics::FUNCTION},
	{Component_Type::Micro_Controller, Electronics::FUNCTION},
	{Component_Type::Voltage_Regulator, Electronics::POWER},
	{Component_Type::Battery, Electronics::POWER}};

unordered_map<string, double(*)(const doublevec &)> in_cuurent_map{
	{Component_Type::H_Bridge, Current_Operation::max},
	{Component_Type::Micro_Controller, Current_Operation::max},
	{Component_Type::Voltage_Regulator, Current_Operation::add},
	{Component_Type::Battery, Current_Operation::add},
	{Component_Type::Motor, Current_Operation::max} };

str_strvec_map type_infer_map{ 
	{Component_Type::Motor, stringvec{Component_Type::H_Bridge}},
	{Component_Type::H_Bridge, stringvec{Component_Type::Micro_Controller,
	Component_Type::Battery}},
	{Component_Type::Micro_Controller, stringvec{Component_Type::Battery}},
	{Component_Type::Voltage_Regulator, stringvec{Component_Type::Battery}},
	{Component_Type::Battery, stringvec{Component_Type::None}},
	{Component_Type::Encoder, stringvec{Component_Type::Motor,
	Component_Type::Micro_Controller, Component_Type::Battery}},
	{Component_Type::Camera, stringvec{Component_Type::Micro_Controller, 
	Component_Type::Battery}},
	{Component_Type::Bluetooth, stringvec{Component_Type::Micro_Controller, 
	Component_Type::Battery}},
	{Component_Type::Force_Sensor, stringvec{Component_Type::Micro_Controller,
	Component_Type::Battery}}, 
};


/*
stringvec preprocessing(const std::string &type, const doublepairs &torqs, const doublepairs &vels)
{
	bool validity = inputsValidityCheck(torqs, vels);
	stringvec versions(torqs.size());
	auto &vsbeg = versions.begin();
	if (type == motor_type && validity)
	{
		std::string component_name;
		for (auto &tbeg = torqs.begin(), &vbeg = vels.begin(); tbeg != torqs.end() && vbeg != vels.end(); ++tbeg, ++vbeg)
		{
			*vsbeg++ = preprocessing(*tbeg, *vbeg);
		}
	}
	return versions;
}

string preprocessing(const doublepair &torq, const doublepair &vel)
{
	std::string component_name;
	double max_torq, max_vel, vel_h, vel_l;
	for (const auto &entry : fs::directory_iterator(motor_path))
	{
		component_name = entry.path().string();
		Motor obj(component_name);
		max_torq = obj.getMaxTorq();
		max_vel = obj.getMaxVel();

		if (max_torq >= torq.second && max_vel >= vel.second)
		{
			vel_h = obj.getVel(torq.first);
			vel_l = obj.getVel(torq.second);
			if (vel_l >= vel.first && vel_h >= vel.second)
			{
				return component_name;
			}
		}
	}
	return no_component;
}

stringvec2d allPreprocessing(const std::string &type, const doublepairs &torqs, const doublepairs &vels)
{
	bool validity = inputsValidityCheck(torqs, vels);
	stringvec2d versions(torqs.size());
	auto &vsbeg = versions.begin();
	if (type == motor_type && validity)
	{
		std::string component_name;
		for (auto &tbeg = torqs.begin(), &vbeg = vels.begin(); tbeg != torqs.end() && vbeg != vels.end(); ++tbeg, ++vbeg)
		{
			*vsbeg++ = allPreprocessing(*tbeg, *vbeg);
		}
	}
	return versions;
}

stringvec2d allPreprocessing(const std::string &type, const doublepairs &inputs)
{
	stringvec2d versions(inputs.size());
	auto &vsbeg = versions.begin();
	if (type == motor_type)
	{
		for (auto &ibeg = inputs.begin(); ibeg != inputs.end(); ibeg++, vsbeg++)
		{
			std::string component_name;
			for (const auto &entry : fs::directory_iterator(motor_path))
			{
				component_name = entry.path().string();
				Motor obj(component_name);
				if (obj.getVel(ibeg->second) >= ibeg->first)
				{
					vsbeg->insert(vsbeg->end(), component_name);
				}
			}
		}
	}
	return versions;;
}

stringvec allPreprocessing(const doublepair &torq, const doublepair &vel)
{
	std::string component_name;
	stringvec component_names;
	double max_torq, max_vel, vel_h, vel_l;
	for (const auto &entry : fs::directory_iterator(motor_path))
	{
		component_name = entry.path().string();
		Motor obj(component_name);
		max_torq = obj.getMaxTorq();
		max_vel = obj.getMaxVel();

		if (max_torq >= torq.second && max_vel >= vel.second)
		{
			vel_h = obj.getVel(torq.first);
			vel_l = obj.getVel(torq.second);
			if (vel_l >= vel.first && vel_h >= vel.second)
			{
				component_names.push_back(component_name);
			}
		}
	}
	return component_names;
}
*/
/*
nodeptrvec initialization(const std::string &type, const doublepairs &torqs, const doublepairs &vels)
{
	stringvec &versions = preprocessing(type, torqs, vels), component_names;
	nodeptrvec node_vec(versions.size());

	auto &nbeg = node_vec.begin();
	for (auto &vbeg = versions.begin(); vbeg != versions.end(); vbeg++)
	{
		*nbeg++ = std::make_shared<InferredNode>(type, *vbeg);
	}

	nbeg = node_vec.begin();
	for (auto &vbeg = versions.begin(); vbeg != versions.end(); vbeg++, nbeg++)
	{
		unsigned copy_pf = 0;
		std::string component_name = connection_path_convert(*vbeg);
		(*nbeg)->component_used_for_connection = Connection_Structure(component_name);
		component_name = makeReplicate(component_names, component_name, copy_pf);
		component_names.push_back(component_name);
		(*nbeg)->component_used_for_connection.setComponentName(component_name);
	}
	return node_vec;
}
*/
/*
bbnodevec allInitialization(const std::string &type, const doublepairs &torqs, const doublepairs &vels)
{
	// reduction inputs by grouping identical inputs together
	auto &t_results = exactrRangeCover(torqs), &v_results = exactrRangeCover(vels);
	auto &t_index = std::get<1>(t_results), &v_index = std::get<1>(v_results);
	unsignedvec2d group_index;
	for (auto &tibeg = t_index.begin(); tibeg != t_index.end(); tibeg++)
	{
		for (auto &vibeg = v_index.begin(); vibeg != v_index.end(); vibeg++)
		{
			unsignedvec tempvec(std::max(tibeg->size(), vibeg->size()));
			auto &it = std::set_intersection(tibeg->begin(), tibeg->end(), vibeg->begin(), vibeg->end(), tempvec.begin());
			tempvec.resize(it - tempvec.begin());
			if (!tempvec.empty())
			{
				group_index.insert(group_index.end(), tempvec);
			}
		}
	}
	doublepairs &intersect_torqs = generateIntersections(group_index, torqs),
		&intersect_vels = generateIntersections(group_index, vels);

	stringvec2d &versions = allPreprocessing(type, intersect_torqs, intersect_vels), component_names;
	bbnodevec bbnode_vec;
	intvec index_vec(versions.size());

	auto &ibeg = index_vec.begin();
	for (auto &vbeg = versions.begin(); vbeg != versions.end(); vbeg++, ibeg++)
	{
		*ibeg = static_cast<int>(vbeg->size() - 1);
	}

	// creat basic structure for branch and bound
	discreture::multisets X(index_vec);
	for (auto &&x : X)
	{
		stringvec version_vec(torqs.size());
		auto &vvbeg = version_vec.begin();
		auto &vbeg = versions.begin();
		auto &gibeg = group_index.begin();
		for (auto &xbeg = x.begin(); xbeg != x.end(); xbeg++, vbeg++, gibeg++)
		{
			for (size_t i = 0; i < gibeg->size(); i++)
			{
				*vvbeg++ = (*vbeg)[*xbeg];
			}
		}

		infernodevec &nodes = generateNodeptrVec(type, version_vec);
		BBNode bbnode(infernodevec2d{ nodes }, ++BBNode::num_of_nodes);
		bbnode_vec.push_back(bbnode);
	}
	return bbnode_vec;
}
bbnodevec allInitialization(const std::string &type, const doublepairs &inputs)
{
	// reduction inputs by grouping identical inputs together
	auto &results = exactrRangeCover(inputs);
	auto &indices = std::get<1>(results);
	doublepairs &grouped_inputs = std::get<0>(results);

	stringvec2d &versions = allPreprocessing(type, grouped_inputs);
	bbnodevec bbnode_vec;
	intvec index_vec(versions.size());

	auto &ibeg = index_vec.begin();
	for (auto &vbeg = versions.begin(); vbeg != versions.end(); vbeg++, ibeg++)
	{
		*ibeg = static_cast<int>(vbeg->size() - 1);
	}

	// creat basic structure for branch and bound
	discreture::multisets X(index_vec);
	for (auto &&x : X)
	{
		stringvec version_vec(inputs.size());
		auto &vvbeg = version_vec.begin();
		auto &vbeg = versions.begin();
		auto &ibeg = indices.begin();
		for (auto &xbeg = x.begin(); xbeg != x.end(); xbeg++, vbeg++, ibeg++)
		{
			for (size_t i = 0; i < ibeg->size(); i++)
			{
				*vvbeg++ = (*vbeg)[*xbeg];
			}
		}

		infernodevec &nodes = generateNodeptrVec(type, version_vec);
		BBNode bbnode(infernodevec2d{ nodes }, ++BBNode::num_of_nodes);
		bbnode_vec.push_back(bbnode);
	}
	return bbnode_vec;
}
*/

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

bool doubleCheck(const doublepairs &torq_range, const doublepairs &vel_range, Circuit &circuit, GRBModel &model)
{
	bool success;
	auto &vbeg = vel_range.begin();
	circuit.syncVars(model);
	for (auto &tbeg = torq_range.begin(); tbeg != torq_range.end(); tbeg++, vbeg++)
	{
		circuit.changeMotorWorkingPoint(tbeg->first, vbeg->second, tbeg - torq_range.begin(), &model);
	}
	model.optimize();
	model.get(GRB_IntAttr_Status) == GRB_OPTIMAL ? success = true : success = false;
	return success;
}

bool doubleCheck(const doublepairs &torq_range, const doublepairs &vel_range, BBNode &design)
{
	return doubleCheck(torq_range, vel_range, design.circuit, design.model);
}
*/
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
	auto &iter = type_infer_map.find(type);
	return type_infer_map.find(type)->second;
}

stringvec removeEmptyTypes(stringvec &types)
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

bool hasMatchComponents(const stringvec &types, const infernodevec &infer_nodes)
{
	boolvec type_vec(types.size());
	for (size_t i = 0; i < types.size(); i++)
	{
		for (size_t j = 0; j < infer_nodes.size(); j++)
		{	
			if (types[i] == infer_nodes[j].getType())
			{
				type_vec[i] = true;
			}
		}
	}

	return find(type_vec.begin(), type_vec.end(), false) == type_vec.end();
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

	unordered_map<string, unordered_map<string, Electrical_Component*>>
		all_component_map = { {Component_Type::Motor, MOTOR_PART_MAP},
		{Component_Type::H_Bridge, H_BRIDDE_PART_MAP},
		{Component_Type::Micro_Controller, MICRO_CONTROLLER_PART_MAP},
		{Component_Type::Voltage_Regulator, VOLTAGE_REGULATOR_PART_MAP},
		{Component_Type::Battery, BATTERY_PART_MAP},
		{Component_Type::Encoder, ENCODER_PART_MAP},
		{Component_Type::Camera, CAMERA_PART_MAP},
		{Component_Type::Force_Sensor, FORCE_SENSOR_PART_MAP},
		{Component_Type::Bluetooth, BLUETOOTH_PART_MAP},
		{Component_Type::Servo, SERVO_PART_MAP}
	};

	unordered_map<string, Electrical_Component*> components = 
		all_component_map[type];

	for (auto &beg = components.begin(); beg != components.end(); beg++)
	{
		vol_output_range = beg->second->getOutVolRange(out_vol_map[type]);
		current_limit = beg->second->getOutCurrentLimit(out_vol_map[type]);

		if (isIntersect(vol_range, vol_output_range) && 
			in_cuurent_map[type](currents) <= current_limit)
		{
			versions.push_back(beg->first);		
		}
	}

	// use voltage regulator when can't find battery
	if (type == Component_Type::Battery && versions.empty())
	{
//		type = Component_Type::Voltage_Regulator;
		versions = versionInfer(Component_Type::Voltage_Regulator, 
			vol_range, currents);
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
	// determine number
	infernodevec next_nodes;
	double input_val = 0.0, output_val = 0.0;
	for (size_t i = 0; i < nodes_index.size(); i++)
	{
		next_nodes.push_back(Infer_Node(versions[i]));
		prev_nodes[nodes_index[i][0]].addNextNode(*next_nodes.rbegin());
		output_val = next_nodes.rbegin()->getOutVal(out_vol_map[type]);
		for (size_t j = 0; j < nodes_index[i].size(); j++)
		{
			input_val += prev_nodes[nodes_index[i][j]].getInVal(type)[vol_vol_index[i][j]];
			if (input_val > output_val)
			{
				next_nodes.push_back(Infer_Node(versions[i]));
				prev_nodes[nodes_index[i][j]].addNextNode(*next_nodes.rbegin());
				output_val += output_val;
			}
			else
			{
				if (j != 0)
				{
					prev_nodes[nodes_index[i][j]].addNextNode(*next_nodes.rbegin());
				}
			}
		}
	}
	return  next_nodes;
}
// brutal-force approach
/*
cliquetype minCliqueCover(const doublepairs &voltage_ranges)
{
	if (voltage_ranges.size() == 32)
	{
		bool stop = true;
	}
	doublepairs cliquerange;
	cliqueindex cliqueindex;
	if (!voltage_ranges.empty())
	{
		unsignedvec num_sequence(voltage_ranges.size());
		std::iota(num_sequence.begin(), num_sequence.end(), 0);

		for (size_t clique_size = voltage_ranges.size(); clique_size != 1; clique_size--)
		{
			auto &X = discreture::combinations(num_sequence, clique_size);
			for (auto &&x : X)
			{
				doublevec minvec, maxvec;
				double min, max;
				for (auto &beg = x.begin(); beg != x.end(); beg++)
				{
					minvec.insert(minvec.end(), voltage_ranges[*beg].first);
					maxvec.insert(maxvec.end(), voltage_ranges[*beg].second);
				}
				min = *std::max_element(minvec.begin(), minvec.end());
				max = *std::min_element(maxvec.begin(), maxvec.end());

				if (compareWithInTol(min, max))
				{
					// clique exists
					cliquerange.push_back(std::make_pair(min, max));
					cliqueindex.push_back(unsignedvec(x.begin(), x.end()));

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

		for (auto &beg = num_sequence.begin(); beg != num_sequence.end(); beg++)
		{
			cliquerange.push_back(voltage_ranges[*beg]);
			cliqueindex.push_back(unsignedvec{ *beg });
		}
	}
endloop:
	return std::make_tuple(cliquerange, cliqueindex);
}
*/
/*
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
			auto &X = discreture::combinations(num_sequence, clique_size);
			for (auto &&x : X)
			{
				doublepairs temppairs(x.size());
				auto &tpbeg = temppairs.begin();
				for (auto &beg = x.begin(); beg != x.end(); beg++)
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

		for (auto &beg = num_sequence.begin(); beg != num_sequence.end(); beg++)
		{
			cliquerange.insert(cliquerange.end(), ranges[*beg]);
			cliqueindex.insert(cliqueindex.end(), intvec{ *beg });
		}
	}
endloop:
	return std::make_tuple(cliquerange, cliqueindex);
	return cliquetype();
}

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
	for (auto &entry : std::experimental::filesystem::directory_iterator(battery_path))
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
/*
void writeDesign(const BBNode &bbnode)
{
	std::ofstream design;
	str_unsigned_uomap components_map;
	design.open(design_path + design_pf + "_" + std::to_string(getFileNumInDirectory(design_path)) + text_pf);

	design << "METRICS:" << std::endl;
	design << "NUM OF COMPONENTS: " << bbnode.returnComponenetNum() << std::endl;
	design << "NUM OF COMPONENT PINS: " << bbnode.returnComponentsPinNum() << std::endl;
	design << "NUM OF CONNECTIONS: " << bbnode.returnConnectionsNum() << std::endl;
	design << "PRICE:  " << bbnode.returnPrice() << std::endl;
	design << "WEIGHT: " << bbnode.returnWeight() << std::endl;
	design << "POWER: " << bbnode.returnPower() << std::endl;
	design << std::endl;
  
	std::vector<Actu_components> components = bbnode.circuit.getComponents();
	for (auto &cbeg = components.begin(); cbeg != components.end(); cbeg++)
	{
		std::string component_name = removeComponentPrePostfix(cbeg->getName());
		auto &iter = components_map.find(component_name);
		if (iter == components_map.end())
		{
			components_map.insert(std::make_pair(component_name, 1));
		}
		else
		{
			iter->second++;
		}
	}

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
	auto &relations = bbnode.final_connections;
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

bool inputsValidityCheck(const doublepairs &torqs, const doublepairs &vels)
{
	if (torqs.empty())
	{
		throw "input torques empty";
	}

	if (vels.empty())
	{
		throw "input velocities empty";
	}

	if (torqs.size() != vels.size())
	{
		throw "input torques and velocities size doesn't match";
	}

	for (auto &tbeg = torqs.begin(); tbeg != torqs.end(); tbeg++)
	{
		if (tbeg->first > tbeg->second)
		{
			throw "input torque range invalid";
		}

		if (tbeg->first < 0 || tbeg->second < 0)
		{
			throw "input torque is negative";
		}
	}

	for (auto &vbeg = vels.begin(); vbeg != vels.end(); vbeg++)
	{
		if (vbeg->first > vbeg->second)
		{
			throw "input velocity range invalid";
		}

		if (vbeg->first < 0 || vbeg->second < 0)
		{
			throw "input velocity is negative";
		}
	}
	return true;
}
*/
int branchNBound(bbnodevec &roots)
{
	static bool first_branch = true;
	// same level connect
	// optimize
	// inference (extra connect)

	bbnodevec descendents;
	for (size_t i = 0; i < roots.size(); i++)
	{
		if (!roots[i].empty())
		{
			if (!first_branch)
			{
				roots[i].bound();
			}

			if (first_branch && roots[i].isBottom())
			{
				BBNode::best_metric_val = roots[i].getMetricVal();
				first_branch = false;
			}

			if (roots[i].feasibility)
			{
				descendents = roots[i].branch();
			}
			branchNBound(descendents);
		}
	}
	return 0;
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

	if (bbnode.next_bbnodes.empty())
	{
		getDependentPins(infer_nodes, prev_pin_connections);
		pin_connections = nodeMatch(bbnode.infer_nodes);
		getDependentPins(infer_nodes, pin_connections);
	}
	else
	{
		pin_connections = nodeMatch(bbnode.infer_nodes,
			infer_nodes);
	}
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
		auto &range = connection_map.equal_range(component_types[i]);
		for (auto &beg = range.first; beg != range.second; beg++)
		{
			size_t pos = getPosInVec(beg->second, component_types);
			if (pos != -1)
			{
				component_pairs.push_back(make_pair(infer_nodes[i].getComponent(),
					infer_nodes[pos].getComponent()));
			}
		}
	}
	return component_pairs;
}

vector<Component_Pair> extractComponentPairs(infernodevec &prev_infer_nodes, infernodevec &infer_nodes)
{
	vector<Component_Pair> component_pairs;
	stringvec prev_component_types, component_types;

	for (size_t i = 0; i < prev_infer_nodes.size(); i++)
	{
		prev_component_types.push_back(prev_infer_nodes[i].getType());
	}

	for (size_t i = 0; i < infer_nodes.size(); i++)
	{
		component_types.push_back(infer_nodes[i].getType());
	}

	for (size_t i = 0; i < component_types.size(); i++)
	{
		auto &range = connection_map.equal_range(component_types[i]);
		for (auto &beg = range.first; beg != range.second; beg++)
		{
			size_t pos = getPosInVec(beg->second, prev_component_types);
			if (pos != -1)
			{
				component_pairs.push_back(make_pair(infer_nodes[i].getComponent(),
					prev_infer_nodes[pos].getComponent()));
			}
		}
	}
	return component_pairs;
}

Pin_Connections nodeMatch(infernodevec &infer_nodes)
{
	// add relations 

	vector<Component_Pair> component_pairs = 
		extractComponentPairs(infer_nodes);
	Pin_Connections pin_connections = groupMatch(component_pairs);
	return pin_connections;
}

Pin_Connections nodeMatch(infernodevec &prev_infer_nodes, 
	infernodevec &infer_nodes)
{
	vector<Component_Pair> component_pairs =
		extractComponentPairs(prev_infer_nodes, infer_nodes);
	Pin_Connections pin_connections = groupMatch(component_pairs);

	return pin_connections;
}

void getDependentPins(infernodevec &infer_nodes, Pin_Connections &pin_connections)
{
	stringvec2d pins_vec(infer_nodes.size());
	// process connections
	for (auto &beg = pin_connections.begin(); beg != pin_connections.end();
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
		prev_anscenstors = getAllAnscenstor(*bbnode.prev_bbnode);
		anscentors = bbnode.getAncensotr()->infer_nodes;
	}
	anscentors.insert(anscentors.end(), prev_anscenstors.begin(), prev_anscenstors.end());
	return anscentors;
}

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

Electrical_Component *creatComponent(const string &file)
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

	Electrical_Component *component = nullptr;
	switch (component_code.to_ulong())
	{
	case DC_MOTOR_OPT: component = new Motor(file);
		break;
	case HBRIDGE_OPT: component = new H_Bridge(file);
		break;
	case MICRO_CONTROLLER_OPT: component = new Micro_Controller(file);
		break;
	case VOLTAGE_REGULATOR_OPT: component = new Voltage_Regulator(file);
		break;
	case BATTERY_OPT: component = new Battery(file);	
		break;
	case ENCODER_OPT: // fill here
		break;
	case CAMERA_OPT: // fill here
		break;
	case FORCE_SENSOT_OPT: // fill here
		break;
	case BLUETOOTH_OPT: // fill here
		break;
	case SERVO_OPT: // fill here
		break;
	default: throw COMPONENT_NOT_FOUND;
		break;
	}

	return component;
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
/*
void BBNode::getComponentPinsNum()
{
	for (auto &nvbeg = nodes_vec.begin(); nvbeg != nodes_vec.end(); nvbeg++)
	{
		for (auto &nbeg = nvbeg->begin(); nbeg != nvbeg->end(); nbeg++)
		{
			component_pins_num += nbeg->component_used_for_connection.getPinNum();
		}
	}
}

void BBNode::getConnectionsNum()
{
	connections_num = circuit.getConnectionsNum();
}

void BBNode::computePrice()
{
	for (auto &nvbeg = nodes_vec.begin(); nvbeg != nodes_vec.end(); nvbeg++)
	{
		for (auto &nbeg = nvbeg->begin(); nbeg != nvbeg->end(); nbeg++)
		{
			price += nbeg->component_used_for_connection.getPrice();
		}
	}
}

void BBNode::computeWeight()
{
	for (auto &nvbeg = nodes_vec.begin(); nvbeg != nodes_vec.end(); nvbeg++)
	{
		for (auto &nbeg = nvbeg->begin(); nbeg != nvbeg->end(); nbeg++)
		{
			weight += nbeg->component_used_for_connection.getWeight();
		}
	}
}

void BBNode::computePowerConsump(const doublepairs2d &power_voltage_ranges, const doublepairs2d &func_voltage_ranges)
{
	auto &pvbeg = power_voltage_ranges.begin(), &fvbeg = func_voltage_ranges.begin();
	auto &ctbeg = current_tracker_vec.begin();
	for (auto &nbeg = nodes_vec.begin(); nbeg != nodes_vec.end(); nbeg++, ctbeg++, pvbeg++, fvbeg++)
	{
		if (!nbeg->empty())
		{
			if (nbeg->begin()->type == motor_type)
			{
				auto &cbeg = ctbeg->current_set.begin();
				for (auto &fbeg = fvbeg->begin(); fbeg != fvbeg->end(); fbeg++)
				{
					power += *cbeg*fbeg->first;
				}
			}
			else
			{
				auto &cbeg = ctbeg->current_set.begin();
				for (auto &pbeg = pvbeg->begin(); pbeg != pvbeg->end(); pbeg++)
				{
					power += *cbeg*pbeg->first;
				}
			}
		}
	}
}
*/
void BBNode::computeMetricVal()
{
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
//	circuit.updateReplicates(component_vec);
//	circuit.checkVars(&model);
	circuit.syncVars(model);
	circuit.maxSolve(&model);
	circuit.updateComponents(getComponents());
	circuit.updateConnections(pin_connections);
	circuit.updateVerify(&model);
//	circuit.checkVars(&model);
}

bbnodevec BBNode::branch()
{
	clearPinConnections();
	// connection 
	//TO DO: extablish connection between nodes
	pin_connections = maxNodeMatch(*this, this->infer_nodes);
	optimize();

	// get vals
	for (size_t i = 0; i < infer_nodes.size(); i++)
	{
		infer_nodes[i].computeElectricProperties();
	}

	bbnodevec next_nodes;
	typeInfer();
	minCliqueCover();
	stringvec2d versions = versionInfer();
	infernodevec2d infer_nodes_vec = numberInfer(versions);

	for (size_t i = 0; i < infer_nodes_vec.size(); i++)
	{
		BBNode next_node(infer_nodes_vec[i], this->circuit, this->model, this);
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
	if (this->getMetricVal() > best_metric_val)
	{
		feasibility = false;
	}
	else
	{
		if (this->isBottom())
		{
			best_metric_val = this->getMetricVal();
		}
	}
}

stringvec BBNode::typeInfer()
{
	stringvec all_types;
	stringvec2d types_vec;
	for (size_t i = 0; i < infer_nodes.size(); i++)
	{
		stringvec types = ::typeInfer(infer_nodes[i]);
		infernodevec next_nodes = infer_nodes[i].getDescendents();
		if (hasMatchComponents(types, next_nodes))
		{
			types.clear();
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
				next_type_map[next_types[i]].push_back(j);
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
	infernodevec2d infer_nodes_vec;
	for (size_t i = 0; i < versions_vec.size(); i++)
	{
		infernodevec sub_infer_nodes;
		for (size_t j = 0; j < next_types.size(); j++)
		{
			stringvec sub_versions = getSubVector(versions_vec[i], 
				next_version_map[next_types[j]].first, 
				next_version_map[next_types[j]].second);
			infernodevec _infer_nodes = ::numberInfer(next_types[j], infer_nodes,
				sub_versions, vol_index[j], vol_vol_index[j]);
			sub_infer_nodes.insert(sub_infer_nodes.end(), _infer_nodes.begin(),
				_infer_nodes.end());
		}
		infer_nodes_vec.push_back(sub_infer_nodes);
	}
	// create relations between layers
	for (size_t k = 0; k < infer_nodes_vec.size(); k++)
	{
		for (size_t i = 0; i < infer_nodes.size(); i++)
		{
			infernodevec &descendenst = infer_nodes[i].getDescendents();
			for (size_t j = 0; j < descendenst.size(); j++)
			{
				size_t pos = getPosInVec(descendenst[j], infer_nodes_vec[k]);
				if (pos != -1)
				{
					infer_nodes_vec[k][pos].addPrevNode(infer_nodes[i]);
				}
			}
		}
	}
	return infer_nodes_vec;
}

void BBNode::minCliqueCover()
{
	for (auto &beg = next_type_map.begin(); beg != next_type_map.end(); beg++)
	{
		unordered_map<unsigned, unsigned> component_vol_index_map, vol_vol_index_map;
		doublepairs all_vol_ranges;
		unsigned vol_cnt = 0;
		for (auto &ibeg = beg->second.begin(); ibeg != beg->second.end();
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
	bbnodevec same_level_nodes = this->prev_bbnode->next_bbnodes;
	size_t pos = getPosInVec(*this, same_level_nodes);
	if (pos != 0)
	{
		Pin_Connections prev_pin_connections = same_level_nodes[pos - 1].
			getPinConections();

		infernodevec prev_infer_nodes = getAllAnscenstor(*this);
		vector<Electrical_Component*> components(prev_infer_nodes.size());
		for (size_t i = 0; i < prev_infer_nodes.size(); i++)
		{
			components[i] = prev_infer_nodes[i].getComponent();
		}

		// clear previous component connection
		stringvec2d pins_vec(components.size());
		for (auto &beg = prev_pin_connections.begin(); beg != prev_pin_connections.end();
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

vector<Electrical_Component*> BBNode::getComponents()
{
	vector<Electrical_Component*> components;
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
	return empty();
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

/*
void BBNode::copyAttributes(const BBNode &bbnode)
{
	components_num = bbnode.components_num;
	component_pins_num = bbnode.component_pins_num;
	connections_num = bbnode.connections_num;
	price = bbnode.price;
	weight = bbnode.weight;
	motor_mass = bbnode.motor_mass;
	power = bbnode.power;
	connect_components_vec = bbnode.connect_components_vec;
}

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
//	computePinNum();
//	computeConnectionNum();
//	computePrice();
//	computeWeight();
	computeMetricVal();
}


void Infer_Node::getUsedVarsName(const stringvec &pin_names)
{
	component->getUsedVarsName(pin_names);
}

void Infer_Node::getUsedNonLinVarNames(const stringvec &pin_names)
{
	component->getUsedNonLinVarNames(pin_names);
}

Infer_Node::Infer_Node(const string &file, Infer_Node &prev_node)
{
	id = num_of_nodes++;
	component = creatComponent(file);
	if (!prev_node.empty())
	{
		prev_nodes.push_back(prev_node);
	}
}

Infer_Node::Infer_Node(Electrical_Component *_component, Infer_Node &prev_node):
	component(_component)
{
	id = num_of_nodes++;
	if (!prev_node.empty())
	{
		prev_nodes.push_back(prev_node);
	}
}

void Infer_Node::addPrevNode(Infer_Node &prev_node)
{
	prev_nodes.push_back(prev_node);
}

void Infer_Node::addNextNode(Infer_Node &next_node)
{
	next_nodes.push_back(next_node);
}

void Infer_Node::computeElectricProperties()
{
	pow_in_vol_range = component->getPowerInVolRange(); 
	func_in_vol_range = component->getFuncInVolRange();
	computePowInCurrent();
	func_in_cuurent = component->getFuncInCurrentLimit();
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
	if (in_vol_map[make_pair(getType(), next_type)] == 
		Electronics::CLASS::POWER)
	{
		return pow_in_vol_range;
	}
	else
	{
		return func_in_vol_range;
	}
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
		return func_in_cuurent;
	}
}

void Infer_Node::computePowInCurrent()
{
	string type = getType();	
	pow_in_current = component->getPowerInCurrentLimit();
	
	if (getPosInVec(type, add_components) != -1)
	{
		unsigned prev_pow_index, pow_inedx = this->getMainPowerIndex();
		double current = 0;
		for (size_t i = 0; i < prev_nodes.size(); i++)
		{
			prev_pow_index = prev_nodes[i].getMainPowerIndex(),
			current += prev_nodes[i].getPowerInCurrent()[prev_pow_index];
		}
		pow_in_current[pow_inedx] = current;
	}
	else
	{
		// fill here
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

doublepairs Infer_Node::getFuncInVolRange()
{
	return component->getFuncInVolRange();
}

doublepairs Infer_Node::getPowerInVolRange()
{
	return component->getPowerInVolRange();
}

BBNode::BBNode(const infernodevec &_infer_nodes, BBNode *_prev_bbnode): 
	infer_nodes(_infer_nodes), prev_bbnode(_prev_bbnode)
{
	id = num_of_bbnodes++;
	level = _prev_bbnode->level + 1;
	prev_bbnode->next_bbnodes.push_back(*this);
}

BBNode::BBNode(const infernodevec &_infer_nodes, const Circuit &_circuit,
	GRBModel &_model, BBNode *_prev_bbnode) : circuit(_circuit), model(_model)
{
	id = num_of_bbnodes++;
	level = _prev_bbnode->level + 1;
	infer_nodes = _infer_nodes;
	prev_bbnode = _prev_bbnode;
	prev_bbnode->next_bbnodes.push_back(*this);
}

double Current_Operation::add(const doublevec &vec)
{
	return std::accumulate(vec.begin(), vec.end(), 0);
}

double Current_Operation::max(const doublevec &vec)
{
	return *std::max_element(vec.begin(), vec.end());
}
