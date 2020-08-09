#include "Matching.h"

using std::vector;
using std::string;
using std::unordered_map;
using std::unordered_multimap;
using std::pair;
using std::make_pair;
using std::cin;
using std::cout;
using std::endl;
using std::bitset;
using std::shared_ptr;

// all possible component pairs
unordered_multimap<std::string, std::string> connection_map{
	{Component_Type::H_Bridge, Component_Type::Motor},
//	{Component_Type::Encoder, Component_Type::Motor},
	{Component_Type::Micro_Controller, Component_Type::Encoder},
	{Component_Type::Voltage_Regulator, Component_Type::Encoder},
	{Component_Type::Battery, Component_Type::Encoder},
	{Component_Type::Micro_Controller, Component_Type::Camera},
	{Component_Type::Voltage_Regulator, Component_Type::Camera},
	{Component_Type::Battery, Component_Type::Camera},
	{Component_Type::Micro_Controller, Component_Type::Bluetooth},
	{Component_Type::Voltage_Regulator, Component_Type::Bluetooth},
	{Component_Type::Battery, Component_Type::Bluetooth},
	{Component_Type::Micro_Controller, Component_Type::Force_Sensor},
	{Component_Type::Micro_Controller, Component_Type::H_Bridge},
	{Component_Type::Voltage_Regulator, Component_Type::H_Bridge},
	{Component_Type::Battery, Component_Type::H_Bridge},
	{Component_Type::Voltage_Regulator, Component_Type::Micro_Controller},
	{Component_Type::Battery, Component_Type::Micro_Controller},
	{Component_Type::Battery, Component_Type::Voltage_Regulator},
	{Component_Type::Voltage_Regulator, Component_Type::Servo},
	{Component_Type::Micro_Controller, Component_Type::Servo}, 
	{Component_Type::Battery, Component_Type::Force_Sensor},
};

struct hash_pair {
	template <class T1, class T2>
	size_t operator()(const pair<T1, T2>& p) const
	{
		auto hash1 = std::hash<T1>{}(p.first);
		auto hash2 = std::hash<T2>{}(p.second);
		return hash1 ^ hash2;
	}
};

unordered_map<stringpair, bitset<7>, hash_pair> connection_mask_map {
	{{Component_Type::H_Bridge, Component_Type::Motor}, func_mask},
	{{Component_Type::Micro_Controller, Component_Type::H_Bridge},
	func_mask},
	{{Component_Type::Voltage_Regulator, Component_Type::H_Bridge},
	power_mask},
	{{Component_Type::Battery, Component_Type::H_Bridge}, power_mask},
	{{Component_Type::Voltage_Regulator, Component_Type::Micro_Controller},
	power_mask},
	{{Component_Type::Battery, Component_Type::Micro_Controller},
	power_mask},
	{{Component_Type::Battery, Component_Type::Voltage_Regulator},
	power_mask},
//	{{Component_Type::Motor, Component_Type::Encoder}, power_mask},
	{{Component_Type::Micro_Controller, Component_Type::Encoder},
	func_mask},
	{{Component_Type::Voltage_Regulator, Component_Type::Encoder},
	power_mask },
	{{Component_Type::Battery, Component_Type::Encoder}, power_mask},
	{{Component_Type::Micro_Controller, Component_Type::Camera},
	func_mask},
	{{Component_Type::Voltage_Regulator, Component_Type::Camera },
	power_mask},
	{{Component_Type::Battery, Component_Type::Camera}, power_mask},
	{{Component_Type::Micro_Controller, Component_Type::Bluetooth},
	func_mask},
	{{Component_Type::Voltage_Regulator, Component_Type::Bluetooth},
	power_mask},
	{{Component_Type::Battery, Component_Type::Bluetooth}, power_mask},
	{{Component_Type::Battery, Component_Type::Force_Sensor},
	power_mask},
	{{Component_Type::Micro_Controller, Component_Type::Force_Sensor},
	func_mask},
	{{Component_Type::Micro_Controller, Component_Type::Servo},
	func_mask},
	{{Component_Type::Voltage_Regulator, Component_Type::Servo}, power_mask },
	{{Component_Type::Battery, Component_Type::Servo},  power_mask}
};

unordered_multimap<Electronics::FUNCTION_TYPE, Electronics::FUNCTION_TYPE>
compatible_type_map{ 
	{Electronics::DIGITAL, Electronics::ELECTRICAL },
	{Electronics::PWM, Electronics::ELECTRICAL},
	{Electronics::DIGITAL_SPI_MOSI, Electronics::ELECTRICAL},
	{Electronics::DIGITAL_SPI_MISO, Electronics::ELECTRICAL},
	{Electronics::DIGITAL_SPI_SCK, Electronics::ELECTRICAL },
	{Electronics::DIGITAL_SPI_SS, Electronics::ELECTRICAL },
	{Electronics::DIGITAL_UART_RX, Electronics::ELECTRICAL},
	{Electronics::DIGITAL_UART_TX, Electronics::ELECTRICAL},
	{Electronics::DIGITAL_EXTERNAL_INTERRUPT, Electronics::ELECTRICAL}, 
	{Electronics::PWM_SPI_MOSI, Electronics::ELECTRICAL},
	{Electronics::PWM_SPI_MISO, Electronics::ELECTRICAL},
	{Electronics::PWM_SPI_SCK, Electronics::ELECTRICAL},
	{Electronics::PWM_SPI_SS, Electronics::ELECTRICAL},
	{Electronics::PWM_EXTERNAL_INTERRUPT, Electronics::ELECTRICAL},
	{Electronics::DIGITAL_EXTERNAL_INTERRUPT_I2C_SDA, Electronics::ELECTRICAL},
	{Electronics::DIGITAL_EXTERNAL_INTERRUPT_I2C_SCL, Electronics::ELECTRICAL},
	{Electronics::DIGITAL_EXTERNAL_INTERRUPT_UART_TX, Electronics::ELECTRICAL},
	{Electronics::DIGITAL_EXTERNAL_INTERRUPT_UART_RX, Electronics::ELECTRICAL},
	{Electronics::DIGITAL, Electronics::ENABLE },
	{Electronics::PWM, Electronics::ENABLE},
	{Electronics::DIGITAL_SPI_MOSI, Electronics::ENABLE},
	{Electronics::DIGITAL_SPI_MISO, Electronics::ENABLE},
	{Electronics::DIGITAL_SPI_SCK, Electronics::ENABLE},
	{Electronics::DIGITAL_SPI_SS, Electronics::ENABLE},
	{Electronics::DIGITAL_UART_TX, Electronics::ENABLE},
	{Electronics::DIGITAL_UART_RX, Electronics::ENABLE},
	{Electronics::DIGITAL_EXTERNAL_INTERRUPT, Electronics::ENABLE},
	{Electronics::PWM_SPI_MOSI, Electronics::ENABLE},
	{Electronics::PWM_SPI_MISO, Electronics::ENABLE},
	{Electronics::PWM_SPI_SCK, Electronics::ENABLE},
	{Electronics::PWM_SPI_SS, Electronics::ENABLE},
	{Electronics::PWM_EXTERNAL_INTERRUPT, Electronics::ENABLE},
	{Electronics::DIGITAL_EXTERNAL_INTERRUPT_I2C_SDA, Electronics::ENABLE},
	{Electronics::DIGITAL_EXTERNAL_INTERRUPT_I2C_SCL, Electronics::ENABLE},
	{Electronics::DIGITAL_EXTERNAL_INTERRUPT_UART_TX, Electronics::ENABLE},
	{Electronics::DIGITAL_EXTERNAL_INTERRUPT_UART_RX, Electronics::ENABLE},
	{Electronics::ELECTRICAL, Electronics::MOTOR},
	{Electronics::ELECTRICAL, Electronics::LOGIC},
	{Electronics::ELECTRICAL, Electronics::GND},
//	{Electronics::UART_RX_I2C_SDA, Electronics::UART_TX},
//	{Electronics::UART_TX_I2C_SCL, Electronics::I2C_SCL},
//	{Electronics::UART_TX_I2C_SCL, Electronics::UART_RX},

	{Electronics::DIGITAL_UART_TX, Electronics::UART_RX_I2C_SDA},
	{Electronics::ANALOG_I2C_SDA, Electronics::UART_RX_I2C_SDA},
	{Electronics::ANALOG_I2C_SCL, Electronics::UART_TX_I2C_SCL},
	{Electronics::I2C_SDA, Electronics::UART_RX_I2C_SDA},
	{Electronics::I2C_SCL, Electronics::UART_TX_I2C_SCL},
	{Electronics::DIGITAL_UART_RX, Electronics::UART_TX_I2C_SCL},
	{Electronics::DIGITAL_EXTERNAL_INTERRUPT_UART_TX, Electronics::UART_RX_I2C_SDA},
	{Electronics::DIGITAL_EXTERNAL_INTERRUPT_I2C_SDA, Electronics::UART_RX_I2C_SDA},
	{Electronics::DIGITAL_EXTERNAL_INTERRUPT_I2C_SCL, Electronics::UART_TX_I2C_SCL},
	{Electronics::DIGITAL_EXTERNAL_INTERRUPT_UART_RX, Electronics::UART_TX_I2C_SCL},
	{Electronics::DIGITAL_UART_TX, Electronics::UART_RX},
	{Electronics::DIGITAL_UART_RX, Electronics::UART_TX},
	{Electronics::DIGITAL_EXTERNAL_INTERRUPT_UART_TX, Electronics::UART_RX},
	{Electronics::DIGITAL_EXTERNAL_INTERRUPT_UART_RX, Electronics::UART_TX},
	{Electronics::DIGITAL_SPI_MISO, Electronics::SPI_MISO},
	{Electronics::DIGITAL_SPI_MOSI, Electronics::SPI_MOSI},
	{Electronics::DIGITAL_SPI_SCK, Electronics::SPI_SCK},
	{Electronics::DIGITAL_SPI_SS, Electronics::SPI_SS},
	{Electronics::PWM_SPI_MISO, Electronics::SPI_MISO},
	{Electronics::PWM_SPI_MOSI , Electronics::SPI_MOSI},
	{Electronics::PWM_SPI_SCK , Electronics::SPI_SCK},
	{Electronics::PWM_SPI_SS , Electronics::SPI_SS},
};

vector<bitset<7>> connect_code_vec{ 0b1001000, 0b1000001, 0b0100100, 
0b0100010, 0b0100001, 0b0010100, 0b0010010 };

/*
Connection_Structure::Connection_Structure(const string &file, const stringvec &pin_names): Connection_Structure(file)
{
	addUsablePins(pin_names);
}

void Connection_Structure::setComponentName(const string &_name)
{
	componentName = _name;
}

void Connection_Structure::clearPinConnections()
{
	this->clearPinConnections(power_in);
	this->clearPinConnections(power_out);
	this->clearPinConnections(power_function_in);
	this->clearPinConnections(power_function_out);
	this->clearPinConnections(gnd);
	this->clearPinConnections(functional_in);
	this->clearPinConnections(functional_out);
	this->clearPinConnections(functional_in_out);
	this->input_counter = 0;
	this->enable_counter = 0;
}

void Connection_Structure::addUsablePins(const stringvec &pin_names)
{
	for (auto &pbeg = pin_names.begin(); pbeg != pin_names.end(); pbeg++)
	{
		for (auto &apbeg = all_pins.begin(); apbeg != all_pins.end(); apbeg++)
		{
			if (apbeg->name == *pbeg)
			{
				usable_pins.push_back(*apbeg);
			}
		}
	}
}

void Connection_Structure::assignPinVoltageRange(const doublevec &lbs, const doublevec &ubs)
{
//	usable_pin_voltage_ranges.resize(lbs.size());
//	auto &rbeg = usable_pin_voltage_ranges.begin();
	for (auto &lbeg = lbs.begin(), &ubeg = ubs.begin(); lbeg != lbs.end(); lbeg++, ubeg++)
	{
		usable_pin_voltage_ranges.push_back(std::make_pair(*lbeg, *ubeg));
	}
}

void Connection_Structure::createAllPins()
{
	all_pins.insert(all_pins.end(), power_in.begin(), power_in.end());
	all_pins.insert(all_pins.end(), power_out.begin(), power_out.end());
	all_pins.insert(all_pins.end(), power_function_in.begin(), power_function_in.end());
	all_pins.insert(all_pins.end(), power_function_out.begin(), power_function_out.end());
	all_pins.insert(all_pins.end(), gnd.begin(), gnd.end());
	all_pins.insert(all_pins.end(), functional_in.begin(), functional_in.end());
	all_pins.insert(all_pins.end(), functional_out.begin(), functional_out.end());
	all_pins.insert(all_pins.end(), functional_in_out.begin(), functional_in_out.end());
}

Component_Connection::componentlist Component_Connection::getComponents()
{
	componentlist components;
	std::vector<std::string> version_vec(2 * component_relations.size()), type_vec(2 * component_relations.size()), unique_version, unique_type;
	auto &vbeg = version_vec.begin(), &tbeg = type_vec.begin();
	for (auto &beg = component_relations.begin(); beg != component_relations.end(); beg++)
	{
		*vbeg++ = beg->first.componentName;
		*vbeg++ = beg->second.componentName;
		*tbeg++ = beg->first.componentClass;
		*tbeg++ = beg->second.componentClass;
	}

	unique_version = unique_vec(version_vec);
	for (auto &beg = unique_version.begin(); beg != unique_version.end(); beg++)
	{
		unique_type.push_back(type_vec[std::find(version_vec.begin(), version_vec.end(), *beg) - version_vec.begin()]);
	}

	auto &uvbeg = unique_version.begin(), &utbeg = unique_type.begin();
	for (; uvbeg != unique_version.end(); uvbeg++, utbeg++)
	{
		components.push_back(std::make_tuple(*uvbeg, *utbeg));
	}
	return components;
}

Component_Connection::componentpinslist Component_Connection::getComponentPins()
{
	componentlist components = getComponents();
	componentpinslist component_pin_list(components.size());
	auto &cbeg = component_pin_list.begin();
	for (auto &beg = components.begin(); beg != components.end(); beg++, cbeg++)
	{
		*cbeg = std::make_tuple(path_convert(std::get<0>(*beg)), std::get<1>(*beg), std::vector<std::string>());
	}

	for (auto &beg = component_pin_list.begin(); beg != component_pin_list.end(); beg++)
	{
		for (auto &pbeg = netlist.begin(); pbeg != netlist.end(); pbeg++)
		{
			if (pbeg->first.first == std::get<0>(*beg))
			{
				std::get<2>(*beg).push_back(pbeg->second.first);
			}
			else if (pbeg->first.second == std::get<0>(*beg))
			{
				std::get<2>(*beg).push_back(pbeg->second.second);
			}
		}
	}
	return component_pin_list;
}
*/

Pin_Connections groupMatch(vector<Component_Pair> &component_pairs)
{
	Pin_Connections connection_list;
	boolvec pair_usage_vec(component_pairs.size(), true);
	// get list of unique components and their occurance number, 
	// start with actuator/sensor
	vector<shared_ptr<Electrical_Component>> components(component_pairs.size() * 2), 
		uniq_components;
	for (size_t i = 0; i < component_pairs.size(); i++)
	{
		components[2 * i] = component_pairs[i].first; 
		components[2 * i + 1] = component_pairs[i].second;
	}
	uniq_components = unique_vec(components);

	unsignedvec component_count(uniq_components.size());
	for (size_t i = 0; i < uniq_components.size(); i++)
	{
		component_count[i] = std::count(components.begin(), components.end(), 
			uniq_components[i]);
	}

	while (true)
	{
		if (std::all_of(component_count.begin(), component_count.end(),
			[](int i) {return i == INT_MAX; }))
		{
			break;
		}

		shared_ptr<Electrical_Component> least_component = nullptr;
		for (size_t i = 0; i < uniq_components.size(); i++)
		{
			if (component_count[i] != INT_MAX &&
				(uniq_components[i]->getComponentClass() ==
				Component_Class::Actuator ||
				uniq_components[i]->getComponentClass() ==
				Component_Class::Sensor))
			{
				least_component = uniq_components[i];
			}
		}

		if (least_component == nullptr)
		{
			size_t min_index = std::min_element(component_count.begin(), 
				component_count.end()) - component_count.begin();
			least_component = uniq_components[min_index];
		}

		for (size_t i = 0; i < component_pairs.size(); i++)
		{
			if (pair_usage_vec[i] &&
				(component_pairs[i].first == least_component || 
				component_pairs[i].second == least_component))
			{
				Pin_Connections pair_connection_list = 
					individualMatch(component_pairs[i]);
				connection_list.insert(pair_connection_list.begin(), 
					pair_connection_list.end());
				size_t left_index = std::find(uniq_components.begin(),
					uniq_components.end(), component_pairs[i].first) 
					- uniq_components.begin();
				size_t right_index = std::find(uniq_components.begin(),
					uniq_components.end(), component_pairs[i].second) 
					- uniq_components.begin();
				pair_usage_vec[i] = false;
				component_count[left_index]--;
				component_count[right_index]--;
				if (component_count[left_index] == 0)
				{
					component_count[left_index] = INT_MAX;
				}
				if (component_count[right_index] == 0)
				{
					component_count[right_index] = INT_MAX;
				}
				break;
			}
		}
	}
	
	return connection_list;
}

Pin_Connections individualMatch(Component_Pair &component_pair)
{
	// left output, right input
	shared_ptr<Electrical_Component> left_component = component_pair.first, 
		right_component = component_pair.second;

	vector<Pin*> left_power_out_pins = left_component->getPowerOutPins(),
		left_func_out_pins = left_component->getFuncOutPins(),
		left_func_bidirect_pins = left_component->getFuncBidirectPins(),
		right_power_in_pins = right_component->getPowerInPins(),
		right_func_in_pins = right_component->getFuncInPins(),
		right_func_bidirect_pins = right_component->getFuncBidirectPins(),
		right_both_bidirect_pins = right_component->getBothBidirectPins();

	std::bitset<7> pin_connect_code;
	left_power_out_pins.empty() ? pin_connect_code[6] = 0 : 
		pin_connect_code[6] = 1;
	left_func_out_pins.empty() ? pin_connect_code[5] = 0 : 
		pin_connect_code[5] = 1;
	left_func_bidirect_pins.empty() ? pin_connect_code[4] : 
		pin_connect_code[4] = 1;
	right_power_in_pins.empty() ? pin_connect_code[3] = 0 : 
		pin_connect_code[3] = 1;
	right_func_in_pins.empty() ? pin_connect_code[2] = 0 : 
		pin_connect_code[2] = 1;
	right_func_bidirect_pins.empty() ? pin_connect_code[1] = 0 : 
		pin_connect_code[1] = 1;
	right_both_bidirect_pins.empty() ? pin_connect_code[0] = 0 :
		pin_connect_code[0] = 1;

	// force mask
	bitset<7> mask = connection_mask_map[make_pair(
		component_pair.first->getComponentType(), 
		component_pair.second->getComponentType())];
	pin_connect_code &= mask;

	vector<bitset<7>> compatible_codes;
	for (size_t i = 0; i < pin_connect_code.size(); i++)
	{
		if ((pin_connect_code & connect_code_vec[i]) == connect_code_vec[i])
		{
			compatible_codes.push_back(connect_code_vec[i]);
		}
	}
	
	unordered_map<int, pair<vector<Pin*>, vector<Pin*>>> connection_map;
	connection_map[LPO_RPI] = make_pair(left_power_out_pins, 
		right_power_in_pins);
	connection_map[LPO_RBB] = make_pair(left_power_out_pins,
		right_both_bidirect_pins);
	connection_map[LFO_RFI] = make_pair(left_func_out_pins, 
		right_func_in_pins);
	connection_map[LFO_RFB] = make_pair(left_func_out_pins, 
		right_func_bidirect_pins);
	connection_map[LFO_RBB] = make_pair(left_func_out_pins,
		right_both_bidirect_pins);
	connection_map[LFB_RFI] = make_pair(left_func_bidirect_pins, 
		right_func_in_pins);
	connection_map[LFB_RFB] = make_pair(left_func_bidirect_pins, 
		right_func_bidirect_pins);

	Pin_Connections pin_list;
	for (size_t i = 0; i < compatible_codes.size(); i++)
	{
		Pin_Connections pin_connections = powerPinMatch(
			connection_map[compatible_codes[i].to_ulong()].first,
			connection_map[compatible_codes[i].to_ulong()].second, 
			component_pair);
		pin_list.insert(pin_connections.begin(), pin_connections.end());
	}
	return removeEmptyConnections(pin_list);
}

Pin_Connections powerPinMatch(vector<Pin*> &left_pins, vector<Pin*> &right_pins, 
	Component_Pair &component_pair)
{
	Pin_Connections pow_pin_list;
	for (size_t i = 0; i < left_pins.size(); i++)
	{
		for (size_t j = 0; j < right_pins.size(); j++)
		{
			stringpair pow_connection = grammer(left_pins[i], right_pins[j], 
				component_pair);
			pow_pin_list.insert(pow_connection);
			if (pow_connection.first != "")
			{
				pow_pin_list.insert(matchDutyCycles(left_pins[i],
					right_pins[j], component_pair));
			}
		}
	}
	return pow_pin_list;
}

Pin_Connections funcPinMatch(vector<Pin*> &left_pins, vector<Pin*> &right_pins, 
	Component_Pair &component_pair)
{
	Pin_Connections func_pin_list;
	for (size_t i = 0; i < left_pins.size(); i++)
	{
		for (size_t j = 0; j < right_pins.size(); j++)
		{
			func_pin_list.insert(grammer(left_pins[i], right_pins[j], 
				component_pair));
//			func_pin_list.insert(matchDutyCycles(left_pins[i], 
//				right_pins[j], component_pair));
		}
	}
	return func_pin_list;
}

stringpair grammer(Pin *left_pin, Pin *right_pin, 
	Component_Pair &component_pair)
{
	int ALL_PASS = 0b11111;
	stringpair power_pin_connection;
	std::bitset<5> conditions_code;
	
	// conditions need to satisfy
	// left pin in usable state (see dependents)
	vector<Pin*> dependent_pins = component_pair.second->getDependentPins();
	std::find(dependent_pins.begin(), dependent_pins.end(), right_pin) ==
		dependent_pins.end() ? conditions_code[4] = 0 : conditions_code[4] = 1;
	
	// both pins in active state
	(!right_pin->status || right_pin->connection == Electronics::CONNECTION::OTM) 
		&& (!left_pin->status || left_pin->connection == 
			Electronics::CONNECTION::OTM) ? conditions_code[3] = 1 : 
		conditions_code[3] = 0;

	// both pins in same physical shape
	left_pin->phys_type == right_pin->phys_type ? conditions_code[2] = 1 : 
		conditions_code[2] = 0;

	// voltage ranges match
	isIntersect(right_pin->v_bound, left_pin->v_bound) ? conditions_code[1] = 1 :
		conditions_code[1] = 0;

	// function type match
	funcTypeCompare(left_pin->func_type, 
		right_pin->func_type, component_pair ) ? conditions_code[0] = 1 :
		conditions_code[0] = 0;

	if (conditions_code == ALL_PASS)
	{
		power_pin_connection = make_pair(
			createConnectionName(component_pair.first->getComponentName(), 
				left_pin->name),
			createConnectionName(component_pair.second->getComponentName(), 
				right_pin->name));
		left_pin->status = true;
		right_pin->status = true;
		left_pin->v_bound = getIntersect(right_pin->v_bound, left_pin->v_bound);
	}
	return power_pin_connection;
}

Pin_Connections removeEmptyConnections(Pin_Connections &pin_connections)
{
	pin_connections.erase("");
	return pin_connections;
}

bool funcTypeCompare(Electronics::FUNCTION_TYPE &left_type,
	Electronics::FUNCTION_TYPE &right_type, Component_Pair &component_pair)
{
	unsigned result = false;
	if (left_type == right_type)
	{
		result = true;
	}
	else if (component_pair.first->getComponentType() == 
		Component_Type::Micro_Controller)
	{
		vector<Pin*> pins = component_pair.first->getFuncBidirectPins();
		bool pwm_empty = true, digital_empty = true;
		// if pwm/digital run out, use other pins 
		if (left_type == Electronics::PWM && 
			compatible_type_map.find(left_type)->second == right_type)
		{
			result = true;
			pwm_empty = false;
		}
		else
		{
			for (size_t i = 0; i < pins.size(); i++)
			{
				if (pins[i]->func_type == Electronics::PWM &&
					pins[i]->status == false &&
					compatible_type_map.find(pins[i]->func_type)->second
					== right_type)
				{
					result = false;
					pwm_empty = false;
					break;
				}
			}
		}

		if (pwm_empty)
		{
			if (left_type == Electronics::DIGITAL &&
				compatible_type_map.find(left_type)->second == right_type)
			{
				result = true;
				digital_empty = false;
			}
			else
			{
				for (size_t i = 0; i < pins.size(); i++)
				{
					if (pins[i]->func_type == Electronics::DIGITAL &&
						pins[i]->status == false &&
						compatible_type_map.find(pins[i]->func_type)->second
						== right_type)
					{
						result = false;
						digital_empty = false;
						break;
					}
				}
			}
		}

		if (pwm_empty && digital_empty)
		{
			auto &range = compatible_type_map.equal_range(left_type);
			for (auto &beg = range.first; beg != range.second; beg++)
			{
				if (beg->second == right_type) {
					result = true;
					break;
				}
			}
		}
	}
	else
	{
		auto &range = compatible_type_map.equal_range(left_type);
		for (auto &beg = range.first;  beg != range.second; beg++)
		{
			if (beg->second == right_type) {
				result = true;
				break;
			}
		}
	}
	return result;
}

stringpair matchDutyCycles(Pin *left_pin, Pin *right_pin, Component_Pair &component_pair)
{
	shared_ptr<Electrical_Component> left_component = component_pair.first,
		right_component = component_pair.second;

	if (left_component->getComponentType() ==
		Component_Type::Micro_Controller &&
		right_component->getComponentType() == Component_Type::H_Bridge)
	{
		H_Bridge temp_h_bridge(h_bridge_path +
			removeReplicate(right_component->getComponentName()) + ".txt");
		Micro_Controller temp_micro_controller(micro_controller_path +
			removeReplicate(left_component->getComponentName()) + ".txt");

		unordered_map<string, string> in_duty_cycle_map =
			temp_h_bridge.getInDutyCycleMap(),
			out_duty_cycle_map = temp_micro_controller.getOutDutyCycleMap();

		auto &hbridge_iter = in_duty_cycle_map.find(right_pin->name);
		auto &micro_controller_iter = out_duty_cycle_map.find(left_pin->name);

		if (hbridge_iter == in_duty_cycle_map.end() || 
			micro_controller_iter == out_duty_cycle_map.end())
		{
			return stringpair();
		}
		else
		{
			return make_pair(createConnectionName(left_component->getComponentName(),
				micro_controller_iter->second),
				createConnectionName(right_component->getComponentName(),
					hbridge_iter->second));
		}
	}
	else
	{
		return stringpair();
	}
}

void printPinConnections(Pin_Connections &pin_connections)
{
	for (auto &beg = pin_connections.begin(); beg != pin_connections.end();
		beg++)
	{
		cout << beg->first << " " << beg->second << endl;
	}
}

/*
void matching_test()
{
	//	matching_test1();
	//	matching_test2();
	//	matching_test3();
	matching_test4();
	//	matching_test5();
	//	matching_test6();
}

void matching_test1()
{
	std::string battery_file1 = std::experimental::filesystem::current_path().string() + "\\Connections\\BATTERY\\Computer_USB.txt",
		battery_file2 = std::experimental::filesystem::current_path().string() + "\\Connections\\BATTERY\\AmazonBasics_6LR61.txt",
		h_bridge_file = std::experimental::filesystem::current_path().string() + "\\Connections\\H_BRIDGE\\A3909.txt",
		micro_controller_file = std::experimental::filesystem::current_path().string() + "\\Connections\\MICRO_CONTROLLER\\Arduino_UNO.txt",
		motor_file = std::experimental::filesystem::current_path().string() + "\\Connections\\MOTOR\\Pololu_1124.txt",
		voltage_regulator_file = std::experimental::filesystem::current_path().string() + "\\Connections\\VOLTAGE_REGULATOR\\AZ1117-5.txt";

	Connection_Structure battery1(battery_file1), battery2(battery_file2), h_bridge(h_bridge_file), micro_controller(micro_controller_file),
		motor(motor_file), voltage_regulator(voltage_regulator_file);

	std::vector<std::pair<Connection_Structure&, Connection_Structure&>> component_relations;
	component_relations.push_back(std::pair<Connection_Structure&, Connection_Structure&>(h_bridge, motor));
	component_relations.push_back(std::pair<Connection_Structure&, Connection_Structure&>(voltage_regulator, h_bridge));
	component_relations.push_back(std::pair<Connection_Structure&, Connection_Structure&>(battery2, voltage_regulator));
	component_relations.push_back(std::pair<Connection_Structure&, Connection_Structure&>(micro_controller, h_bridge));
	component_relations.push_back(std::pair<Connection_Structure&, Connection_Structure&>(battery1, micro_controller));

	Component_Connection TestConnection(component_relations);
	TestConnection.components_matching();
	TestConnection.netlist_print();
}

void matching_test2()
{
	std::string battery_file1 = std::experimental::filesystem::current_path().string() + "\\Connections\\BATTERY\\Computer_USB.txt",
		battery_file2 = std::experimental::filesystem::current_path().string() + "\\Connections\\BATTERY\\AmazonBasics_6LR61.txt",
		h_bridge_file = std::experimental::filesystem::current_path().string() + "\\Connections\\H_BRIDGE\\A3909.txt",
		micro_controller_file = std::experimental::filesystem::current_path().string() + "\\Connections\\MICRO_CONTROLLER\\Arduino_UNO.txt",
		motor_file = std::experimental::filesystem::current_path().string() + "\\Connections\\MOTOR\\Pololu_1124.txt",
		voltage_regulator_file = std::experimental::filesystem::current_path().string() + "\\Connections\\VOLTAGE_REGULATOR\\AZ1117-5.txt";

	Connection_Structure battery1(battery_file1), battery2(battery_file2), h_bridge(h_bridge_file), micro_controller(micro_controller_file),
		motor1(motor_file), motor2(motor_file), voltage_regulator(voltage_regulator_file);

	std::vector<std::pair<Connection_Structure&, Connection_Structure&>> component_relations;
	component_relations.push_back(std::pair<Connection_Structure&, Connection_Structure&>(h_bridge, motor1));
	component_relations.push_back(std::pair<Connection_Structure&, Connection_Structure&>(h_bridge, motor2));
	component_relations.push_back(std::pair<Connection_Structure&, Connection_Structure&>(voltage_regulator, h_bridge));
	component_relations.push_back(std::pair<Connection_Structure&, Connection_Structure&>(battery2, voltage_regulator));
	component_relations.push_back(std::pair<Connection_Structure&, Connection_Structure&>(micro_controller, h_bridge));
	component_relations.push_back(std::pair<Connection_Structure&, Connection_Structure&>(battery1, micro_controller));

	Component_Connection TestConnection(component_relations);
	TestConnection.components_matching();
	TestConnection.netlist_print();
}

void matching_test3()
{
	// components
	std::string battery_file1 = std::experimental::filesystem::current_path().string() + "\\Connections\\BATTERY\\Computer_USB.txt",
		battery_file2 = std::experimental::filesystem::current_path().string() + "\\Connections\\BATTERY\\AmazonBasics_6LR61.txt",
		h_bridge_file = std::experimental::filesystem::current_path().string() + "\\Connections\\H_BRIDGE\\A3909.txt",
		micro_controller_file = std::experimental::filesystem::current_path().string() + "\\Connections\\MICRO_CONTROLLER\\Arduino_UNO.txt",
		motor_file = std::experimental::filesystem::current_path().string() + "\\Connections\\MOTOR\\Pololu_3060.txt",
		voltage_regulator_file = std::experimental::filesystem::current_path().string() + "\\Connections\\VOLTAGE_REGULATOR\\AZ1117-ADJ.txt";

	Connection_Structure battery1(battery_file1), battery2(battery_file2), h_bridge(h_bridge_file), micro_controller(micro_controller_file),
		motor1(motor_file), voltage_regulator(voltage_regulator_file);

	std::vector<std::pair<Connection_Structure&, Connection_Structure&>> component_relations;
	component_relations.push_back(std::pair<Connection_Structure&, Connection_Structure&>(h_bridge, motor1));
	component_relations.push_back(std::pair<Connection_Structure&, Connection_Structure&>(voltage_regulator, h_bridge));
	component_relations.push_back(std::pair<Connection_Structure&, Connection_Structure&>(battery2, voltage_regulator));
	component_relations.push_back(std::pair<Connection_Structure&, Connection_Structure&>(micro_controller, h_bridge));
	component_relations.push_back(std::pair<Connection_Structure&, Connection_Structure&>(battery1, micro_controller));

	Component_Connection TestConnection(component_relations);
	TestConnection.components_matching();
	TestConnection.netlist_print();
}

void matching_test4()
{
	// components
	std::string battery_file1 = std::experimental::filesystem::current_path().string() + "\\Connections\\BATTERY\\Computer_USB.txt",
		battery_file2 = std::experimental::filesystem::current_path().string() + "\\Connections\\BATTERY\\AmazonBasics_6LR61.txt",
		h_bridge_file = std::experimental::filesystem::current_path().string() + "\\Connections\\H_BRIDGE\\A3909.txt",
		micro_controller_file = std::experimental::filesystem::current_path().string() + "\\Connections\\MICRO_CONTROLLER\\Arduino_UNO.txt",
		motor_file = std::experimental::filesystem::current_path().string() + "\\Connections\\MOTOR\\Pololu_3060.txt",
		voltage_regulator_file = std::experimental::filesystem::current_path().string() + "\\Connections\\VOLTAGE_REGULATOR\\AZ1117-ADJ.txt";

	Connection_Structure battery1(battery_file1), battery2(battery_file2), h_bridge(h_bridge_file), micro_controller(micro_controller_file),
		motor1(motor_file), motor2(motor_file), voltage_regulator(voltage_regulator_file);

	std::vector<std::pair<Connection_Structure&, Connection_Structure&>> component_relations;
	component_relations.push_back(std::pair<Connection_Structure&, Connection_Structure&>(h_bridge, motor1));
	component_relations.push_back(std::pair<Connection_Structure&, Connection_Structure&>(h_bridge, motor2));
	component_relations.push_back(std::pair<Connection_Structure&, Connection_Structure&>(voltage_regulator, h_bridge));
	component_relations.push_back(std::pair<Connection_Structure&, Connection_Structure&>(battery2, voltage_regulator));
	component_relations.push_back(std::pair<Connection_Structure&, Connection_Structure&>(micro_controller, h_bridge));
	component_relations.push_back(std::pair<Connection_Structure&, Connection_Structure&>(battery1, micro_controller));

	Component_Connection TestConnection(component_relations);
	TestConnection.components_matching();
	TestConnection.netlist_print();
}

void matching_test5()
{
	std::string battery_file1 = std::experimental::filesystem::current_path().string() + "\\Connections\\BATTERY\\Computer_USB.txt",
		battery_file2 = std::experimental::filesystem::current_path().string() + "\\Connections\\BATTERY\\AmazonBasics_6LR61.txt",
		h_bridge_file = std::experimental::filesystem::current_path().string() + "\\Connections\\H_BRIDGE\\SN754410.txt",
		micro_controller_file = std::experimental::filesystem::current_path().string() + "\\Connections\\MICRO_CONTROLLER\\Arduino_UNO.txt",
		motor_file = std::experimental::filesystem::current_path().string() + "\\Connections\\MOTOR\\Pololu_3036.txt",
		voltage_regulator_file1 = std::experimental::filesystem::current_path().string() + "\\Connections\\VOLTAGE_REGULATOR\\AZ1117-ADJ.txt",
		voltage_regulator_file2 = std::experimental::filesystem::current_path().string() + "\\Connections\\VOLTAGE_REGULATOR\\AZ1117-5.txt";

	Connection_Structure battery1(battery_file1), battery2(battery_file2), h_bridge(h_bridge_file), micro_controller(micro_controller_file),
		motor1(motor_file), voltage_regulator1(voltage_regulator_file1), voltage_regulator2(voltage_regulator_file2);

	std::vector<std::pair<Connection_Structure&, Connection_Structure&>> component_relations;
	component_relations.push_back(std::pair<Connection_Structure&, Connection_Structure&>(h_bridge, motor1));
	component_relations.push_back(std::pair<Connection_Structure&, Connection_Structure&>(voltage_regulator1, h_bridge));
	component_relations.push_back(std::pair<Connection_Structure&, Connection_Structure&>(voltage_regulator2, h_bridge));
	component_relations.push_back(std::pair<Connection_Structure&, Connection_Structure&>(battery2, voltage_regulator1));
	component_relations.push_back(std::pair<Connection_Structure&, Connection_Structure&>(battery2, voltage_regulator2));
	component_relations.push_back(std::pair<Connection_Structure&, Connection_Structure&>(micro_controller, h_bridge));
	component_relations.push_back(std::pair<Connection_Structure&, Connection_Structure&>(battery1, micro_controller));

	Component_Connection TestConnection(component_relations);
	TestConnection.components_matching();
	TestConnection.netlist_print();
}

void matching_test6()
{
	std::string battery_file1 = std::experimental::filesystem::current_path().string() + "\\Connections\\BATTERY\\Computer_USB.txt",
		battery_file2 = std::experimental::filesystem::current_path().string() + "\\Connections\\BATTERY\\AmazonBasics_6LR61.txt",
		h_bridge_file = std::experimental::filesystem::current_path().string() + "\\Connections\\H_BRIDGE\\SN754410.txt",
		micro_controller_file = std::experimental::filesystem::current_path().string() + "\\Connections\\MICRO_CONTROLLER\\Arduino_UNO.txt",
		motor_file = std::experimental::filesystem::current_path().string() + "\\Connections\\MOTOR\\Pololu_3036.txt",
		voltage_regulator_file1 = std::experimental::filesystem::current_path().string() + "\\Connections\\VOLTAGE_REGULATOR\\AZ1117-ADJ.txt",
		voltage_regulator_file2 = std::experimental::filesystem::current_path().string() + "\\Connections\\VOLTAGE_REGULATOR\\AZ1117-5.txt";

	Connection_Structure battery1(battery_file1), battery2(battery_file2), h_bridge(h_bridge_file), micro_controller(micro_controller_file),
		motor1(motor_file), motor2(motor_file), voltage_regulator1(voltage_regulator_file1), voltage_regulator2(voltage_regulator_file2);

	std::vector<std::pair<Connection_Structure&, Connection_Structure&>> component_relations;
	component_relations.push_back(std::pair<Connection_Structure&, Connection_Structure&>(h_bridge, motor1));
	component_relations.push_back(std::pair<Connection_Structure&, Connection_Structure&>(h_bridge, motor2));
	component_relations.push_back(std::pair<Connection_Structure&, Connection_Structure&>(voltage_regulator1, h_bridge));
	component_relations.push_back(std::pair<Connection_Structure&, Connection_Structure&>(voltage_regulator2, h_bridge));
	component_relations.push_back(std::pair<Connection_Structure&, Connection_Structure&>(battery2, voltage_regulator1));
	component_relations.push_back(std::pair<Connection_Structure&, Connection_Structure&>(battery2, voltage_regulator2));
	component_relations.push_back(std::pair<Connection_Structure&, Connection_Structure&>(micro_controller, h_bridge));
	component_relations.push_back(std::pair<Connection_Structure&, Connection_Structure&>(battery1, micro_controller));

	Component_Connection TestConnection(component_relations);
	TestConnection.components_matching();
	TestConnection.netlist_print();
}
Connection_Structure clearPinConnections(Connection_Structure &component)
{
	component.clearPinConnections();
	return component;
}

std::vector<Connection_Structure> initializeConnectionVec(const std::string &path, stringvec &files)
{
	unsigned filenum = getFileNumInDirectory(path);
	std::vector<Connection_Structure> vec(filenum);
	files.resize(filenum);

	auto &vbeg = vec.begin();
	auto &fbeg = files.begin();
	for (auto &entry: std::experimental::filesystem::directory_iterator(path))
	{
		std::string &file = entry.path().string();
		*vbeg++ = Connection_Structure(file);
		*fbeg++ = file;
	}
	return vec;
}

unsigned determineConnectionFolder(const std::string &file)
{
	if (file.find(battery_connection_path) != std::string::npos)
	{
		return IN_BATTERY_CONNECTION_FOLDER;
	}
	else if (file.find(voltage_regulator_connection_path) != std::string::npos)
	{
		return IN_VOLTAGE_REGULATOR_CONNECTION_FOLDER;
	}
	else if (file.find(hbridge_connection_path) != std::string::npos)
	{
		return IN_HBRIDGE_CONNECTION_FOLDER;
	}
	else if (file.find(micro_controller_connection_path) != std::string::npos)
	{
		return IN_MICRO_CONTROLLER_CONNECTION_FOLDER;
	}
	else if (file.find(motor_connection_path) != std::string::npos)
	{
		return IN_MOTOR_CONNECTION_FOLDER;
	}
}

/*
void matching_optimization_module_test()
{
	// matching_optimization_module_test1();
	matching_optimization_module_test2();
}

void matching_optimization_module_test1()
{
	std::string battery_file1 = std::experimental::filesystem::current_path().string() + "\\Connections\\BATTERY\\Computer_USB.txt",
		battery_file2 = std::experimental::filesystem::current_path().string() + "\\Connections\\BATTERY\\AmazonBasics_6LR61.txt",
		h_bridge_file = std::experimental::filesystem::current_path().string() + "\\Connections\\H_BRIDGE\\A3909.txt",
		micro_controller_file = std::experimental::filesystem::current_path().string() + "\\Connections\\MICRO_CONTROLLER\\Arduino_UNO.txt",
		motor_file = std::experimental::filesystem::current_path().string() + "\\Connections\\MOTOR\\Pololu_1124.txt",
		voltage_regulator_file = std::experimental::filesystem::current_path().string() + "\\Connections\\VOLTAGE_REGULATOR\\AZ1117-5.txt";

	Connection_Structure battery1(battery_file1), battery2(battery_file2), h_bridge(h_bridge_file), micro_controller(micro_controller_file),
		motor(motor_file), voltage_regulator(voltage_regulator_file);

	std::vector<std::pair<Connection_Structure&, Connection_Structure&>> component_relations;
	component_relations.push_back(std::pair<Connection_Structure&, Connection_Structure&>(h_bridge, motor));
	component_relations.push_back(std::pair<Connection_Structure&, Connection_Structure&>(voltage_regulator, h_bridge));
	component_relations.push_back(std::pair<Connection_Structure&, Connection_Structure&>(battery2, voltage_regulator));
	component_relations.push_back(std::pair<Connection_Structure&, Connection_Structure&>(micro_controller, h_bridge));
	component_relations.push_back(std::pair<Connection_Structure&, Connection_Structure&>(battery1, micro_controller));

	Component_Connection TestConnection(component_relations);
	TestConnection.components_matching();
	TestConnection.netlist_print();
	Component_Connection::Netlist relation = TestConnection.get_netlist();

	try
	{
		GRBEnv env = GRBEnv();
		GRBModel model = GRBModel(env);

		Battery battery1_opt(path_convert(battery_file1)), battery2_opt(path_convert(battery_file2));
		H_bridge h_bridge_opt(path_convert(h_bridge_file));
		Micro_Controller micro_controller_opt(path_convert(micro_controller_file));
		Motor motor_opt(path_convert(motor_file), 1e-3, 0.12);
		V_regulator voltage_regulator_opt(path_convert(voltage_regulator_file));

		std::vector<Actu_components> components{ battery1_opt, battery2_opt, h_bridge_opt, micro_controller_opt, motor_opt, voltage_regulator_opt };

		// verification
		Circuit circuit(components, relation);
		circuit.verify(&model);
		model.write(std::experimental::filesystem::current_path().string() + "\\Tests\\test1.mps");
	}
	catch (const GRBException &e)
	{
		std::cout << "ERROR CODE: " << e.getErrorCode() << std::endl;
		std::cout << e.getMessage() << std::endl;
	}
	catch (const std::exception &e)
	{
		std::cout << "ERROR MESSAGE: " << e.what() << std::endl;
	}
}

void matching_optimization_module_test2()
{
	std::string battery_file1 = std::experimental::filesystem::current_path().string() + "\\Connections\\BATTERY\\Computer_USB.txt",
		battery_file2 = std::experimental::filesystem::current_path().string() + "\\Connections\\BATTERY\\AmazonBasics_6LR61.txt",
		h_bridge_file = std::experimental::filesystem::current_path().string() + "\\Connections\\H_BRIDGE\\A3909.txt",
		micro_controller_file = std::experimental::filesystem::current_path().string() + "\\Connections\\MICRO_CONTROLLER\\Arduino_UNO.txt",
		motor_file = std::experimental::filesystem::current_path().string() + "\\Connections\\MOTOR\\Pololu_1124.txt",
		voltage_regulator_file = std::experimental::filesystem::current_path().string() + "\\Connections\\VOLTAGE_REGULATOR\\AZ1117-5.txt";

	Connection_Structure battery1(battery_file1), battery2(battery_file2), h_bridge(h_bridge_file), micro_controller(micro_controller_file),
		motor1(motor_file), motor2(motor_file), voltage_regulator(voltage_regulator_file);

	std::vector<std::pair<Connection_Structure&, Connection_Structure&>> component_relations;
	component_relations.push_back(std::pair<Connection_Structure&, Connection_Structure&>(h_bridge, motor1));
	component_relations.push_back(std::pair<Connection_Structure&, Connection_Structure&>(h_bridge, motor2));
	component_relations.push_back(std::pair<Connection_Structure&, Connection_Structure&>(voltage_regulator, h_bridge));
	component_relations.push_back(std::pair<Connection_Structure&, Connection_Structure&>(battery2, voltage_regulator));
	component_relations.push_back(std::pair<Connection_Structure&, Connection_Structure&>(micro_controller, h_bridge));
	component_relations.push_back(std::pair<Connection_Structure&, Connection_Structure&>(battery1, micro_controller));

	Component_Connection TestConnection(component_relations);
	TestConnection.components_matching();
	TestConnection.netlist_print();
	Component_Connection::Netlist relation = TestConnection.get_netlist();

	try
	{
		GRBEnv env = GRBEnv();
		GRBModel model = GRBModel(env);

		Battery battery1_opt(path_convert(battery_file1)), battery2_opt(path_convert(battery_file2));
		H_bridge h_bridge_opt(path_convert(h_bridge_file));
		Micro_Controller micro_controller_opt(path_convert(micro_controller_file));
		Motor motor1_opt(path_convert(motor_file), 1e-3, 0.12), motor2_opt(path_convert(motor_file), 1e-3, 0.12);
		V_regulator voltage_regulator_opt(path_convert(voltage_regulator_file));

		std::vector<Actu_components> components{ battery1_opt, battery2_opt, h_bridge_opt, micro_controller_opt, motor1_opt, motor2_opt, voltage_regulator_opt };

		// verification
		Circuit circuit(components, relation);
		circuit.verify(&model);
		model.write(std::experimental::filesystem::current_path().string() + "\\Tests\\test1.mps");
	}
	catch (const GRBException &e)
	{
		std::cout << "ERROR CODE: " << e.getErrorCode() << std::endl;
		std::cout << e.getMessage() << std::endl;
	}
	catch (const std::exception &e)
	{
		std::cout << "ERROR MESSAGE: " << e.what() << std::endl;
	}
}
*/

