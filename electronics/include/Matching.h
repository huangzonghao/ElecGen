#pragma once
#ifndef MATCHING
#define MATCHING
#include "actuator.h"
#include <bitset>

// used to determine which two groups of pins will be connected
// left_power_out_pins: 0/1
// left_func_out_pins: 0/1
// left_func_bidirect_pins: 0/1
// right_power_in_pins: 0/1
// right_func_in_pins: 0/1
// right_func_bidirect_pins: 0/1
// right_both_bidrect_pins: 0/1

const int LPO_RPI = 0b1001000, LPO_RBB = 0b1000001, LFO_RFI = 0b0100100,
LFO_RFB = 0b0100010, LFO_RBB = 0b0100001, LFB_RFI = 0b0010100,
LFB_RFB = 0b0010010;

const std::bitset<7> power_mask = 0b1001001, func_mask = 0b0110111;

/*
class Connection_Structure
{
public:
	Connection_Structure() = default;
	Connection_Structure(const std::string &);
	Connection_Structure(const std::string &, const stringvec &);
	Connection_Structure(const Connection_Structure &);

//	void write(const std::string& filename);
	void parameters();
	std::string get_componentClass() const;
	void setComponentName(const std::string &);
	unsigned getUsedOutputPinNum() { return output_counter;  }
	unsigned getPinNum() { return power_in.size() + power_out.size() + power_function_in.size() + 
		power_function_out.size() + gnd.size() + functional_in.size() + functional_out.size() + functional_in_out.size();}
	double getPrice() { return price; }
	double getWeight() { return weight; }
	void clearPinConnections();
	void addUsablePins(const stringvec &);
	std::vector<Pin> getUsablePins() { return usable_pins; }
	void assignPinVoltageRange(const doublevec &, const doublevec &);

	friend Component_Connection;
	friend Circuit;

private:
	void read(const std::string &);
	void createAllPins();
	void extract(const Electronics::component_structure &);
	void clearPinConnections(std::vector<Pin> &);

	std::vector<Pin> power_in;
	std::vector<Pin> power_out;
	std::vector<Pin> power_function_in;
	std::vector<Pin> power_function_out;
	std::vector<Pin> gnd;
	std::vector<Pin> functional_in;
	std::vector<Pin> functional_out;
	std::vector<Pin> functional_in_out;
	std::vector<Pin> all_pins;
	std::vector<Pin> usable_pins;
	doublepairs usable_pin_voltage_ranges;
	unsigned output_counter = 0, input_counter = 0, enable_counter = 0;
};
*/
/*
class Component_Connection
{
public:
	using componentlist = std::vector<std::tuple<std::string, std::string>>;
	using componentpinslist = std::vector<std::tuple<std::string, std::string, std::vector<std::string>>>;
	Component_Connection() = default;
	Component_Connection(std::vector<std::pair<Connection_Structure &, Connection_Structure &>> &);
	void individual_matching(std::pair<Connection_Structure &, Connection_Structure &> *);
	void components_matching();
	void pow_pins_matching(const std::string &, const std::string &, std::vector<Pin> &, 
		std::vector<Pin> &, std::vector<Pin> &, doublepairs &);
	void func_pins_matching(Connection_Structure &, Connection_Structure &, std::vector<Pin> &, std::vector<Pin> &);
	void netlist_print();
	connection_relation get_netlist();
	componentlist getComponents();
	componentpinslist getComponentPins();

private:
	std::vector<std::pair<Connection_Structure &, Connection_Structure &>> component_relations;
	connection_relation netlist;
};
*/
using Component_Pair = std::pair<std::shared_ptr<Electrical_Component>, std::shared_ptr<Electrical_Component>>;
using Pin_Connections = std::unordered_multimap<std::string, std::string>;
Pin_Connections groupMatch(std::vector<Component_Pair> &);
Pin_Connections individualMatch(Component_Pair &);
Pin_Connections powerPinMatch(std::vector<Pin*> &, std::vector<Pin*> &, Component_Pair &);
Pin_Connections funcPinMatch(std::vector<Pin*> &, std::vector<Pin*> &, Component_Pair &);
stringpair grammer(Pin *, Pin *, Component_Pair &);
stringpair matchDutyCycles(Pin *, Pin *, Component_Pair &);
void printPinConnections(Pin_Connections &);
Pin_Connections removeEmptyConnections(Pin_Connections &);
bool funcTypeCompare(Electronics::FUNCTION_TYPE &, Electronics::FUNCTION_TYPE &, 
	Component_Pair &);


template <typename T> std::vector<T> unique_vec(std::vector<T>);

// void matching_test();
// void matching_test1();
// void matching_test2();
// void matching_test3();
// void matching_test4();
// void matching_test5();
// void matching_test6();

#endif // !MATCHING

template <typename T>
inline std::vector<T> unique_vec(std::vector<T> vec)
{
	std::sort(vec.begin(), vec.end());
	auto index = std::unique(vec.begin(), vec.end());
	vec.resize(std::distance(vec.begin(), index));
	return vec;
}


// Connection_Structure clearPinConnections(Connection_Structure &);
// std::vector<Connection_Structure> initializeConnectionVec(const std::string &, stringvec &);
// unsigned determineConnectionFolder(const std::string &);
