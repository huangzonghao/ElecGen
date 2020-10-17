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

using Component_Pair = std::pair<std::shared_ptr<Electrical_Component>, std::shared_ptr<Electrical_Component>>;
using Pin_Connections = std::unordered_multimap<std::string, std::string>;
Pin_Connections groupMatch(const std::vector<Component_Pair> &);
Pin_Connections individualMatch(const Component_Pair &);
Pin_Connections powerPinMatch(std::vector<Pin*> &, std::vector<Pin*> &, const Component_Pair &);
Pin_Connections funcPinMatch(std::vector<Pin*> &, std::vector<Pin*> &, Component_Pair &);
stringpair grammer(Pin *, Pin *, const Component_Pair &);
stringpair matchDutyCycles(const stringpair &, const Component_Pair &);
void printPinConnections(Pin_Connections &);
Pin_Connections removeEmptyConnections(Pin_Connections &);
std::pair<bool, Pin*> funcTypeCompare(Electronics::FUNCTION_TYPE &, Electronics::FUNCTION_TYPE &,
	const Component_Pair &);


template <typename T> std::vector<T> unique_vec(std::vector<T>);

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
