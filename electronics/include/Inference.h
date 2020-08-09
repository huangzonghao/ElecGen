﻿#pragma once
// #include <memory>
#include "Circuit.h"
// #include <sstream>
// #include <cstring>
// #include <stdlib.h> 
// #include "boost/variant/variant.hpp"
// #include <discreture.hpp>
// #include <type_traits>
// #include <iomanip>

struct Infer_Node;
struct BBNode;
//using bbglobalmap = std::vector<std::tuple<BBNode, unsigned>>;

using infernodevec = std::vector<Infer_Node>;
using infernodevec2d = std::vector<infernodevec>;
using bbnodevec = std::vector<BBNode>;
using cliqueindex = std::vector<intvec>;
using cliqueindex2d = std::vector<cliqueindex>;

static GRBEnv env = GRBEnv();
// using compoundtype = std::tuple<std::vector<Electrical_Component*>, unsignedpairs, unsignedpairs, unsignedpairs>;
// using compoundsettype = std::tuple<doublepairs, doublepairs, cliqueindex, cliqueindex, doublevec, doublepairs, doublepairs>;

const int DC_MOTOR_OPT = 0b00000000001, HBRIDGE_OPT = 0b00000000010,
MICRO_CONTROLLER_OPT = 0b00000000100, VOLTAGE_REGULATOR_OPT = 0b00000001000,
BATTERY_OPT = 0b00000010000, ENCODER_OPT = 0b00000100000,
CAMERA_OPT = 0b00010000000, FORCE_SENSOT_OPT = 0b00100000000,
BLUETOOTH_OPT = 0b01000000000, SERVO_OPT = 0b10000000000;

const stringvec end_components{ Component_Type::Motor, Component_Type::Encoder,
Component_Type::Camera, Component_Type::Bluetooth, Component_Type::Servo, 
Component_Type::Micro_Controller };

const stringvec add_components{ Component_Type::H_Bridge, 
Component_Type::Voltage_Regulator, Component_Type::Battery };

struct BBNode
{
	infernodevec infer_nodes;
	Circuit circuit;
	GRBModel model = GRBModel(env);
//	cctrackvec current_tracker_vec;
	doublepairs2d vol_ranges;
	cliqueindex2d vol_index;
	cliqueindex2d vol_vol_index;
	doublevec3d current_set;
//	stringvec connect_components_vec;

	unsigned id = 0;
	unsigned level = 0;
	unsigned component_num = 0;
	unsigned pin_num = 0;
	unsigned connection_num = 0;
	double price = 0.0;
	double weight = 0.0;
	doublevec component_weights;
//	doublevec motor_mass = doublevec();
	double power_consumption = 0.0;
	double eval_metric = 0.0;
	bool feasibility = true;
	static doublevec coefficients;
	static unsigned num_of_bbnodes;
	static unsigned last_level;
	static double best_metric_val;
//	static connection_relation final_connections;

	stringvec next_types;
	std::unordered_map<std::string, unsignedvec> next_type_map; // type-prev components
	std::unordered_map<std::string, unsignedpair> next_version_map;
	Pin_Connections pin_connections;

	BBNode() = default;
	BBNode(const infernodevec &, BBNode * = new BBNode);
	BBNode(const infernodevec &, const Circuit &, GRBModel &, BBNode * = new BBNode);

	bool operator==(const BBNode &bbnode) const { return id == bbnode.id; }

//	void copyAttributes(const BBNode &);
//	void getRangeNIndex(const doublepairs &, const doublepairs &, const cliqueindex &, const cliqueindex &, const doublevec &);
	void evaluate();
	void computeComponentNum();
//	void computePinNum();
//	void computeConnectionNum();
//	void computePrice();
//	void computeWeight();
	void computeMetricVal();
	//	void computePowerConsump (const doublepairs2d &, const doublepairs2d &);

	void optimize();
	bbnodevec branch();
	void bound();
	stringvec typeInfer();
	stringvec2d versionInfer();
	infernodevec2d numberInfer(const stringvec2d &);
	void minCliqueCover();
	void clearPinConnections();
	std::vector<Electrical_Component*> getComponents();
	Pin_Connections getPinConections() { return pin_connections; }
	void updateInPrevNode();
	unsigned getComponenetNum() const { return component_num; }
	unsigned getPinNum() const { return pin_num; }
	unsigned getConnectionsNum() const { return connection_num; }	
	double getPrice() const { return price; }
	double getWeight() const { return weight; }
	double getPowerConsump() const { return power_consumption; }
//	void getMotorMass();
	double getMetricVal() const;
	bool empty() const { return infer_nodes.empty(); }

	BBNode *prev_bbnode;
	bbnodevec next_bbnodes;
	void addDescendent(BBNode &next_node) { next_bbnodes.push_back(next_node); }
	void addAncenstor(BBNode *prev_node) { prev_bbnode = prev_node; }
	BBNode* getAncensotr() { return prev_bbnode; }
	bool isBottom();
};

struct Infer_Node
{
//	double extra_current = 0;
	bool operator==(const Infer_Node &obj) const { return id == obj.id; }
	
	std::string getType() const { return component->getComponentType(); }
	std::string getName() const { return component->getComponentName(); }
	void getUsedVarsName(const stringvec &);
	void getUsedNonLinVarNames(const stringvec &);

	Infer_Node() = default;
	Infer_Node(const std::string &, Infer_Node & = Infer_Node());
	Infer_Node(Electrical_Component *, Infer_Node & = Infer_Node());
//	~Infer_Node() { delete component; }
	bool empty() { return component == nullptr; }
	void addPrevNode(Infer_Node &);
	void addNextNode(Infer_Node &);
	void computeElectricProperties();
	doublevec getInVal(const std::string &);
	double getOutVal(const Electronics::CLASS &);
	doublepairs getInVolRange(const std::string &);
	doublevec getInCurrentLimit(const std::string &);
	doublevec getPowerInCurrent() { return pow_in_current; }
	unsigned getMainPowerIndex() { return component->getMainPowerIndex(); }
	void computePowInCurrent();
	Electrical_Component *getComponent() { return component; }
	infernodevec &getDescendents() { return next_nodes; }
	infernodevec &getAnscentors() { return prev_nodes; }

private:
	Electrical_Component *component;
	infernodevec prev_nodes;
	infernodevec next_nodes;
	unsigned id;
	bool visited = false;
	doublevec pow_in_current = doublevec();
	doublevec func_in_cuurent = doublevec();
	doublepairs pow_in_vol_range = doublepairs();
	doublepairs func_in_vol_range = doublepairs();
	static unsigned num_of_nodes;

	double getFuncOutSize();
	double getPowerOutLimit();
	double getFuncInSize();
	doublevec getPowerInVal();
	doublepairs getFuncInVolRange();
	doublepairs getPowerInVolRange();
//	Infer_Node(const std::string &_type, const std::string &_version): type(_type), version(_version) { id = ++num_of_nodes; }
//	Infer_Node(const std::string &_type, const std::string &_version, const double &_current): type(_type),
//		version(_version), extra_current(_current){
//		id = ++num_of_nodes;
//	}
};

// initialization function
/*
stringvec preprocessing(const std::string &, const doublepairs &, const doublepairs &);
std::string preprocessing(const doublepair &, const doublepair &);
stringvec2d allPreprocessing(const std::string &, const doublepairs &, const doublepairs &);
stringvec2d allPreprocessing(const std::string &, const doublepairs &);
stringvec allPreprocessing(const doublepair &, const doublepair &);
nodeptrvec initialization(const std::string &, const doublepairs &, const doublepairs &);
bbnodevec allInitialization(const std::string &, const doublepairs &, const doublepairs &);
bbnodevec allInitialization(const std::string &, const doublepairs &);
*/

// version/number graph formulation function

/*
compoundtype generateNeedVec(const infernodevec &, const doublepairs & = doublepairs(), const doublepairs & = doublepairs());
compoundsettype generateNeedSet(Circuit &, GRBModel &, compstructptrvec &, CopyCurrentTracker &ctracker, const std::vector<Actu_components> &,
	unsignedpairs &, unsignedpairs &, unsignedpairs & = unsignedpairs());

bool doubleCheck(const doublepairs &, const doublepairs &, Circuit &, GRBModel &);
bool doubleCheck(const doublepairs &, const doublepairs &, BBNode &);

*/
// component type inference rules
bool backTrack(BBNode &);

stringvec typeInfer(const Infer_Node &);
stringvec typeInfer(const std::string &);
stringvec removeEmptyTypes(stringvec &);
bool hasMatchComponents(const stringvec &, const infernodevec &);

stringvec2d versionInfer(const std::string &, const doublepairs &, const doublepairs &, const doublevec &);
stringvec2d versionInfer(const std::string &, const doublepairs &, const doublevec2d &);
stringvec versionInfer(const std::string &, const doublepair &, const doublevec &);
double getCurrentVal(const std::string &, const doublevec & = doublevec());

 // infernodevec numberInfer(const std::string &, infernodevec &, const stringvec &,
//	const cliqueindex &, const cliqueindex &);
 infernodevec numberInfer(const std::string &, infernodevec &, const stringvec &,
	 const cliqueindex &, const cliqueindex &);

cliquetype minCliqueCover(const doublepairs &);
/*
cliquetype exactrRangeCover(const doublepairs &);

doublevec getCurrentClique(const cliqueindex &, const doublevec &);
doublevec changeCurrentSet(doublevec &, const connect_relation &, const unsigned &);

// void findMatchNodes(const nodeptrvec &, const std::string &, nodeptrvec &);
bool findMatchBattery(const doublepair &, const double &);
connection_relation sameLevelComponentsReduction(BBNode &);
connection_relation_vec sameLevelComponentsReduction(bbnodevec &);

// nodeptr findAvailPowerComponent(nodeptrvec &, const stringvec &, const double &);

// auxiliary function 
// visualization function
// void treeVisualize(nodeptr root, unsigned cnt = 0);

std::string indexComponent(std::string);
// void writeTree(const nodeptrvec &, std::ofstream &, str_unsigned_uomap &);
// void writeDesign(const nodeptrvec &, const Circuit &, str_unsigned_uomap &);
void writeDesign(const BBNode &);
void writeRawDesign(const BBNode &);

// This function return different types of components and their indices as vector
// std::tuple<nodeptrvec2d, unsignedvec2d, connection_relation_vec> identifyTypes(const nodeptrvec &, const connection_relation_vec &);
// std::tuple<nodeptrvec2d, unsignedvec2d> identifyTypes(const nodeptrvec &);
std::tuple<infernodevec2d, unsignedvec2d> identifyTypes(const infernodevec &);

// This function return nodes that have descendents
// nodeptrvec getAproNodes(const nodeptrvec &);
// nodeptrvec generateNodeptrVec(const std::string &, const stringvec &);
infernodevec generateNodeptrVec(const std::string &, const stringvec &);

// This function checks the inputs' validity
bool inputsValidityCheck(const doublepairs &, const doublepairs &);
*/
int branchNBound(bbnodevec & = bbnodevec());

//void expandMap(const bbnodevec &, bbglobalmap &);

// BBNode getOptimalDesign(const bbglobalmap &, const int &);

void evaluate(bbnodevec &, connection_relation_vec2d & = connection_relation_vec2d(),
	const doublepairs & = doublepairs(), const doublepairs & = doublepairs());
void evaluate(BBNode &, connection_relation_vec & = connection_relation_vec(), 
	const doublepairs & = doublepairs(), const doublepairs & = doublepairs());
/*
compstructptrvec getComponentConnections(infernodevec &);

bool checkTermination(const bbglobalmap &);
// void disableBBNode(const unsigned &, bbglobalmap &);
void printConnections(const BBNode &, const bbnodevec &, const connection_relation_vec2d &);
std::vector<Actu_components> generateComponents(stringvec &, testinput &);
*/
std::vector<Component_Pair> extractComponentPairs(infernodevec &);
std::vector<Component_Pair> extractComponentPairs(infernodevec &, infernodevec &);
Pin_Connections maxNodeMatch(BBNode &, infernodevec & = infernodevec());
Pin_Connections nodeMatch(infernodevec &);
Pin_Connections nodeMatch(infernodevec &, infernodevec &);
void getDependentPins(infernodevec &, Pin_Connections &);
infernodevec getAllAnscenstor(BBNode &);

Electrical_Component *creatComponent(const std::string &);

namespace Current_Operation {
	double add(const doublevec &vec);
	double max(const doublevec &vec);
}