#pragma once
#include "Circuit.h"
struct Infer_Node;
struct BBNode;

using infernodevec = std::vector<Infer_Node>;
using infernodevec2d = std::vector<infernodevec>;
using bbnodevec = std::vector<BBNode>;
using cliqueindex = std::vector<intvec>;
using cliqueindex2d = std::vector<cliqueindex>;

extern GRBEnv env;

const int DC_MOTOR_OPT = 0b0000000001, HBRIDGE_OPT = 0b0000000010,
MICRO_CONTROLLER_OPT = 0b0000000100, VOLTAGE_REGULATOR_OPT = 0b0000001000,
BATTERY_OPT = 0b0000010000, ENCODER_OPT = 0b0000100000,
CAMERA_OPT = 0b0001000000, FORCE_SENSOT_OPT = 0b0010000000,
BLUETOOTH_OPT = 0b0100000000, SERVO_OPT = 0b1000000000;

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
	doublepairs2d vol_ranges;
	cliqueindex2d vol_index;
	cliqueindex2d vol_vol_index;
	doublevec3d current_set;
	stringvec component_names;
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
	static bool pruning_enable;
//	static connection_relation final_connections;

	stringvec next_types;
	std::unordered_map<std::string, unsignedvec> next_type_map; // type-prev components
	std::unordered_map<std::string, unsignedpair> next_version_map;
	Pin_Connections pin_connections;

	BBNode() = default;
	BBNode(const infernodevec &, BBNode * = NULL);
	BBNode(const infernodevec &, const Circuit &, GRBModel &, BBNode * = NULL);

	bool operator==(const BBNode &bbnode) const { return id == bbnode.id; }

	void copyMetrics(const BBNode &);
//	void getRangeNIndex(const doublepairs &, const doublepairs &, const cliqueindex &, const cliqueindex &, const doublevec &);
	void evaluate();
	void reevaluate();
	void computeComponentNum();
	void computePinNum();
	void computeConnectionNum();
	void computePrice();
	void computeWeight();
	void computeMetricVal();
	void computePowerConsump();

	void optimize();
	void updateConnections(const Pin_Connections &);
	bbnodevec branch();
	void bound();
	stringvec typeInfer();
	stringvec2d versionInfer();
	infernodevec2d numberInfer(const stringvec2d &);
	void minCliqueCover();
	void clearPinConnections();
	void makeReplicates();
	std::vector<std::shared_ptr<Electrical_Component>> getComponents();
	Pin_Connections getPinConections() { return pin_connections; }
	void updateInPrevNode();
	unsigned getComponenetNum() const { return component_num; }
	unsigned getPinNum() const { return pin_num; }
	unsigned getConnectionsNum() const { return connection_num; }
	double getPrice() const { return price; }
	double getWeight() const { return weight; }
	double getPowerConsump() const { return power_consumption; }
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

	std::string getType() const;
	std::string getName() const;
	double getPrice() const;
	double getWeight() const;
	unsigned getPinNum() const;
	void setName(const std::string &name) { component->setComponentName(name); }
	unsigned getId() const { return id; }
	void getUsedVarsName(const stringvec &);
	void getUsedNonLinVarNames(const stringvec &);

	Infer_Node() = default;
    Infer_Node(const std::string &);
	Infer_Node(const std::string &, Infer_Node &);
	Infer_Node(std::shared_ptr<Electrical_Component>);
	Infer_Node(std::shared_ptr<Electrical_Component>, Infer_Node &);
//	~Infer_Node() { std::cout << "INFER NODE DESTRUCTOR" << std::endl; }
	bool empty() const;
	void addPrevNode(Infer_Node &);
	void addNextNode(Infer_Node &);
	void computeElectricProperties(Pin_Connections &);;
	doublevec getInVal(const std::string &);
	double getOutVal(const Electronics::CLASS &);
	doublepairs getInVolRange(const std::string &);
	doublevec getInCurrentLimit(const std::string &);
	stringvec getInPinNames(const std::string &) const;
	stringvec getOutPinNames(const std::string &) const;
	doublevec getPowerInCurrent() const { return pow_in_current; }
	doublevec getFuncInCurrent() const { return func_in_current; }
	doublepairs getPowerInVolRange() const { return pow_in_vol_range; }
	doublepairs getFuncInVolRange() const { return func_in_vol_range; }
	doublepair getPowerOutVolRange() const;
	doublepair getFuncOutVolRange() const;
	unsigned getMainPowerIndex();
	void computePowInCurrent(Pin_Connections &);
	void computePowInPinNames();
	void computePowOutPinNames();
	void computeFuncInPinNames();
	void computeFuncOutPinNames();
	void setPowerExistVec(const size_t &) const;
	void setFuncExistVec(const size_t &) const;
	stringvec getPowInPinNames() const { return pow_in_pin_names; }
	stringvec getPowOutPinNames() const { return pow_out_pin_names; }
	stringvec getFuncInPinNames() const { return func_in_pin_names; }
	stringvec getFuncOutPinNames() const { return func_out_pin_names; }
	std::shared_ptr<Electrical_Component> getComponent();
	infernodevec &getDescendants() { return next_nodes; }
	infernodevec getAncestors();
	void updatePrevNode(const Infer_Node &);
	void updateNextNode(const Infer_Node &);

private:
	std::shared_ptr<Electrical_Component> component;
	unsignedvec prev_node_ids;
	infernodevec next_nodes;
	unsigned id;
	bool visited = false;
	mutable boolvec pow_in_exist_vec = boolvec();
	mutable boolvec func_in_exist_vec = boolvec();
	doublevec pow_in_current = doublevec();
	doublevec func_in_current = doublevec();
	doublepairs pow_in_vol_range = doublepairs();
	doublepairs func_in_vol_range = doublepairs();
	stringvec pow_in_pin_names = stringvec();
	stringvec pow_out_pin_names = stringvec();
	stringvec func_in_pin_names = stringvec();
	stringvec func_out_pin_names = stringvec();
	double extra_current = 0.0;
	static unsigned num_of_nodes;

	double getFuncOutSize();
	double getPowerOutLimit();
	double getFuncInSize();
	doublevec getPowerInVal();
//	doublepairs getFuncInVolRange();
//	doublepairs getPowerInVolRange();
};


// initialization function
stringvec2d preprocess(const stringvec &, const doublepairs & = doublepairs(),
	const doublepairs & = doublepairs());
stringvec2d sensorPreprocess(const stringvec &);
stringvec2d actuatorPreprocess(const stringvec &, const doublepairs & = doublepairs(),
	const doublepairs & = doublepairs());
infernodevec2d initialize(const stringvec2d &, const doublepairs & = doublepairs(),
	const doublepairs & = doublepairs());
bbnodevec initialize(const infernodevec2d &);
std::vector<stringpair> postprocessing(const Pin_Connections &, const stringvec &,
	const unsignedvec &, const unsignedvec &);
std::string replaceConnections(const std::string &, const std::string &);
doublevec getMassVec(const BBNode &, const unsigned &);
bool doubleCheck(BBNode &, const doublepairs &, const doublepairs &);

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
stringvec removeEmptyTypes(const stringvec &);
boolvec hasMatchComponents(const stringvec &, const infernodevec &,
	const Infer_Node &, const Pin_Connections &);

stringvec2d versionInfer(const std::string &, const doublepairs &, const doublepairs &, const doublevec &);
stringvec2d versionInfer(const std::string &, const doublepairs &, const doublevec2d &);
stringvec versionInfer(const std::string &, const doublepair &, const doublevec &);
double getCurrentVal(const std::string &, const doublevec & = doublevec());

 // infernodevec numberInfer(const std::string &, infernodevec &, const stringvec &,
//	const cliqueindex &, const cliqueindex &);
infernodevec numberInfer(const std::string &, infernodevec &, const stringvec &,
	 const cliqueindex &, const cliqueindex &);
bool hasTwoVoltageRegulators(const std::string &, const std::string &);

/*
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
*/
void writeDesign(const BBNode &);
void writeRawDesign(const BBNode &);
/*
// This function return different types of components and their indices as vector
// std::tuple<nodeptrvec2d, unsignedvec2d, connection_relation_vec> identifyTypes(const nodeptrvec &, const connection_relation_vec &);
// std::tuple<nodeptrvec2d, unsignedvec2d> identifyTypes(const nodeptrvec &);
std::tuple<infernodevec2d, unsignedvec2d> identifyTypes(const infernodevec &);

// This function return nodes that have descendents
// nodeptrvec getAproNodes(const nodeptrvec &);
// nodeptrvec generateNodeptrVec(const std::string &, const stringvec &);
infernodevec generateNodeptrVec(const std::string &, const stringvec &);
*/
// This function checks the inputs' validity
bool inputsValidityCheck(const doublepairs &, const doublepairs &);

std::shared_ptr<BBNode> branchNBound(bbnodevec * = new bbnodevec);

//void expandMap(const bbnodevec &, bbglobalmap &);

// BBNode getOptimalDesign(const bbglobalmap &, const int &);
/*
compstructptrvec getComponentConnections(infernodevec &);

bool checkTermination(const bbglobalmap &);
// void disableBBNode(const unsigned &, bbglobalmap &);
void printConnections(const BBNode &, const bbnodevec &, const connection_relation_vec2d &);
std::vector<Actu_components> generateComponents(stringvec &, testinput &);
*/
std::vector<Component_Pair> extractComponentPairs(infernodevec &);
std::vector<Component_Pair> extractComponentPairs(infernodevec &,
	const unsignedpair &, const unsignedpair &);
std::vector<Component_Pair> extractConsecutiveComponentPairs(infernodevec &, infernodevec &);
Pin_Connections maxNodeMatch(BBNode &, const infernodevec & = infernodevec());
Pin_Connections nodeMatch(infernodevec &);
Pin_Connections nodeMatch(infernodevec &, const unsignedpair &, const unsignedpair &);
Pin_Connections consecutiveNodeMatch(infernodevec &, infernodevec &);

void createLinks(infernodevec &, Pin_Connections &);
void createLinks(infernodevec &, infernodevec &, Pin_Connections &);
void getDependentPins(infernodevec &, Pin_Connections &);
infernodevec getAllAnscenstor(BBNode &);
void addInferNodeMap(const infernodevec &);
std::string makeReplicate(const std::string &, const stringvec &);
std::string makeReplicate(const std::string &);

std::shared_ptr<Electrical_Component> creatComponent(const std::string &);
stringvec getAllSensorVersions(const std::string &);
stringvec getAllVersions(const std::string &);
stringvec2d getAllActuatorVersions(const std::string &, const std::vector<std::pair<doublepair, doublepair>> &);
bool operator==(const std::pair<doublepair, doublepair> &,
	const std::pair<doublepair, doublepair> &);
stringvec2d getMotorVersions(const std::string &, const std::vector<std::pair<doublepair, doublepair>> &);
stringvec getMotorVersions(const std::string &, const std::pair<doublepair, doublepair> &);

namespace Current_Operation {
	double add(const doublevec &vec);
	double max(const doublevec &vec);
}
