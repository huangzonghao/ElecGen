﻿#ifndef CIRCUIT_H_DMCIUXFF
#define CIRCUIT_H_DMCIUXFF

#include "Matching.h"

class Electrical_Component;

std::vector<GRBLinExpr> operator*(const Eigen::MatrixXd &, const std::vector<GRBVar> &);

namespace OPT_METHOD {
	const std::string NEWTON_RAPHSON = "NEWTON_RAPHSON",
		EM = "EXPECTATION_MAXIMIZATION",
		STOC = "STOCHASTIC";
}

enum verifyMode
{
	min, max, normal
};

class Circuit
{
public:
	typedef bool(Circuit::*func)(std::vector<std::shared_ptr<Electrical_Component>>, GRBModel *); // just declaration
	Circuit() = default;
	Circuit(std::vector<std::shared_ptr<Electrical_Component>> &, const Pin_Connections & = Pin_Connections());

	// methods
	void minUpdateConnectionsSolve(GRBModel *);
	void maxSolve(GRBModel *);
	void checkVars(GRBModel *);
	void verify(GRBModel *, verifyMode = min, const std::string & = OPT_METHOD::NEWTON_RAPHSON);
	void updateVerify(GRBModel *, verifyMode = min, const std::string & = OPT_METHOD::NEWTON_RAPHSON);
	void updateMaxObjs(GRBModel *);
	void updateMinObjs(GRBModel *);
//	void updateReplicates(std::vector<Electrical_Component*> &);
	void report();
	void updateComponents(const std::vector<std::shared_ptr<Electrical_Component>> &_components);
	void updateConnections(const Pin_Connections &);
	void setMotorWorkPoint1(GRBModel *, const doublepairs &, const doublepairs &);
	void setMotorWorkPoint2(GRBModel *, const doublepairs &, const doublepairs &);
	doublevec getVals(unsigned &, unsigned &, unsigned &);
	doublevec getMaxVals(unsigned &, unsigned &, unsigned &);
	unsigned getComponentsSize() { return static_cast<unsigned>(components.size()); }

//	std::vector<Electrical_Component*>& getComponents() { return components; }
	std::vector<std::shared_ptr<Electrical_Component>> getComponents() const { return components; }

	void syncVars(const GRBModel &);
	unsigned getMotorNumber();

	// get multiple solution tests
//	void getMultiVals(GRBModel *);
	void printComponents();
	void printRelations();
	Pin_Connections getComponentRelations() const { return pin_connections; }
	unsigned getConnectionsSize() const { return static_cast<unsigned>(pin_connections.size()); }
	int add(int x, int y) { return x + y; }
	int subtract(int x, int y) { return x - y; }
	static bool enable_reporting;

private:
	// one time
	void addVars(GRBModel *);
	void addObjs(GRBModel *);
	void addMinObjs(GRBModel *);
	void addMaxObjs(GRBModel *);
	void addModelCons(GRBModel *);
	void addLinCons(GRBModel *);
	void addCompositions(GRBModel *);

	// recursive
	void updateVars(GRBModel *);
	void updateObjs(GRBModel *);
	void updateModelCons(GRBModel *);
	void updateLinCons(GRBModel *);
	void updateCompositions(GRBModel *);
	void restorePrevModelCons(GRBModel *);

	bool newton_raphson(std::vector<std::shared_ptr<Electrical_Component>>, GRBModel *);
	bool expect_maximize(std::vector<std::shared_ptr<Electrical_Component>>, GRBModel *) { return false; }
	bool stochastic_optimize(std::vector<std::shared_ptr<Electrical_Component>>, GRBModel *) { return false; }
//	void getComponentPins(str_strvec_uomap &, const std::string &, const std::string &);


	std::vector<std::shared_ptr<Electrical_Component>> components;
	std::vector<std::shared_ptr<Electrical_Component>> new_components;
	std::unordered_map<std::string, std::shared_ptr<Electrical_Component>> component_maps;
	stringvec component_names;
	Pin_Connections pin_connections;
	Pin_Connections new_pin_connections;
	std::vector<GRBConstr> cons; // constraint vector
	std::vector<GRBConstr> model_cons;
	stringvec model_cons_names;
	std::vector<GRBConstr> lin_cons;
 	GRBLinExpr objective;
	bool first_flag = true;

	std::unordered_map<std::string, func> method_maps = { {OPT_METHOD::NEWTON_RAPHSON, &Circuit::newton_raphson},
	{OPT_METHOD::EM, &Circuit::expect_maximize},
	{OPT_METHOD::STOC, &Circuit::stochastic_optimize} };
};
bool isDutyCycle(const std::string &);

#endif /* end of include guard: CIRCUIT_H_DMCIUXFF */
