#include "Circuit.h"
bool Circuit::enable_reporting = false;

using std::vector;
using std::string;
using std::cout;
using std::endl;
using Eigen::MatrixXd;
using std::unordered_map;
using std::shared_ptr;

unordered_map<Electronics::RELATION, char> sign_map{
	{Electronics::RELATION::GREATER, GRB_GREATER_EQUAL},
	{Electronics::RELATION::EQUAL, GRB_EQUAL},
	{Electronics::RELATION::LESS, GRB_LESS_EQUAL}
};


Circuit::Circuit(vector<shared_ptr<Electrical_Component>> &_components, const Pin_Connections &_connections):
	components(_components), pin_connections(_connections)
{
	for (size_t i = 0; i < components.size(); i++)
	{
		component_names.push_back(components[i]->getComponentName());
		component_maps[*component_names.rbegin()] = components[i];
	}
}
/*
void Circuit::getMultiVals(GRBModel *model)
{
	std::cout << "Objective value: " << model->get(GRB_DoubleAttr_ObjVal) << std::endl;
	std::cout << "MIP? " << model->get(GRB_IntAttr_IsMIP) << std::endl;
	std::cout << "Number: " << model->get(GRB_IntAttr_SolCount) << std::endl;
	model->set(GRB_IntParam_SolutionNumber, 0);
	std::cout << components.begin()->vars[0].get(GRB_DoubleAttr_Xn) << std::endl;
}
*/
void Circuit::printComponents()
{
	for (size_t i = 0; i < components.size(); i++)
	{
		cout << components[i]->getComponentName() << endl;
	}
}

void Circuit::printRelations()
{
	for (auto beg = pin_connections.begin(); beg != pin_connections.end(); beg++)
	{
		cout << beg->first << " " << beg->second << endl;
	}
}

void Circuit::addVars(GRBModel *model) {

	for (size_t i = 0; i < components.size(); i++)
	{
		MatrixXd var_bound_mat = components[i]->getVarsBound();
		vector<char> var_types = components[i]->getVarsType();
		stringvec var_names = components[i]->getVarsName();
		for (size_t j = 0; j < components[i]->getVarsSize(); j++)
		{
			components[i]->vars[j] = model->addVar(var_bound_mat(j, 0), var_bound_mat(j, 1), 0, var_types[j], var_names[j]);
		}
		components[i]->constructVarMaps(); // sync vars
	}
}

void Circuit::addObjs(GRBModel *model) {

	model->setObjective(GRBLinExpr(0));
}

void Circuit::addMinObjs(GRBModel *model)
{
	for (size_t i = 0; i < components.size(); i++)
	{
		stringvec used_var_names = components[i]->getUsedVarsName();
		for (size_t j = 0; j < used_var_names.size(); j++)
		{
			if (isDutyCycle(used_var_names[j]))
			{
				objective -= 10*components[i]->vars[j];
			}
			else
			{
				objective += components[i]->vars[j];
			}
		}
	}
	model->setObjective(objective, GRB_MINIMIZE);
}

void Circuit::addMaxObjs(GRBModel *model)
{
	for (size_t i = 0; i < components.size(); i++)
	{
		stringvec used_var_names = components[i]->getUsedVarsName();
		for (size_t j = 0; j < used_var_names.size(); j++)
		{
			if (isDutyCycle(used_var_names[j]))
			{
				objective -= 10*components[i]->vars[j];
			}
			else
			{
				objective += components[i]->vars[j];
			}
		}
	}
	model->setObjective(objective, GRB_MAXIMIZE);
}

void Circuit::addModelCons(GRBModel *model)
{
	for (size_t i = 0; i < components.size(); i++)
	{
		// only solve used model
		stringvec used_var_names = components[i]->getUsedVarsName();
		unsignedvec used_index;;
		for (size_t j = 0; j < used_var_names.size(); j++)
		{
			const auto &iter = components[i]->model_index_map.find(used_var_names[j]);
			if (iter != components[i]->model_index_map.end())
			{
				used_index.push_back(iter->second);
			}
		}
        const vector<GRBLinExpr>& lin_exp = components[i]->getModelMat()*components[i]->vars;
		vector<char> model_relations = components[i]->getModelRelations();
		stringvec model_names = components[i]->getModelNames();
		for (size_t j = 0; j < used_index.size(); j++)
		{
			size_t row = used_index[j];
			model_cons.push_back(model->addConstr(lin_exp[row], model_relations[row], 0,
				model_names[row]));
		}
	}
}


void Circuit::addLinCons(GRBModel *model)
{
	for (size_t i = 0;  i< components.size(); i++)
	{
		if (components[i]->has_linear_constraints())
		{
			for (size_t j = 0; j < components[i]->linear_constraints.size(); j++)
			{
				stringvec var_names = components[i]->linear_constraints[j].var_names;
				MatrixXd coefficents = components[i]->linear_constraints[j].coefficients;
				Electronics::RELATION relation = components[i]->linear_constraints[j].type;
				double constant = components[i]->linear_constraints[j].constant;
				size_t var_size = var_names.size();
				vector<GRBVar> vars(var_size);
				for (size_t k = 0; k < var_size; k++)
				{
					vars[k] = *components[i]->var_maps[var_names[k]];
				}

				const vector<GRBLinExpr> &lin_cons = coefficents*vars;
				model->addConstr(lin_cons[0], sign_map[relation], constant,
					components[i]->getComponentName() + "LINEAR CONS");
			}
		}
	}
}


void Circuit::addCompositions(GRBModel *model) {

	for (auto beg = pin_connections.begin(); beg != pin_connections.end(); beg++)
	{
		stringpair left_pair = separateNames(beg->first);
		stringpair right_pair = separateNames(beg->second);
		shared_ptr<Electrical_Component> left_component = component_maps[left_pair.first],
			right_component = component_maps[right_pair.first];

		model->addConstr(*left_component->var_maps[left_pair.second] == *right_component->var_maps[right_pair.second]);
	}
}

void Circuit::minUpdateConnectionsSolve(GRBModel *model)
{
	updateMinObjs(model);
	updateModelCons(model);
	updateCompositions(model);
	model->update();
	model->optimize();
	if (model->get(GRB_IntAttr_Status) == GRB_OPTIMAL)
	{
		vector<shared_ptr<Electrical_Component>> nonlin_components;
		for (size_t i = 0; i < components.size(); i++)
		{
			if (components[i]->isNoninComponent())
			{
				nonlin_components.push_back(components[i]);
			}
		}

		bool indicator = std::invoke(method_maps[OPT_METHOD::NEWTON_RAPHSON], this, nonlin_components, model);
		if (indicator)
		{
			cout << MODEL_SOLVED << endl;
		}
		else
		{
			throw MODEL_IS_INFEASIBLE;
		}
	}
	else
	{
		throw MODEL_IS_INFEASIBLE;
	}

	// update voltage bounds
	for (size_t i = 0; i < components.size(); i++)
	{
		components[i]->updateUsedPinsVolLB();
	}

	// update dc motor/servo motor current bounds
	for (size_t i = 0; i < components.size(); i++)
	{
		if (components[i]->component_type == Component_Type::Motor ||
			components[i]->component_type == Component_Type::Servo)
		{
			double limit_current = components[i]->
				var_maps["I"]->get(GRB_DoubleAttr_X);
			components[i]->i_bound_mat(0, 1) = limit_current;
		}
	}
	if (Circuit::enable_reporting)
	{
		model->write("min_model.lp");
		report();
	}
	return;
}

void Circuit::maxSolve(GRBModel* model)
{
	updateMaxObjs(model);
	restorePrevModelCons(model);
	model->update();
	model->optimize();
	if (model->get(GRB_IntAttr_Status) == GRB_OPTIMAL)
	{
		vector<shared_ptr<Electrical_Component>> nonlin_components;
		for (size_t i = 0; i < components.size(); i++)
		{
			if (components[i]->isNoninComponent())
			{
				nonlin_components.push_back(components[i]);
			}
		}

		bool indicator = std::invoke(method_maps[OPT_METHOD::NEWTON_RAPHSON], this, nonlin_components, model);
		if (indicator)
		{
			cout << MODEL_SOLVED << endl;
		}
		else
		{
			throw MODEL_IS_INFEASIBLE;
		}
	}
	else
	{
		throw MODEL_IS_INFEASIBLE;
	}

	// update voltage bounds
	for (size_t i = 0; i < components.size(); i++)
	{
		components[i]->updateUsedPinsVolUB();
	}

	if (Circuit::enable_reporting)
	{
		model->write("max_model.lp");
		report();
	}
	return;
}



void Circuit::checkVars(GRBModel *model)
{
	GRBVar *model_var = model->getVars();
	for (size_t i = 0; i < components.size(); i++)
	{
		for (size_t j = 0; j < components[i]->getVarsSize(); j++)
		{
			cout << (model_var++->sameAs(components[i]->vars[j])) << endl;
		}
	}
}

void Circuit::updateVars(GRBModel *model)
{
	for (size_t i = 0; i < new_components.size(); i++)
	{
		MatrixXd var_bound_mat = new_components[i]->getVarsBound();
		vector<char> var_types = new_components[i]->getVarsType();
		stringvec var_names = new_components[i]->getVarsName();
		for (size_t j = 0; j < new_components[i]->getVarsSize(); j++)
		{
			new_components[i]->vars[j] = model->addVar(var_bound_mat(j, 0),
				var_bound_mat(j, 1), 0, var_types[j], var_names[j]);
		}
		new_components[i]->constructVarMaps();
	}
}

void Circuit::updateObjs(GRBModel *model)
{
	for (size_t i = 0; i < new_components.size(); i++)
	{
		stringvec used_var_names = new_components[i]->getUsedVarsName();
		for (size_t j = 0; j < used_var_names.size(); j++)
		{
			if (isDutyCycle(used_var_names[j]))
			{
				new_components[i]->var_maps[used_var_names[j]]->
				set(GRB_DoubleAttr_Obj, -10);
			}
			else
			{
				new_components[i]->var_maps[used_var_names[j]]->
				set(GRB_DoubleAttr_Obj, 1);
			}
		}
	}
}


void Circuit::updateMinObjs(GRBModel *model)
{
	model->set(GRB_IntAttr_ModelSense, GRB_MINIMIZE);
}


/*
void Circuit::updateReplicates(vector<Electrical_Component*> &_components)
{
	for (auto &beg = _components.begin(); beg != _components.end(); beg++)
	{
		unsigned copy_pf = 0;
		beg->component_name = makeReplicate(component_names, beg->component_name, copy_pf);
		component_names.push_back(beg->component_name);
		name_maps[beg->component_name] = components.size() + (beg - _components.begin());
	}
}
*/
void Circuit::updateMaxObjs(GRBModel *model)
{
	model->set(GRB_IntAttr_ModelSense, GRB_MAXIMIZE);
}

void Circuit::updateModelCons(GRBModel *model)
{
	for (size_t i = 0; i < new_components.size(); i++)
	{
		stringvec used_var_names = new_components[i]->getUsedVarsName();
		unsignedvec used_index;;
		for (size_t j = 0; j < used_var_names.size(); j++)
		{
			const auto &iter = new_components[i]->model_index_map.find(used_var_names[j]);
			if (iter != new_components[i]->model_index_map.end())
			{
				used_index.push_back(iter->second);
			}
		}
		const vector<GRBLinExpr> &lin_exp = new_components[i]->getModelMat()*
			new_components[i]->vars;
		vector<char> model_relations = new_components[i]->getModelRelations();
		stringvec model_names = new_components[i]->getModelNames();
		for (size_t j = 0; j < used_index.size(); j++)
		{
			size_t row = used_index[j];
			if (getPosInVec(model_names[row], model_cons_names) == -1)
			{
				model_cons.push_back(model->addConstr(lin_exp[row],
					model_relations[row], 0, model_names[row]));
				model_cons_names.push_back(model_names[row]);
			}
		}
	}
}

void Circuit::updateLinCons(GRBModel *model)
{
	for (size_t i = 0; i < new_components.size(); i++)
	{
		if (new_components[i]->has_linear_constraints())
		{
			for (size_t j = 0; j < new_components[i]->linear_constraints.size(); j++)
			{
				stringvec var_names = new_components[i]->linear_constraints[j].var_names;
				MatrixXd coefficents = new_components[i]->linear_constraints[j].coefficients;
				Electronics::RELATION relation = new_components[i]->linear_constraints[j].type;
				double constant = new_components[i]->linear_constraints[j].constant;
				size_t var_size = var_names.size();
				vector<GRBVar> vars(var_size);
				for (size_t k = 0; k < var_size; k++)
				{
					vars[k] = *new_components[i]->var_maps[var_names[k]];
				}

				const vector<GRBLinExpr> &lin_cons = coefficents * vars;
				model->addConstr(lin_cons[0], sign_map[relation], constant,
					new_components[i]->getComponentName() + "LINEAR CONS");
			}
		}
	}
}

void Circuit::updateCompositions(GRBModel *model)
{
	for (auto beg = new_pin_connections.begin(); beg != new_pin_connections.end(); beg++)
	{
		stringpair left_pair = separateNames(beg->first),
			right_pair = separateNames(beg->second);
		shared_ptr<Electrical_Component> left_component = component_maps[left_pair.first],
			right_component = component_maps[right_pair.first];
		model->addConstr(*left_component->var_maps[left_pair.second] == *right_component->var_maps[right_pair.second]);
	}
}

void Circuit::restorePrevModelCons(GRBModel* model)
{
	for (size_t i = 0; i < new_components.size(); i++)
	{
		stringvec used_var_names = new_components[i]->getUsedVarsName();
		unsignedvec used_index;;
		for (size_t j = 0; j < used_var_names.size(); j++)
		{
			auto& iter = new_components[i]->model_index_map.find(used_var_names[j]);
			if (iter != new_components[i]->model_index_map.end())
			{
				used_index.push_back(iter->second);
			}
		}
		vector<GRBLinExpr>& lin_exp = new_components[i]->getModelMat() *
			new_components[i]->vars;
		vector<char> model_relations = new_components[i]->getModelRelations();
		stringvec model_names = new_components[i]->getModelNames();
		for (size_t j = 0; j < used_index.size(); j++)
		{
			size_t row = used_index[j];
			GRBConstr& cons = model->getConstrByName(model_names[row]);
			for (size_t k = 0; k < lin_exp[row].size(); k++)
			{
				model->chgCoeff(cons, lin_exp[row].getVar(k), lin_exp[row].getCoeff(k));
			}
		}
	}
}

void Circuit::verify(GRBModel *model, verifyMode mode, const string &method)
{
	addVars(model);
	mode == min ? addMinObjs(model) : addMaxObjs(model);
	addModelCons(model);
	addLinCons(model);
	addCompositions(model);
	model->update();
	model->optimize();
	if (model->get(GRB_IntAttr_Status) == GRB_OPTIMAL)
	{
		// check nonlinear components optimality, update it if not optimal
		vector<shared_ptr<Electrical_Component>> nonlin_components;
		for (size_t i = 0; i < components.size(); i++)
		{
			if (components[i]->isNoninComponent())
			{
				nonlin_components.push_back(components[i]);
			}
		}

		bool indicator = std::invoke(method_maps[method], this, nonlin_components, model);
		if (indicator)
		{
			cout << MODEL_SOLVED << endl;
		}
		else
		{
			throw MODEL_IS_INFEASIBLE;
		}
	}
	else if (model->get(GRB_IntAttr_Status) == GRB_INFEASIBLE)
	{
		model->computeIIS();
		model->write("model.ilp");
		throw MODEL_IS_INFEASIBLE;
	}

	// update voltage bounds
	for (size_t i = 0; i < components.size(); i++)
	{
		components[i]->updateUsedPinsVolLB();
	}

	if (Circuit::enable_reporting)
	{
		report();
	}
	return;
}

void Circuit::updateVerify(GRBModel *model, verifyMode mode, const std::string &method)
{
	// need modification in bnd
	updateVars(model);
	updateObjs(model);
	updateMinObjs(model);
	updateModelCons(model);
	updateLinCons(model);
	updateCompositions(model);
	model->optimize();

	if (model->get(GRB_IntAttr_Status) == GRB_OPTIMAL)
	{
		vector<shared_ptr<Electrical_Component>> nonlin_components;
		for (size_t i = 0; i < components.size(); i++)
		{
			if (components[i]->isNoninComponent())
			{
				nonlin_components.push_back(components[i]);
			}
		}

		bool indicator = std::invoke(method_maps[method], this, nonlin_components, model);
		if (indicator)
		{
			cout << MODEL_SOLVED << endl;
		}
		else
		{
			throw MODEL_IS_INFEASIBLE;
		}
	}
	else
	{
		model->computeIIS();
		model->write("model.ilp");
		throw MODEL_IS_INFEASIBLE;
	}

	for (size_t i = 0; i < components.size(); i++)
	{
		components[i]->updateUsedPinsVolLB();
	}

	for (size_t i = 0; i < components.size(); i++)
	{
		if (components[i]->component_type == Component_Type::Motor ||
			components[i]->component_type == Component_Type::Servo)
		{
			double limit_current = components[i]->
				var_maps["I"]->get(GRB_DoubleAttr_X);
			components[i]->i_bound_mat(0, 1) = limit_current;
		}
	}

	if (Circuit::enable_reporting)
	{
		report();
		model->write("min_model.lp");
	}
	return;
}

void Circuit::report() {

	for (size_t i = 0; i < components.size(); i++)
	{
		cout << endl;
		cout << components[i]->getComponentName() << endl;
		stringvec used_var_names = components[i]->getUsedVarsName();
		for (size_t j = 0; j < used_var_names.size(); j++)
		{
			cout << used_var_names[j] << ": " <<
				components[i]->var_maps[used_var_names[j]]->
				get(GRB_DoubleAttr_X) << endl;
		}
	}
}

void Circuit::updateComponents(const vector<shared_ptr<Electrical_Component>>
	&_components)
{
	components.insert(components.end(), _components.begin(), _components.end());
	new_components = _components;
	for (size_t i = 0; i < _components.size(); i++)
	{
		component_names.push_back(_components[i]->getComponentName());
		component_maps[_components[i]->getComponentName()] = _components[i];
	}
}

void Circuit::updateConnections(const Pin_Connections &_pin_connections)
{
	pin_connections.insert(_pin_connections.begin(), _pin_connections.end());
	new_pin_connections = _pin_connections;
}

void Circuit::setMotorWorkPoint2(GRBModel *model,
	const doublepairs &torqs, const doublepairs &vels)
{
	for (size_t i = 0; i < torqs.size(); i++)
	{
		double torq = torqs[i].second, vel = vels[i].first;
		unsigned torq_cons_index = components[i]->model_index_map["TORQ"],
			vel_cons_index = components[i]->model_index_map["VEL"];
		string torq_cons_name = components[i]->model_names[torq_cons_index],
			vel_cons_name = components[i]->model_names[vel_cons_index];
		GRBConstr torq_cons = model->getConstrByName(torq_cons_name),
			vel_cons = model->getConstrByName(vel_cons_name);

		model->chgCoeff(torq_cons, *components[i]->var_maps["TORQ"], -torq);
		model->chgCoeff(vel_cons, *components[i]->var_maps["VEL"], -vel);
	}
}

void Circuit::setMotorWorkPoint1(GRBModel* model,
	const doublepairs& torqs, const doublepairs& vels)
{
	for (size_t i = 0; i < torqs.size(); i++)
	{
		double torq = torqs[i].first, vel = vels[i].second;
		unsigned torq_cons_index = components[i]->model_index_map["TORQ"],
			vel_cons_index = components[i]->model_index_map["VEL"];
		string torq_cons_name = components[i]->model_names[torq_cons_index],
			vel_cons_name = components[i]->model_names[vel_cons_index];
		GRBConstr torq_cons = model->getConstrByName(torq_cons_name),
			vel_cons = model->getConstrByName(vel_cons_name);

		model->chgCoeff(torq_cons, *components[i]->var_maps["TORQ"], -torq);
		model->chgCoeff(vel_cons, *components[i]->var_maps["VEL"], -vel);
	}
}


doublevec Circuit::getVals(unsigned &comp_idx, unsigned &var_beg, unsigned &var_end)
{
	doublevec val_vec;
	for (size_t beg = var_beg; beg < var_end; beg++)
	{
		val_vec.push_back(components[comp_idx]->vars[beg].get(GRB_DoubleAttr_X));
	}
	return val_vec;
}

doublevec Circuit::getMaxVals(unsigned &comp_idx, unsigned &var_beg, unsigned &var_end)
{
	doublevec val_vec;
	for (size_t beg = var_beg; beg < var_end; beg++)
	{
		val_vec.push_back(components[comp_idx]->vars[beg].get(GRB_DoubleAttr_UB));
	}
	return val_vec;
}

void Circuit::syncVars(const GRBModel &model)
{
	GRBVar *vars = model.getVars();
	for (size_t i = 0; i < components.size(); i++)
	{
		for (auto vbeg = components[i]->vars.begin(); vbeg != components[i]->vars.end(); vbeg++)
		{
			*vbeg = *vars++;
		}
	}
}

unsigned Circuit::getMotorNumber()
{
	unsigned cnt = 0;
	for (size_t i = 0; i < components.size(); i++)
	{
		if (components[i]->getComponentType() == Component_Type::Motor)
		{
			cnt++;
		}
	}
	return cnt;
}

bool Circuit::newton_raphson(vector<shared_ptr<Electrical_Component>> nonlin_components, GRBModel *model)
{
	bool indicator = false;
	boolvec indicator_vec(nonlin_components.size());
	unsigned cnt = 0;
	while (cnt++ < 1e3)
	{
		for (size_t i = 0; i < nonlin_components.size(); i++)
		{
			indicator_vec[i] = nonlin_components[i]->thresholdCheck();
		}

		if (std::find(indicator_vec.begin(), indicator_vec.end(), false) == indicator_vec.end())
		{
			indicator = true;
			break;
		}

		for (size_t i = 0; i < nonlin_components.size(); i++)
		{
			if (!indicator_vec[i])
			{
				nonlin_components[i]->updateCoefficients(model, model_cons);
			}
		}
		model->update();
		model->optimize();

		if (model->get(GRB_IntAttr_Status) == GRB_INFEASIBLE)
		{
			throw MODEL_IS_INFEASIBLE;
		}
	}
	return indicator;
}
/*
bool Circuit::expect_maximize(const std::vector<std::vector<GRBVar*>> &nonlin_vars_vec, std::vector<bool> &indicator_vec, GRBModel *model)
{
	// extract boundary conditions
	doublepairs2d var_bounds_vec(nonlin_vars_vec.size());
	auto &vbvbeg = var_bounds_vec.begin();
	for (auto &nvvbeg = nonlin_vars_vec.begin(); nvvbeg != nonlin_vars_vec.end(); nveg++, vbvbeg++)
	{
		vbvbeg->resize(nvvbeg->size());
		auto &vbbeg = vbvbeg->begin();
		for (auto &nvbeg = nvvbeg->begin(); nvbeg != nvvbeg->end(); nvbeg++)
		{
			vbbeg->first = (*nvbeg)->get(GRB_DoubleAttr_LB);
			vbbeg->first = (*nvbeg)->get(GRB_DoubleAttr_UB);
		}
	}
	return false;
}

bool Circuit::stochastic_optimize(const std::vector<std::vector<GRBVar*>> &nonlin_vars_vec, std::vector<bool> &indicator_vec, GRBModel *model)
{
	return false;
}
*/

/*
void Circuit::getComponentPins(str_strvec_uomap &componentpins, const std::string &component, const std::string &pin)
{
	auto &riter = refer_componentpins.find(component);
	if ((riter == refer_componentpins.end() ||
		std::find(riter->second.begin(), riter->second.end(), pin) == riter->second.end()) &&
		pin.substr(0, 4) != duty)
	{
		auto &iter = componentpins.find(component);
		if (iter == componentpins.end())
		{
			componentpins.insert(std::make_pair(component, stringvec{ pin }));
		}
		else if (std::find(iter->second.begin(), iter->second.end(), pin) == iter->second.end())
		{
			iter->second.push_back(pin);
		}
	}
}
*/
std::vector<GRBLinExpr> operator*(const Eigen::MatrixXd &mat, const std::vector<GRBVar> &vars)
{
	vector<GRBLinExpr> model_cons(mat.rows());
	doublevec vec(mat.cols());

	auto con = model_cons.begin();
	for (unsigned i = 0; i < mat.rows(); i++)
	{
		GRBLinExpr temp_exp(0);
		auto vbeg = vec.begin();
		for (unsigned j = 0; j < mat.cols(); j++)
		{
			*vbeg++ = mat(i, j);
		}

		temp_exp.addTerms(&vec[0], &vars[0], static_cast<int>(vars.size()));
		*con++ = temp_exp;
	}
	return model_cons;
}

bool isDutyCycle(const string &pin_name)
{
	string duty_cycle = "DUTY";
	return pin_name.substr(0, duty_cycle.size()) == duty_cycle;
}

