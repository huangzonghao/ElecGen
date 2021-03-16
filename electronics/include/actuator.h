#ifndef ACTUATOR_H_OVPFYACD
#define ACTUATOR_H_OVPFYACD

#include <gurobi_c++.h>
#include <Eigen/Dense>
#include <fcntl.h>
#include <google/protobuf/text_format.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include "utility_functions.h"
#include "component_structure.pb.h"

class Circuit;

namespace Component_Class {
	const std::string Actuator = "ACTUATOR",
		Sensor = "SENSOR",
		Other = "SENSOR";
}

namespace Component_Type {
	const std::string Motor = "MOTOR",
		H_Bridge = "H_BRIDGE",
		Micro_Controller = "MICRO_CONTROLLER",
		Voltage_Regulator = "VOLTAGE_REGULATOR",
		Battery = "BATTERY",
		Encoder = "ENCODER",
		Bluetooth = "BLUETOOTH",
		Camera = "CAMERA",
		Force_Sensor = "FORCE_SENSOR",
		Servo = "SERVO",
		Power_Supply = "POWER_SUPPLY", // battery + voltage regulator
		None = "";
}

struct Lin_Cons
{
	stringvec var_names;
	Eigen::MatrixXd coefficients;
	double constant;
	Electronics::RELATION type;

	Lin_Cons(const stringvec &_names, const Eigen::MatrixXd &_coefficients,
		const double &_constant, const Electronics::RELATION _type) : var_names(_names),
		coefficients(_coefficients), constant(_constant), type(_type)
	{}
};
Lin_Cons lincons2LinCons(const Electronics::lin_cons &);

struct Pin
{
	std::string name;
	Electronics::CLASS pin_class;
	Electronics::FUNCTION_TYPE func_type;
	Electronics::PHYSICAL_TYPE phys_type;
	Electronics::IO io;
	Electronics::CONNECTION connection;
	bool status = false;
	doublepair v_bound;
	doublepair i_bound;
	stringvec dependents;

	bool operator==(const Pin &pin) const { return this->name == pin.name; }
	bool operator==(Pin *pin) { return this->name == pin->name; }
};
Pin pin2Pin(const Electronics::pin &);

class Electrical_Component
{
public:
	friend Circuit;
	Electrical_Component() = default;
	Electrical_Component(const std::string &);
	virtual ~Electrical_Component() = default;

	void setComponentName(const std::string &name);
	std::string getComponentName() { return component_name; }
	std::string getComponentType() { return component_type; }
	std::string getComponentClass() { return component_class; }
	double getPrice() { return component_price; }
	double getWeight() { return component_weight; }
	unsigned getPinNum() { return pins.size(); }
	size_t getVarsSize() { return vars.size(); }
	Eigen::MatrixXd getVarsBound() { return var_bound_mat; }
	Eigen::MatrixXd getModelMat() { return model_mat; }
	std::vector<char> getVarsType() { return var_types; }
	std::vector<char> getModelRelations() { return model_relations; }
	stringvec getModelNames() { return model_names; }
 	stringvec getVarsName() { return var_names; }
	stringvec getUsedVarsName() { return used_var_names; }
	bool isNoninComponent() { return nonlin; }
	void updateUsedPinsVolLB();
	void updateUsedPinsVolUB();
	void restorePinVolBound(const std::string &);
	bool has_linear_constraints() { return !linear_constraints.empty(); }

	doublepair getOutVolRange(const Electronics::CLASS &);
//	doublepair getInVolRange(const Electronics::CLASS &);
	double getOutCurrentLimit(const Electronics::CLASS &);
//	double getInCurrentLimit(const Electronics::CLASS &);

	unsigned getPowerInSize()
	{ return static_cast<unsigned>(getPowerInPins().size() +
		getBothInPins().size() + getBothBidirectPins().size()); }
	virtual unsigned getFuncInSize() = 0;
	unsigned getPowerOutSize()
	{ return static_cast<unsigned>(getPowerOutPins().size() +
		getBothOutPins().size() + getBothBidirectPins().size()); }
	virtual unsigned getFuncOutSize() = 0;
	void constructVarMaps();

	std::vector<Pin*> getPowerInPins();
	std::vector<Pin*> getPowerOutPins();
	std::vector<Pin*> getFuncInPins();
	std::vector<Pin*> getFuncOutPins();
	std::vector<Pin*> getFuncBidirectPins();
	std::vector<Pin*> getBothInPins();
	std::vector<Pin*> getBothOutPins();
	std::vector<Pin*> getBothBidirectPins();
	std::vector<Pin> getUsablePins() { return usable_pins; }
	std::vector<Pin*> getDependentPins(const std::string &);
	std::vector<Pin*> getDependentPins() { return dependent_pins; }
	std::vector<Pin> getAllPins() { return pins; }
	void clearConnections(const stringvec &);

	void setName(const std::string &_new_name) { component_name = _new_name; }

//	friend std::vector<Actu_components>& replicate(std::vector<Actu_components> &);

	virtual void parameters() const = 0;
	virtual stringvec getNonlinVarNames() = 0;
	virtual stringvec getUsedNonLinVarNames(const stringvec &) = 0;
	virtual bool thresholdCheck() = 0;
	virtual void updateCoefficients(GRBModel *, std::vector<GRBConstr> &) = 0;
	virtual doublevec getFuncInCurrentLimit() = 0;
	virtual doublevec getPowerInCurrentLimit() = 0;
	virtual double getFuncOutCurrentLimit() = 0;
	virtual double getPowerOutCurrentLimit() = 0;
	virtual doublepairs getFuncInVolRange() = 0;
	virtual doublepairs getPowerInVolRange() = 0;
	virtual doublepair getFuncOutVolRange() = 0;
	virtual doublepair getPowerOutVolRange() = 0;
	virtual stringvec getPowerInPinNames() = 0;
	virtual stringvec getPowerOutPinNames() = 0;
	virtual stringvec getFuncInPinNames() = 0;
	virtual stringvec getFuncOutPinNames() = 0;
	virtual unsigned getMainPowerIndex() = 0;
	virtual void getUsedVarsName(const stringvec &) = 0;

protected:
	std::string component_name;
	std::string component_type;
	std::string component_class;
	std::vector<Pin> pins;
	std::vector<Pin> usable_pins;
	std::vector<Pin*> dependent_pins;
	std::vector<GRBVar> vars;
	stringvec var_names;
	stringvec used_var_names;
	std::unordered_map<std::string, GRBVar*> var_maps;
	stringvec nonlin_var_names;
	stringvec used_nonlin_var_names;
	unsignedvec used_nonlin_var_index;
	std::vector<char> var_types;
	Eigen::MatrixXd v_bound_mat;
	Eigen::MatrixXd i_bound_mat;
	Eigen::MatrixXd var_bound_mat;
	Eigen::MatrixXd model_mat;
	std::vector<char> model_relations;
	stringvec model_names;
	std::unordered_map<std::string, unsigned> model_index_map;
	std::vector<Lin_Cons> lin_cons;
	std::vector<Lin_Cons> linear_constraints;
	bool nonlin;
	doublevec component_size = doublevec(3);
	double component_weight = 0;
	double component_price = 0;
	unsigned linear_constraint_size = 0;

	Electronics::Component* read(const std::string &);
	void constructUsablePins() { usable_pins = pins; }

private:
	void extractInfo(const Electronics::Component *);
	std::vector<Pin*> getPins(Electronics::CLASS, Electronics::IO);
};

class Motor: public Electrical_Component
{
public:
	Motor() = default;
	Motor(const std::string &, double = 0.0, double = 0.0);

	double getMaxTorq() { return torq_ub; }
	double getMaxVel() { return vel_ub; }
	double getVel(double torq) { return -r * torq / (ke * kt) + v / ke; }
//	double getTorq(double vel) { return (-ke * kt * vel + kt * v) / r; }

	void parameters() const override;
	stringvec getNonlinVarNames() override { return stringvec(); }
	stringvec getUsedNonLinVarNames(const stringvec &used_pins) override { return stringvec(); }
	bool thresholdCheck() override { return false; }
	void updateCoefficients(GRBModel *, std::vector<GRBConstr> &) override { return;  }
	doublevec getFuncInCurrentLimit() override;
	doublevec getPowerInCurrentLimit() override;
	double getFuncOutCurrentLimit() override { return 0.0; }
	double getPowerOutCurrentLimit() override { return 0.0; }
	doublepairs getFuncInVolRange() override;
	doublepairs getPowerInVolRange() override;
	doublepair getFuncOutVolRange() override { return doublepair(); }
	doublepair getPowerOutVolRange() override { return doublepair(); }
	stringvec getPowerInPinNames() override;
	stringvec getPowerOutPinNames() override { return stringvec(); }
	stringvec getFuncInPinNames() override;
	stringvec getFuncOutPinNames() override { return stringvec(); }
	void getUsedVarsName(const stringvec &pin_names) override {
		if (used_var_names.empty())
		{
			used_var_names = var_names;
			for (size_t i = 0; i < pins.size(); i++)
			{
				dependent_pins.push_back(&pins[i]);
			}
		}
	}
	unsigned getMainPowerIndex() override { return 0; }
	unsigned getFuncInSize() override;
	unsigned getFuncOutSize() override { return 0; }


	void setWorkPoint(double, double);
private:
	void extractInfo(const Electronics::Component *);
	bool isServo() { return pins.size() == 3; }
	double v;
	double i;
	double kt;
	double ke;
	double r;
	double torq_ub;
	double vel_ub;
	double torq_des;
	double vel_des;
	double ang_vel;
	double torque;
};
void printPinInfo(const Pin &);
void initializeAllMotors(const std::string &);
void initializeAllServos(const std::string &);


class Voltage_Regulator: public Electrical_Component
{
public:
	Voltage_Regulator() = default;
	Voltage_Regulator(const std::string &);

	void parameters() const override;
	stringvec getNonlinVarNames() override { return stringvec(); }
	stringvec getUsedNonLinVarNames(const stringvec &) override { return stringvec(); }
	bool thresholdCheck() override { return true; }
	void updateCoefficients(GRBModel *, std::vector<GRBConstr> &) override {}
	doublevec getFuncInCurrentLimit() override { return doublevec(); }
	doublevec getPowerInCurrentLimit() override {
		return doublevec{ i_bound_mat(0, 1) };
	}
	double getFuncOutCurrentLimit() override { return 0.0; }
	double getPowerOutCurrentLimit() override {
		return i_bound_mat(pow_in_pins.size(), 1); }
	doublepairs getFuncInVolRange() override { return doublepairs(); }
	doublepairs getPowerInVolRange() override {
		return doublepairs{ std::make_pair(pins[0].v_bound.first,
			pins[0].v_bound.second) };
	}
	doublepair getFuncOutVolRange() override { return doublepair(); }
	doublepair getPowerOutVolRange() override {
		size_t pos = pow_in_pins.size();
		return std::make_pair(pins[pos].v_bound.first,
				pins[pos].v_bound.second);
	}
	stringvec getPowerInPinNames() override;
	stringvec getPowerOutPinNames() override;
	stringvec getFuncInPinNames() override { return stringvec(); }
	stringvec getFuncOutPinNames() override { return stringvec(); }
	void getUsedVarsName(const stringvec &) override;
	unsigned getMainPowerIndex() override { return 0; }
	unsigned getFuncInSize() override { return 0; }
	unsigned getFuncOutSize() override { return 0; }

private:
	void extractInfo(Electronics::Component *);
	void getPinNumberInfo();

	// pins
	std::vector<Pin> pow_in_pins;
	std::vector<Pin> pow_out_pins;
	std::vector<Pin> gnd_pins;
};
void initializeAllVoltageRegulators(const std::string &);

class H_Bridge: public Electrical_Component
{
public:
	H_Bridge() = default;
	H_Bridge(const std::string &);

	void parameters() const override;
	stringvec getNonlinVarNames() override;
	stringvec getUsedNonLinVarNames(const stringvec &) override;
	bool thresholdCheck() override;
	void updateCoefficients(GRBModel *, std::vector<GRBConstr> &) override;
	doublevec getFuncInCurrentLimit() override;
	doublevec getPowerInCurrentLimit() override;
	double getFuncOutCurrentLimit() override;
	double getPowerOutCurrentLimit() override { return 0.0; }
	doublepairs getFuncInVolRange() override;
	doublepairs getPowerInVolRange() override;
	doublepair getFuncOutVolRange() override;
	stringvec getPowerInPinNames() override;
	stringvec getPowerOutPinNames() override;
	stringvec getFuncInPinNames() override;
	stringvec getFuncOutPinNames() override { return getPowerOutPinNames(); }
	doublepair getPowerOutVolRange() override { return doublepair(); }
	void getUsedVarsName(const stringvec &) override;
	unsigned getMainPowerIndex() override;
	unsigned getFuncInSize() override;
	unsigned getFuncOutSize() override { return out_pin_num; }

	std::unordered_map<std::string, std::string> getInDutyCycleMap() { return in_duty_cycle_map; }

private:
	void extractInfor(const Electronics::Component *);
	void constructDutyCycleMap();
	unsignedvec getPinNumberInfo(const std::vector<Pin> &);

	std::unordered_map<std::string, std::string> in_duty_cycle_map;
	std::unordered_map<std::string, std::string> out_duty_cycle_map;
	// pins
	doublepair logic_level;
	unsigned log_pin_num;
	unsigned mot_pin_num;
	unsigned ena_pin_num;
	unsigned in_pin_num;
	unsigned out_pin_num;
	unsigned gnd_pin_num;
	unsigned other_pin_num;
	unsigned total_pin_num;
};
void initializeAllHBridges(const std::string &);


class Micro_Controller : public Electrical_Component
{
public:
	Micro_Controller() = default;
	Micro_Controller(const std::string &);

	void parameters() const override;
	stringvec getNonlinVarNames() override { return stringvec(); }
	stringvec getUsedNonLinVarNames(const stringvec &) override { return stringvec(); }
	bool thresholdCheck() override { return true; }
	void updateCoefficients(GRBModel *, std::vector<GRBConstr> &) override {}
	doublevec getFuncInCurrentLimit() override;
	doublevec getPowerInCurrentLimit() override;
	double getFuncOutCurrentLimit() override;
	double getPowerOutCurrentLimit() override;
	doublepairs getFuncInVolRange() override { return doublepairs(); }
	doublepairs getPowerInVolRange() override;
	doublepair getFuncOutVolRange() override;
	stringvec getPowerInPinNames() override;
	stringvec getPowerOutPinNames() override { return stringvec(); }
	stringvec getFuncInPinNames() override;
	stringvec getFuncOutPinNames() override;
	doublepair getPowerOutVolRange() override;
	void getUsedVarsName(const stringvec &) override;
	unsigned getMainPowerIndex() override { return 0; }
	unsigned getFuncInSize() override { return 0; }
	unsigned getFuncOutSize() override { return digital_pins.size() + analog_pins.size(); }

	std::unordered_map<std::string, std::string> getOutDutyCycleMap() { return out_duty_cycle_map; }

private:
	void extractInfo(Electronics::Component *);
	void constructDutyCycleMap();
	void getPinNumberInfo();

	// pins
	std::vector<Pin> pow_in_pins;
	std::vector<Pin> pow_out_pins;
	std::vector<Pin> gnd_pins;
	std::vector<Pin> digital_pins;
	std::vector<Pin> pwm_pins;
	std::vector<Pin> analog_pins;
	std::vector<Pin> spi_pins;
	std::vector<Pin> uart_pins;
	std::vector<Pin> i2c_pins;
	std::vector<Pin> external_interrupt_pins;
	std::vector<Pin> other_pins;
	std::vector<Pin> digital_only_pins;
	std::vector<Pin> pwm_only_pins;
	std::vector<Pin> i2c_only_pins;

	doublepair logic_level;
	std::unordered_map<std::string, std::string> out_duty_cycle_map;
};
void initializeAllMicroController(const std::string &);

class Battery: public Electrical_Component
{
public:
	Battery() = default;
	Battery(const std::string &);

	void parameters() const override;
	stringvec getNonlinVarNames() override { return stringvec(); }
	stringvec getUsedNonLinVarNames(const stringvec &) override { return stringvec(); }
	bool thresholdCheck() override { return true; }
	void updateCoefficients(GRBModel *, std::vector<GRBConstr> &) override {}
	doublevec getFuncInCurrentLimit() override { return doublevec(); }
	doublevec getPowerInCurrentLimit() override { return doublevec(1); }
	double getFuncOutCurrentLimit() override { return 0.0; }
	double getPowerOutCurrentLimit() override { return i_bound_mat(0, 1); }
	doublepairs getFuncInVolRange() override { return doublepairs(); }
	doublepairs getPowerInVolRange() override { return doublepairs(); }
	doublepair getFuncOutVolRange() override { return doublepair(); }
	doublepair getPowerOutVolRange() override { return
		std::make_pair(pins[0].v_bound.first, pins[0].v_bound.second); }
	stringvec getPowerInPinNames() override { return stringvec(); }
	stringvec getPowerOutPinNames() override;
	stringvec getFuncInPinNames() override { return stringvec(); }
	stringvec getFuncOutPinNames() override { return stringvec(); }
	void getUsedVarsName(const stringvec &) override;
	unsigned getMainPowerIndex() override { return 0; }
	unsigned getFuncInSize() override { return 0; }
	unsigned getFuncOutSize() override { return 0; }

private:
	void extractInfo(Electronics::Component *);
	double capacity;
};
void initializeAllBatteries(const std::string &);


class Encoder: public Electrical_Component
{
public:
	Encoder() = default;
	Encoder(const std::string &);

	void parameters() const override;
	stringvec getNonlinVarNames() override { return stringvec(); }
	stringvec getUsedNonLinVarNames(const stringvec &) override { return stringvec(); }
	bool thresholdCheck() override { return true; }
	void updateCoefficients(GRBModel *, std::vector<GRBConstr> &) override {}
	doublevec getFuncInCurrentLimit() override;
	doublevec getPowerInCurrentLimit() override;
	double getFuncOutCurrentLimit() override { return 0.0; }
	double getPowerOutCurrentLimit() override { return 0.0; }
	doublepairs getFuncInVolRange() override;
	doublepairs getPowerInVolRange() override;
	doublepair getFuncOutVolRange() override { return doublepair(); }
	doublepair getPowerOutVolRange() override { return doublepair(); }
	stringvec getPowerInPinNames() override;
	stringvec getPowerOutPinNames() override { return stringvec(); }
	stringvec getFuncInPinNames() override;
	stringvec getFuncOutPinNames() override { return stringvec(); }
	void getUsedVarsName(const stringvec &) override;
	unsigned getMainPowerIndex() override { return 0; }
	unsigned getFuncInSize() override { return signal_pins.size(); }
	unsigned getFuncOutSize() override { return 0; }

private:
	void extractInfo(Electronics::Component *);
	void getPinNumberInfo();

	// pins
	std::vector<Pin> pow_in_pins;
	std::vector<Pin> gnd_pins;
	std::vector<Pin> signal_pins;
	std::vector<Pin> other_pins;

	double frequency;
};
void initializeAllEncoders(const std::string &);

class Camera: public Electrical_Component
{
public:
	Camera() = default;
	Camera(const std::string &);

	void parameters() const override;
	stringvec getNonlinVarNames() override { return stringvec(); }
	stringvec getUsedNonLinVarNames(const stringvec &) override { return stringvec(); }
	bool thresholdCheck() override { return true; }
	void updateCoefficients(GRBModel *, std::vector<GRBConstr> &) override {}
	doublevec getFuncInCurrentLimit() override;
	doublevec getPowerInCurrentLimit() override;
	double getFuncOutCurrentLimit() override { return 0.0; }
	double getPowerOutCurrentLimit() override { return 0.0; }
	doublepairs getFuncInVolRange() override;
	doublepairs getPowerInVolRange() override;
	doublepair getFuncOutVolRange() override { return doublepair(); }
	doublepair getPowerOutVolRange() override { return doublepair(); }
	stringvec getPowerInPinNames() override;
	stringvec getPowerOutPinNames() override { return stringvec(); }
	stringvec getFuncInPinNames() override;
	stringvec getFuncOutPinNames() override { return stringvec(); }
	void getUsedVarsName(const stringvec &) override;
	unsigned getMainPowerIndex() override { return 0; }
	unsigned getFuncInSize() override { return uart_pins.size(); }
	unsigned getFuncOutSize() override { return 0; }

private:
	void extractInfo(Electronics::Component *);
	void getPinNumberInfo();

	// pins
	std::vector<Pin> pow_in_pins;
	std::vector<Pin> gnd_pins;
	std::vector<Pin> uart_pins;
	std::vector<Pin> i2c_pins;
	std::vector<Pin> other_pins;

	double frequency;
};
void initializeAllCameras(const std::string &);


class Bluetooth: public Electrical_Component
{
public:
	Bluetooth() = default;
	Bluetooth(const std::string &);

	void parameters() const override;
	stringvec getNonlinVarNames() override { return stringvec(); }
	stringvec getUsedNonLinVarNames(const stringvec &) override { return stringvec(); }
	bool thresholdCheck() override { return true; }
	void updateCoefficients(GRBModel *, std::vector<GRBConstr> &) override {}
	doublevec getFuncInCurrentLimit() override;
	doublevec getPowerInCurrentLimit() override;
	double getFuncOutCurrentLimit() override { return 0.0; }
	double getPowerOutCurrentLimit() override { return 0.0; }
	doublepairs getFuncInVolRange() override;
	doublepairs getPowerInVolRange() override;
	doublepair getFuncOutVolRange() override { return doublepair(); }
	doublepair getPowerOutVolRange() override { return doublepair(); }
	stringvec getPowerInPinNames() override;
	stringvec getPowerOutPinNames() override { return stringvec(); }
	stringvec getFuncInPinNames() override;
	stringvec getFuncOutPinNames() override { return stringvec(); }
	void getUsedVarsName(const stringvec &) override;
	unsigned getMainPowerIndex() override { return 0; }
	unsigned getFuncInSize() override { return uart_pins.size() + spi_pins.size(); }
	unsigned getFuncOutSize() override { return 0; }

private:
	void extractInfo(Electronics::Component *);
	void getPinNumberInfo();

	// pins
	std::vector<Pin> pow_in_pins;
	std::vector<Pin> gnd_pins;
	std::vector<Pin> uart_pins;
	std::vector<Pin> spi_pins;
	std::vector<Pin> interrupt_pins;
	std::vector<Pin> digital_pins;
	std::vector<Pin> other_pins;

	double frequency;
};
void initializeAllBluetooths(const std::string &);


class Force_Sensor: public Electrical_Component
{
public:
	Force_Sensor() = default;
	Force_Sensor(const std::string &);

	void parameters() const override;
	stringvec getNonlinVarNames() override { return stringvec(); }
	stringvec getUsedNonLinVarNames(const stringvec &) override { return stringvec(); }
	bool thresholdCheck() override { return true; }
	void updateCoefficients(GRBModel *, std::vector<GRBConstr> &) override {}
	doublevec getFuncInCurrentLimit() override;
	doublevec getPowerInCurrentLimit() override;
	double getFuncOutCurrentLimit() override { return 0.0; }
	double getPowerOutCurrentLimit() override { return  0.0; }
	doublepairs getFuncInVolRange() override;
	doublepairs getPowerInVolRange() override;
	doublepair getFuncOutVolRange() override { return doublepair(); }
	doublepair getPowerOutVolRange() override { return doublepair(); }
	stringvec getPowerInPinNames() override;
	stringvec getPowerOutPinNames() override { return stringvec(); }
	stringvec getFuncInPinNames() override;
	stringvec getFuncOutPinNames() override { return stringvec(); }
	void getUsedVarsName(const stringvec &) override;
	unsigned getMainPowerIndex() override { return 0; }
	unsigned getFuncInSize() override { return 1; }
	unsigned getFuncOutSize() override { return 0; }

private:
	void extractInfo(Electronics::Component *);
};
void initializeAllForceSensors(const std::string &);


// auxiliary functions

// template<typename T>
// inline int sgn(T val)
// {
// 	return (T(0) < val) - (T(0) > val);
// }

// std::vector<Actu_components>& replicate(std::vector<Actu_components> &);

#endif /* end of include guard: ACTUATOR_H_OVPFYACD */
