#include "stdafx.h"
#include "actuator.h"

using std::vector;
using std::string;
using std::cin;
using std::cout;
using std::cerr;
using std::ends;
using std::endl;
using Eigen::MatrixXd;
using std::filesystem::directory_iterator;
using google::protobuf::io::FileInputStream;
using google::protobuf::io::ZeroCopyInputStream;
using google::protobuf::TextFormat;
using std::make_pair;
using std::unordered_map;
using std::to_string;
using std::shared_ptr;
using std::make_shared;

unordered_map<string, Motor> MOTOR_MAP, SERVO_MAP;
unordered_map<string, H_Bridge> H_BRIDGE_MAP;
unordered_map<string, Micro_Controller> MICRO_CONTROLLER_MAP;
unordered_map<string, Voltage_Regulator> VOLTAGE_REGULATOR_MAP;
unordered_map<string, Battery> BATTERY_MAP;
unordered_map<string, Encoder> ENCODER_MAP;
unordered_map<string, Camera> CAMERA_MAP;
unordered_map<string, Bluetooth> BLUETOOTH_MAP;
unordered_map<string, Force_Sensor> FORCE_SENSOR_MAP;

unordered_map<string, shared_ptr<Electrical_Component>> MOTOR_PART_MAP,
H_BRIDDE_PART_MAP, MICRO_CONTROLLER_PART_MAP, VOLTAGE_REGULATOR_PART_MAP,
BATTERY_PART_MAP, ENCODER_PART_MAP, CAMERA_PART_MAP, FORCE_SENSOR_PART_MAP, 
BLUETOOTH_PART_MAP, SERVO_PART_MAP;

Electrical_Component::Electrical_Component(const string &file)
{
	Electronics::Component *base_component = read(file);
	extractInfo(base_component);
	delete base_component;
}

void Electrical_Component::setComponentName(const std::string & name)
{
	component_name = name;
	for (size_t i = 0; i < model_names.size(); i++)
	{
		model_names[i] = component_name + model_names[i];
	}
}

Voltage_Regulator::Voltage_Regulator(const string& file) : Electrical_Component(file)
{
	if (!VOLTAGE_REGULATOR_MAP.empty())
	{
		*this = VOLTAGE_REGULATOR_MAP[file];
	}
	else
	{
		Electronics::Component *voltage_regulator = read(file);
		extractInfo(voltage_regulator);
		delete voltage_regulator;
	}
}

void Voltage_Regulator::parameters() const
{
	cout << component_name << " PARAMETERS: " << endl;
	for (size_t i = 0; i < linear_constraints.size(); i++)
	{
		for (size_t j = 0; j < linear_constraints[i].var_names.size(); j++)
		{
			cout << "VAR: " << linear_constraints[i].var_names[j] <<
				" COEFF: " << linear_constraints[i].coefficients(0, j) << endl;
		}
		cout << "CONST: " << linear_constraints[i].constant << endl;
		cout << "RELATION: " << linear_constraints[i].type << endl;
		cout << endl;
	}
	cout << endl;

	for (size_t i = 0; i < pins.size(); i++)
	{
		printPinInfo(pins[i]);
	}

	cout << endl;
	cout << model_mat << endl;
}

Micro_Controller::Micro_Controller(const std::string &file) : Electrical_Component(file)
{
	if (!MICRO_CONTROLLER_MAP.empty())
	{
		*this = MICRO_CONTROLLER_MAP[file];
	}
	else
	{
		Electronics::Component *micro_controller = read(file);
		extractInfo(micro_controller);
		delete micro_controller;
	}
}

void Micro_Controller::parameters() const
{
	cout << component_name << " PARAMETERS: " << endl;
	cout << "LOGIC LEVEL [" << logic_level.first << ", "  << 
		logic_level.second << "]" << endl;
	cout << endl;

	for (size_t i = 0; i < pins.size(); i++)
	{
		printPinInfo(pins[i]);
	}

	cout << endl;
	cout << model_mat << endl;
}

doublevec Micro_Controller::getFuncInCurrentLimit()
{
	size_t digital_pos = pow_in_pins.size() + pow_out_pins.size() + 
		gnd_pins.size();
	return doublevec{ i_bound_mat(digital_pos, 1) };
}

doublevec Micro_Controller::getPowerInCurrentLimit()
{
	unsigned dig_pin_cnt = 0;
	for (size_t i = 0; i < dependent_pins.size(); i++)
	{
		size_t pos = getPosInVec(*dependent_pins[i], digital_pins);
		if (pos != -1)
		{
			dig_pin_cnt++;
		}
	}

	return doublevec{ dig_pin_cnt * digital_pins[0].i_bound.second/2000 };
}

double Micro_Controller::getFuncOutCurrentLimit()
{
	size_t digital_pos = pow_in_pins.size() + pow_out_pins.size() +
		gnd_pins.size();
	return i_bound_mat(digital_pos, 1);
}

double Micro_Controller::getPowerOutCurrentLimit()
{
	if (pow_out_pins.empty())
	{
		return 0.0;
	}
	else
	{
		size_t pow_out_pos = pow_in_pins.size();
		return i_bound_mat(pow_out_pos, 1);
	}
}

doublepairs Micro_Controller::getPowerInVolRange()
{
	return doublepairs{ make_pair(pins[0].v_bound.first, 
		pins[0].v_bound.second) };
}

doublepair Micro_Controller::getFuncOutVolRange()
{
	size_t digtal_pos = pow_in_pins.size() + pow_out_pins.size() + 
		gnd_pins.size();
	return make_pair(pins[digtal_pos].v_bound.first, 
		pins[digtal_pos].v_bound.second);
}

stringvec Micro_Controller::getPowerInPinNames()
{
	return stringvec{ pins[0].name };
}

stringvec Micro_Controller::getFuncInPinNames()
{
	stringvec func_in_pin_names(digital_pins.size() + analog_pins.size());
	for (size_t i = 0; i < func_in_pin_names.size(); i++)
	{
		if (i < digital_pins.size())
		{
			func_in_pin_names[i] = digital_pins[i].name;
		}
		else
		{
			func_in_pin_names[i] = analog_pins[i - digital_pins.size()].name;
		}
	}
	return func_in_pin_names;
}

stringvec Micro_Controller::getFuncOutPinNames()
{
	stringvec func_in_pin_names(digital_pins.size());
	for (size_t i = 0; i < func_in_pin_names.size(); i++)
	{

		func_in_pin_names[i] = digital_pins[i].name;
	}
	return func_in_pin_names;
}

doublepair Micro_Controller::getPowerOutVolRange()
{
	if (pow_out_pins.empty())
	{
		return doublepair();
	}
	else
	{
		size_t pow_out_size = pow_in_pins.size();
		return make_pair(pins[pow_out_size].v_bound.first, 
			pins[pow_out_size].v_bound.second);
	}
}

void Micro_Controller::getUsedVarsName(const stringvec &pin_names)
{
	for (size_t i = 0; i < pin_names.size(); i++)
	{
		getDependentPins(pin_names[i]);
	}

	for (size_t i = 0; i < dependent_pins.size(); i++)
	{
		if (getPosInVec(dependent_pins[i]->name, used_var_names) == -1)
		{
			used_var_names.push_back(dependent_pins[i]->name);
		}
	}

	size_t used_vars_size = used_var_names.size();
	for (size_t i = 0; i < used_vars_size; i++)
	{
		auto &iter = out_duty_cycle_map.find(used_var_names[i]);
		if (iter != out_duty_cycle_map.end())
		{
			if (getPosInVec(iter->second, used_var_names) == -1)
			{
				used_var_names.push_back(iter->second);
			}
		}
	}
}

void Micro_Controller::extractInfo(Electronics::Component *micro_controller)
{
	Electronics::Micro_Controller ext = micro_controller->GetExtension(
		Electronics::Micro_Controller::micro_controller);
	logic_level.first = ext.logic_level().lb();
	logic_level.second = ext.logic_level().ub();


	getPinNumberInfo();
	unsigned pow_in_pin_num = pow_in_pins.size(),
		pow_out_pin_num = pow_out_pins.size(),
		gnd_pin_num = gnd_pins.size(),
		dig_pin_num = digital_pins.size(),
		ana_pin_num = analog_pins.size(),
		other_pin_num = other_pins.size(),
		total_pin_num = pins.size();

	// config part
	unsigned extra_var_size = dig_pin_num + 1,
		duty_cycle_col = static_cast<unsigned> (vars.size());
	var_bound_mat = MatrixXd::Zero(vars.size() + extra_var_size, 2);
	for (size_t i = 0; i < extra_var_size; i++)
	{
		vars.push_back(GRBVar());
		var_types.push_back(GRB_CONTINUOUS);
		if (i == extra_var_size - 1)
		{
			var_names.push_back("CONST_MICRO_CONTROLLER");
			var_bound_mat.row(i + duty_cycle_col) << 1, 1;
		}
		else
		{
			var_names.push_back("DUTY_CYCYLE" + std::to_string(i));
			var_bound_mat.row(i + duty_cycle_col) << 0, 1;
		}
	}
	constructDutyCycleMap();

	// formulating boundary matrix
	var_bound_mat.block(0, 0, v_bound_mat.rows(), v_bound_mat.cols()) =
		v_bound_mat;

	// coefficient matrix
	unsigned var_size = static_cast<unsigned>(vars.size());
	model_mat = MatrixXd::Zero(pins.size(), vars.size());
	unsigned range1 = pow_in_pin_num + pow_out_pin_num + gnd_pin_num,
		range2 = range1 + dig_pin_num, range3 = range2 + ana_pin_num;

	model_relations.resize(total_pin_num);
	model_names.resize(total_pin_num);
	for (size_t i = 0; i < total_pin_num; i++)
	{
		model_mat(i, i) = 1;
		if (i < range1)
		{

			model_mat(i, var_size - 1) = -var_bound_mat(i, 0);
			model_relations[i] = GRB_GREATER_EQUAL;
			model_names[i] = "VOLTAGE CONS" + to_string(i);
		}
		else if (i >= range1 && i < range2)
		{
			model_mat(i, i - range1 + duty_cycle_col) = -var_bound_mat(i, 1);
			model_relations[i] = GRB_EQUAL;
			model_names[i] = "DIGITAL CONS" + to_string(i - range1);
		}
		else if (i >= range2 && i < range3)
		{
			model_mat(i, var_size - 1) = -var_bound_mat(i, 0);
			model_relations[i] = GRB_GREATER_EQUAL;
			model_names[i] = "ANALOG CONS" + to_string(i - range2);
		}
		else
		{
			// not used 
			model_relations[i] = GRB_GREATER_EQUAL;
			model_names[i] = "OTHER CONS" + to_string(i - range3);
		}
		model_index_map[var_names[i]] = i;
	}
}

void Micro_Controller::constructDutyCycleMap()
{
	unsigned pre_pin_num = pow_in_pins.size() + pow_out_pins.size() +
		gnd_pins.size();
	for (size_t i = 0; i < digital_pins.size(); i++)
	{
		out_duty_cycle_map[var_names[pre_pin_num + i]] =
			var_names[pins.size() + i];
	}
}


void Micro_Controller::getPinNumberInfo()
{
	for (auto &beg = pins.begin(); beg != pins.end(); beg++)
	{
		if (beg->pin_class == Electronics::POWER)
		{
			switch (beg->func_type)
			{
			case Electronics::ELECTRICAL: 
				if (beg->io == Electronics::IN) 
				{
					pow_in_pins.push_back(*beg);
				}
				else
				{
					pow_out_pins.push_back(*beg);
				}
				break;
			case Electronics::GND: gnd_pins.push_back(*beg);
				break;
			default:
				break;
			}
		}
		else
		{
			switch (beg->func_type)
			{
			case Electronics::DIGITAL: digital_pins.push_back(*beg);
				digital_only_pins.push_back(*beg);
				break;
			case Electronics::DIGITAL_SPI_MOSI: digital_pins.push_back(*beg);
				spi_pins.push_back(*beg);
				break;
			case Electronics::DIGITAL_SPI_MISO: digital_pins.push_back(*beg);
				spi_pins.push_back(*beg);
				break;
			case Electronics::DIGITAL_SPI_SCK: digital_pins.push_back(*beg);
				spi_pins.push_back(*beg);
				break;
			case Electronics::DIGITAL_SPI_SS: digital_pins.push_back(*beg);
				spi_pins.push_back(*beg);
				break;
			case Electronics::DIGITAL_UART_TX: digital_pins.push_back(*beg);
				uart_pins.push_back(*beg);
				break;
			case Electronics::DIGITAL_UART_RX: digital_pins.push_back(*beg);
				uart_pins.push_back(*beg);
				break;
			case Electronics::DIGITAL_EXTERNAL_INTERRUPT: 
				digital_pins.push_back(*beg);
				external_interrupt_pins.push_back(*beg);
				break;
			case Electronics::DIGITAL_EXTERNAL_INTERRUPT_I2C_SDA:
				digital_pins.push_back(*beg);
				external_interrupt_pins.push_back(*beg);
				i2c_pins.push_back(*beg);
				break;
			case Electronics::DIGITAL_EXTERNAL_INTERRUPT_I2C_SCL:
				digital_pins.push_back(*beg);
				external_interrupt_pins.push_back(*beg);
				i2c_pins.push_back(*beg);
				break;
			case Electronics::DIGITAL_EXTERNAL_INTERRUPT_UART_TX:
				digital_pins.push_back(*beg);
				external_interrupt_pins.push_back(*beg);
				i2c_pins.push_back(*beg);
				break;
			case Electronics::DIGITAL_EXTERNAL_INTERRUPT_UART_RX:
				digital_pins.push_back(*beg);
				external_interrupt_pins.push_back(*beg);
				i2c_pins.push_back(*beg);
				break;
			case Electronics::ANALOG: analog_pins.push_back(*beg);
				break;
			case Electronics::ANALOG_I2C_SDA: analog_pins.push_back(*beg);
				i2c_pins.push_back(*beg);
				break;
			case Electronics::ANALOG_I2C_SCL: analog_pins.push_back(*beg);
				i2c_pins.push_back(*beg);
				break;
			case Electronics::SPI_MOSI: spi_pins.push_back(*beg);
				break;
			case Electronics::SPI_MISO: spi_pins.push_back(*beg);
				break;
			case Electronics::SPI_SCK: spi_pins.push_back(*beg);
				break;
			case Electronics::SPI_SS: spi_pins.push_back(*beg);
				break;
			case Electronics::PWM_SPI_MOSI: pwm_pins.push_back(*beg);
				digital_pins.push_back(*beg);
				spi_pins.push_back(*beg);
				break;
			case Electronics::PWM_SPI_MISO: pwm_pins.push_back(*beg);
				digital_pins.push_back(*beg);
				spi_pins.push_back(*beg);
				break;
			case Electronics::PWM_SPI_SCK: pwm_pins.push_back(*beg);
				digital_pins.push_back(*beg);
				spi_pins.push_back(*beg);
				break;
			case Electronics::PWM_SPI_SS: pwm_pins.push_back(*beg);
				digital_pins.push_back(*beg);
				spi_pins.push_back(*beg);
				break;
			case Electronics::UART_TX: uart_pins.push_back(*beg);
				break;
			case Electronics::UART_RX: uart_pins.push_back(*beg);
				break;
			case Electronics::I2C_SDA: i2c_pins.push_back(*beg);
				i2c_only_pins.push_back(*beg);
				break;
			case Electronics::I2C_SCL: i2c_pins.push_back(*beg);
				i2c_only_pins.push_back(*beg);
				break;
			case Electronics::PWM: pwm_pins.push_back(*beg);
				digital_pins.push_back(*beg);
				pwm_only_pins.push_back(*beg);
				break;
			case Electronics::PWM_EXTERNAL_INTERRUPT: 
				pwm_pins.push_back(*beg);
				digital_pins.push_back(*beg);
				external_interrupt_pins.push_back(*beg);
				break;
			case Electronics::OTHER: other_pins.push_back(*beg);
				break;
			default:
				break;
			}
		}
	}

	pins.clear();
	pins.insert(pins.end(), pow_in_pins.begin(), pow_in_pins.end());
	pins.insert(pins.end(), pow_out_pins.begin(), pow_out_pins.end());
	pins.insert(pins.end(), digital_pins.begin(), digital_pins.end());
	pins.insert(pins.end(), analog_pins.begin(), analog_pins.end());
	pins.insert(pins.end(), i2c_only_pins.begin(), i2c_only_pins.end());
	pins.insert(pins.end(), other_pins.begin(), other_pins.end());
	for (size_t i = 0; i < pins.size(); i++)
	{
		var_names[i] = pins[i].name;
		v_bound_mat.row(i) << pins[i].v_bound.first, pins[i].v_bound.second;
		i_bound_mat.row(i) << pins[i].i_bound.first, pins[i].i_bound.second;
	}
}

Battery::Battery(const std::string &file):Electrical_Component(file)
{
	if (!BATTERY_MAP.empty())
	{
		*this = BATTERY_MAP[file];
	}
	else
	{
		Electronics::Component *battery = read(file);
		extractInfo(battery);
		delete battery;
	}
}

void Battery::parameters() const
{
	cout << component_name << " PARAMETERS: " << endl;
	cout << "Capacity: " << capacity << endl;
	cout << endl;

	for (size_t i = 0; i < pins.size(); i++)
	{
		printPinInfo(pins[i]);
	}

	cout << endl;
	cout << model_mat << endl;
}

stringvec Battery::getPowerOutPinNames()
{
	return stringvec{pins[0].name};
}

void Battery::getUsedVarsName(const stringvec &pin_names)
{
	for (size_t i = 0; i < pin_names.size(); i++)
	{
		getDependentPins(pin_names[i]);
	}

	for (size_t i = 0; i < dependent_pins.size(); i++)
	{
		if (getPosInVec(dependent_pins[i]->name, used_var_names) == -1)
		{
			used_var_names.push_back(dependent_pins[i]->name);
		}
	}
}

void Battery::extractInfo(Electronics::Component *battery)
{
	Electronics::Battery ext = battery->GetExtension(
		Electronics::Battery::battery);
	capacity = ext.capacity();

	// config info
	size_t extra_var_size = 1;
	stringvec extra_var_names = {"CONST_BATTERY" };
	unsigned pos_col = 0, neg_col = 1, const_col = 2, pos_row = 0,
		neg_row = 1, model_row_size = 2;

	for (size_t i = 0; i < extra_var_size; i++)
	{
		vars.push_back(GRBVar());
		var_types.push_back(GRB_CONTINUOUS);
		var_names.push_back(extra_var_names[i]);
	}
	constructUsablePins();

	// boundary condition matrix
	var_bound_mat = MatrixXd::Zero(vars.size(), v_bound_mat.cols());
	var_bound_mat.block(0, 0, v_bound_mat.rows(), v_bound_mat.cols()) = 
		v_bound_mat;
	var_bound_mat.row(const_col) << 1, 1;


	// coefficient matrix
	model_mat = MatrixXd::Zero(model_row_size, vars.size());
	model_mat(pos_row, pos_col) = 1;
	model_mat(pos_row, const_col) = -v_bound_mat(pos_col, 0);
	model_mat(neg_row, neg_col) = 1;
	model_mat(neg_row, const_col) = -v_bound_mat(neg_col, 0);


	model_relations.resize(model_mat.rows());
	model_names.resize(model_mat.rows());
	model_relations[pos_row] = GRB_EQUAL;
	model_relations[neg_row] = GRB_EQUAL;
	model_names[pos_row] = "POSITIVE VOLTAGE CONS";
	model_names[neg_row] = "NEGATIVE VOLTAGE CONS";
	model_index_map[var_names[pos_col]] = pos_row;
	model_index_map[var_names[neg_col]] = neg_row;
}

/*

void encoder_model_test()
{
	for (const auto &entry: directory_iterator(encoder_path))
	{
		Encoder tempObj(entry.path().string());
		Actu_components obj = tempObj;
		obj.parameters();
	}
}


std::vector<Actu_components>& replicate(std::vector<Actu_components> &components)
{
	std::map<std::string, unsigned> countMap;
	for (auto &entry : components)
	{
		auto results = countMap.insert(std::pair<std::string, unsigned>(entry.component_name, 1));
		if (results.second == false)
		{
			entry.component_name = entry.component_name + "_" + std::to_string(++(results.first->second) - 1);
		}
	}
	return components;
}

Encoder::Encoder(const std::string &file): name(file)
{
	Encoder_Reader encoder_rd(file);
	unsignedvec ranges(encoder_rd.params.size());
	for (size_t i = 1; i < ranges.size(); i++)
	{
		ranges[i] = ranges[i - 1] + encoder_rd.params[i - 1];
	}

	unsigned var_size = *(ranges.end() - 1) + 1, pins_size = var_size - 1;
	var_vec.resize(var_size);
	for (size_t i = 0; i < var_size; i++)
	{
		GRBVar var;
		var_vec[i] = var;
	}
	type_vec = vector<char>(var_size, 'C');

	pin_names = encoder_rd.names;
	pin_names.push_back("CONST_ENCODER");

	MatrixXd vol_bound_mat(var_size, 2);
	vol_bound_mat.block(0, 0, pins_size, 2) = encoder_rd.vol_bound_mat;
	vol_bound_mat.row(pins_size) << 1, 1;

	MatrixXd coef_mat = MatrixXd::Zero(pins_size, var_size);
	for (size_t i = 0; i < pins_size; i++)
	{
		coef_mat(i, i) = 1;
		coef_mat(i, pins_size) = -vol_bound_mat(i, 0);
	}

}
*/
Camera::Camera(const string &file): Electrical_Component(file)
{
	if (!CAMERA_MAP.empty())
	{
		*this = CAMERA_MAP[file];
	}
	else
	{
		Electronics::Component *camera = read(file);
		extractInfo(camera);
		delete camera;
	}
}
void Camera::parameters() const
{
	cout << component_name << " PARAMETERS: " << endl;
	cout << "FREQUENCY: " << frequency << endl;
	cout << endl;

	for (size_t i = 0; i < pins.size(); i++)
	{
		printPinInfo(pins[i]);
	}

	cout << endl;
	cout << model_mat << endl;
}
doublevec Camera::getFuncInCurrentLimit()
{
	size_t uart_pos = pow_in_pins.size() + gnd_pins.size();
	return doublevec{i_bound_mat(uart_pos, 1)};
}
doublevec Camera::getPowerInCurrentLimit()
{
	return doublevec{i_bound_mat(0, 1)};
}
doublepairs Camera::getFuncInVolRange()
{
	size_t uart_pos = pow_in_pins.size() + gnd_pins.size();
	return doublepairs{make_pair(pins[uart_pos].v_bound.first, 
		pins[uart_pos].v_bound.second)};
}
doublepairs Camera::getPowerInVolRange()
{
	return doublepairs{make_pair(pins[0].v_bound.first, 
		pins[0].v_bound.second)};
}

Bluetooth::Bluetooth(const string &file): Electrical_Component(file)
{
	if (!BLUETOOTH_MAP.empty())
	{
		*this = BLUETOOTH_MAP[file];
	}
	else
	{
		Electronics::Component *bluetooth = read(file);
		extractInfo(bluetooth);
		delete bluetooth;
	}
}

Electronics::Component* Electrical_Component::read(const std::string &file)
{
	Electronics::Component *base_component = new Electronics::Component;

	int fd = _open(file.c_str(), O_RDONLY);
	if (fd == -1)
	{
		throw file + FILE_DOES_NOT_EXSIT;
	}
	else
	{
		ZeroCopyInputStream *input = new FileInputStream(fd);
		if (!TextFormat::Parse(input, base_component))
		{
			throw CAN_NOT_PARSE_FILE + file;
		}
		delete input;
		_close(fd);
	}

	return base_component;
}

void Electrical_Component::extractInfo(const Electronics::Component *base_component)
{
	component_name = base_component->component_name();
	component_type = base_component->component_type();
	component_class = base_component->component_class();
	nonlin = base_component->nonlin();
	component_size[0] = base_component->size().length();
	component_size[1] = base_component->size().width();
	component_size[2] = base_component->size().height();
	component_price = base_component->price();
	component_weight = base_component->weight();

	vars = std::vector<GRBVar>(base_component->pin_size());
	var_types = vector<char>(base_component->pin_size(), GRB_CONTINUOUS);

	pins.resize(base_component->pin_size());
	v_bound_mat = MatrixXd::Zero(base_component->pin_size(), 2);
	i_bound_mat = MatrixXd::Zero(base_component->pin_size(), 2);
	for (size_t i = 0; i < base_component->pin_size(); i++)
	{
		const Electronics::pin pin = base_component->pin(static_cast<int>(i));
		pins[i] = pin2Pin(pin);

		var_names.push_back(pin.name());
		v_bound_mat.row(i) << pin.v_bound().lb(), pin.v_bound().ub();
		i_bound_mat.row(i) << pin.i_bound().lb(), pin.i_bound().ub();
	}

}

void Electrical_Component::constructVarMaps()
{
	for (size_t i = 0; i < vars.size(); i++)
	{
		var_maps[var_names[i]] = &vars[i];
	}
}

vector<Pin*> Electrical_Component::getPins(Electronics::CLASS pin_class, Electronics::IO pin_io)
{
	vector<Pin*> filtered_pins;
	for (size_t i = 0; i < pins.size(); i++)
	{
		if (pins[i].pin_class == pin_class && pins[i].io == pin_io)
		{
			filtered_pins.push_back(&pins[i]);
		}
	}
	return filtered_pins;
}

Lin_Cons lincons2LinCons(const Electronics::lin_cons &cons)
{
	stringvec var_names(cons.var_name_size());
	MatrixXd coefficients = MatrixXd::Zero(1, cons.var_name_size());
	for (size_t i = 0; i < cons.var_name_size(); i++)
	{
		var_names[i] = cons.var_name(i);
		coefficients(0, i) = cons.coefficient(i);
	}

	return Lin_Cons(var_names, coefficients, cons.constant(), cons.type());
}

Pin pin2Pin(const Electronics::pin &pin)
{
	Pin myPin;
	myPin.name = pin.name();
	myPin.pin_class = pin.pin_class();
	myPin.func_type = pin.func_type();
	myPin.phys_type = pin.phys_type();
	myPin.io = pin.io();
	myPin.connection = pin.connection();
	myPin.status = pin.status();
	myPin.v_bound = make_pair(pin.v_bound().lb(), pin.v_bound().ub());
	myPin.i_bound = make_pair(pin.i_bound().lb(), pin.i_bound().ub());
	myPin.dependents.resize(pin.dependents().others_size());
	for (size_t i = 0; i < pin.dependents().others_size(); i++)
	{
		myPin.dependents[i] = pin.dependents().others(static_cast<int>(i));
	}
	return myPin;
}

void printPinInfo(const Pin &pin)
{
	cout << "pin: " << pin.name << endl;
	cout << "pin class: " << pin.pin_class << endl;
	cout << "function type: " << pin.func_type << endl;
	cout << "physical type: " << pin.phys_type << endl;
	cout << "io: " << pin.io << endl;
	cout << "connection: " << pin.connection << endl;
	cout << "status: " << pin.status << endl;
	cout << "voltage bounds: [" << pin.v_bound.first << ", " << pin.v_bound.second << "]" << endl;
	cout << "current bounds: [" << pin.i_bound.first << ", " << pin.i_bound.second << "]" << endl;
	cout << "dependent pins: " << std::ends;
	for (size_t i = 0; i < pin.dependents.size(); i++)
	{
		if (i != pin.dependents.size() - 1)
		{
			cout << pin.dependents[i] << ", " << ends;
		}
		else
		{
			cout << pin.dependents[i] << endl;
		}
	}
}

void initializeAllMotors(const string &dir)
{
	unordered_map<string, Motor> motor_map;
	for (auto &entry : directory_iterator(dir))
	{
		string component = entry.path().string();
		motor_map[component] = Motor(component);
		MOTOR_PART_MAP[component] = make_shared<Motor>(component);
	}
	MOTOR_MAP = motor_map; // avoid conflict with constructor
}

void initializeAllServos(const std::string &dir)
{
	unordered_map<string, Motor> servo_map;
	for (auto &entry : directory_iterator(dir))
	{
		string component = entry.path().string();
		servo_map[component] = Motor(component);
		SERVO_PART_MAP[component] = make_shared<Motor>(component);
	}
	SERVO_MAP = servo_map; 
}

void initializeAllVoltageRegulators(const string &dir)
{
	unordered_map<string, Voltage_Regulator> voltage_regulator_map;
	for (auto &entry : directory_iterator(dir))
	{
		string component = entry.path().string();
		voltage_regulator_map[component] = Voltage_Regulator(component);
		VOLTAGE_REGULATOR_PART_MAP[component] = make_shared<Voltage_Regulator>(component);
	}
	VOLTAGE_REGULATOR_MAP = voltage_regulator_map; 
}

void initializeAllHBridges(const string &dir)
{
	unordered_map<string, H_Bridge> h_bridge_map;
	for (auto &entry : directory_iterator(dir))
	{
		string component = entry.path().string();
		h_bridge_map[component] = H_Bridge(component);
		H_BRIDDE_PART_MAP[component] = make_shared<H_Bridge>(component);
	}
	H_BRIDGE_MAP = h_bridge_map;
}

void initializeAllMicroController(const std::string &dir)
{
	unordered_map<string, Micro_Controller> micro_controller_map;
	for (auto &entry : directory_iterator(dir))
	{
		string component = entry.path().string();
		micro_controller_map[component] = Micro_Controller(component);
		MICRO_CONTROLLER_PART_MAP[component] = make_shared<Micro_Controller>(component);
	}
	MICRO_CONTROLLER_MAP = micro_controller_map;
}

void initializeAllBatteries(const string &dir)
{
	unordered_map<string, Battery> battery_map;
	for (auto &entry : directory_iterator(dir))
	{
		string component = entry.path().string();
		battery_map[component] = Battery(component);
		BATTERY_PART_MAP[component] = make_shared<Battery>(component);
	}
	BATTERY_MAP = battery_map;
}

void initializeAllEncoders(const string &dir)
{
	unordered_map<string, Encoder> encoder_map;
	for (auto &entry : directory_iterator(dir))
	{
		string component = entry.path().string();
		encoder_map[component] = Encoder(component);
		ENCODER_PART_MAP[component] = make_shared<Encoder>(component);
	}
	ENCODER_MAP = encoder_map;
}

void initializeAllCameras(const string &dir)
{
	unordered_map<string, Camera> camera_map;
	for (auto &entry : directory_iterator(dir))
	{
		string component = entry.path().string();
		camera_map[component] = Camera(component);
		CAMERA_PART_MAP[component] = make_shared<Camera>(component);
	}
	CAMERA_MAP = camera_map;
}

void initializeAllBluetooths(const string &dir)
{
	unordered_map<string, Bluetooth> bluetooth_map;
	for (auto &entry : directory_iterator(dir))
	{
		string component = entry.path().string();
		bluetooth_map[component] = Bluetooth(component);
		BLUETOOTH_PART_MAP[component] = make_shared<Bluetooth>(component);
	}
	BLUETOOTH_MAP = bluetooth_map;
}

void initializeAllForceSensors(const string &dir)
{
	unordered_map<string, Force_Sensor> force_sensor_map;
	for (auto &entry : directory_iterator(dir))
	{
		string component = entry.path().string();
		force_sensor_map[component] = Force_Sensor(component);
		FORCE_SENSOR_PART_MAP[component] = make_shared<Force_Sensor>(component);
	}
	FORCE_SENSOR_MAP = force_sensor_map;
}

void Motor::parameters() const
{
	cout << component_name << " PARAMETERS: " << endl;
	cout << "kt: " << kt << endl;
	cout << "ke: " << ke << endl;
	cout << "r: " << r << endl;
	cout << "v: " << v << endl;
	cout << "i: " << i << endl;
	cout << "torque: " << torq_ub << endl;
	cout << "velocity: " << vel_ub << endl;
	cout << endl;

	for (size_t i = 0; i < pins.size(); i++)
	{
		printPinInfo(pins[i]);
	}

	cout << endl;
	cout << model_mat << endl;
}

doublevec Motor::getFuncInCurrentLimit()
{
	if (isServo())
	{
		return doublevec{ pins[2].i_bound.second/100 };
	}
	else
	{
		return doublevec();
	}
}

doublevec Motor::getPowerInCurrentLimit()
{
	return doublevec{ var_maps["I"]->get(GRB_DoubleAttr_X) };
}

doublepairs Motor::getFuncInVolRange()
{
	if (isServo())
	{
		return doublepairs{ make_pair(pins[2].v_bound.first,
			pins[2].v_bound.second) };
	}
	else
	{
		return doublepairs();
	}
}

doublepairs Motor::getPowerInVolRange()
{
	return doublepairs{ make_pair(pins[0].v_bound.first, pins[0].v_bound.second) };
}

stringvec Motor::getPowerInPinNames()
{
	return stringvec{ pins[0].name };
}

stringvec Motor::getFuncInPinNames()
{
	if (isServo())
	{
		return stringvec{pins[2].name};
	}
	else
	{
		return stringvec();
	}
}

unsigned Motor::getFuncInSize()
{
	if (isServo())
	{
		return 1;
	}
	else
	{
		return pins.size();
	}
}

void Motor::setWorkPoint(double _torq_des, double _vel_des)
{
	torq_des = _torq_des;
	vel_des = _vel_des;

	unsigned torq_col = 3, vel_col = 4;
	if (isServo())
	{
		torq_col++;
		vel_col++;
	}
	var_bound_mat.row(torq_col) << torq_des, torq_des;
	var_bound_mat.row(vel_col) << vel_des, vel_des;
}

void Motor::extractInfo(const Electronics::Component *motor)
{
	Electronics::Motor ext = motor->GetExtension(Electronics::Motor::motor);
	v = v_bound_mat(0, 1);
	i = i_bound_mat(0, 1);
	kt = ext.kt();
	ke = ext.ke();
	r = ext.r();
	torq_ub = ext.torq();
	vel_ub = ext.vel();
	
	// config info
	size_t extra_var_size = 4;
	stringvec extra_var_names = { "I", "TORQ", "VEL", "CONST_MOTOR" };
	unsigned lead1_col = 0, lead2_col = 1, sig_col, i_col = 2, torq_col = 3, vel_col = 4, const_col = 5,
		vol_row = 0, torq_row = 1, vel_row = 2, sig_row, model_row_size = 3;

	// servo condition
	if (isServo())
	{
		sig_col = 2;
		i_col++;
		torq_col++;
		vel_col++;
		const_col++;
		sig_row = 3;
		model_row_size++;
	}

	for (size_t i = 0; i < extra_var_size; i++)
	{
		vars.push_back(GRBVar());
		var_types.push_back(GRB_CONTINUOUS);
		var_names.push_back(extra_var_names[i]);
	}
	constructUsablePins();

	// boundary condition matrix
	var_bound_mat = MatrixXd::Zero(vars.size(), v_bound_mat.cols());
	var_bound_mat.block(0, 0, v_bound_mat.rows(), v_bound_mat.cols()) = v_bound_mat;
	var_bound_mat.row(i_col) << 0, i_bound_mat(0, 1);
	var_bound_mat.row(torq_col) << torq_des, torq_des;
	var_bound_mat.row(vel_col) << vel_des, vel_des;
	var_bound_mat.row(const_col) << 1, 1;
	
	// coefficient matrix
	model_mat = MatrixXd::Zero(model_row_size, vars.size());
	model_mat(vol_row, lead1_col) = 1;
	model_mat(vol_row, lead2_col) = -1;
	model_mat(vol_row, const_col) = -v_bound_mat(0, 1);
	model_mat(torq_row, i_col) = kt;
	model_mat(torq_row, torq_col) = -1;
	model_mat(vel_row, lead1_col) = 1/ke;
	model_mat(vel_row, lead2_col) = -1/ke;
	model_mat(vel_row, i_col) = -r/ke;
	model_mat(vel_row, vel_col) = -1;
	if (isServo())
	{
		model_mat(sig_row, sig_col) = 1;
		model_mat(sig_row, const_col) = -v_bound_mat(sig_col, 1);
	}

	model_relations.resize(model_mat.rows());
	model_names.resize(model_mat.rows());
	model_relations[vol_row] = GRB_LESS_EQUAL;
	model_relations[torq_row] = GRB_GREATER_EQUAL;
	model_relations[vel_row] = GRB_GREATER_EQUAL;
	model_names[vol_row] = "VOLTAGE CONS";
	model_names[torq_row] = "TORQUE CONS";
	model_names[vel_row] = "VELOCITY_CONS";
	model_index_map[var_names[lead1_col]] = vol_row;
	model_index_map[var_names[torq_col]] = torq_row;
	model_index_map[var_names[vel_col]] = vel_row;
	if (isServo())
	{
		model_relations[sig_row] = GRB_LESS_EQUAL;
		model_names[sig_row] = "SIGNAL CONS";
		model_index_map[var_names[sig_row]] = sig_row;
	}

}

Motor::Motor(const string &file, double _torq_des, double _vel_des) : Electrical_Component(file),
torq_des(_torq_des), vel_des(_vel_des)
{
	if (isDCMotor(file) && !MOTOR_MAP.empty()) 
	{
		*this = MOTOR_MAP[file];
	}
	else if (::isServo(file) && !SERVO_MAP.empty())
	{
		*this = SERVO_MAP[file];
	}
	else
	{
		Electronics::Component *motor = read(file);
		extractInfo(motor);
		delete motor;
	}
}

H_Bridge::H_Bridge(const string &file) : Electrical_Component(file)
{
	if (!H_BRIDGE_MAP.empty())
	{
		*this = H_BRIDGE_MAP[file];
	}
	else
	{
		Electronics::Component *h_bridge = read(file);
		extractInfor(h_bridge);
		delete h_bridge;
	}
}

doublepairs H_Bridge::getFuncInVolRange()
{
	size_t in_pos = log_pin_num + mot_pin_num + ena_pin_num + gnd_pin_num;
	return doublepairs{ make_pair(pins[in_pos].v_bound.first,
		pins[in_pos].v_bound.second) };
}

doublepairs H_Bridge::getPowerInVolRange()
{
	doublepair log_vol_range = make_pair(pins[0].v_bound.first,
		pins[0].v_bound.second),
		mot_vol_range = make_pair(pins[log_pin_num].v_bound.first, 
			pins[log_pin_num].v_bound.second);

	if (isIntersect(log_vol_range, mot_vol_range))
	{
		return doublepairs{ getIntersect(log_vol_range, mot_vol_range) };
	}
	else
	{
		return doublepairs{ log_vol_range, mot_vol_range };
	}
}

doublepair H_Bridge::getFuncOutVolRange()
{
	size_t out_pos = log_pin_num + mot_pin_num + ena_pin_num + gnd_pin_num +
		in_pin_num;
	return make_pair(pins[out_pos].v_bound.first, pins[out_pos].v_bound.second);
}

stringvec H_Bridge::getPowerInPinNames()
{
	doublepair log_vol_range = make_pair(pins[0].v_bound.first,
		pins[0].v_bound.second),
		mot_vol_range = make_pair(pins[log_pin_num].v_bound.first,
			pins[log_pin_num].v_bound.second);

	if (isIntersect(log_vol_range, mot_vol_range))
	{
		return stringvec{ pins[0].name };
	}
	else
	{
		return stringvec{ pins[0].name, pins[log_pin_num].name };
	}
}

stringvec H_Bridge::getPowerOutPinNames()
{
	stringvec out_pin_names;
	size_t out_pos = log_pin_num + mot_pin_num + ena_pin_num + gnd_pin_num +
		in_pin_num;
	for (size_t i = 0; i < out_pin_num; i++)
	{
		out_pin_names.push_back(pins[i+out_pos].name);
	}
	return out_pin_names;
}

stringvec H_Bridge::getFuncInPinNames()
{
	stringvec in_pin_names;
	size_t in_pos = log_pin_num + mot_pin_num + ena_pin_num + gnd_pin_num;
	for (size_t i = 0; i < in_pin_num; i++)
	{
		in_pin_names.push_back(pins[i + in_pos].name);
	}
	return in_pin_names;
}

void H_Bridge::getUsedVarsName(const stringvec &pin_names)
{
	for (size_t i = 0; i < pin_names.size(); i++)
	{
		getDependentPins(pin_names[i]);
	}

	for (size_t i = 0; i < dependent_pins.size(); i++)
	{
		if (getPosInVec(dependent_pins[i]->name, used_var_names) == -1)
		{
			used_var_names.push_back(dependent_pins[i]->name);
		}
	}

	size_t used_vars_size = used_var_names.size();
	for (size_t i = 0; i < used_vars_size; i++)
	{
		auto &iter = in_duty_cycle_map.find(used_var_names[i]);
		if (iter != in_duty_cycle_map.end())
		{
			if (getPosInVec(iter->second, used_var_names) == -1)
			{
				used_var_names.push_back(iter->second);
			}
		} 
	}
}

unsigned H_Bridge::getMainPowerIndex()
{
	doublepair log_vol_range = make_pair(pins[0].v_bound.first,
		pins[0].v_bound.second),
		mot_vol_range = make_pair(pins[log_pin_num].v_bound.first,
			pins[log_pin_num].v_bound.second);

	if (isIntersect(log_vol_range, mot_vol_range))
	{
		return 0;
	}
	else
	{
		return 1;
	}
}

unsigned H_Bridge::getFuncInSize()
{
	unsigned func_in_size = 0;
	size_t out_pos = log_pin_num + mot_pin_num + ena_pin_num + gnd_pin_num +
		in_pin_num;
	for (size_t i = 0; i < out_pin_num; i++)
	{
		if (pins[i + out_pos].status)
		{
			func_in_size++;
		}
	}
	return func_in_size;
}

void H_Bridge::extractInfor(const Electronics::Component *h_bridge)
{
	Electronics::H_Bridge ext = h_bridge->GetExtension(Electronics::H_Bridge::h_bridge);
	logic_level.first = ext.logic_level().lb();
	logic_level.second = ext.logic_level().ub();

	unsignedvec pin_nums = getPinNumberInfo(pins);
	log_pin_num = pin_nums[0], mot_pin_num = pin_nums[1], 
		ena_pin_num = pin_nums[2], in_pin_num = pin_nums[3],
		out_pin_num = pin_nums[4], gnd_pin_num = pin_nums[5],	
		other_pin_num = pin_nums[6],
		total_pin_num = std::accumulate(pin_nums.begin(), pin_nums.end(), 0);

	// config part
	unsigned extra_var_size = in_pin_num + 1, 
		duty_cycle_col = static_cast<unsigned> (vars.size());
	var_bound_mat = MatrixXd::Zero(vars.size() + extra_var_size, 2);
	for (size_t i = 0; i < extra_var_size; i++)
	{
		vars.push_back(GRBVar());
		var_types.push_back(GRB_CONTINUOUS);
		if (i== extra_var_size - 1)
		{
			var_names.push_back("CONST_HBRIDGE");
			var_bound_mat.row(i + duty_cycle_col) << 1, 1;
		}
		else
		{
			var_names.push_back("DUTY_CYCYLE" + std::to_string(i));
			var_bound_mat.row(i + duty_cycle_col) << 0, 1;
		}
	}
	constructDutyCycleMap();
	getNonlinVarNames();

	// formulating boundary matrix
	var_bound_mat.block(0, 0, v_bound_mat.rows(), v_bound_mat.cols()) = 
		v_bound_mat;

	// coefficient matrix
	unsigned var_size = static_cast<unsigned>(vars.size());
	model_mat = MatrixXd::Zero(pins.size(), vars.size());
	unsigned range1 = log_pin_num + mot_pin_num + ena_pin_num + gnd_pin_num,
		range2 = range1 + in_pin_num, range3 = range2 + out_pin_num,
		ena_range_start = log_pin_num + mot_pin_num,
		ena_range_end = ena_range_start + ena_pin_num;

	model_relations.resize(total_pin_num);
	model_names.resize(total_pin_num);
	for (size_t i = 0; i < total_pin_num; i++)
	{
		model_mat(i, i) = 1;
		if (i < range1)
		{
			if (i >= ena_range_start && i < ena_range_end)
			{
				model_mat(i, var_size - 1) = -logic_level.second;
			}
			else
			{
				model_mat(i, var_size - 1) = -v_bound_mat(i, 0);
			}
			model_relations[i] = GRB_GREATER_EQUAL;
			model_names[i] = "VOLTAGE CONS" + to_string(i);
		}
		else if (i >= range1 && i < range2)
		{
			model_mat(i, i - range1 + duty_cycle_col) = -logic_level.second;
			model_relations[i] = GRB_GREATER_EQUAL;
			model_names[i] = "INPUT CONS" + to_string(i - range1);
		}
		else if (i >= range2 && i < range3)
		{
			model_mat(i, log_pin_num) = -var_bound_mat(i - range2 + duty_cycle_col, 1);
			model_mat(i, i - range2 + duty_cycle_col) = -var_bound_mat(log_pin_num, 1);
			model_mat(i, var_size - 1) = var_bound_mat(i - range2 + duty_cycle_col, 1)*var_bound_mat(log_pin_num, 1);
			model_relations[i] = GRB_EQUAL;
			model_names[i] = "OUTPUT CONS" + to_string(i - range2);
		}
		else
		{
			// not used 
			model_relations[i] = GRB_EQUAL;
			model_names[i] = "OTHER CONS" + to_string(i - range3);
		}
		model_index_map[var_names[i]] = i;
	}
}

void H_Bridge::constructDutyCycleMap()
{
	unsigned pre_pin_num = log_pin_num + mot_pin_num + ena_pin_num + 
		gnd_pin_num;
	for (size_t i = 0; i < in_pin_num; i++)
	{
		in_duty_cycle_map[var_names[pre_pin_num + i]] = 
			var_names[total_pin_num + i];
	}

	pre_pin_num += in_pin_num;
	for (size_t i = 0; i < out_pin_num; i++)
	{
		out_duty_cycle_map[var_names[pre_pin_num + i]] = 
			var_names[total_pin_num + i];
	}
}

unsignedvec H_Bridge::getPinNumberInfo(const vector<Pin> &_pins)
{	
	// reorder pins
	vector<Pin> log_pins, mot_pins, ena_pins, gnd_pins,
		in_pins, out_pins, other_pins;
	unsigned log_pin_num = 0, mot_pin_num = 0, ena_pin_num = 0, 
		in_pin_num = 0, out_pin_num = 0, gnd_pin_num = 0, other_pin_num = 0;
	for (auto &beg = _pins.begin(); beg != _pins.end(); beg++)
	{
		if (beg->pin_class == Electronics::POWER)
		{
			switch (beg->func_type)
			{
			case Electronics::LOGIC: log_pin_num++;
				log_pins.push_back(*beg);
				break;
			case Electronics::MOTOR: mot_pin_num++;
				mot_pins.push_back(*beg);
				break;
			case Electronics::GND: gnd_pin_num++;
				gnd_pins.push_back(*beg);
				break;
			default:
				break;
			}
		}
		else
		{
			switch (beg->func_type)
			{
			case Electronics::ENABLE: ena_pin_num++;
				ena_pins.push_back(*beg);
				break;
			case Electronics::ELECTRICAL: 
				if (beg->io == Electronics::IN)
				{
					in_pin_num++;
					in_pins.push_back(*beg);
				}
				else
				{
					out_pin_num++; 
					out_pins.push_back(*beg);
				}
				break;
			case Electronics::OTHER: other_pin_num++;
				other_pins.push_back(*beg);
				break;
			default:
				break;
			}
		}
	}

	pins.clear();
	pins.insert(pins.end(), log_pins.begin(), log_pins.end());
	pins.insert(pins.end(), mot_pins.begin(), mot_pins.end());
	pins.insert(pins.end(), ena_pins.begin(), ena_pins.end());
	pins.insert(pins.end(), gnd_pins.begin(), gnd_pins.end());
	pins.insert(pins.end(), in_pins.begin(), in_pins.end());
	pins.insert(pins.end(), out_pins.begin(), out_pins.end());
	pins.insert(pins.end(), other_pins.begin(), other_pins.end());
	for (size_t i = 0; i < pins.size(); i++)
	{
		var_names[i] = pins[i].name;
		v_bound_mat.row(i) << pins[i].v_bound.first, pins[i].v_bound.second;
		i_bound_mat.row(i) << pins[i].i_bound.first, pins[i].i_bound.second;
	}
	return {log_pin_num, mot_pin_num, ena_pin_num, in_pin_num, out_pin_num, 
		gnd_pin_num, other_pin_num};
}

void H_Bridge::parameters() const
{
	for (size_t i = 0; i < pins.size(); i++)
	{
		printPinInfo(pins[i]);
	}

	cout << "LOGIC LEVEL << [" << logic_level.first << ", " << logic_level.second << "]" << endl;
	cout << model_mat << endl;
}

stringvec H_Bridge::getNonlinVarNames()
{
	for (size_t i = 0; i < out_pin_num; i++)
	{
		nonlin_var_names.push_back(var_names[i + log_pin_num + mot_pin_num + 
			ena_pin_num + gnd_pin_num + in_pin_num]);
	}
	return nonlin_var_names;
}

stringvec H_Bridge::getUsedNonLinVarNames(const stringvec &used_pins)
{
	if (used_nonlin_var_names.empty())
	{
		for (size_t i = 0; i < used_pins.size(); i++)
		{
			if (getPosInVec(used_pins[i], nonlin_var_names) != -1)
			{
				used_nonlin_var_names.push_back(var_names[log_pin_num]);
				used_nonlin_var_names.push_back(*var_names.rbegin());
				break;
			}
		}
	}

	for (size_t i = 0; i < used_pins.size(); i++)
	{
		if (getPosInVec(used_pins[i], nonlin_var_names) != -1)
		{
			used_nonlin_var_names.push_back(used_pins[i]);
			used_nonlin_var_names.push_back(out_duty_cycle_map[used_pins[i]]);
		}
	}
	return used_nonlin_var_names;
}

bool H_Bridge::thresholdCheck()
{
	bool indicator = false;
	double mot_pin_val;
	doublevec out_pin_vals, duty_cycle_vals;
	boolvec indicator_vec;
	for (size_t i = 0; i < used_nonlin_var_names.size(); i += 2)
	{
		if (i == 0)
		{
			mot_pin_val = var_maps[used_nonlin_var_names[0]]->get(GRB_DoubleAttr_X);
		}
		else if (i >= 2 && i < used_nonlin_var_names.size())
		{
			out_pin_vals.push_back(var_maps[used_nonlin_var_names[i]]->get(GRB_DoubleAttr_X));
			duty_cycle_vals.push_back(var_maps[used_nonlin_var_names[i+1]]->get(GRB_DoubleAttr_X));
		}
	}

	for (size_t i = 0; i < out_pin_vals.size(); i++)
	{
		indicator_vec.push_back(abs(out_pin_vals[i] - mot_pin_val*duty_cycle_vals[i]) < tol ? true : false);
	}
	std::find(indicator_vec.begin(), indicator_vec.end(), false) == indicator_vec.end() ? indicator = true : indicator = false;	
	return indicator;
}

void H_Bridge::updateCoefficients(GRBModel *model, vector<GRBConstr> &model_cons)
{
	boolvec indicator_vec;
	double mot_pin_val;
	doublevec out_pin_vals, duty_cycle_vals;
	for (size_t i = 0; i < used_nonlin_var_names.size(); i += 2)
	{
		if (i == 0)
		{
			mot_pin_val = var_maps[used_nonlin_var_names[0]]->get(GRB_DoubleAttr_X);
		}
		else if (i >= 2 && i < used_nonlin_var_names.size())
		{
			out_pin_vals.push_back(var_maps[used_nonlin_var_names[i]]->get(GRB_DoubleAttr_X));
			duty_cycle_vals.push_back(var_maps[used_nonlin_var_names[i+1]]->get(GRB_DoubleAttr_X));
		}
	}

	for (size_t i = 0; i < out_pin_vals.size(); i++)
	{
		indicator_vec.push_back(abs(out_pin_vals[i] - mot_pin_val * duty_cycle_vals[i]) < tol ? true : false);
	}

	for (size_t i = 0, j = 0; i < out_pin_vals.size(); i++, j += 2)
	{
		if (!indicator_vec[i])
		{
			string cons_name = model_names[model_index_map[used_nonlin_var_names[2+j]]];
			GRBConstr &cons = model->getConstrByName(cons_name);
			model->chgCoeff(cons, *var_maps[used_nonlin_var_names[0]],
				-duty_cycle_vals[i]);
			model->chgCoeff(cons, *var_maps[used_nonlin_var_names[j + 3]],
				-mot_pin_val);
			model->chgCoeff(cons, *var_maps[used_nonlin_var_names[1]],
				duty_cycle_vals[i] * mot_pin_val);
		}
	}
}

doublevec H_Bridge::getFuncInCurrentLimit()
{
	size_t in_pos = log_pin_num + mot_pin_num + ena_pin_num + gnd_pin_num;
	return doublevec{ i_bound_mat(in_pos, 1) };
}

doublevec H_Bridge::getPowerInCurrentLimit()
{
	doublepair log_vol_range = make_pair(pins[0].v_bound.first,
		pins[0].v_bound.second),
		mot_vol_range = make_pair(pins[log_pin_num].v_bound.first,
			pins[log_pin_num].v_bound.second);

	if (isIntersect(log_vol_range, mot_vol_range))
	{
		return doublevec{ i_bound_mat(log_pin_num, 1) };
	}
	else
	{
		return doublevec{ i_bound_mat(0, 1), i_bound_mat(log_pin_num, 1) };
	}
	
}

double H_Bridge::getFuncOutCurrentLimit()
{
	size_t out_pos = log_pin_num + mot_pin_num + ena_pin_num + gnd_pin_num +
		in_pin_num;
	return i_bound_mat(out_pos, 1);
}

void Electrical_Component::updateUsedPinsVolLB()
{
	for (size_t i = 0; i < dependent_pins.size(); i++)
	{
		double lb = var_maps[dependent_pins[i]->name]->get(GRB_DoubleAttr_X);
		dependent_pins[i]->v_bound.first = lb;
	}
}

void Electrical_Component::updateUsedPinsVolUB()
{
	for (size_t i = 0; i < dependent_pins.size(); i++)
	{
		double ub = var_maps[dependent_pins[i]->name]->get(GRB_DoubleAttr_X);
		dependent_pins[i]->v_bound.second = ub;
	}
}

void Electrical_Component::restorePinVolBound(const string &pin_name)
{
	for (size_t i = 0; i < dependent_pins.size(); i++)
	{
		if (dependent_pins[i]->name == pin_name)
		{
			size_t pos = getPosInVec(*dependent_pins[i], pins);
			dependent_pins[i]->v_bound.first = v_bound_mat(pos, 0);
			dependent_pins[i]->v_bound.second = v_bound_mat(pos, 1);
		}

	}
}

doublepair Electrical_Component::getOutVolRange(const Electronics::CLASS
	&pin_class)
{
	if (pin_class == Electronics::CLASS::POWER)
	{
		return getPowerOutVolRange();
	}
	else {
		return getFuncOutVolRange();
	}
}

// doublepair Electrical_Component::getInVolRange(const Electronics::CLASS 
//	&pin_class)
// {
//	if (pin_class == Electronics::CLASS::POWER)
//	{
//		return getPowerInVolRange();
//	}
//	else
//	{
//		return getFuncInVolRange();
//	}
// }

double Electrical_Component::getOutCurrentLimit(const Electronics::CLASS 
	&pin_class)
{
	if (pin_class == Electronics::CLASS::POWER)
	{
		return getPowerOutCurrentLimit();
	}
	else
	{
		return getFuncOutCurrentLimit();
	}
}

// double Electrical_Component::getInCurrentLimit(const Electronics::CLASS 
//	&pin_class)
// {
//	if (pin_class == Electronics::CLASS::POWER)
//	{
//		return getPowerInCurrentLimit();
//	}
//	else
//	{
//		return getFuncInCurrentLimit();
//	}
// }

vector<Pin*> Electrical_Component::getPowerInPins()
{
	return getPins(Electronics::CLASS::POWER, Electronics::IO::IN);
}

vector<Pin*> Electrical_Component::getPowerOutPins()
{
	return getPins(Electronics::CLASS::POWER, Electronics::IO::OUT);
}

vector<Pin*> Electrical_Component::getFuncInPins()
{
	return getPins(Electronics::CLASS::FUNCTION, Electronics::IO::IN);
}

vector<Pin*> Electrical_Component::getFuncOutPins()
{
	return getPins(Electronics::CLASS::FUNCTION, Electronics::IO::OUT);
}

vector<Pin*> Electrical_Component::getFuncBidirectPins()
{
	return getPins(Electronics::CLASS::FUNCTION, Electronics::IO::BIDIRECT);
}

vector<Pin*> Electrical_Component::getBothInPins()
{
	return getPins(Electronics::CLASS::BOTH, Electronics::IO::IN);
}

vector<Pin*> Electrical_Component::getBothOutPins()
{
	return getPins(Electronics::CLASS::BOTH, Electronics::IO::OUT);
}

vector<Pin*> Electrical_Component::getBothBidirectPins()
{
	return getPins(Electronics::CLASS::BOTH, Electronics::IO::BIDIRECT);
}

vector<Pin*> Electrical_Component::getDependentPins(const string &pin_name)
{
	for (size_t i = 0; i < pins.size(); i++)
	{
		if (pins[i].name == pin_name && 
			getPosInVec(&pins[i], dependent_pins) == -1)
		{
			dependent_pins.push_back(&pins[i]);
			for (size_t j = 0; j < pins[i].dependents.size(); j++)
			{
				getDependentPins(pins[i].dependents[j]);
			}
		}
	}
	return dependent_pins;
}

void Electrical_Component::clearConnections(const stringvec &pin_names)
{
	for (size_t i = 0; i < pins.size(); i++)
	{
		if (getPosInVec(pins[i].name, pin_names) != -1)
		{
			pins[i].status = false;
		}
	}
}

Force_Sensor::Force_Sensor(const string &file):Electrical_Component(file)
{
	if (!FORCE_SENSOR_MAP.empty())
	{
		*this = FORCE_SENSOR_MAP[file];
	}
	else
	{
		Electronics::Component *force_sensor = read(file);
		extractInfo(force_sensor);
		delete force_sensor;
	}
}

void Force_Sensor::parameters() const
{
	cout << component_name << " PARAMETERS: " << endl;
	cout << endl;

	for (size_t i = 0; i < pins.size(); i++)
	{
		printPinInfo(pins[i]);
	}

	cout << endl;
	cout << model_mat << endl;
}

doublevec Force_Sensor::getFuncInCurrentLimit()
{
	return doublevec{ i_bound_mat(1, 1) };
}

doublevec Force_Sensor::getPowerInCurrentLimit()
{
	return doublevec{ i_bound_mat(0, 1) };
}

doublepairs Force_Sensor::getFuncInVolRange()
{
	return doublepairs{ make_pair(pins[1].v_bound.first,
		pins[1].v_bound.second) };
}

doublepairs Force_Sensor::getPowerInVolRange()
{
	return doublepairs{ make_pair(pins[0].v_bound.first, 
		pins[0].v_bound.second) };
}

stringvec Force_Sensor::getPowerInPinNames()
{
	return stringvec{pins[0].name};
}

stringvec Force_Sensor::getFuncInPinNames()
{
	return stringvec{ pins[1].name };
}

void Force_Sensor::getUsedVarsName(const stringvec &pin_names)
{
	for (size_t i = 0; i < pins.size(); i++)
	{
		dependent_pins.push_back(&pins[i]);
	}

	for (size_t i = 0; i < dependent_pins.size(); i++)
	{
		if (getPosInVec(dependent_pins[i]->name, used_var_names) == -1)
		{
			used_var_names.push_back(dependent_pins[i]->name);
		}
	}
}

void Force_Sensor::extractInfo(Electronics::Component *force_sensor)
{
	// config info
	size_t extra_var_size = 1;
	stringvec extra_var_names = { "CONST_FORCE_SENSOR" };
	unsigned lead1_col = 0, lead2_col = 1, const_col = 2,
		lead1_row = 0, lead2_row = 1, model_row_size = 2;

	for (size_t i = 0; i < extra_var_size; i++)
	{
		vars.push_back(GRBVar());
		var_types.push_back(GRB_CONTINUOUS);
		var_names.push_back(extra_var_names[i]);
	}

	// boundary condition matrix
	var_bound_mat = MatrixXd::Zero(vars.size(), v_bound_mat.cols());
	var_bound_mat.block(0, 0, v_bound_mat.rows(), v_bound_mat.cols()) = v_bound_mat;
	var_bound_mat.row(const_col) << 1, 1;

	// coefficient matrix
	model_mat = MatrixXd::Zero(model_row_size, vars.size());
	model_mat(lead1_row, lead1_col) = 1;
	model_mat(lead1_row, const_col) = -v_bound_mat(0, 1);
	model_mat(lead2_row, lead2_col) = 1;
	model_mat(lead2_row, const_col) = -v_bound_mat(1, 1);

	model_relations.resize(model_mat.rows());
	model_names.resize(model_mat.rows());
	model_relations[lead1_row] = GRB_LESS_EQUAL;
	model_relations[lead2_row] = GRB_LESS_EQUAL;
	model_names[lead1_row] = "LEAD1 CONS";
	model_names[lead2_row] = "LEAD2 CONS";
	model_index_map[var_names[lead1_col]] = lead1_row;
	model_index_map[var_names[lead2_col]] = lead2_row;
}

stringvec Voltage_Regulator::getPowerInPinNames()
{
	return stringvec{pins[0].name};
}

stringvec Voltage_Regulator::getPowerOutPinNames()
{
	size_t pow_out_pos = pow_in_pins.size();
	return stringvec{pins[pow_out_pos].name};
}

void Voltage_Regulator::getUsedVarsName(const stringvec &pin_names)
{
	for (size_t i = 0; i < pin_names.size(); i++)
	{
		getDependentPins(pin_names[i]);
	}

	for (size_t i = 0; i < dependent_pins.size(); i++)
	{
		if (getPosInVec(dependent_pins[i]->name, used_var_names) == -1)
		{
			used_var_names.push_back(dependent_pins[i]->name);
		}
	}
}

void Voltage_Regulator::extractInfo(Electronics::Component *voltage_regulator)
{
	Electronics::Voltage_Regulator ext = voltage_regulator->
		GetExtension(Electronics::Voltage_Regulator::voltage_regulator);
	for (size_t i = 0; i < ext.cons_size(); i++)
	{
		linear_constraints.push_back(lincons2LinCons(ext.cons(i)));
	}

	this->getPinNumberInfo();

	// config info
	size_t extra_var_size = 1;
	stringvec extra_var_names = { "CONST_VOLTAGE_REGULATOR" };
	unsigned pow_in_pin_num = pow_in_pins.size(),
		pow_out_pin_num = pow_out_pins.size(), gnd_pin_num = gnd_pins.size(),
		range1 = pow_in_pin_num, range2 = range1 + pow_out_pin_num,
		model_row_size = pow_in_pin_num + pow_out_pin_num + gnd_pin_num;

	for (size_t i = 0; i < extra_var_size; i++)
	{
		vars.push_back(GRBVar());
		var_types.push_back(GRB_CONTINUOUS);
		var_names.push_back(extra_var_names[i]);
	}

	// boundary condition matrix
	var_bound_mat = MatrixXd::Zero(vars.size(), v_bound_mat.cols());
	var_bound_mat.block(0, 0, v_bound_mat.rows(), v_bound_mat.cols()) =
		v_bound_mat;
	var_bound_mat.row(vars.size() - 1) << 1, 1;


	// coefficient matrix
	model_mat = MatrixXd::Zero(model_row_size, vars.size());
	model_relations.resize(model_mat.rows());
	model_names.resize(model_mat.rows());
	for (size_t i = 0; i < model_row_size; i++)
	{
		model_mat(i, i) = 1;
		model_mat(i, vars.size() - 1) = -v_bound_mat(i, 1);
		model_relations[i] = GRB_LESS_EQUAL;
		model_index_map[var_names[i]] = i;
		if (i < range1)
		{
			model_names[i] = "POWER IN CONS";
		}
		else if (i >= range1 && i < range2)
		{
			model_names[i] = "POWER OUT CONS";
		}
		else
		{
			model_names[i] = "GND CONS";
		}

	}
}

void Voltage_Regulator::getPinNumberInfo()
{
	// reorder pins
	for (auto &beg = pins.begin(); beg != pins.end(); beg++)
	{
		if (beg->pin_class == Electronics::POWER)
		{
			switch (beg->func_type)
			{
			case Electronics::ELECTRICAL: 
				if (beg->io == Electronics::IN)
				{
					pow_in_pins.push_back(*beg);
				}
				else
				{
					pow_out_pins.push_back(*beg);
				}
				break;
			case Electronics::GND: gnd_pins.push_back(*beg);
				break;
			default:
				break;
			}
		}
	}

	pins.clear();
	pins.insert(pins.end(), pow_in_pins.begin(), pow_in_pins.end());
	pins.insert(pins.end(), pow_out_pins.begin(), pow_out_pins.end());
	pins.insert(pins.end(), gnd_pins.begin(), gnd_pins.end());

	for (size_t i = 0; i < pins.size(); i++)
	{
		var_names[i] = pins[i].name;
		v_bound_mat.row(i) << pins[i].v_bound.first, pins[i].v_bound.second;
		i_bound_mat.row(i) << pins[i].i_bound.first, pins[i].i_bound.second;
	}
}

stringvec Camera::getPowerInPinNames()
{
	return stringvec{pins[0].name};
}

stringvec Camera::getFuncInPinNames()
{
	stringvec func_in_pin_names(uart_pins.size());
	size_t comm_pos = pow_in_pins.size() + gnd_pins.size();
	for (size_t i = 0; i < uart_pins.size(); i++)
	{
		func_in_pin_names[i] = uart_pins[i].name;
	}
	return func_in_pin_names;
}

void Camera::getUsedVarsName(const stringvec &pin_names)
{
	if (pin_names.empty())
	{
		for (size_t i = 0; i < pins.size() - other_pins.size(); i++)
		{
			dependent_pins.push_back(&pins[i]);
		}
	}
	else
	{
		for (size_t i = 0; i < pin_names.size(); i++)
		{
			getDependentPins(pin_names[i]);
		}
	}

	for (size_t i = 0; i < dependent_pins.size(); i++)
	{
		if (getPosInVec(dependent_pins[i]->name, used_var_names) == -1)
		{
			used_var_names.push_back(dependent_pins[i]->name);
		}
	}
}

void Camera::extractInfo(Electronics::Component *camera)
{
	Electronics::Camera ext = camera->GetExtension(
		Electronics::Camera::camera);
	frequency = ext.frequency();

	this->getPinNumberInfo();

	// config info
	size_t extra_var_size = 1;
	stringvec extra_var_names = { "CONST_BATTERY" };
	unsigned pow_in_pin_num = pow_in_pins.size(),
		gnd_pin_num = gnd_pins.size(), uart_pin_num = uart_pins.size(),
		other_pin_num = other_pins.size(), range1 = pow_in_pin_num,
		range2 = range1 + gnd_pin_num, range3 = range2 + uart_pin_num, 
		model_row_size = pins.size();

	for (size_t i = 0; i < extra_var_size; i++)
	{
		vars.push_back(GRBVar());
		var_types.push_back(GRB_CONTINUOUS);
		var_names.push_back(extra_var_names[i]);
	}

	// boundary condition matrix
	var_bound_mat = MatrixXd::Zero(vars.size(), v_bound_mat.cols());
	var_bound_mat.block(0, 0, v_bound_mat.rows(), v_bound_mat.cols()) =
		v_bound_mat;
	var_bound_mat.row(vars.size() - 1) << 1, 1;


	// coefficient matrix
	model_mat = MatrixXd::Zero(model_row_size, vars.size());
	model_relations.resize(model_mat.rows());
	model_names.resize(model_mat.rows());
	for (size_t i = 0; i < model_row_size; i++)
	{
		model_mat(i, i) = 1;
		model_mat(i, vars.size() - 1) = -v_bound_mat(i, 1);
		model_relations[i] = GRB_LESS_EQUAL;
		model_index_map[var_names[i]] = i;
		if (i < range1)
		{
			model_names[i] = "POWER INPUT CONS";
		}
		else if (i >= range1 && i < range2)
		{
			model_names[i] = "GND CONS";
		}
		else if (i >= range2 && i < range3)
		{
			model_names[i] = "UART CONS";
		}
		else
		{
			model_names[i] = "OTHER CONS";
		}
	}
}

void Camera::getPinNumberInfo()
{
	// reorder pins
	for (auto &beg = pins.begin(); beg != pins.end(); beg++)
	{
		if (beg->pin_class == Electronics::POWER)
		{
			switch (beg->func_type)
			{
			case Electronics::ELECTRICAL: pow_in_pins.push_back(*beg);
				break;
			case Electronics::GND: gnd_pins.push_back(*beg);
				break;
			default:
				break;
			}
		}
		else
		{
			switch (beg->func_type)
			{
			case Electronics::UART_TX: uart_pins.push_back(*beg);
				break;
			case Electronics::UART_RX: uart_pins.push_back(*beg);
				break;
			case Electronics::I2C_SDA: i2c_pins.push_back(*beg);
				break;
			case Electronics::I2C_SCL: i2c_pins.push_back(*beg);
				break;
			case Electronics::UART_TX_I2C_SDA: i2c_pins.push_back(*beg);
				uart_pins.push_back(*beg);
				break;
			case Electronics::UART_TX_I2C_SCL: i2c_pins.push_back(*beg);
				uart_pins.push_back(*beg);
				break;
			case Electronics::UART_RX_I2C_SDA: i2c_pins.push_back(*beg);
				uart_pins.push_back(*beg);
				break;
			case Electronics::UART_RX_I2C_SCL: i2c_pins.push_back(*beg);
				uart_pins.push_back(*beg);
				break;
			case Electronics::OTHER: other_pins.push_back(*beg);
				break;
			default:
				break;
			}
		}
	}

	pins.clear();
	pins.insert(pins.end(), pow_in_pins.begin(), pow_in_pins.end());
	pins.insert(pins.end(), gnd_pins.begin(), gnd_pins.end());
	pins.insert(pins.end(), uart_pins.begin(), uart_pins.end());
	pins.insert(pins.end(), other_pins.begin(), other_pins.end());

	for (size_t i = 0; i < pins.size(); i++)
	{
		var_names[i] = pins[i].name;
		v_bound_mat.row(i) << pins[i].v_bound.first, pins[i].v_bound.second;
		i_bound_mat.row(i) << pins[i].i_bound.first, pins[i].i_bound.second;
	}
}

void Bluetooth::parameters() const
{

	cout << component_name << " PARAMETERS: " << endl;
	cout << "FREQUENCY: " << frequency << endl;
	cout << endl;

	for (size_t i = 0; i < pins.size(); i++)
	{
		printPinInfo(pins[i]);
	}

	cout << endl;
	cout << model_mat << endl;
}

doublevec Bluetooth::getFuncInCurrentLimit()
{
	size_t comm_pos = pow_in_pins.size() + gnd_pins.size();
	return doublevec{ i_bound_mat(comm_pos, 1) };
}

doublevec Bluetooth::getPowerInCurrentLimit()
{
	return doublevec{ i_bound_mat(0, 1) };
}

doublepairs Bluetooth::getFuncInVolRange()
{
	size_t comm_pos = pow_in_pins.size() + gnd_pins.size();
	return doublepairs{ make_pair(pins[comm_pos].v_bound.first,
		pins[comm_pos].v_bound.second) };
}

doublepairs Bluetooth::getPowerInVolRange()
{
	return doublepairs{make_pair(pins[0].v_bound.first, 
		pins[0].v_bound.second)};
}

stringvec Bluetooth::getPowerInPinNames()
{
	return stringvec{pins[0].name};
}

stringvec Bluetooth::getFuncInPinNames()
{
	stringvec func_out_pin_names(uart_pins.size() + spi_pins.size());
	size_t comm_pos = pow_in_pins.size() + gnd_pins.size();
	for (size_t i = 0; i < func_out_pin_names.size(); i++)
	{
		if (i < uart_pins.size())
		{
			func_out_pin_names[i] = uart_pins[i].name;
		}
		else
		{
			func_out_pin_names[i] = spi_pins[i - uart_pins.size()].name;
		}
	}
	return func_out_pin_names;
}

void Bluetooth::getUsedVarsName(const stringvec &pin_names)
{
	if (pin_names.empty())
	{
		size_t used_size = pow_in_pins.size() + gnd_pins.size() +
			uart_pins.size() + spi_pins.size();
		for (size_t i = 0; i < used_size; i++)
		{
			dependent_pins.push_back(&pins[i]);
		}
	}
	else
	{
		for (size_t i = 0; i < pin_names.size(); i++)
		{
			getDependentPins(pin_names[i]);
		}
	}

	for (size_t i = 0; i < dependent_pins.size(); i++)
	{
		if (getPosInVec(dependent_pins[i]->name, used_var_names) == -1)
		{
			used_var_names.push_back(dependent_pins[i]->name);
		}
	}
}

void Bluetooth::extractInfo(Electronics::Component *bluetooth)
{
	Electronics::Bluetooth ext = bluetooth->GetExtension(
		Electronics::Bluetooth::bluetooth);
	frequency = ext.frequency();

	this->getPinNumberInfo();

	// config info
	size_t extra_var_size = 1;
	stringvec extra_var_names = { "CONST_BATTERY" };
	unsigned pow_in_pin_num = pow_in_pins.size(),
		gnd_pin_num = gnd_pins.size(), uart_pin_num = uart_pins.size(),
		spi_pin_num = spi_pins.size(), dig_pin_num = digital_pins.size(),
		inter_pin_num = interrupt_pins.size(),
		other_pin_num = other_pins.size(), range1 = pow_in_pin_num,
		range2 = range1 + gnd_pin_num, range3 = range2 + uart_pin_num,
		range4 = range3 + spi_pin_num, range5 = range4 + dig_pin_num,
		range6 = range5 + inter_pin_num, model_row_size = pins.size();

	for (size_t i = 0; i < extra_var_size; i++)
	{
		vars.push_back(GRBVar());
		var_types.push_back(GRB_CONTINUOUS);
		var_names.push_back(extra_var_names[i]);
	}

	// boundary condition matrix
	var_bound_mat = MatrixXd::Zero(vars.size(), v_bound_mat.cols());
	var_bound_mat.block(0, 0, v_bound_mat.rows(), v_bound_mat.cols()) =
		v_bound_mat;
	var_bound_mat.row(vars.size() - 1) << 1, 1;


	// coefficient matrix
	model_mat = MatrixXd::Zero(model_row_size, vars.size());
	model_relations.resize(model_mat.rows());
	model_names.resize(model_mat.rows());
	for (size_t i = 0; i < model_row_size; i++)
	{
		model_mat(i, i) = 1;
		model_mat(i, vars.size() - 1) = -v_bound_mat(i, 1);
		model_relations[i] = GRB_LESS_EQUAL;
		model_index_map[var_names[i]] = i;
		if (i < range1)
		{
			model_names[i] = "POWER INPUT CONS";
		}
		else if (i >= range1 && i < range2)
		{
			model_names[i] = "GND CONS";
		}
		else if (i >= range2 && i < range3)
		{
			model_names[i] = "UART CONS";
		}
		else if (i >= range3 && i < range4)
		{
			model_names[i] = "SPI CONS";
		}
		else if (i >= range4 && i < range5)
		{
			model_names[i] = "DIGITAL CONS";
		}
		else if (i >= range5 && i < range6)
		{
			model_names[i] = "EXTERNAL INTERRUPT CONS";
		}
		else
		{
			model_names[i] = "OTHER CONS";
		}
	}
}

void Bluetooth::getPinNumberInfo()
{
	// reorder pins
	for (auto &beg = pins.begin(); beg != pins.end(); beg++)
	{
		if (beg->pin_class == Electronics::POWER)
		{
			switch (beg->func_type)
			{
			case Electronics::ELECTRICAL: pow_in_pins.push_back(*beg);
				break;
			case Electronics::GND: gnd_pins.push_back(*beg);
				break;
			default:
				break;
			}
		}
		else
		{
			switch (beg->func_type)
			{
			case Electronics::UART_RX: uart_pins.push_back(*beg);
				break;
			case Electronics::UART_TX: uart_pins.push_back(*beg);
				break;
			case Electronics::SPI_MISO: spi_pins.push_back(*beg);
				break;
			case Electronics::SPI_MOSI: spi_pins.push_back(*beg);
				break;
			case Electronics::SPI_SCK: spi_pins.push_back(*beg);
				break;
			case Electronics::SPI_SS: spi_pins.push_back(*beg);
				break;
			case Electronics::DIGITAL: digital_pins.push_back(*beg);
				break;
			case Electronics::EXTERNAL_INTERRUPT: 
				interrupt_pins.push_back(*beg);
				break;
			case Electronics::OTHER: other_pins.push_back(*beg);
				break;
			default:
				break;
			}
		}
	}

	pins.clear();
	pins.insert(pins.end(), pow_in_pins.begin(), pow_in_pins.end());
	pins.insert(pins.end(), gnd_pins.begin(), gnd_pins.end());
	pins.insert(pins.end(), uart_pins.begin(), uart_pins.end());
	pins.insert(pins.end(), spi_pins.begin(), spi_pins.end());
	pins.insert(pins.end(), digital_pins.begin(), digital_pins.end());
	pins.insert(pins.end(), interrupt_pins.begin(), interrupt_pins.end());
	pins.insert(pins.end(), other_pins.begin(), other_pins.end());
	for (size_t i = 0; i < pins.size(); i++)
	{
		var_names[i] = pins[i].name;
		v_bound_mat.row(i) << pins[i].v_bound.first, pins[i].v_bound.second;
		i_bound_mat.row(i) << pins[i].i_bound.first, pins[i].i_bound.second;
	}
}

void Encoder::extractInfo(Electronics::Component *encoder)
{
	Electronics::Encoder ext = encoder->GetExtension(
		Electronics::Encoder::encoder);
	frequency = ext.frequency();

	this->getPinNumberInfo();

	// config info
	size_t extra_var_size = 1;
	stringvec extra_var_names = { "CONST_BATTERY" };
	unsigned pow_in_pin_num = pow_in_pins.size(),
		gnd_pin_num = gnd_pins.size(), signal_pin_num = signal_pins.size(),
		other_pin_num = other_pins.size(), range1 = pow_in_pin_num,
		range2 = range1 + gnd_pin_num, range3 = range2 + signal_pin_num,
		model_row_size = pins.size();

	for (size_t i = 0; i < extra_var_size; i++)
	{
		vars.push_back(GRBVar());
		var_types.push_back(GRB_CONTINUOUS);
		var_names.push_back(extra_var_names[i]);
	}

	// boundary condition matrix
	var_bound_mat = MatrixXd::Zero(vars.size(), v_bound_mat.cols());
	var_bound_mat.block(0, 0, v_bound_mat.rows(), v_bound_mat.cols()) =
		v_bound_mat;
	var_bound_mat.row(vars.size() - 1) << 1, 1;


	// coefficient matrix
	model_mat = MatrixXd::Zero(model_row_size, vars.size());
	model_relations.resize(model_mat.rows());
	model_names.resize(model_mat.rows());
	for (size_t i = 0; i < model_row_size; i++)
	{
		model_mat(i, i) = 1;
		model_mat(i, vars.size() - 1) = -v_bound_mat(i, 1);
		model_relations[i] = GRB_LESS_EQUAL;
		model_index_map[var_names[i]] = i;
		if (i < range1)
		{
			model_names[i] = "POWER INPUT CONS";
		}
		else if (i >= range1 && i < range2)
		{
			model_names[i] = "GND CONS";
		}
		else if (i >= range2 && i < range3)
		{
			model_names[i] = "SIGNAL CONS";
		}
		else
		{
			model_names[i] = "OTHER CONS";
		}
	}
}

void Encoder::getPinNumberInfo()
{
	// reorder pins
	for (auto &beg = pins.begin(); beg != pins.end(); beg++)
	{
		if (beg->pin_class == Electronics::POWER)
		{
			switch (beg->func_type)
			{
			case Electronics::ELECTRICAL: pow_in_pins.push_back(*beg);
				break;
			case Electronics::GND: gnd_pins.push_back(*beg);
				break;
			default:
				break;
			}
		}
		else
		{
			switch (beg->func_type)
			{
			case Electronics::DIGITAL: signal_pins.push_back(*beg);
				break;
			case Electronics::OTHER: other_pins.push_back(*beg);
				break;
			default:
				break;
			}
		}
	}

	pins.clear();
	pins.insert(pins.end(), pow_in_pins.begin(), pow_in_pins.end());
	pins.insert(pins.end(), gnd_pins.begin(), gnd_pins.end());
	pins.insert(pins.end(), signal_pins.begin(), signal_pins.end());
	pins.insert(pins.end(), other_pins.begin(), other_pins.end());

	for (size_t i = 0; i < pins.size(); i++)
	{
		var_names[i] = pins[i].name;
		v_bound_mat.row(i) << pins[i].v_bound.first, pins[i].v_bound.second;
		i_bound_mat.row(i) << pins[i].i_bound.first, pins[i].i_bound.second;
	}
}

Encoder::Encoder(const string &file): Electrical_Component(file)
{
	if (!ENCODER_MAP.empty())
	{
		*this = ENCODER_MAP[file];
	}
	else
	{
		Electronics::Component *encoder = read(file);
		extractInfo(encoder);
		delete encoder;
	}
}

void Encoder::parameters() const
{
	cout << component_name << " PARAMETERS: " << endl;
	cout << "FREQUENCY: " << frequency << endl;
	cout << endl;

	for (size_t i = 0; i < pins.size(); i++)
	{
		printPinInfo(pins[i]);
	}

	cout << endl;
	cout << model_mat << endl;
}

doublevec Encoder::getFuncInCurrentLimit()
{
	size_t sig_pos = pow_in_pins.size() + gnd_pins.size();
	return doublevec{i_bound_mat(sig_pos, 1)};
}

doublevec Encoder::getPowerInCurrentLimit()
{
	return doublevec{i_bound_mat(0, 1)};
}

doublepairs Encoder::getFuncInVolRange()
{
	size_t sig_pos = pow_in_pins.size() + gnd_pins.size();
	return doublepairs{make_pair(pins[sig_pos].v_bound.first, 
		pins[sig_pos].v_bound.second)};
}

doublepairs Encoder::getPowerInVolRange()
{
	return doublepairs{make_pair(pins[0].v_bound.first, 
		pins[0].v_bound.second)};
}

stringvec Encoder::getPowerInPinNames()
{
	stringvec pow_in_pin_names;
	for (size_t i = 0; i < pow_in_pins.size(); i++)
	{
		pow_in_pin_names.push_back(pins[i].name);
	}
	return pow_in_pin_names;
}

stringvec Encoder::getFuncInPinNames()
{
	size_t sig_pos = pow_in_pins.size() + gnd_pins.size();
	stringvec func_in_pin_names;
	for (size_t i = 0; i < signal_pins.size(); i++)
	{
		func_in_pin_names.push_back(pins[i + sig_pos].name);
	}
	return func_in_pin_names;
}

void Encoder::getUsedVarsName(const stringvec &pin_names)
{
	if (pin_names.empty())
	{
		for (size_t i = 0; i < pins.size() - other_pins.size(); i++)
		{
			dependent_pins.push_back(&pins[i]);
		}
	}
	else
	{
		for (size_t i = 0; i < pin_names.size(); i++)
		{
			getDependentPins(pin_names[i]);
		}
	}

	for (size_t i = 0; i < dependent_pins.size(); i++)
	{
		if (getPosInVec(dependent_pins[i]->name, used_var_names) == -1)
		{
			used_var_names.push_back(dependent_pins[i]->name);
		}
	}
}
