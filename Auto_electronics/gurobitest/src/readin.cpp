#include "readin.h"
using Eigen::MatrixXd;
using std::cerr;
using std::cout;
using std::endl;
using std::vector;
using std::string;
using google::protobuf::io::FileInputStream;
using google::protobuf::io::ZeroCopyInputStream;
using google::protobuf::TextFormat;
using Electronics::bounds;
using std::experimental::filesystem::directory_iterator;

bool Hbridge_Reader::initialize_all = true, Read_Micro_Controller::initialize_all = true, Motor_Reader::initialize_all = true,
Read_Vregulator::initialize_all = true, Battery_Interface::initialize_all = true, Encoder_Reader::initialize_all = true,
ForceSensor_Reader::initialize_all = true, Bluetooth_Reader::initialize_all = true, Camera_Reader::initialize_all = true;

vector<Battery_Interface> _BATTERY_SPECIFICATION_VEC_;
vector<Read_Vregulator> _VOLTAGE_REGULATOR_SPECIFICATION_VEC_;
vector<Read_Micro_Controller> _MICRO_CONTROLLER_SPECIFICATIONS_VEC_;
vector<Hbridge_Reader> _HBRIDGE_SPECIFICATIONS_VEC_;
vector<Motor_Reader> _MOTOR_SPECIFICATIONS_VEC_;
vector<Encoder_Reader> _ENCODER_SPECIFICATIONS_VEC_;
vector<ForceSensor_Reader> _FORCESENSOR_SPECIFICATIONS_VEC_;
vector<Bluetooth_Reader> _BLUETOOTH_SPECIFICATIONS_VEC_;
vector<Camera_Reader> _CAMERA_SPECIFICATIONS_VEC_;

stringvec _BATTERY_SPECIFICATION_FILES_, _VOLTAGE_REGULATOR_SPECIFICATION_FILES_,
_MICRO_CONTROLLER_SPECIFICATIONS_FILES_, _HBRIDGE_SPECIFICATIONS_FILES_, _MOTOR_SPECIFICATIONS_FILES_,
_ENCODER_SPECIFICATIONS_FILES_, _FORCESENSOR_SPECIFICATIONS_FILES_, _BLUETOOTH_SPECIFICATIONS_FILES_,
_CAMERA_SPECIFICATIONS_FILES_;

// This function extract information from protobuf object
void Read_Vregulator::extract_info(const Electronics::Vregulator& myVregulator) {

	// intrinsic parameters
	params = { double(myVregulator.in_pin_num()), double(myVregulator.en_pin_num()), double(myVregulator.adj_pin_num()), 
	double(myVregulator.out_pin_num()), double(myVregulator.gnd_pin_num())};


	if (myVregulator.adj_pin_num() != 0)
	{
		params.push_back(myVregulator.v_ref());
		params.push_back(myVregulator.i_adj());

		const Electronics::bounds &reference_resist = myVregulator.resistor();
		params.push_back(reference_resist.lb());
		params.push_back(reference_resist.ub());
	}

	// inference related parameters
	power_input_num = myVregulator.in_pin_num();
	power_output_num = myVregulator.out_pin_num();

	unsigned in_pin_num = myVregulator.in_pin_num(),
		en_pin_num = myVregulator.en_pin_num(),
		adj_pin_num = myVregulator.adj_pin_num(),
		out_pin_num = myVregulator.out_pin_num(),
		gnd_pin_num = myVregulator.gnd_pin_num(),
		total_pin_num = in_pin_num + en_pin_num + adj_pin_num + out_pin_num + gnd_pin_num;

	// boundary condition matrix
	MatrixXd temp_vol_mat(total_pin_num, 2), temp_cur_mat(total_pin_num, 2), temp_pin_mat(total_pin_num, total_pin_num);
	for (unsigned i = 0; i < total_pin_num; i++)
	{
		if (i < in_pin_num)
		{
			const bounds& in_pin_vol = myVregulator.in_pin_v_bounds(i);
			temp_vol_mat.row(i) << in_pin_vol.lb(), in_pin_vol.ub();

			// inference related parameters
			power_input.second = in_pin_vol.ub();
		}
		else if (i >= in_pin_num && i < in_pin_num + en_pin_num)
		{
			const bounds &en_pin_vol = myVregulator.en_pin_v_bounds(i - in_pin_num);
			temp_vol_mat.row(i) << en_pin_vol.lb(), en_pin_vol.ub();
			
			// inference related parameters
			power_input.second = std::min(power_input.second, en_pin_vol.ub());
		}
		else if (i >= in_pin_num + en_pin_num && i < in_pin_num + en_pin_num + adj_pin_num)
		{
			const bounds &adj_pin_vol = myVregulator.adj_pin_v_bounds(i - in_pin_num - en_pin_num);
			temp_vol_mat.row(i) << adj_pin_vol.lb(), adj_pin_vol.ub();
		}
		else if (i >= in_pin_num + en_pin_num + adj_pin_num && i < in_pin_num + en_pin_num +adj_pin_num + out_pin_num)
		{
			const bounds &out_pin_vol = myVregulator.out_pin_v_bounds(i - in_pin_num - en_pin_num - adj_pin_num);
			temp_vol_mat.row(i) << out_pin_vol.lb(), out_pin_vol.ub();

			// inference related parameters 
			power_output.first = out_pin_vol.lb();
			power_output.second = out_pin_vol.ub();
		}
		else
		{
			const bounds &gnd_pin_vol = myVregulator.gnd_pin_v_bounds(i - in_pin_num - en_pin_num - out_pin_num - adj_pin_num);
			temp_vol_mat.row(i) << gnd_pin_vol.lb(), gnd_pin_vol.ub();
		}
	}

	if (en_pin_num != 0)
	{
		const bounds &logic_level = myVregulator.logic_bounds();
		temp_vol_mat.conservativeResize(temp_vol_mat.rows() + 1, temp_vol_mat.cols());
		temp_vol_mat.row(temp_vol_mat.rows() - 1) << logic_level.lb(), logic_level.ub();
	}
	bound_mat = temp_vol_mat;

	for (unsigned i = 0; i < total_pin_num; i++)
	{
		if (i < in_pin_num)
		{
			if (myVregulator.in_pin_i_bounds_size() != 0)
			{
				const bounds &in_pin_cur = myVregulator.in_pin_i_bounds(i);
				temp_cur_mat.row(i) << in_pin_cur.lb(), in_pin_cur.ub();
			}
		}
		else if (i >= in_pin_num && i < in_pin_num+en_pin_num)
		{
			if (myVregulator.en_pin_i_bounds_size() != 0)
			{
				const bounds &en_pin_cur = myVregulator.en_pin_i_bounds(i - in_pin_num);
				temp_cur_mat.row(i) << en_pin_cur.lb(), en_pin_cur.ub();
			}
		}
		else if (i >= in_pin_num+en_pin_num && i < in_pin_num+en_pin_num+adj_pin_num)
		{
			if (myVregulator.adj_pin_i_bounds_size() != 0)
			{
				const bounds &adj_pin_cur = myVregulator.adj_pin_i_bounds(i - in_pin_num - en_pin_num);
				temp_cur_mat.row(i) << adj_pin_cur.lb(), adj_pin_cur.ub();
			}
		}
		else if (i >= in_pin_num+en_pin_num+adj_pin_num && i < in_pin_num+en_pin_num+adj_pin_num+out_pin_num)
		{
			if (myVregulator.out_pin_i_bounds_size() !=  0)
			{
				const bounds &out_pin_cur = myVregulator.out_pin_i_bounds(i-in_pin_num-en_pin_num-adj_pin_num);
				temp_cur_mat.row(i) << out_pin_cur.lb(), out_pin_cur.ub();
			}
		}
		else
		{
			if (myVregulator.gnd_pin_i_bounds_size() != 0)
			{
				const bounds &gnd_pin_cur = myVregulator.gnd_pin_i_bounds(i - in_pin_num - en_pin_num - out_pin_num - adj_pin_num);
				temp_cur_mat.row(i) << gnd_pin_cur.lb(), gnd_pin_cur.ub();
			}
		}
	}
	current_mat = temp_cur_mat;

	// names for pins
	for (unsigned i = 0; i < in_pin_num + en_pin_num + adj_pin_num + out_pin_num + gnd_pin_num; i++)
	{
		if (i < in_pin_num)
		{
			const string &input_pin = myVregulator.in_pin_name(i);
			names.push_back(input_pin);
		}
		else if (i >= in_pin_num && i < in_pin_num + en_pin_num)
		{
			const string &enable_pin = myVregulator.en_pin_name(i - in_pin_num);
			names.push_back(enable_pin);
		}
		else if (i >= in_pin_num + en_pin_num && i < in_pin_num + en_pin_num + adj_pin_num)
		{
			const string &adj_pin = myVregulator.adj_pin_name(i - in_pin_num - en_pin_num);
			names.push_back(adj_pin);
		}
		else if (i >= in_pin_num + en_pin_num + adj_pin_num && i < in_pin_num + en_pin_num + adj_pin_num + out_pin_num)
		{
			const string &output_pin = myVregulator.out_pin_name(i - in_pin_num - en_pin_num - adj_pin_num);
			names.push_back(output_pin);
			nonlin_var_names.push_back(output_pin);
		}
		else
		{
			const string &gnd_pin = myVregulator.gnd_pin_name(i - in_pin_num - en_pin_num - adj_pin_num - out_pin_num);
			names.push_back(gnd_pin);
		}
	}

	// nonlinearity of component
	nonlin_flag = myVregulator.nonlin();

	// constraints
	for (size_t i = 0; i < myVregulator.constraints_size(); i++)
	{
		const Electronics::Linear_Constraint &cons_info = myVregulator.constraints(i);
		stringvec var_names(cons_info.var_name_size());
		MatrixXd coefficients(1, cons_info.coefficient_size());
		double constant = cons_info.constant();
		Electronics::Connection_type type = cons_info.type();
		for (size_t i = 0; i < cons_info.var_name_size(); i++)
		{
			var_names[i] = cons_info.var_name(i);
			coefficients(0, i) = cons_info.coefficient(i);
		}
		constraints.push_back(linear_constraint_expression(var_names, coefficients, constant, type));		
	}

	// pin relation matrix
	for (size_t i = 0; i < in_pin_num; i++)
	{
		for (auto beg = myVregulator.in_pin_row(i).c_str(); *beg != '\0'; beg++)
		{
			temp_pin_mat(i, beg - myVregulator.in_pin_row(i).c_str()) = static_cast<int>(*beg - '0');
		}
	}

	for (size_t i = 0; i < en_pin_num; i++)
	{
		for (auto beg = myVregulator.en_pin_row(i).c_str(); *beg != '\0'; beg++)
		{
			temp_pin_mat(i + in_pin_num, beg - myVregulator.en_pin_row(i).c_str()) = static_cast<int>(*beg - '0');
		}
	}

	for (size_t i = 0; i < adj_pin_num; i++)
	{
		for (auto beg = myVregulator.adj_pin_row(i).c_str(); *beg != '\0'; beg++)
		{
			temp_pin_mat(i + in_pin_num + en_pin_num, beg - myVregulator.adj_pin_row(i).c_str()) = static_cast<int>(*beg - '0');
		}
	}

	for (size_t i = 0; i < out_pin_num; i++)
	{
		for (auto beg = myVregulator.out_pin_row(i).c_str(); *beg != '\0'; beg++)
		{
			temp_pin_mat(i + in_pin_num + en_pin_num + adj_pin_num, beg - myVregulator.out_pin_row(i).c_str()) = static_cast<int>(*beg - '0');
		}
	}

	for (size_t i = 0; i < gnd_pin_num; i++)
	{
		for (auto beg = myVregulator.gnd_pin_row(i).c_str(); *beg != '\0'; beg++)
		{
			temp_pin_mat(i + in_pin_num + en_pin_num + adj_pin_num + out_pin_num, beg - myVregulator.gnd_pin_row(i).c_str()) = static_cast<int>(*beg - '0');
		}
	}
	pin_relation_mat = temp_pin_mat;
}

Read_Vregulator::Read_Vregulator(const string &file)
{
	if (initialize_all)
	{
		if (_VOLTAGE_REGULATOR_SPECIFICATION_VEC_.empty())
		{
			initialize_all = false;
			_VOLTAGE_REGULATOR_SPECIFICATION_VEC_ = initializeVoltageRegulatorVec(_VOLTAGE_REGULATOR_SPECIFICATION_FILES_);
			initialize_all = true;
		}
		*this = _VOLTAGE_REGULATOR_SPECIFICATION_VEC_[getPosInVec(file, _VOLTAGE_REGULATOR_SPECIFICATION_FILES_)];
	}
	else
	{
		read(file);
	}
}

Read_Vregulator::Read_Vregulator(const Read_Vregulator &obj)
{
	this->params = obj.params;
	this->names = obj.names;
	this->nonlin_var_names = obj.nonlin_var_names;
	this->constraints = obj.constraints;
	this->bound_mat = obj.bound_mat;
	this->current_mat = obj.current_mat;
	this->pin_relation_mat = obj.pin_relation_mat;
	this->power_input = obj.power_input;
	this->power_output = obj.power_output;
	this->func_input = obj.func_input;
	this->func_output = obj.func_output;
	this->power_input_num = obj.power_input_num;
	this->power_output_num = obj.power_output_num;
	this->func_input_num = obj.func_input_num;
	this->func_output_num = obj.func_output_num;
	this->nonlin_flag = obj.nonlin_flag;
}

// This function reads information from a voltage regulator file
int Read_Vregulator::read(const string &file_name) {

	Electronics::Vregulator myVregulator;

	// file descriptor (only when a file exists)
	int fd = _open(file_name.c_str(), O_RDONLY);

	// check a file existance
	if (fd == -1)
	{
		std::cout << "File created. Please restart the program." << std::endl;
		return 0;
	}
	else
	{
		google::protobuf::io::ZeroCopyInputStream* input = new google::protobuf::io::FileInputStream(fd);

		// check parse state
		if (!google::protobuf::TextFormat::Parse(input, &myVregulator))
		{
			std::cerr << "Fail to parse voltage regulator file" << std::endl;
			return -1;
		}

		delete input;
		_close(fd);
	}

	// extract information
	extract_info(myVregulator);

	std::cout << file_name + " Loading completed" << std::endl;
	return 0;
}

void Read_Vregulator::parameters()
{
	std::cout << "INPUT PIN #: " << params[0] << std::endl;
	std::cout << "ENABLE PIN #: " << params[1] << std::endl;
	std::cout << "ADJ PIN #: " << params[2] << std::endl;
	std::cout << "OUTPUT PIN #: " << params[3] << std::endl;
	std::cout << "GND PIN #: " << params[4] << std::endl;

	if (params[2] != 0)
	{
		std::cout << "REFERENCE VOLTAGE: " << params[5] << std::endl;
		std::cout << "ADJ CURRENT: " << params[6] << std::endl;
		std::cout << "R_LB: " << params[7] << "R_UB" << params[8] << std::endl;
	}

	for (size_t i = 0; i < params[0]; i++)
	{
		std::cout << "INPUT PIN NAME: " << names[i] << std::endl;
	}

	for (size_t i = params[0]; i < params[0] + params[1]; i++)
	{
		std::cout << "ENABLE PIN NAME: " << names[i] << std::endl;
	}

	for (size_t i = params[0] + params[1]; i < params[0] + params[1] + params[2]; i++)
	{
		std::cout << "ADJ PIN NAME: " << names[i] << std::endl;
	}

	for (size_t i = params[0] + params[1] + params[2]; i < params[0] + params[1] + params[2] + params[3]; i++)
	{
		std::cout << "OUTPUT PIN NAME: " << names[i] << std::endl;
	}

	for (size_t i = params[0] + params[1] + params[2] + params[3]; i < params[0] + params[1] + params[2] + params[3] + params[4]; i++)
	{
		std::cout << "GND PIN NAME: " << names[i] << std::endl;
	}

	std::cout << "BOUNDARY CONDITIONS:" << std::endl;
	std::cout << bound_mat << std::endl;

	std::cout << "NONLINEARIRY: " << nonlin_flag << std::endl;
	std::cout << std::endl;

	for (auto &beg = constraints.begin(); beg != constraints.end(); ++beg)
	{
		std::cout << "LINEAR CONSTRAINT:" << std::endl;
		for (auto vnbeg = beg->var_names.begin(); vnbeg != beg->var_names.end(); vnbeg++)
		{
			std::cout << "VARIABLE NAME: " << *vnbeg << std::endl;
		}
		std::cout << "COEFFICIENT: " << beg->coefficients << std::endl;
		std::cout << "CONSTANT: " << beg->constant << std::endl;
		std::cout << "CONSTRAINT TYPE: " << beg->type << std::endl;
		std::cout << std::endl;
	}
}

/********************************************************************/
// This function extract motor info into a certain form
void Motor_Reader::extract_info(const Electronics::Motor &myMotor) {

	// reform the data shape
	params = { myMotor.kt(), myMotor.ke(), myMotor.r() };
	const Electronics::bounds &vin = myMotor.vin(), &i = myMotor.i(),  &torq = myMotor.torq(), &vel = myMotor.vel();

	// boundary conditions 
	MatrixXd temp_mat(4, 2);
	temp_mat.row(0) << vin.lb(), vin.ub();
	temp_mat.row(1) << i.lb(), i.ub();
	temp_mat.row(2) << torq.lb(), torq.ub();
	temp_mat.row(3) << vel.lb(), vel.ub();
	bound_mat = temp_mat;

	current_mat(0, 1) = i.ub();

	// nonlinearity
	nonlin = myMotor.nonlin();

	// inference related parameters
	func_input.second = vin.ub();
	func_input_num = 2;
}

void Motor_Reader::parameters()
{
	cout << "Kt: " << params[0] << endl;
	cout << "Ke: " << params[1] << endl;
	cout << "R: " << params[2] << endl;

	cout << "BOUNDARY CONDITIONS: " << endl;
	cout << bound_mat << endl;

	cout << "NONLINEARITY: " << nonlin << endl;
	cout << endl;
}

Motor_Reader::Motor_Reader(const std::string &file)
{
	if (initialize_all)
	{
		if (_MOTOR_SPECIFICATIONS_VEC_.empty())
		{
			initialize_all = false;
			_MOTOR_SPECIFICATIONS_VEC_ = initializeMotorVec(_MOTOR_SPECIFICATIONS_FILES_);
			initialize_all = true;
		}
		*this = _MOTOR_SPECIFICATIONS_VEC_[getPosInVec(file, _MOTOR_SPECIFICATIONS_FILES_)];
	}
	else
	{
		read(file);
	}
}

Motor_Reader::Motor_Reader(const Motor_Reader &obj)
{
	this->params = obj.params;
	this->bound_mat = obj.bound_mat;
	this->current_mat = obj.current_mat;
	this->power_input = obj.power_input;
	this->power_output = obj.power_output;
	this->func_input = obj.func_input;
	this->func_output = obj.func_output;
	this->power_input_num = obj.power_input_num;
	this->power_output_num = obj.power_output_num;
	this->func_input_num = obj.func_input_num;
	this->func_output_num = obj.func_output_num;
	this->nonlin = obj.nonlin;
}

// This function reads all information about motor in
int Motor_Reader::read(const std::string &file_name) {

	Electronics::Motor myMotor;

	// file descriptor (only when a file exists)
	int fd = _open(file_name.c_str(), O_RDONLY);

	// check file existance 
	if (fd == -1)
	{
		cout << "File doesn't exist. Please create the file and restart the program." << endl;
		return 0;
	}
	else
	{
		google::protobuf::io::ZeroCopyInputStream *input = new google::protobuf::io::FileInputStream(fd);

		// check parse state
		if (!google::protobuf::TextFormat::Parse(input, &myMotor))
		{
			std::cerr << "Fail to parse motor information." << std::endl;
			return -1;
		}
		delete input;
		_close(fd);
	}

	// extract info
	extract_info(myMotor);

	cout << file_name + " Loading completed" << endl;
	return 0;
}

/******************************************************************************************/
// This function extract hbridge info into a certain form
void Hbridge_Reader::extract_info(const Electronics::Hbridge &myHbridge) {

	// vector structure: #power pins|#enable pins|#input pins|#output pins|#gnd pins
	params = { double(myHbridge.pow_pin_num()), double(myHbridge.mot_pin_num()), double(myHbridge.en_pin_num()), 
		double(myHbridge.in_pin_num()), double(myHbridge.out_pin_num()), double(myHbridge.gnd_pin_num()),
		double(myHbridge.other_pin_num())};

	power_input_num = myHbridge.pow_pin_num();
	func_input_num = myHbridge.in_pin_num();
	func_output_num = myHbridge.out_pin_num();

	// matrix structure: power_bds|output_bds
	int range1 = int(params[0] - params[1]), range2 = int(params[0]), range3 = range2 + int(params[2]),
		range4 = range3 + int(params[3]), range5 = range4 + int(params[4]), range6 = range5 + int(params[5]),
		total_pin_num = range6 + params[6];

	// formulate boundary conditions matrix
	MatrixXd temp_vol_mat(total_pin_num+1, 2), temp_cur_mat(total_pin_num, 2), temp_pin_mat(total_pin_num, total_pin_num); // +1 is for logic level
	names.resize(total_pin_num);
	for (auto i = 0; i < total_pin_num + 1; i++)
	{
		if (i < range1)
		{
			const bounds &log_pin_vol = myHbridge.log_pin_v_bounds(i), &log_pin_cur = myHbridge.log_pin_i_bounds(i);
			temp_vol_mat.row(i) << log_pin_vol.lb(), log_pin_vol.ub();
			temp_cur_mat.row(i) << log_pin_cur.lb(), log_pin_cur.ub();

			const string &log_pin_name = myHbridge.log_pin_name(i);
			names[i] = log_pin_name;

			for (auto j = 0; j < total_pin_num; j++)
			{
				temp_pin_mat(i, j) = static_cast<int>(myHbridge.log_pin_row(i)[j] - '0');
			}

			// inference related parameters
			power_input.second = log_pin_vol.ub();
		}
		else if (i >= range1 && i < range2)
		{
			const bounds &mot_pin_vol = myHbridge.mot_pin_v_bounds(i - range1), &mot_pin_cur = myHbridge.mot_pin_i_bounds(i - range1);
			temp_vol_mat.row(i) << mot_pin_vol.lb(), mot_pin_vol.ub();
			temp_cur_mat.row(i) << mot_pin_cur.lb(), mot_pin_cur.ub();

			const string &mot_pin_name = myHbridge.mot_pin_name(i - range1);
			names[i] = mot_pin_name;

			for (auto j = 0; j < total_pin_num; j++)
			{
				temp_pin_mat(i, j) = static_cast<int>(myHbridge.mot_pin_row(i - range1)[j] - '0');
			}

			// inference related parameters
			power_input.second = std::min(power_input.second, mot_pin_vol.ub());
		}
		else if (i >= range2 && i < range3)
		{
			const bounds &en_pin_vol = myHbridge.en_pin_v_bounds(i - range2), &en_pin_cur = myHbridge.en_pin_i_bounds(i - range2);
			temp_vol_mat.row(i) << en_pin_vol.lb(), en_pin_vol.ub();
			temp_cur_mat.row(i) << en_pin_cur.lb(), en_pin_cur.ub();

			const string &en_pin_name = myHbridge.en_pin_name(i - range2);
			names[i] = en_pin_name;

			for (auto j = 0; j < total_pin_num; j++)
			{
				temp_pin_mat(i, j) = static_cast<int>(myHbridge.en_pin_row(i - range2)[j] - '0');
			}

			// inference related parameters
			power_input.second = std::min(power_input.second, en_pin_vol.ub());
		}
		else if (i >= range3 && i < range4)
		{
			const bounds &in_pin_vol = myHbridge.in_pin_v_bounds(i - range3), &in_pin_cur = myHbridge.in_pin_i_bounds(i - range3);
			temp_vol_mat.row(i) << in_pin_vol.lb(), in_pin_vol.ub();
			temp_cur_mat.row(i) << in_pin_cur.lb(), in_pin_cur.ub();

			const string &in_pin_name = myHbridge.in_pin_name(i - range3);
			names[i] = in_pin_name;

			for (auto j = 0; j < total_pin_num; j++)
			{
				temp_pin_mat(i, j) = static_cast<int>(myHbridge.in_pin_row(i - range3)[j] - '0');
			}

			// inference related parameters
			func_input.second = in_pin_vol.ub();
		}
		else if (i >= range4 && i < range5)
		{
			const bounds &out_pin_vol = myHbridge.out_pin_v_bounds(i - range4), &out_pin_cur = myHbridge.out_pin_i_bounds(i - range4);
			temp_vol_mat.row(i) << out_pin_vol.lb(), out_pin_vol.ub();
			temp_cur_mat.row(i) << out_pin_cur.lb(), out_pin_cur.ub();

			const string &out_pin_name = myHbridge.out_pin_name(i - range4);
			names[i] = out_pin_name;

			for (auto j = 0; j < total_pin_num; j++)
			{
				temp_pin_mat(i, j) = static_cast<int>(myHbridge.out_pin_row(i - range4)[j] - '0');
			}

			// inference related parameters
			func_output.first = out_pin_vol.lb();
			func_output.second = out_pin_vol.ub();
		}
		else if (i >= range5 && i < range6)
		{
			const bounds &gnd_pin_vol = myHbridge.gnd_pin_v_bounds(i - range5), &gnd_pin_cur = myHbridge.gnd_pin_i_bounds(i - range5);
			temp_vol_mat.row(i) << gnd_pin_vol.lb(), gnd_pin_vol.ub();
			temp_cur_mat.row(i) << gnd_pin_cur.lb(), gnd_pin_cur.ub();

			const string &gnd_pin_name = myHbridge.gnd_pin_name(i - range5);
			names[i] = gnd_pin_name;

			for (auto j = 0; j < total_pin_num; j++)
			{
				temp_pin_mat(i, j) = static_cast<int>(myHbridge.gnd_pin_row(i - range5)[j] - '0');
			}
		}
		else if (i >= range6 && i < total_pin_num)
		{
			const bounds &other_pin_vol = myHbridge.other_pin_v_bounds(i - range6), &other_pin_cur = myHbridge.other_pin_i_bounds(i - range6);
			temp_vol_mat.row(i) << other_pin_vol.lb(), other_pin_vol.ub();
			temp_cur_mat.row(i) << other_pin_cur.lb(), other_pin_cur.ub();

			const string &other_pin_name = myHbridge.other_pin_name(i - range6);
			names[i] = other_pin_name;

			for (auto j = 0; j < total_pin_num; j++)
			{
				temp_pin_mat(i, j) = static_cast<int>(myHbridge.other_pin_row(i - range6)[j] - '0');
			}
		}
		else
		{
			const bounds &logic_level = myHbridge.logic_level();
			temp_vol_mat.row(i) << logic_level.lb(), logic_level.ub();
		}
	}
	vol_bound_mat = temp_vol_mat;
	cur_bound_mat = temp_cur_mat;
	pin_relation_mat = temp_pin_mat;

	// nonlinearity 
	nonlin_flag = myHbridge.nonlin();

	// constraints
    for (size_t i = 0; i < myHbridge.constraints_size(); i++)
	{
		const Electronics::Linear_Constraint &cons_info = myHbridge.constraints(i);
		stringvec var_names(cons_info.var_name_size());
		MatrixXd coefficients(1, cons_info.coefficient_size());
		double constant = cons_info.constant();
		Electronics::Connection_type type = cons_info.type();
		for (size_t i = 0; i < cons_info.var_name_size(); i++)
		{
			var_names[i] = cons_info.var_name(i);
			coefficients(0, i) = cons_info.coefficient(i);
		}
		constraints.push_back(linear_constraint_expression(var_names, coefficients, constant, type));
	}
}

void Hbridge_Reader::parameters()
{
	int range1 = params[0] - params[1], range2 = params[0], range3 = range2 + params[2],
		range4 = range3 + params[3], range5 = range4 + params[4], range6 = range5 + params[5],
		range7 = range6 + params[6];

	for (size_t i = 0; i < range1; i++)
	{
		cout << "LOGIC PIN: " << names[i] << endl;
		cout << "VOL RANGE: [" << vol_bound_mat(i, 0) << ", " << vol_bound_mat(i, 1) << "]" << endl;
		cout << "CUR RANGE: [" << cur_bound_mat(i, 0) << ", " << cur_bound_mat(i, 1) << "]" << endl;
	}

	for (size_t i = range1; i < range2; i++)
	{
		cout << "MOTOR PIN: " << names[i] << endl;
		cout << "VOL RANGE: [" << vol_bound_mat(i, 0) << ", " << vol_bound_mat(i, 1) << "]" << endl;
		cout << "CUR RANGE: [" << cur_bound_mat(i, 0) << ", " << cur_bound_mat(i, 1) << "]" << endl;
	}

	for (size_t i = range2; i < range3; i++)
	{
		cout << "ENABLE PIN: " << names[i] << endl;
		cout << "VOL RANGE: [" << vol_bound_mat(i, 0) << ", " << vol_bound_mat(i, 1) << "]" << endl;
		cout << "CUR RANGE: [" << cur_bound_mat(i, 0) << ", " << cur_bound_mat(i, 1) << "]" << endl;
	}

	for (size_t i = range3; i < range4; i++)
	{
		cout << "INPUT PIN: " << names[i] << endl;
		cout << "VOL RANGE: [" << vol_bound_mat(i, 0) << ", " << vol_bound_mat(i, 1) << "]" << endl;
		cout << "CUR RANGE: [" << cur_bound_mat(i, 0) << ", " << cur_bound_mat(i, 1) << "]" << endl;
	}

	for (size_t i = range4; i < range5; i++)
	{
		std::cout << "OUTPUT PIN NAME: " << names[i] << std::endl;
		cout << "VOL RANGE: [" << vol_bound_mat(i, 0) << ", " << vol_bound_mat(i, 1) << "]" << endl;
		cout << "CUR RANGE: [" << cur_bound_mat(i, 0) << ", " << cur_bound_mat(i, 1) << "]" << endl;
	}

	for (size_t i = range5; i < range6; i++)
	{
		cout << "GND PIN NAME: " << names[i] << endl;
		cout << "VOL RANGE: [" << vol_bound_mat(i, 0) << ", " << vol_bound_mat(i, 1) << "]" << endl;
		cout << "CUR RANGE: [" << cur_bound_mat(i, 0) << ", " << cur_bound_mat(i, 1) << "]" << endl;
	}

	for (size_t i = range6; i < range7; i++)
	{
		cout << "OTHER PIN NAME: " << names[i] << endl;
		cout << "VOL RANGE: [" << vol_bound_mat(i, 0) << ", " << vol_bound_mat(i, 1) << "]" << endl;
		cout << "CUR RANGE: [" << cur_bound_mat(i, 0) << ", " << cur_bound_mat(i, 1) << "]" << endl;
	}

	cout << endl;
}

int Hbridge_Reader::read(const string& file_name) {

	Electronics::Hbridge myHbridge;
	// file descriptor (only when a file exists)
	int fd = _open(file_name.c_str(), O_RDONLY);

	// check file existance
	if (fd == -1)
	{
		std::cout << "File Created. Please restart the program." << std::endl;
		return 0;
	}
	else
	{
		google::protobuf::io::ZeroCopyInputStream* input = new google::protobuf::io::FileInputStream(fd);

		// check parse state
		if (!google::protobuf::TextFormat::Parse(input, &myHbridge))
		{
			std::cerr << "Fail to parse H-bridge information" << std::endl;
			return -1;
			_close(fd);
		}
	}

	// extract information
	extract_info(myHbridge);

	std::cout << file_name + " Loading completed" << std::endl;
	return 0;
}

Hbridge_Reader::Hbridge_Reader(const string &file)
{
	if (initialize_all)
	{
		if (_HBRIDGE_SPECIFICATIONS_VEC_.empty())
		{
			initialize_all = false;
			_HBRIDGE_SPECIFICATIONS_VEC_ = initializeHbridgeVec(_HBRIDGE_SPECIFICATIONS_FILES_);
			initialize_all = true;
		}
		*this = _HBRIDGE_SPECIFICATIONS_VEC_[getPosInVec(file, _HBRIDGE_SPECIFICATIONS_FILES_)];
	}
	else
	{
		read(file);
	}
}

Hbridge_Reader::Hbridge_Reader(const Hbridge_Reader &obj)
{
	this->params = obj.params;
	this->names = obj.names;
	this->nonlin_var_names = obj.nonlin_var_names;
	this->correspond_input_names = obj.correspond_input_names;
	this->constraints = obj.constraints;
	this->vol_bound_mat = obj.vol_bound_mat;
	this->pin_relation_mat = obj.pin_relation_mat;
	this->cur_bound_mat = obj.cur_bound_mat;
	this->power_input = obj.power_input;
	this->power_output = obj.power_output;
	this->func_input = obj.func_input;
	this->func_output = obj.func_output;
	this->power_input_num = obj.power_input_num;
	this->power_output_num = obj.power_output_num;
	this->func_input_num = obj.func_input_num;
	this->func_output_num = obj.func_output_num;
	this->nonlin_flag = obj.nonlin_flag;
}

// micro-controller part:
// This functiom reads micro-controller information from a .txt file
int Read_Micro_Controller::read(const std::string& file_name) {

	Electronics::MicroController myMicrocontroller;
	// file descriptor
	int fd = _open(file_name.c_str(), _O_RDONLY);

	// check file existance
	if (fd == -1)
	{
		std::cout << "File Created. Please restart the program." << std::endl;
		return 0;
	}
	else
	{
		google::protobuf::io::ZeroCopyInputStream* input = new google::protobuf::io::FileInputStream(fd);

		// check parse states
		if (!google::protobuf::TextFormat::Parse(input, &myMicrocontroller))
		{
			std::cerr << "Failed to parse micro-controller file." << std::endl;
			return -1;
		}
		delete input;
		_close(fd);
	}

	// extract information
	extract_info(myMicrocontroller);

	std::cout << file_name +  " Loading Completed." << std::endl;
	return 0;
}

Read_Micro_Controller::Read_Micro_Controller(const std::string &file)
{
	if (initialize_all)
	{
		if (_MICRO_CONTROLLER_SPECIFICATIONS_VEC_.empty())
		{
			initialize_all = false;
			_MICRO_CONTROLLER_SPECIFICATIONS_VEC_ = initializeMicroControllerVec(_MICRO_CONTROLLER_SPECIFICATIONS_FILES_);
			initialize_all = true;
		}
		*this = _MICRO_CONTROLLER_SPECIFICATIONS_VEC_[getPosInVec(file, _MICRO_CONTROLLER_SPECIFICATIONS_FILES_)];
	}
	else
	{
		read(file);
	}
}

Read_Micro_Controller::Read_Micro_Controller(const Read_Micro_Controller &obj)
{
	this->params = obj.params;
	this->names = obj.names;
	this->bound_mat = obj.bound_mat;
	this->current_mat = obj.current_mat;
	this->pin_relation_mat = obj.pin_relation_mat;
	this->power_input = obj.power_input;
	this->power_output = obj.power_output;
	this->func_input = obj.func_input;
	this->func_output = obj.func_output;
	this->power_input_num = obj.power_input_num;
	this->power_output_num = obj.power_output_num;
	this->func_input_num = obj.func_input_num;
	this->func_output_num = obj.func_output_num;
	this->nonlin_flag = obj.nonlin_flag;
}

// This function parse the information of a micro-controller object based on protobuf
void Read_Micro_Controller::extract_info(const Electronics::MicroController &myMicrocontroller) {

	// numbers of pins information
	params = { double(myMicrocontroller.pow_in_pin_num()), double(myMicrocontroller.pow_out_pin_num()), 
		double(myMicrocontroller.pwm_pin_num()), double(myMicrocontroller.dig_pin_num()), double(myMicrocontroller.gnd_pin_num()), 
		double(myMicrocontroller.etn_itp_pin_num()), double(myMicrocontroller.spi_pin_num()), double(myMicrocontroller.twi_pin_num()), 
		double(myMicrocontroller.adc_in_pin_num()) };

	power_input_num = myMicrocontroller.pow_in_pin_num();
	power_output_num = myMicrocontroller.pow_out_pin_num();
	func_input_num = myMicrocontroller.pwm_pin_num() + myMicrocontroller.dig_pin_num();
	func_output_num = myMicrocontroller.pwm_pin_num() + myMicrocontroller.dig_pin_num();

	unsigned pow_in_pin_num = myMicrocontroller.pow_in_pin_num(),
		pow_out_pin_num = myMicrocontroller.pow_out_pin_num(),
		gnd_pin_num = myMicrocontroller.gnd_pin_num(),
		dig_pin_num = myMicrocontroller.dig_pin_num(),
		pwm_pin_num = myMicrocontroller.pwm_pin_num(),
		ent_itp_pin_num = myMicrocontroller.etn_itp_pin_num(),
		spi_pin_num = myMicrocontroller.spi_pin_num(),
		twi_pin_num = myMicrocontroller.twi_pin_num(),
		adc_in_pin_num = myMicrocontroller.adc_in_pin_num(),
		total_pin_num = pow_in_pin_num + pow_out_pin_num + gnd_pin_num + dig_pin_num +
		pwm_pin_num + ent_itp_pin_num + spi_pin_num + twi_pin_num + adc_in_pin_num,
		used_total_pin_num = pow_in_pin_num + pow_out_pin_num + pwm_pin_num + dig_pin_num + gnd_pin_num;

	// names of pins information
	for (unsigned i = 0; i < pow_in_pin_num; i++)
	{
		const std::string &pow_in_pin = myMicrocontroller.pow_in_pin_name(i);
		names.push_back(pow_in_pin);
	}

	for (unsigned i = 0; i < pow_out_pin_num; i++)
	{
		const std::string &pow_out_pin = myMicrocontroller.pow_out_pin_name(i);
		names.push_back(pow_out_pin);
	}
	for (unsigned i = 0; i < pwm_pin_num; i++)
	{
		const std::string &pwm_pin = myMicrocontroller.pwm_pin_name(i);
		names.push_back(pwm_pin);
	}

	for (unsigned i = 0; i < dig_pin_num; i++)
	{
		const std::string &dig_pin = myMicrocontroller.dig_pin_name(i);
		names.push_back(dig_pin);
	}

	for (unsigned i = 0; i < gnd_pin_num; i++)
	{
		const std::string &gnd_pin = myMicrocontroller.gnd_pin_name(i);
		names.push_back(gnd_pin);
	}

	for (unsigned i = 0; i < ent_itp_pin_num; i++)
	{
		const std::string &ent_ipt_pin = myMicrocontroller.ent_itp_pin_name(i);
		names.push_back(ent_ipt_pin);
	}

	for (unsigned i = 0; i < spi_pin_num; i++)
	{
		const std::string &spi_pin = myMicrocontroller.spi_pin_name(i);
		names.push_back(spi_pin);
	}

	for (unsigned i = 0; i < twi_pin_num; i++)
	{
		const std::string &twi_pin = myMicrocontroller.twi_pin_name(i);
		names.push_back(twi_pin);
	}

	for (unsigned i = 0; i < adc_in_pin_num; i++)
	{
		const std::string &adc_in_pin = myMicrocontroller.adc_in_pin_name(i);
		names.push_back(adc_in_pin);
	}

	// boundary conditions of pins part
	Eigen::MatrixXd temp_vol_mat(total_pin_num+1, 2), temp_cur_mat(total_pin_num, 2), temp_pin_mat(total_pin_num, total_pin_num);
	for (unsigned i = 0; i < pow_in_pin_num; i++)
	{
		const bounds &pow_in_pin_vol = myMicrocontroller.pow_in_pin_v_bounds(i);
		temp_vol_mat.row(i) << pow_in_pin_vol.lb(), pow_in_pin_vol.ub();

		// inference related parameters
		power_input.second = pow_in_pin_vol.ub();
	}

	for (unsigned i = 0; i < pow_out_pin_num; i++)
	{
		const bounds &pow_out_pin_vol = myMicrocontroller.pow_out_pin_v_bounds(i);
		temp_vol_mat.row(i + pow_in_pin_num) << pow_out_pin_vol.lb(), pow_out_pin_vol.ub();

		// inference ralated parameters
		power_output.first = pow_out_pin_vol.lb();
		power_output.second = pow_out_pin_vol.ub();
	}

	for (unsigned i = 0; i < pwm_pin_num; i++)
	{
		const bounds &pwm_pin_vol = myMicrocontroller.pwm_pin_v_bounds(i);
		temp_vol_mat.row(i + pow_in_pin_num + pow_out_pin_num) << pwm_pin_vol.lb(), pwm_pin_vol.ub();
	}

	for (unsigned i = 0; i < dig_pin_num; i++)
	{
		const bounds &dig_pin_vol = myMicrocontroller.dig_pin_v_bounds(i);
		temp_vol_mat.row(i + pow_in_pin_num + pow_out_pin_num + pwm_pin_num) << dig_pin_vol.lb(), dig_pin_vol.ub();

		// inference related parameters
		func_output.first = dig_pin_vol.lb();
		func_output.second = dig_pin_vol.ub();
	}

	for (unsigned i = 0; i < gnd_pin_num; i++)
	{
		const bounds &gnd_pin_vol = myMicrocontroller.gnd_pin_v_bounds(i);
		temp_vol_mat.row(i + pow_in_pin_num + pow_out_pin_num + pwm_pin_num + dig_pin_num) << gnd_pin_vol.lb(), gnd_pin_vol.ub();
	}

	for (unsigned i = 0; i < ent_itp_pin_num; i++)
	{
		const bounds &ent_ipt_pin_vol = myMicrocontroller.ent_itp_pin_v_bounds(i);
		temp_vol_mat.row(i + pow_in_pin_num + pow_out_pin_num + 
			gnd_pin_num + dig_pin_num + pwm_pin_num) << ent_ipt_pin_vol.lb(), ent_ipt_pin_vol.ub();
	}

	for (unsigned i = 0; i < spi_pin_num; i++)
	{
		const bounds &spi_pin_vol = myMicrocontroller.spi_pin_v_bounds(i);
		temp_vol_mat.row(i + pow_in_pin_num + pow_out_pin_num + gnd_pin_num + 
			dig_pin_num + pwm_pin_num + ent_itp_pin_num) << spi_pin_vol.lb(), spi_pin_vol.ub();
	}

	for (unsigned i = 0; i < twi_pin_num; i++)
	{
		const bounds &twi_pin_vol = myMicrocontroller.twi_pin_v_bounds(i);
		temp_vol_mat.row (i + pow_in_pin_num + pow_out_pin_num + gnd_pin_num + 
			         dig_pin_num + pwm_pin_num + ent_itp_pin_num + spi_pin_num) << twi_pin_vol.lb(), twi_pin_vol.ub();
	}

	for (unsigned i = 0; i < adc_in_pin_num; i++)
	{
		const bounds &adc_in_pin_vol = myMicrocontroller.adc_in_pin_v_bounds(i);
		temp_vol_mat.row(i + pow_in_pin_num + pow_out_pin_num + gnd_pin_num + dig_pin_num + 
			     pwm_pin_num + ent_itp_pin_num + spi_pin_num + twi_pin_num) << adc_in_pin_vol.lb(), adc_in_pin_vol.ub();

		// inference related parameters
		func_input.first =  adc_in_pin_vol.lb();
		func_input.second = adc_in_pin_vol.ub();
	}

	const bounds &logic_level = myMicrocontroller.logic_level();
	temp_vol_mat.row(pow_in_pin_num + pow_out_pin_num + gnd_pin_num + dig_pin_num +
		pwm_pin_num + ent_itp_pin_num + spi_pin_num + twi_pin_num + adc_in_pin_num) << logic_level.lb(), logic_level.ub();
	bound_mat = temp_vol_mat;

	// nonlinearity 
	nonlin_flag = myMicrocontroller.nonlin();

	// boundary conditions of pins part
	for (unsigned i = 0; i < pow_in_pin_num; i++)
	{
		if (myMicrocontroller.pow_in_pin_i_bounds_size() != 0)
		{
			const bounds &pow_in_pin_cur = myMicrocontroller.pow_in_pin_i_bounds(i);
			temp_cur_mat.row(i) << pow_in_pin_cur.lb(), pow_in_pin_cur.ub();
		}
	}

	for (unsigned i = 0; i < pow_out_pin_num; i++)
	{
		if (myMicrocontroller.pow_out_pin_i_bounds_size() != 0)
		{
			const bounds &pow_out_pin_cur = myMicrocontroller.pow_out_pin_i_bounds(i);
			temp_cur_mat.row(i + pow_in_pin_num) << pow_out_pin_cur.lb(), pow_out_pin_cur.ub();
		}
	}

	for (unsigned i = 0; i < pwm_pin_num; i++)
	{
		if (myMicrocontroller.pwm_pin_i_bounds_size() != 0)
		{
			const bounds &pwm_pin_cur = myMicrocontroller.pwm_pin_i_bounds(i);
			temp_cur_mat.row(i + pow_in_pin_num + pow_out_pin_num) << pwm_pin_cur.lb(), pwm_pin_cur.ub();
		}
	}

	for (unsigned i = 0; i < dig_pin_num; i++)
	{
		if (myMicrocontroller.dig_pin_i_bounds_size() != 0)
		{
			const bounds &dig_pin_cur = myMicrocontroller.dig_pin_i_bounds(i);
			temp_cur_mat.row(i + pow_in_pin_num + pow_out_pin_num + pwm_pin_num) << dig_pin_cur.lb(), dig_pin_cur.ub();
		}
	}

	for (unsigned i = 0; i < gnd_pin_num; i++)
	{
		if (myMicrocontroller.gnd_pin_i_bounds_size() != 0)
		{
			const bounds &gnd_pin_cur = myMicrocontroller.gnd_pin_i_bounds(i);
			temp_cur_mat.row(i + pow_in_pin_num + pow_out_pin_num + pwm_pin_num + dig_pin_num) << gnd_pin_cur.lb(), gnd_pin_cur.ub();
		}
	}

	for (unsigned i = 0; i < ent_itp_pin_num; i++)
	{
		if (myMicrocontroller.ent_itp_pin_i_bounds_size() != 0)
		{
			const bounds &ent_ipt_pin_cur = myMicrocontroller.ent_itp_pin_i_bounds(i);
			temp_cur_mat.row(i + pow_in_pin_num + pow_out_pin_num + 
				gnd_pin_num + dig_pin_num + pwm_pin_num) << ent_ipt_pin_cur.lb(), ent_ipt_pin_cur.ub();
		}
	}

	for (unsigned i = 0; i < spi_pin_num; i++)
	{
		if (myMicrocontroller.spi_pin_i_bounds_size() != 0)
		{
			const bounds &spi_pin_cur = myMicrocontroller.spi_pin_i_bounds(i);
			temp_cur_mat.row(i + pow_in_pin_num + pow_out_pin_num + 
				gnd_pin_num + dig_pin_num + pwm_pin_num + ent_itp_pin_num) << spi_pin_cur.lb(), spi_pin_cur.ub();
		}
	}

	for (unsigned i = 0; i < twi_pin_num; i++)
	{
		if (myMicrocontroller.twi_pin_i_bounds_size() != 0)
		{
			const bounds &twi_pin_cur = myMicrocontroller.twi_pin_i_bounds(i);
			temp_cur_mat.row(i + pow_in_pin_num + pow_out_pin_num + gnd_pin_num + dig_pin_num +
				pwm_pin_num + ent_itp_pin_num + spi_pin_num) << twi_pin_cur.lb(), twi_pin_cur.ub();
		}
	}

	for (unsigned i = 0; i < adc_in_pin_num; i++)
	{
		if (myMicrocontroller.adc_in_pin_i_bounds_size() != 0)
		{
			const bounds &adc_in_pin_cur = myMicrocontroller.adc_in_pin_i_bounds(i);
			temp_cur_mat.row(i + pow_in_pin_num + pow_out_pin_num + gnd_pin_num + dig_pin_num +
				pwm_pin_num + ent_itp_pin_num + spi_pin_num + twi_pin_num) << adc_in_pin_cur.lb(), adc_in_pin_cur.ub();
		}
	}
	current_mat = temp_cur_mat;

	// pin relation matrix
	for (size_t i = 0; i < pow_in_pin_num; i++)
	{
		for (auto beg = myMicrocontroller.pow_in_pin_row(i).c_str(); *beg != '\0'; beg++)
		{
			temp_pin_mat(i, beg - myMicrocontroller.pow_in_pin_row(i).c_str()) = static_cast<int>(*beg - '0');
		}
	}

	for (size_t i = 0; i < pow_out_pin_num; i++)
	{
		for (auto beg = myMicrocontroller.pow_out_pin_row(i).c_str(); *beg != '\0'; beg++)
		{
			temp_pin_mat(i + pow_in_pin_num, beg - myMicrocontroller.pow_out_pin_row(i).c_str()) = static_cast<int>(*beg - '0');
		}
	}

	for (size_t i = 0; i < pwm_pin_num; i++)
	{
		for (auto beg = myMicrocontroller.pwm_pin_row(i).c_str(); *beg != '\0'; beg++)
		{
			temp_pin_mat(i + pow_in_pin_num + pow_out_pin_num, beg - myMicrocontroller.pwm_pin_row(i).c_str()) = static_cast<int>(*beg - '0');
		}
	}

	for (size_t i = 0; i < dig_pin_num; i++)
	{
		for (auto beg = myMicrocontroller.dig_pin_row(i).c_str(); *beg != '\0'; beg++)
		{
			temp_pin_mat(i+pow_in_pin_num+pow_out_pin_num+pwm_pin_num, beg - myMicrocontroller.dig_pin_row(i).c_str()) = static_cast<int>(*beg - '0');
		}
	}

	for (size_t i = 0; i < gnd_pin_num; i++)
	{
		for (auto beg = myMicrocontroller.gnd_pin_row(i).c_str(); *beg != '\0'; beg++)
		{
			temp_pin_mat(i+pow_in_pin_num+pow_out_pin_num+pwm_pin_num+dig_pin_num, 
				beg-myMicrocontroller.gnd_pin_row(i).c_str()) = static_cast<int>(*beg - '0');
		}
	}
	pin_relation_mat = temp_pin_mat;
}

void Read_Micro_Controller::parameters()
{
	std::cout << "POWER INPUT PIN #: " << params[0] << std::endl;
	std::cout << "POWER OUTPUT PIN #: " << params[1] << std::endl;
	std::cout << "PWM PIN #: " << params[2] << std::endl;
	std::cout << "DIGITAL I/O PIN #: " << params[3] << std::endl;
	std::cout << "GND PIN #: " << params[4] << std::endl;
	std::cout << "EXTERNAL INTERRUPT PIN #: " << params[5] << std::endl;
	std::cout << "SPI PIN #: " << params[6] << std::endl;
	std::cout << "TWI PIN #: " << params[7] << std::endl;
	std::cout << "ADC INPUT PIN #: " << params[8] << std::endl;

	for (size_t i = 0; i < params[0]; i++)
	{
		std::cout << "POWER INPUT PIN NAME: " << names[i] << std::endl;
	}

	for (size_t i = params[0]; i < params[0] + params[1]; i++)
	{
		std::cout << "POWER OUTPUT PIN NAME: " << names[i] << std::endl;
	}

	for (size_t i = params[0] + params[1]; i < params[0] + params[1] + params[2]; i++)
	{
		std::cout << "PWM PIN NAME: " << names[i] << std::endl;
	}

	for (size_t i = params[0] + params[1] + params[2]; i < params[0] + params[1] + params[2] + params[3]; i++)
	{
		std::cout << "DIGITAL I/O PIN NAME: " << names[i] << std::endl;
	}

	for (size_t i = params[0] + params[1] + params[2] + params[3]; i < params[0] + params[1] + params[2] + params[3] + params[4]; i++)
	{
		std::cout << "GND PIN NAME: " << names[i] << std::endl;
	}

	for (size_t i = params[0] + params[1] + params[2] + params[3] + params[4]; i < params[0] + params[1] + params[2] + params[3] 
		+ params[4] + params[5]; i++)
	{
		std::cout << "EXTERNAL INTERRUPT PIN NAME: " << names[i] << std::endl;
	}

	for (size_t i = params[0] + params[1] + params[2] + params[3] + params[4] + params[5]; i < params[0] + 
		params[1] + params[2] + params[3] + params[4] + params[5] + params[6]; i++)
	{
		std::cout << "SPI PIN NAME: " << names[i] << std::endl;
	}

	for (size_t i = params[0] + params[1] + params[2] + params[3] + params[4] + params[5] + params[6]; i < params[0] +
		params[1] + params[2] + params[3] + params[4] + params[5] + params[6] + params[7]; i++)
	{
		std::cout << "TWI PIN NAME: " << names[i] << std::endl;
	}

	for (size_t i = params[0] + params[1] + params[2] + params[3] + params[4] + params[5] + params[6] + params[7]; i < params[0] +
		params[1] + params[2] + params[3] + params[4] + params[5] + params[6] + params[7] + params[8]; i++)
	{
		std::cout << "ADJ INPUT PIN NAME: " << names[i] << std::endl;
	}

	std::cout << "BOUNDARY CONDITION: " << std::endl;
	std::cout << bound_mat << std::endl;

	std::cout << "NONLINEARITY: " << nonlin_flag << std::endl;
	std::cout << std::endl;

}

Battery_Interface::Battery_Interface(const std::string &file)
{
	if (initialize_all)
	{
		if (_BATTERY_SPECIFICATION_VEC_.empty())
		{
			initialize_all = false;
			_BATTERY_SPECIFICATION_VEC_ = initializeBatteryVec(_BATTERY_SPECIFICATION_FILES_);
			initialize_all = true;
		}
		*this = _BATTERY_SPECIFICATION_VEC_[getPosInVec(file, _BATTERY_SPECIFICATION_FILES_)];
	}
	else
	{
		read(file);
	}
}

Battery_Interface::Battery_Interface(const Battery_Interface &obj)
{
	this->params = obj.params;
	this->names = obj.names;
	this->bound_mat = obj.bound_mat;
	this->current_mat = obj.current_mat;
	this->pin_relation_mat = obj.pin_relation_mat;
	this->power_input = obj.power_input;
	this->power_output = obj.power_output;
	this->func_input = obj.func_input;
	this->func_output = obj.func_output;
	this->power_input_num = obj.power_input_num;
	this->power_output_num = obj.power_output_num;
	this->func_input_num = obj.func_input_num;
	this->func_output_num = obj.func_output_num;
	this->nonlin_flag = obj.nonlin_flag;
}

void Battery_Interface::parameters()
{
	std::cout << "POWER PIN #: " << params[0] << std::endl;

	for (size_t i = 0; i < params[0]; i++)
	{
		std::cout << "POWER PIN NAME: " << names[i] << std::endl;
	}

	std::cout << "BOUNDARY CONDITION: (Unit: V)" << std::endl;
	std::cout << bound_mat << std::endl;

	std::cout << "NONLINEARITY: " << nonlin_flag << std::endl;
	std::cout << std::endl;

}

void Battery_Interface::read(const string& file)
{
	Electronics::Battery BatteryObj;
	
	int fd = _open(file.c_str(), _O_RDONLY, _S_IREAD);
	if (fd == -1)
	{
		if (errno == EDOM)
		{
			std::perror("CAN'T DETECT FILE: ");
		}

		std::cout << "File Created. Please restart the program." << std::endl;
		return;
	}
	else
	{
		google::protobuf::io::ZeroCopyInputStream* input = new google::protobuf::io::FileInputStream(fd);

		if (!google::protobuf::TextFormat::Parse(input, &BatteryObj))
		{
			std::cerr << "Fail to parse battery file." << std::endl;
			return;
		}
		delete input;
		_close(fd);
	}

	extract(BatteryObj);

	std::cout << file + " Battery file loaded" << std::endl;

}

void Battery_Interface::extract(const Electronics::Battery &BatteryObj)
{
	power_input_num = BatteryObj.pow_pin_num();
	params.push_back(power_input_num);

	MatrixXd temp_vol_mat(power_input_num, 2), temp_cur_mat(power_input_num, 2), temp_pin_mat(power_input_num, power_input_num);
	for (size_t i = 0; i < power_input_num; i++)
	{
		const bounds &pow_pin_vol = BatteryObj.pow_pin_v_bounds(i);
		temp_vol_mat(i, 0) = pow_pin_vol.lb();
		temp_vol_mat(i, 1) = pow_pin_vol.ub();

		if (i == 0)
		{
			// inference related parameters
			power_output.first = pow_pin_vol.lb();
			power_output.second = pow_pin_vol.ub();
		}
		names.push_back(BatteryObj.pow_pin_name(i));
	}
	bound_mat = temp_vol_mat;

	for (size_t i = 0; i < power_input_num; i++)
	{
		if (BatteryObj.pow_pin_i_bounds_size() != 0)
		{
			const bounds &pow_pin_cur = BatteryObj.pow_pin_i_bounds(i);
			temp_cur_mat.row(i) << pow_pin_cur.lb(), pow_pin_cur.ub();
		}
	}
	current_mat = temp_cur_mat;

	// nonlineariy
	nonlin_flag = BatteryObj.nonlin();
	// pin relation matrix
	for (size_t i = 0; i < power_input_num; i++)
	{
		for (auto beg = BatteryObj.pow_pin_row(i).c_str(); *beg != '\0'; beg++)
		{
			temp_pin_mat(i, beg - BatteryObj.pow_pin_row(i).c_str()) = static_cast<int>(*beg - '0');
		}
	}
	pin_relation_mat = temp_pin_mat;
}

vector<Battery_Interface> initializeBatteryVec(stringvec &files)
{
	vector<Battery_Interface> vec;
	for(auto &entry : directory_iterator(battery_path))
	{
		files.push_back(entry.path().string());
		vec.push_back(Battery_Interface(entry.path().string()));
	}
	return vec;
}

vector<Read_Vregulator> initializeVoltageRegulatorVec(stringvec &files)
{
	vector<Read_Vregulator> vec;
	for (auto &entry : directory_iterator(voltage_regulator_path))
	{
		files.push_back(entry.path().string());
		vec.push_back(Read_Vregulator(entry.path().string()));
	}
	return vec;
}

vector<Read_Micro_Controller> initializeMicroControllerVec(stringvec &files)
{
	vector<Read_Micro_Controller> vec;
	for (auto &entry : directory_iterator(micro_controller_path))
	{
		files.push_back(entry.path().string());
		vec.push_back(Read_Micro_Controller(entry.path().string()));
	}
	return vec;
}

vector<Hbridge_Reader> initializeHbridgeVec(stringvec &files)
{
	vector<Hbridge_Reader> vec;
	for (auto &entry : directory_iterator(hbridge_path))
	{
		files.push_back(entry.path().string());
		vec.push_back(Hbridge_Reader(entry.path().string()));
	}
	return vec;
}

vector<Motor_Reader> initializeMotorVec(stringvec &files)
{
	vector<Motor_Reader> vec;
	for (auto &entry : std::experimental::filesystem::directory_iterator(motor_path))
	{
		files.push_back(entry.path().string());
		vec.push_back(Motor_Reader(entry.path().string()));
	}
	return vec;
}

vector<Encoder_Reader> initializeEncoderVec(stringvec &files)
{
	vector<Encoder_Reader> vec;
	for (auto &entry : directory_iterator(encoder_path))
	{
		files.push_back(entry.path().string());
		vec.push_back(Encoder_Reader(entry.path().string()));
	}
	return vec;
}

vector<ForceSensor_Reader> initializeForceSensorVec(stringvec &files)
{
	vector<ForceSensor_Reader> vec;
	for (auto &entry: directory_iterator(forcesensor_path))
	{
		files.push_back(entry.path().string());
		vec.push_back(ForceSensor_Reader(entry.path().string()));
	}
	return vec;
}

vector<Bluetooth_Reader> initializeBluetoothVec(stringvec &files)
{
	vector<Bluetooth_Reader> vec;
	for (auto &entry: directory_iterator(bluetooth_path))
	{
		files.push_back(entry.path().string());
		vec.push_back(Bluetooth_Reader(entry.path().string()));
	}
	return vec;
}

vector<Camera_Reader> initializeCameraVec(stringvec &files)
{
	vector<Camera_Reader> vec;
	for (auto &entry : directory_iterator(camera_path))
	{
		files.push_back(entry.path().string());
		vec.push_back(Camera_Reader(entry.path().string()));
	}
	return vec;
}

void voltage_regulator_io_test()
{	
	for (const auto &entry : directory_iterator(voltage_regulator_path))
	{
		Read_Vregulator voltage_regulator_obj(entry.path().string());
		voltage_regulator_obj.parameters();
	}
}

void motor_io_test()
{
	for (const auto &entry : directory_iterator(motor_path))
	{
		Motor_Reader motor_obj(entry.path().string());
		motor_obj.parameters();
	}
}

void h_bridge_io_test()
{
	for (const auto &entry : directory_iterator(hbridge_path))
	{
		Hbridge_Reader hbridge_obj(entry.path().string());
		hbridge_obj.parameters();
	}
}

void micro_controller_io_test()
{
	for (const auto &entry : directory_iterator(micro_controller_path))
	{
		Read_Micro_Controller microcontroller_obj(entry.path().string());
		microcontroller_obj.parameters();

	}
}

void battery_io_test()
{
	string &path = std::experimental::filesystem::current_path().string() + "\\Specifications\\BATTERY\\";
	for (const auto &entry : std::experimental::filesystem::directory_iterator(path))
	{
		Battery_Interface obj(entry.path().string());
		obj.parameters();
	}
}

void encoder_io_test()
{
	for (const auto &entry : directory_iterator(encoder_path))
	{
		Encoder_Reader encoder_obj(entry.path().string());
		encoder_obj.parameters();
	}
}

void force_sensor_io_test()
{
	for (const auto &entry : directory_iterator(forcesensor_path))
	{
		ForceSensor_Reader forcesensor_obj(entry.path().string());
		forcesensor_obj.parameters();
	}
}

void bluetooth_io_test()
{
	for (const auto &entry : directory_iterator(bluetooth_path))
	{
		Bluetooth_Reader bluetooth_obj(entry.path().string());
		bluetooth_obj.parameters();
	}
}

void camera_io_test()
{
	for (const auto &entry : directory_iterator(camera_path))
	{
		Camera_Reader camera_obj(entry.path().string());
		camera_obj.parameters();
	}
}

Encoder_Reader::Encoder_Reader(const std::string &file)
{
	if (initialize_all)
	{
		if (_ENCODER_SPECIFICATIONS_VEC_.empty())
		{
			initialize_all = false;
			_ENCODER_SPECIFICATIONS_VEC_ = initializeEncoderVec(_ENCODER_SPECIFICATIONS_FILES_);
			initialize_all = true;
		}
		*this = _ENCODER_SPECIFICATIONS_VEC_[getPosInVec(file, _ENCODER_SPECIFICATIONS_FILES_)];
	}
	else
	{
		read(file);
	}
}

Encoder_Reader::Encoder_Reader(const Encoder_Reader &encoderObj)
{
	this->params = encoderObj.params;
	this->names = encoderObj.names;
	this->vol_bound_mat = encoderObj.vol_bound_mat;
	this->cur_bound_mat = encoderObj.cur_bound_mat;
	this->pin_dep_mat = encoderObj.pin_dep_mat;
	this->pow_in_num = encoderObj.pow_in_num;
	this->pow_out_num = encoderObj.pow_out_num;
	this->func_in_num = encoderObj.func_in_num;
	this->func_out_num = encoderObj.func_out_num;
	this->nonlin = encoderObj.nonlin;
}

void Encoder_Reader::parameters()
{
	int range1 = params[0], range2 = params[0] + params[1], range3 = params[0] + params[1] + params[2],
		range4 = std::accumulate(params.begin(), params.end(), 0);
	for (size_t i = 0; i < range1; i++)
	{
		cout << "POWER PIN: " << names[i] << endl;
		cout << "VOL RANGE: [" << vol_bound_mat(i, 0) << ", " << vol_bound_mat(i, 1) << "]" << endl;
		cout << "CUR RANGE: [" << cur_bound_mat(i, 0) << ", " << cur_bound_mat(i, 1) << "]" << endl;
	}

	for (size_t i = range1; i < range2; i++)
	{
		cout << "MOT PIN: " << names[i] << endl;
		cout << "VOL RANGE: [" << vol_bound_mat(i, 0) << ", " << vol_bound_mat(i, 1) << "]" << endl;
		cout << "CUR RANGE: [" << cur_bound_mat(i, 0) << ", " << cur_bound_mat(i, 1) << "]" << endl;
	}

	for (size_t i = range2; i < range3; i++)
	{
		cout << "SIG PIN: " << names[i] << endl;
		cout << "VOL RANGE: [" << vol_bound_mat(i, 0) << ", " << vol_bound_mat(i, 1) << "]" << endl;
		cout << "CUR RANGE: [" << cur_bound_mat(i, 0) << ", " << cur_bound_mat(i, 1) << "]" << endl;

	}

	for (size_t i = range3; i < range4; i++)
	{
		cout << "GND PIN: " << names[i] << endl;
		cout << "VOL RANGE: [" << vol_bound_mat(i, 0) << ", " << vol_bound_mat(i, 1) << "]" << endl;
		cout << "CUR RANGE: [" << cur_bound_mat(i, 0) << ", " << cur_bound_mat(i, 1) << "]" << endl;
	}

	cout << endl;
}

void Encoder_Reader::read(const std::string &file)
{
	Electronics::Encoder myEncoder;
	
	int fd = _open(file.c_str(), O_RDONLY);
	if (fd == -1)
	{
		cout << "File doesn't exist. Please create the file and restart the program" << endl;
		return;
	}
	else
	{
		ZeroCopyInputStream *input = new FileInputStream(fd);
		if (!TextFormat::Parse(input, &myEncoder))
		{
			cerr << "Failed to parse encoder file" << endl;
			return;
		}
		delete input;
		_close(fd);
	}

	extract(myEncoder);
	cout << file + " Loading Completed" << endl;
	return;
}

void Encoder_Reader::extract(const Electronics::Encoder &encodeObj)
{
	params = { encodeObj.pow_pin_num(), encodeObj.mot_pin_num(), encodeObj.sig_pin_num(), encodeObj.gnd_pin_num() };
	int total_pin_num = std::accumulate(params.begin(), params.end(), 0),
		range1 = params[0], range2 = params[0] + params[1], range3 = params[0] + params[1] + params[2];

	MatrixXd temp_vol_mat(total_pin_num, 2), temp_cur_mat(total_pin_num, 2), temp_pin_mat = MatrixXd::Zero(total_pin_num, total_pin_num);
	names.resize(total_pin_num);
	for (size_t i = 0; i < total_pin_num; i++)
	{
		if (i < range1)
		{
			const bounds &pow_pin_vol = encodeObj.pow_pin_v_bounds(i), &pow_pin_cur = encodeObj.pow_pin_i_bounds(i);
			temp_vol_mat.row(i) << pow_pin_vol.lb(), pow_pin_vol.ub();
			temp_cur_mat.row(i) << pow_pin_cur.lb(), pow_pin_cur.ub();

			const string &pow_pin_name = encodeObj.pow_pin_name(i);
			names[i] = pow_pin_name;

			for (auto j = 0; j < encodeObj.pow_pin_row(i).length(); j++)
			{
				temp_pin_mat(i, j) = static_cast<int>(encodeObj.pow_pin_row(i)[j] - '\0');
			}
		}
		else if (i >= range1 && i < range2)
		{
			const bounds &mot_pin_vol = encodeObj.mot_pin_v_bounds(i - range1), &mot_pin_cur = encodeObj.mot_pin_i_bounds(i - range1);
			temp_vol_mat.row(i) << mot_pin_vol.lb(), mot_pin_vol.ub();
			temp_cur_mat.row(i) << mot_pin_cur.lb(), mot_pin_cur.ub();

			const string &mot_pin_name = encodeObj.mot_pin_name(i - range1);
			names[i] = mot_pin_name;

			for (auto j = 0; j < encodeObj.mot_pin_row(i - range1).length(); j++)
			{
				temp_pin_mat(i, j) = static_cast<int>(encodeObj.mot_pin_row(i - range1)[j] - '\0');
			}
		}
		else if (i >= range2 && i < range3)
		{
			const bounds &sig_pin_vol = encodeObj.sig_pin_v_bounds(i - range2), &sig_pin_cur = encodeObj.sig_pin_i_bounds(i - range2);
			temp_vol_mat.row(i) << sig_pin_vol.lb(), sig_pin_vol.ub();
			temp_cur_mat.row(i) << sig_pin_cur.lb(), sig_pin_cur.ub();

			const string &sig_pin_name = encodeObj.sig_pin_name(i - range2);
			names[i] = sig_pin_name;

			for (auto j = 0; j < encodeObj.sig_pin_row(i - range2).length(); j++)
			{
				temp_pin_mat(i, j) = static_cast<int>(encodeObj.sig_pin_row(i - range2)[j] - '\0');
			}
		}
		else
		{
			const bounds &gnd_pin_vol = encodeObj.gnd_pin_v_bounds(i - range3), &gnd_pin_cur = encodeObj.gnd_pin_i_bounds(i - range3);
			temp_vol_mat.row(i) << gnd_pin_vol.lb(), gnd_pin_vol.ub();
			temp_cur_mat.row(i) << gnd_pin_cur.lb(), gnd_pin_cur.ub();

			const string &gnd_pin_name = encodeObj.gnd_pin_name(i - range3);
			names[i] = gnd_pin_name;

			for (auto j = 0; j < encodeObj.gnd_pin_row(i - range3).length(); j++)
			{
				temp_pin_mat(i, j) = static_cast<int>(encodeObj.gnd_pin_row(i - range3)[j] - '\0');
			}
		}
	}
	vol_bound_mat = temp_vol_mat;
	cur_bound_mat = temp_cur_mat;
	pin_dep_mat = temp_pin_mat;

	nonlin = encodeObj.nonlin();
}


Bluetooth_Reader::Bluetooth_Reader(const std::string &file)
{
	if (initialize_all)
	{
		if (_BLUETOOTH_SPECIFICATIONS_VEC_.empty())
		{
			initialize_all = false;
			_BLUETOOTH_SPECIFICATIONS_VEC_ = initializeBluetoothVec(_BLUETOOTH_SPECIFICATIONS_FILES_);
			initialize_all = true;
		}
		*this = _BLUETOOTH_SPECIFICATIONS_VEC_[getPosInVec(file, _BLUETOOTH_SPECIFICATIONS_FILES_)];
	}
	else
	{
		read(file);
	}
}

Bluetooth_Reader::Bluetooth_Reader(const Bluetooth_Reader &bluetoothObj)
{
	this->params = bluetoothObj.params;
	this->names = bluetoothObj.names;
	this->vol_bound_mat = bluetoothObj.vol_bound_mat;
	this->cur_bound_mat = bluetoothObj.cur_bound_mat;
	this->pin_dep_mat = bluetoothObj.pin_dep_mat;
	this->pow_in_num = bluetoothObj.pow_in_num;
	this->pow_out_num = bluetoothObj.pow_out_num;
	this->func_in_num = bluetoothObj.func_in_num;
	this->func_out_num = bluetoothObj.func_out_num;
	this->nonlin = bluetoothObj.nonlin;
}

void Bluetooth_Reader::parameters()
{
	int range1 = params[0], range2 = range1 + params[1], range3 = range2 + params[2],
		range4 = range3 + params[3];
	
	for (size_t i = 0; i < range1; i++)
	{
		cout << "POW PIN: " << names[i] << endl;
		cout << "VOL RANGE: [" << vol_bound_mat(i, 0) << ", " << vol_bound_mat(i, 1) << "]" << endl;
		cout << "CUR RANGE: [" << cur_bound_mat(i, 0) << ", " << cur_bound_mat(i, 1) << "]" << endl;
	}

	for (size_t i = range1; i < range2; i++)
	{
		cout << "SIG PIN: " << names[i] << endl;
		cout << "VOL RANGE: [" << vol_bound_mat(i, 0) << ", " << vol_bound_mat(i, 1) << "]" << endl;
		cout << "CUR RANGE: [" << cur_bound_mat(i, 0) << ", " << cur_bound_mat(i, 1) << "]" << endl;
	}

	for (size_t i = range2; i < range3; i++)
	{
		cout << "GND PIN: " << names[i] << endl;
		cout << "VOL RANGE: [" << vol_bound_mat(i, 0) << ", " << vol_bound_mat(i, 1) << "]" << endl;
		cout << "CUR RNAGE: [" << cur_bound_mat(i, 0) << ", " << cur_bound_mat(i, 1) << "]" << endl;
	}

	for (size_t i = range3; i < range4; i++)
	{
		cout << "OTHER PIN: " << names[i] << endl;
		cout << "VOL RANGE: [" << vol_bound_mat(i, 0) << ", " << vol_bound_mat(i, 1) << "]" << endl;
		cout << "CUR RANGE: [" << cur_bound_mat(i, 0) << ", " << cur_bound_mat(i, 1) << "]" << endl;
 	}

	cout << endl;
}

void Bluetooth_Reader::read(const std::string &file)
{
	Electronics::Bluetooth myBluetooth;
	int fd = _open(file.c_str(), O_RDONLY);

	if (fd == -1)
	{
		cout << "File doesn't exist. Please create the file and restart the program" << endl;
		return;
	}
	else
	{
		ZeroCopyInputStream *input = new FileInputStream(fd);
		if (!TextFormat::Parse(input, &myBluetooth))
		{
			cerr << "Failed to parse bluetooth file" << endl;
			return;
		}
		delete input;
		_close(fd);
	}

	extract(myBluetooth);
	cout << file + " Loading completed" << endl;
	return;

}

void Bluetooth_Reader::extract(const Electronics::Bluetooth &bluetoothObj)
{
	params = { bluetoothObj.pow_pin_num(), bluetoothObj.sig_pin_num(), bluetoothObj.gnd_pin_num(), bluetoothObj.other_pin_num() };
	int total_pin_num = std::accumulate(params.begin(), params.end(), 0), range1 = params[0], range2 = range1 + params[1],
		range3 = range2 + params[2];

	MatrixXd temp_vol_mat(total_pin_num, 2), temp_cur_mat(total_pin_num, 2), temp_pin_mat(total_pin_num, total_pin_num);
	names.resize(total_pin_num);
	for (size_t i = 0; i < total_pin_num; i++)
	{
		if (i < range1)
		{
			const bounds &pow_pin_vol = bluetoothObj.pow_pin_v_bounds(i), &pow_pin_cur = bluetoothObj.pow_pin_i_bounds(i);
			temp_vol_mat.row(i) << pow_pin_vol.lb(), pow_pin_vol.ub();
			temp_cur_mat.row(i) << pow_pin_cur.lb(), pow_pin_cur.ub();

			const string &pow_pin_name = bluetoothObj.pow_pin_name(i);
			names[i] = pow_pin_name;

			for (size_t j = 0; j < bluetoothObj.pow_pin_row(i).length(); j++)
			{
				temp_pin_mat(i, j) = static_cast<int>(bluetoothObj.pow_pin_row(i)[j] - '\0');
			}
		}
		else if (i >= range1 && i < range2)
		{
			const bounds &sig_pin_vol = bluetoothObj.sig_pin_v_bounds(i - range1), &sig_pin_cur = bluetoothObj.sig_pin_i_bounds(i - range1);
			temp_vol_mat.row(i) << sig_pin_vol.lb(), sig_pin_vol.ub();
			temp_cur_mat.row(i) << sig_pin_cur.lb(), sig_pin_cur.ub();

			const string &sig_pin_name = bluetoothObj.sig_pin_name(i - range1);
			names[i] = sig_pin_name;

			for (size_t j = 0; j < bluetoothObj.sig_pin_row(i - range1).length(); j++)
			{
				temp_pin_mat(i, j) = static_cast<int>(bluetoothObj.sig_pin_row(i - range1)[j] - '\0');
			}
		}
		else if (i >= range2 && i < range3)
		{
			const bounds &gnd_pin_vol = bluetoothObj.gnd_pin_v_bounds(i - range2), &gnd_pin_cur = bluetoothObj.gnd_pin_i_bounds(i - range2);
			temp_vol_mat.row(i) << gnd_pin_vol.lb(), gnd_pin_vol.ub();
			temp_cur_mat.row(i) << gnd_pin_cur.lb(), gnd_pin_cur.ub();

			const string &gnd_pin_name = bluetoothObj.gnd_pin_name(i - range2);
			names[i] = gnd_pin_name;

			for (size_t j = 0; j < bluetoothObj.gnd_pin_row(i - range2).length(); j++)
			{
				temp_pin_mat(i, j) = static_cast<int>(bluetoothObj.gnd_pin_row(i - range2)[j] - '\0');
			}
		}
		else
		{
			const bounds &other_pin_vol = bluetoothObj.other_pin_v_bounds(i - range3),
				&other_pin_cur = bluetoothObj.other_pin_i_bounds(i - range3);
			temp_vol_mat.row(i) << other_pin_vol.lb(), other_pin_vol.ub();
			temp_cur_mat.row(i) << other_pin_cur.lb(), other_pin_cur.ub();

			const string &other_pin_name = bluetoothObj.other_pin_name(i - range3);
			names[i] = other_pin_name;

			for (size_t j = 0; j < bluetoothObj.other_pin_row(i - range3).length(); j++)
			{
				temp_pin_mat(i, j) = static_cast<int>(bluetoothObj.other_pin_row(i - range3)[j] - '\0');
			}
		}
	}
	vol_bound_mat = temp_vol_mat;
	cur_bound_mat = temp_cur_mat;
	pin_dep_mat = temp_pin_mat;

	nonlin = bluetoothObj.nonlin();
}


Camera_Reader::Camera_Reader(const std::string &file)
{
	if (initialize_all)
	{
		if (_CAMERA_SPECIFICATIONS_VEC_.empty())
		{
			initialize_all = false;
			_CAMERA_SPECIFICATIONS_VEC_ = initializeCameraVec(_CAMERA_SPECIFICATIONS_FILES_);
			initialize_all = true;
		}
		*this = _CAMERA_SPECIFICATIONS_VEC_[getPosInVec(file, _CAMERA_SPECIFICATIONS_FILES_)];
	}
	else
	{
		read(file);
	}
}

Camera_Reader::Camera_Reader(const Camera_Reader &cameraObj)
{
	this->params = cameraObj.params;
	this->names = cameraObj.names;
	this->vol_bound_mat = cameraObj.vol_bound_mat;
	this->cur_bound_mat = cameraObj.cur_bound_mat;
	this->pin_dep_mat = cameraObj.pin_dep_mat;
	this->pow_in_num = cameraObj.pow_in_num;
	this->pow_out_num = cameraObj.pow_out_num;
	this->func_in_num = cameraObj.func_in_num;
	this->func_out_num = cameraObj.func_out_num;
	this->nonlin = cameraObj.nonlin;
}

void Camera_Reader::parameters()
{
	int range1 = params[0], range2 = range1 + params[1], range3 = range2 + params[2],
		range4 = range3 + params[3];

	for (size_t i = 0; i < range1; i++)
	{
		cout << "POW PIN: " << names[i] << endl;
		cout << "VOL RANGE: [" << vol_bound_mat(i, 0) << ", " << vol_bound_mat(i, 1) << "]" << endl;
		cout << "CUR RANGE: [" << cur_bound_mat(i, 0) << ", " << cur_bound_mat(i, 1) << "]" << endl;
	}

	for (size_t i = range1; i < range2; i++)
	{
		cout << "SIG PIN: " << names[i] << endl;
		cout << "VOL RANGE: [" << vol_bound_mat(i, 0) << ", " << vol_bound_mat(i, 1) << "]" << endl;
		cout << "CUR_RANGE: [" << cur_bound_mat(i, 0) << ", " << cur_bound_mat(i, 1) << "]" << endl;
	}

	for (size_t i = range2; i < range3; i++)
	{
		cout << "GND PIN: " << names[i] << endl;
		cout << "VOL RANGE: [" << vol_bound_mat(i, 0) << ", " << vol_bound_mat(i, 1) << "]" << endl;
		cout << "CUR RANGE: [" << cur_bound_mat(i, 0) << ", " << cur_bound_mat(i, 1) << "]" << endl;
	}

	for (size_t i = range3; i < range4; i++)
	{
		cout << "OTHER PIN: " << names[i] << endl;
		cout << "VOL RANGE: [" << vol_bound_mat(i, 0) << ", " << vol_bound_mat(i, 1) << "]" << endl;
		cout << "CUR RANGE: [" << cur_bound_mat(i, 0) << ", " << cur_bound_mat(i, 1) << "]" << endl;
	}

	cout << endl;
}

void Camera_Reader::read(const std::string &file)
{
	Electronics::Camera myCamera;
	int fd = _open(file.c_str(), O_RDONLY);

	if (fd == -1)
	{
		cout << "File doesn't exist. Please create the file and restart the program" << endl;
		return;
	}
	else
	{
		ZeroCopyInputStream *input = new FileInputStream(fd);
		if (!TextFormat::Parse(input, &myCamera))
		{
			cerr << "Failed to parse bluetooth file" << endl;
			return;
		}
		delete(input);
		_close(fd);
	}

	extract(myCamera);
	cout << file + "Loading completed" << endl;
	return;
}

void Camera_Reader::extract(const Electronics::Camera &cameraObj)
{
	params = { cameraObj.pow_pin_num(), cameraObj.sig_pin_num(), cameraObj.gnd_pin_num(), cameraObj.other_pin_num() };
	int total_pin_num = std::accumulate(params.begin(), params.end(), 0), range1 = params[0], range2 = range1 + params[1],
		range3 = range2 + params[2];

	MatrixXd temp_vol_mat(total_pin_num, 2), temp_cur_mat(total_pin_num, 2), temp_pin_mat(total_pin_num, total_pin_num);
	names.resize(total_pin_num);
	for (size_t i = 0; i < total_pin_num; i++)
	{
		if (i < range1)
		{
			const bounds &pow_pin_vol = cameraObj.pow_pin_v_bounds(i), &pow_pin_cur = cameraObj.pow_pin_i_bounds(i);
			temp_vol_mat.row(i) << pow_pin_vol.lb(), pow_pin_vol.ub();
			temp_cur_mat.row(i) << pow_pin_cur.lb(), pow_pin_cur.ub();

			const string pow_pin_name = cameraObj.pow_pin_name(i);
			names[i] = pow_pin_name;

			for (size_t j = 0; j < cameraObj.pow_pin_row(i).length(); j++)
			{
				temp_pin_mat(i, j) = static_cast<int>(cameraObj.pow_pin_row(i)[j] - '\0');
			}
		}
		else if (i >= range1 && i < range2)
		{
			const bounds &sig_pin_vol = cameraObj.sig_pin_v_bounds(i - range1), &sig_pin_cur = cameraObj.sig_pin_i_bounds(i - range1);
			temp_vol_mat.row(i) << sig_pin_vol.lb(), sig_pin_vol.ub();
			temp_cur_mat.row(i) << sig_pin_cur.lb(), sig_pin_cur.ub();

			const string sig_pin_name = cameraObj.sig_pin_name(i - range1);
			names[i] = sig_pin_name;

			for (size_t j = 0; j < cameraObj.sig_pin_row(i - range1).length(); j++)
			{
				temp_pin_mat(i, j) = static_cast<int>(cameraObj.sig_pin_row(i - range1)[j] - '\0');
			}
		}
		else if (i >= range2 && i < range3)
		{
			const bounds &gnd_pin_vol = cameraObj.gnd_pin_v_bounds(i - range2), &gnd_pin_cur = cameraObj.gnd_pin_i_bounds(i - range2);
			temp_vol_mat.row(i) << gnd_pin_vol.lb(), gnd_pin_vol.ub();
			temp_cur_mat.row(i) << gnd_pin_cur.lb(), gnd_pin_cur.ub();

			const string &gnd_pin_name = cameraObj.gnd_pin_name(i - range2);
			names[i] = gnd_pin_name;

			for (size_t j = 0; j < cameraObj.gnd_pin_row(i - range2).length(); j++)
			{
				temp_pin_mat(i, j) = static_cast<int>(cameraObj.gnd_pin_row(i - range2)[j] - '\0');
			}
		}
		else
		{
			const bounds &other_pin_vol = cameraObj.other_pin_v_bounds(i - range3), &other_pin_cur = cameraObj.other_pin_i_bounds(i - range3);
			temp_vol_mat.row(i) << other_pin_vol.lb(), other_pin_vol.ub();
			temp_cur_mat.row(i) << other_pin_cur.lb(), other_pin_cur.ub();

			const string other_pin_name = cameraObj.other_pin_name(i - range3);
			names[i] = other_pin_name;

			for (size_t j = 0; j < cameraObj.other_pin_row(i - range3).length(); j++)
			{
				temp_pin_mat(i, j) = static_cast<int>(cameraObj.other_pin_row(i - range3)[j] - '\0');
			}
		}
	}
	vol_bound_mat = temp_vol_mat;
	cur_bound_mat = temp_cur_mat;
	pin_dep_mat = temp_pin_mat;

	nonlin = cameraObj.nonlin();
}


ForceSensor_Reader::ForceSensor_Reader(const std::string &file)
{
	if (initialize_all)
	{
		if (_FORCESENSOR_SPECIFICATIONS_VEC_.empty())
		{
			initialize_all = false;
			_FORCESENSOR_SPECIFICATIONS_VEC_ = initializeForceSensorVec(_FORCESENSOR_SPECIFICATIONS_FILES_);
			initialize_all = true;
		}
		*this = _FORCESENSOR_SPECIFICATIONS_VEC_[getPosInVec(file, _FORCESENSOR_SPECIFICATIONS_FILES_)];
	}
	else
	{
		read(file);
	}
}

ForceSensor_Reader::ForceSensor_Reader(const ForceSensor_Reader &forcesensorObj)
{
	this->params = forcesensorObj.params;
	this->names = forcesensorObj.names;
	this->vol_bound_mat = forcesensorObj.vol_bound_mat;
	this->cur_bound_mat = forcesensorObj.cur_bound_mat;
	this->pin_dep_mat = forcesensorObj.pin_dep_mat;
	this->pow_in_num = forcesensorObj.pow_in_num;
	this->pow_out_num = forcesensorObj.pow_out_num;
	this->func_in_num = forcesensorObj.func_in_num;
	this->func_out_num = forcesensorObj.func_out_num;
	this->nonlin = forcesensorObj.nonlin;
}

void ForceSensor_Reader::parameters()
{
	for (size_t i = 0; i < params[0]; i++)
	{
		cout << "PIN NAME: " << names[i] << endl;
		cout << "VOL RANGE: [" << vol_bound_mat(i, 0) << ", " << vol_bound_mat(i, 1) << "]" << endl;
		cout << "CUR RANGE: [" << cur_bound_mat(i, 0) << ", " << cur_bound_mat(i, 1) << "]" << endl;
	}

	cout << endl;
}

void ForceSensor_Reader::read(const std::string &file)
{
	Electronics::Force_Sensor myForceSensor;
	int fd = _open(file.c_str(), _O_RDONLY);

	if (fd == -1)
	{
		cout << "File doesn't exist. Please create the file and restart the program" << endl;
		return;
	}
	else
	{
		ZeroCopyInputStream *input = new FileInputStream(fd);
		if (!TextFormat::Parse(input, &myForceSensor))
		{
			cerr << "Failed to parse force sensor file" << endl;
			return;
		}
		delete(input);
		_close(fd);
	}

	extract(myForceSensor);
	cout << file + "Loading completed" << endl;
	return;
}

void ForceSensor_Reader::extract(const Electronics::Force_Sensor &forcesensorObj)
{
	params = { forcesensorObj.pow_pin_num() };
	int total_pin_num = params[0];

	MatrixXd temp_vol_mat(total_pin_num, 2), temp_cur_mat(total_pin_num, 2), temp_pin_mat(total_pin_num, total_pin_num);
	names.resize(total_pin_num);
	for (size_t i = 0; i < total_pin_num; i++)
	{
		const bounds &pow_pin_vol = forcesensorObj.pow_pin_v_bounds(i), &pow_pin_cur = forcesensorObj.pow_pin_i_bounds(i);
		temp_vol_mat.row(i) << pow_pin_vol.lb(), pow_pin_vol.ub();
		temp_cur_mat.row(i) << pow_pin_cur.lb(), pow_pin_cur.ub();

		const string pow_pin_name = forcesensorObj.pow_pin_name(i);
		names[i] = pow_pin_name;

		for (size_t j = 0; j < forcesensorObj.pow_pin_row(i).length(); j++)
		{
			temp_pin_mat(i, j) = static_cast<int>(forcesensorObj.pow_pin_row(i)[j] - '\0');
		}
	}
	vol_bound_mat = temp_vol_mat;
	cur_bound_mat = temp_cur_mat;
	pin_dep_mat = temp_pin_mat;

	nonlin = forcesensorObj.nonlin();
}

