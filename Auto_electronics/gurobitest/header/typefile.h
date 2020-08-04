#pragma once
#ifndef TYPE_FILE
#define TYPE_FILE
#include <string>
#include <vector>
#include <tuple>
#include <unordered_map>
#include <experimental/filesystem>


//extern struct CurrentTracker;
//extern struct CopyCurrentTracker;

//using compstructptrvec = std::vector<Connection_Structure*>;

// using bbglobalmap = std::vector<std::tuple<BBNode, unsigned>>;

//using cctrackvec = std::vector<CopyCurrentTracker>;
//using cctrackvec2d = std::vector<cctrackvec>;
//using ctrac = std::shared_ptr<CurrentTracker>;
//using ctrackvec = std::vector<ctrac>;


using intvec = std::vector<int>;
using intvec2d = std::vector<intvec>;

using unsignedvec = std::vector<unsigned>;
using unsignedvec2d = std::vector<unsignedvec>;
using unsignedpair = std::pair<unsigned, unsigned>;
using unsignedpairs = std::vector<unsignedpair>;

using stringpair = std::pair<std::string, std::string>;
using stringvec = std::vector<std::string>;
using stringvec2d = std::vector<stringvec>;
using doublevec = std::vector<double>;
using doublevec2d = std::vector<doublevec>;
using doublevec3d = std::vector<doublevec2d>;
using doublepair = std::pair<double, double>;
using doublepairs = std::vector<doublepair>;
using doublepairs2d = std::vector<doublepairs>;

using connect_relation = std::vector<std::pair<unsigned, unsigned>>;
using componentspecs = std::vector<std::pair<std::vector<std::string>, std::vector<unsigned>>>;
using initnode = std::pair<std::vector<std::string>, std::vector<unsigned>>;
using cliquetype = std::tuple<doublepairs, std::vector<intvec>>;

using str_strvec_map = std::unordered_map<std::string, std::vector<std::string>>;
using str_unsigned_uomap = std::unordered_map<std::string, unsigned>;

using boolvec = std::vector<bool>;
using testinput = std::pair<doublepairs, doublepairs>;

using connection_relation = std::vector<std::pair<std::pair<std::string, std::string>, std::pair<std::string, std::string>>>;
using connection_relation_vec = std::vector<connection_relation>;
using connection_relation_vec2d = std::vector<connection_relation_vec>;

// file path
const std::string current_path = std::experimental::filesystem::current_path().string(),
	dc_motor_path = current_path + "\\Electronic_Components\\DC_MOTOR\\",
	battery_path = current_path + "\\Electronic_Components\\BATTERY\\",
	h_bridge_path = current_path + "\\Electronic_Components\\H_BRIDGE\\",
	micro_controller_path = current_path + "\\Electronic_Components\\MICRO_CONTROLLER\\",
	voltage_regulator_path = current_path + "\\Electronic_Components\\VOLTAGE_REGULATOR\\",
	encoder_path = current_path + "\\Electronic_Components\\ENCODER\\",
	bluetooth_path = current_path + "\\Electronic_Components\\BLUETOOTH\\",
	camera_path = current_path + "\\Electronic_Components\\CAMERA\\",
	forcesensor_path = current_path + "\\Electronic_Components\\FORCE_SENSOR\\",
	servo_path = current_path + "\\Electronic_Components\\SERVO\\";
// design_path = current_path + "\\Designs\\";
// design_path = current_path + "\\files\\";

// prefix, postfix
const std::string design_pf = "circuit_specs",
text_pf = "_.txt",
raw_text_pf = "_raw.txt";

// separation line
const std::string sline = "----------------------------------------";

const std::string duty = "DUTY";

const unsigned IN_BATTERY_CONNECTION_FOLDER = 0,
IN_VOLTAGE_REGULATOR_CONNECTION_FOLDER = 1,
IN_HBRIDGE_CONNECTION_FOLDER = 2,
IN_MICRO_CONTROLLER_CONNECTION_FOLDER = 3,
IN_MOTOR_CONNECTION_FOLDER = 4,
IN_ENCODER_CONNECTION_FOLDER = 5,
IN_FORCESENSOR_CONNECTION_FOLDER = 6,
IN_BLUETOOTH_CONNECTION_FOLDER = 7,
IN_CAMERA_CONNECTION_FOLDER = 8;

const std::string FILE_DOES_NOT_EXSIT = "File doesn't exit. Please create the file and restart the program.",
CAN_NOT_PARSE_FILE = "Fail to parse file: ",
FILE_LOADED = "Has loaded file: ",
MODEL_IS_INFEASIBLE = "The optimization model is infeasible.",
MODEL_SOLVED = "The optimization model is solved",
INTERSECT_ERROR = "Minimum value is greated than maximum value",
COMPONENT_NOT_FOUND = "This component is not found: ";

const double tol = 1e-3;

#endif // !
