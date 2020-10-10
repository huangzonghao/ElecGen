#pragma once
#ifndef TYPE_FILE
#define TYPE_FILE
#include <string>
#include <vector>
#include <tuple>
#include <unordered_map>
#include <filesystem>
#include "data_dir_path.h"


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
const std::string data_root(std::string(DATA_DIRECTORY_PATH) + "/");
const std::string elec_root(data_root + "electronics/");

const std::string
    dc_motor_path          (elec_root + "DC_MOTOR/"),
    battery_path           (elec_root + "BATTERY/"),
    h_bridge_path          (elec_root + "H_BRIDGE/"),
    micro_controller_path  (elec_root + "MICRO_CONTROLLER/"),
    voltage_regulator_path (elec_root + "VOLTAGE_REGULATOR/"),
    encoder_path           (elec_root + "ENCODER/"),
    bluetooth_path         (elec_root + "BLUETOOTH/"),
    camera_path            (elec_root + "CAMERA/"),
    forcesensor_path       (elec_root + "FORCE_SENSOR/"),
    servo_path             (elec_root + "SERVO/"),
    design_path            (data_root + "Designs/");


// prefix, postfix
const std::string design_pf = "circuit_specs",
text_pf = "_.txt",
raw_text_pf = "_raw.txt";

// separation line
const std::string sline = "----------------------------------------";

const std::string duty = "DUTY";

const std::string FILE_DOES_NOT_EXSIT = "File doesn't exit. Please create the file and restart the program.",
CAN_NOT_PARSE_FILE = "Fail to parse file: ",
FILE_LOADED = "Has loaded file: ",
MODEL_IS_INFEASIBLE = "The optimization model is infeasible.",
MODEL_SOLVED = "The optimization model is solved",
INTERSECT_ERROR = "Minimum value is greated than maximum value",
COMPONENT_NOT_FOUND = "This component is not found: ",
EMPTY_TORQUE = "Input torques empty",
EMPTY_VELOCITY = "Input velocities empty",
TORQUE_VELOCITY_SIZE_NOT_MATCH = "input torques and velocities size doesn't match",
TORQUE_RANGE_INVALID = "Minimum torque is greater than maximum torque",
NEGATIVE_TORQUE = "input torque is negative",
VELOCITY_RANGE_INVALID = "Minimum velocity is greater than maximum velocity",
NEGATIVE_VELOCITY = "input velocity is negative";


const double tol = 1e-3;

#endif // !
