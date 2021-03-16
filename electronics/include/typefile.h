#ifndef TYPEFILE_H_LGAK8JKF
#define TYPEFILE_H_LGAK8JKF

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
const std::string data_root(ELECGEN_DATA_PATH);
const std::string output_root(ELECGEN_OUTPUT_PATH);
const std::string elec_root(data_root + "/electronics/");

const std::string
    dc_motor_path          (std::filesystem::path(elec_root + "/DC_Motor/").string()),
    battery_path           (std::filesystem::path(elec_root + "/BATTERY/").string()),
    h_bridge_path          (std::filesystem::path(elec_root + "/H_BRIDGE/").string()),
    micro_controller_path  (std::filesystem::path(elec_root + "/MICRO_CONTROLLER/").string()),
    voltage_regulator_path (std::filesystem::path(elec_root + "/VOLTAGE_REGULATOR/").string()),
    encoder_path           (std::filesystem::path(elec_root + "/ENCODER/").string()),
    bluetooth_path         (std::filesystem::path(elec_root + "/BLUETOOTH/").string()),
    camera_path            (std::filesystem::path(elec_root + "/CAMERA/").string()),
    forcesensor_path       (std::filesystem::path(elec_root + "/FORCE_SENSOR/").string()),
    servo_path             (std::filesystem::path(elec_root + "/SERVO/").string()),
    design_path            (std::filesystem::path(output_root + "/Designs/").string());


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

#endif /* end of include guard: TYPEFILE_H_LGAK8JKF */
