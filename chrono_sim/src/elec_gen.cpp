#include <iostream>
#include <Eigen/Core>
#include "SimulationManager.h"
#include "Inference.h"

#include "testfile.h"

#include "data_dir_path.h"

const double s_friction = 2.0;
const double k_friction = 1.9;
const std::string env_file = std::string(ELECGEN_DATA_PATH) + "maps/env1.bmp";
const std::string urdf_file = std::string(ELECGEN_DATA_PATH) + "robots/fourleg3.urdf";

int main(int argc, char *argv[]) {

    motor_read();
    h_bridge_read();
    battery_read();
    micro_controller_read();
    force_sensor_read();
    voltage_regulator_read();
    servo_read();
    camera_read();
    bluetooth_read();
    encoder_read();

    SimulationManager sm;
    sm.SetTimeout(50);
    sm.SetUrdfFile(urdf_file);
    sm.SetEnv(env_file, 1, 1, 0.08);
    sm.SetFrictionK(k_friction);
    sm.SetFrictionS(s_friction);

    sm.AddMotor("SERVO", "chassis", "chassis-fl_cyl",    0.01, 0.01, 0.01, 0.01);
    sm.AddMotor("MOTOR", "chassis", "fl_cyl-fl_upper",   0.01, 0.01, 0.01, 0.01);
    sm.AddMotor("MOTOR", "chassis", "fl_upper-fl_lower", 0.01, 0.01, 0.01, 0.01);
    sm.AddMotor("SERVO", "chassis", "chassis-bl_cyl",    0.01, 0.01, 0.01, 0.01);
    sm.AddMotor("MOTOR", "chassis", "bl_cyl-bl_upper",   0.01, 0.01, 0.01, 0.01);
    sm.AddMotor("MOTOR", "chassis", "bl_upper-bl_lower", 0.01, 0.01, 0.01, 0.01);
    sm.AddMotor("SERVO", "chassis", "chassis-fr_cyl",    0.01, 0.01, 0.01, 0.01);
    sm.AddMotor("MOTOR", "chassis", "fr_cyl-fr_upper",   0.01, 0.01, 0.01, 0.01);
    sm.AddMotor("MOTOR", "chassis", "fr_upper-fr_lower", 0.01, 0.01, 0.01, 0.01);
    sm.AddMotor("SERVO", "chassis", "chassis-br_cyl",    0.01, 0.01, 0.01, 0.01);
    sm.AddMotor("MOTOR", "chassis", "br_cyl-br_upper",   0.01, 0.01, 0.01, 0.01);
    sm.AddMotor("MOTOR", "chassis", "br_upper-br_lower", 0.01, 0.01, 0.01, 0.01);

    sm.AddComponent("ENCODER", "chassis",  0.01, 0.01, 0.01, 0.01);
    sm.AddComponent("ENCODER", "chassis",  0.01, 0.01, 0.01, 0.01);
    sm.AddComponent("ENCODER", "chassis",  0.01, 0.01, 0.01, 0.01);
    sm.AddComponent("ENCODER", "chassis",  0.01, 0.01, 0.01, 0.01);
    sm.AddComponent("ENCODER", "chassis",  0.01, 0.01, 0.01, 0.01);
    sm.AddComponent("ENCODER", "chassis",  0.01, 0.01, 0.01, 0.01);
    sm.AddComponent("ENCODER", "chassis",  0.01, 0.01, 0.01, 0.01);
    sm.AddComponent("ENCODER", "chassis",  0.01, 0.01, 0.01, 0.01);
    sm.AddComponent("CAMERA", "chassis",  0.01, 0.01, 0.01, 0.01);

    sm.AddWaypoint(0.3, 0.3, 0.4);
    sm.AddWaypoint(0.9, 0.9, 0.2);

    stringvec input_types;
    doublepairs input_vels;
    doublepairs input_torqs;
    std::shared_ptr<BBNode> best_node;

    bool sim_done = false;
    bool task_done = false;
    int cnt = 0;
    sm.GetComponentTypes(input_types);

    do {
        task_done = sm.RunSimulation(true);
        sm.GetActuatorVels(input_vels);
        sm.GetActuatorTorques(input_torqs);

        // std::cout << "Input to electronic generator" << std::endl;
        // std::cout << "Torques:" << std::endl;
        // for (size_t i = 0; i < input_torqs.size(); i++) {
            // std::cout << input_torqs[i].first << " " << input_torqs[i].second << std::endl;
        // }
        // std::cout << std::endl;

        // std::cout << "Velocities:" << std::endl;
        // for (size_t i = 0; i < input_vels.size(); i++) {
            // std::cout << input_vels[i].first << " " << input_vels[i].second << std::endl;
        // }
        // std::cout << std::endl;

        if (best_node && reEvaluate(*best_node, input_torqs, input_vels)){
            std::cout << "Successfully generated electronics design" << std::endl;
            sim_done = true;
        }
        else {
            stringvec2d component_versions = preprocess(input_types, input_torqs, input_vels);
            infernodevec2d infer_nodes_vec = initialize(component_versions, input_torqs, input_vels);
            bbnodevec bbnodes = initialize(infer_nodes_vec);
            best_node = branchNBound(&bbnodes);

            if (doubleCheck(*best_node, input_torqs, input_vels)) {
                writeDesign(*best_node);
            }
            else {
                std::cout <<  "Unable to find a feasible design";
                return 1;
            }
            sm.UpdateMassInfo(getMassVec(*best_node, input_torqs.size()));
        }

    } while (!sim_done);

    return 0;
}
