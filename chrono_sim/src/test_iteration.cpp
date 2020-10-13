#include <iostream>
#include "SimulationManager.h"

const std::string urdf_filename = "../robots/fourwheels.urdf";
// const std::string urdf_filename = "../robots/fourleg.urdf";
const std::string env_filename = "";

const double s_friction = 2.0;
const double k_friction = 1.9;

int main(){

    // first init simulation manager
    SimulationManager sm;
    sm.SetUrdfFile(urdf_filename);
    sm.SetEnvFile(env_filename);
    sm.SetFrictionK(k_friction);
    sm.SetFrictionS(s_friction);

    // this part will be done by UI
    // sm.AddMotor("chassis_wheel_rl", 1,0.1,0.1,0.1);
    // sm.AddMotor("chassis_wheel_rr", 1,0.1,0.1,0.1);

	/* Following part needs to be filled */
	// TEST CASE: QUADRUPED
    sm.AddMotor("base_link", "base_link_link1", 1,0.1,0.1,0.1);
    sm.AddMotor("base_link", "base_link_link2", 1,0.1,0.1,0.1);
    sm.AddMotor("base_link", "base_link_link3", 1,0.1,0.1,0.1);
    sm.AddMotor("base_link", "base_link_link4", 1,0.1,0.1,0.1);
	sm.AddMotor(...); 
	sm.AddMotor(...); 
	sm.AddMotor(...);
	sm.AddMotor(...); // dc motor*8
	sm.AddServo(...);
	sm.AddServo(...);
	sm.AddServo(...);
	sm.AddServo(...); // servo motor*4
	sm.AddEncoder(...);
	sm.AddEncoder(...);
	sm.AddEncoder(...);
	sm.AddEncoder(...); 
	sm.AddEncoder(...);
	sm.AddEncoder(...);
	sm.AddEncoder(...);
	sm.AddEncoder(...); // encoder*8
	sm.AddCamera(...); // camera*1

	// TEST CASE: MANIPULATOR
	sm.AddServo(...);
	sm.AddServo(...);
	sm.AddServo(...); 
	sm.AddServo(...); // servo*4
	sm.AddForceSensor(...); 
	sm.AddForceSensor(...); // force sensor*2

	// n + 1 vector: n for input components masses,
	// 1 for sum of all other components mass
	doublvec mass_vec; 
	stringvec input_types;
	shared_ptr<BBNode> best_node;
	/*END*/
 

    sm.AddWaypoint(0, 0, 1);
    sm.AddWaypoint(5, 0, 1);
    sm.AddWaypoint(8, 0, 1);

    // use this loop for iterations
    bool sim_done = false;
	unsigned cnt = 0;
    while (!sim_done){

        bool task_done = sm.RunSimulation();

		/* The following parts need to be filled*/
		if (task_done && !cnt)
		{
			input_types = sm.getComponentTypes();
			doublepairs input_vels = sm.getActuatorVels(),
				input_torqs = sm.getActuatorTorqs();

			stringvec2d component_versions = preprocess(input_types, input_torqs, input_vels);
			infernodevec2d infer_nodes_vec = initialize(component_versions, input_torqs, input_vels);
			bbnodevec bbnodes = initialize(infer_nodes_vec);
			best_node = branchNBound(bbnodes);
			writeDesign(*best_node);
		}
		else if (task_done)
		{
			doublepairs input_vels = sm.getActuatorVels(),
				input_torqs = sm.getActuatorTorqs();

			// verify design
			if (doubleCheck(best_node, input_vels, input_torqs))
			{
				sim_done = true;
			}
			else
			{
				stringvec2d component_versions = preprocess(input_types, input_torqs, input_vels);
				infernodevec2d infer_nodes_vec = initialize(component_versions, input_torqs, input_vels);
				bbnodevec bbnodes = initialize(infer_nodes_vec);
				best_node = branchNBound(bbnodes);
				writeDesign(*best_node);
			}
		}
		mass_vec = best_node->getMassVec(); // feed this into dynamic simulation again
		sm.updateMassInfo(mass_vec);
		cnt++;
		/* END */

    }

    return 0;
}
