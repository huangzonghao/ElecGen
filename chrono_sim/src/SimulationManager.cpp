#include <chrono>
#include "chrono_irrlicht/ChIrrApp.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"

#include "SimulationManager.h"
#include "data_dir_path.h"

using namespace chrono;

SimulationManager::SimulationManager(double step_size,
                                     double timeout,
                                     double system_friction_k,
                                     double system_friction_s,
                                     SystemType system_type):
    step_size(step_size), timeout(timeout),
    system_friction_k(system_friction_k),
    system_friction_s(system_friction_s),
    system_type(NSC)
{
    payloads.clear();
    motors.clear();
    waypoints.clear();
    SetChronoDataPath(CHRONO_DATA_DIR);
}

void SimulationManager::SetUrdfFile(std::string filename){
    urdf_doc = std::make_shared<ChUrdfDoc>(filename);
}

const std::string& SimulationManager::GetUrdfFileName(){
    if(!urdf_doc){
        std::cerr << "Error: URDF file not set yet, call SetUrdfFile() first" << std::endl;
        exit(EXIT_FAILURE);
    }
    return urdf_doc->GetUrdfFileName();
}

void SimulationManager::AddPayload(const std::string& body_name, double mass,
                                   double size_x, double size_y, double size_z,
                                   double pos_x, double pos_y, double pos_z){
    payloads.push_back(std::make_shared<SimPayload>(body_name, mass,
                                                    size_x, size_y, size_z,
                                                    pos_x, pos_y, pos_z));
    auxrefs.insert(body_name);
}

void SimulationManager::AddMotor(const std::string& link_name,
                                 double mass, double size_x, double size_y, double size_z,
                                 double pos_x, double pos_y, double pos_z){
    motors.push_back(std::make_shared<SimMotor>(link_name, mass,
                                                size_x, size_y, size_z,
                                                pos_x, pos_y, pos_z));

    if (!urdf_doc){
        std::cerr << "Error: URDF file not set yet, call SetUrdfFile() first" << std::endl;
        return;
    }
    auto& body_name = urdf_doc->GetLinkBodyName(link_name, 2);
    auxrefs.insert(body_name);
}

void SimulationManager::AddMotor(const std::string& body_name, const std::string& link_name,
                                 double mass, double size_x, double size_y, double size_z,
                                 double pos_x, double pos_y, double pos_z){
    motors.push_back(std::make_shared<SimMotor>(body_name, link_name, mass,
                                                size_x, size_y, size_z,
                                                pos_x, pos_y, pos_z));
    auxrefs.insert(body_name);
}

bool SimulationManager::RunSimulation(bool do_viz){
    task_done = false;

    if (system_type == NSC){
        sim_system = chrono_types::make_shared<ChSystemNSC>();
    }
    else if (system_type == SMC){
        sim_system = chrono_types::make_shared<ChSystemSMC>();
    }
    else{
        std::cerr << "Wrong system type: " << system_type << std::endl;
        exit(EXIT_FAILURE);
    }

    sim_system->Set_G_acc(ChVector<>(0, 0, -9.81));
    sim_system->SetSolverMaxIterations(20);  // the higher, the easier to keep the constraints satisifed.

    auto ground_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    ground_mat->SetSfriction(system_friction_s);
    ground_mat->SetKfriction(system_friction_k);
    ground_mat->SetRestitution(0.01f);

    if (env_file.empty()){
        std::cout << "Map file not initialized, building default ground" << std::endl;
        // ground body
        auto my_ground = chrono_types::make_shared<ChBodyEasyBox>(50, 50, 0.9, 1.0, true, true, ground_mat);
        my_ground->SetPos(ChVector<>(0, 0., -0.5));
        // my_ground->SetRot(Q_from_AngAxis(0.1, VECT_Y));
        my_ground->SetBodyFixed(true);
        auto ground_texture = chrono_types::make_shared<ChColorAsset>();
        ground_texture->SetColor(ChColor(0.2f, 0.2f, 0.2f));
        my_ground->AddAsset(ground_texture);
        sim_system->AddBody(my_ground);
    }
    else if (env_file.find(".bmp") != std::string::npos){
        vehicle::RigidTerrain terrain(sim_system.get());
        auto patch = terrain.AddPatch(ground_mat, ChCoordsys<>(ChVector<>(0, -0.5, 0), QUNIT),
                                      env_file, "ground_mesh", 50, 50, 0, 2);
        patch->SetColor(ChColor(0.2, 0.2, 0.2));
        terrain.Initialize();
    }
    else if (env_file.find(".obj") != std::string::npos){
        vehicle::RigidTerrain terrain(sim_system.get());
        auto patch = terrain.AddPatch(ground_mat, ChCoordsys<>(ChVector<>(0, -0.5, 0), QUNIT),
                                      env_file, "ground_mesh");
        patch->SetColor(ChColor(0.2, 0.2, 0.2));
        terrain.Initialize();
    }
    else {
        std::cerr << "Map file " << env_file << " not recognized, exiting" << std::endl;
        return 1;
    }


    if (urdf_doc){
        urdf_doc->SetAuxRef(auxrefs);

        bool add_ok;
        if (!waypoints.empty()){
            add_ok = urdf_doc->AddtoSystem(sim_system, waypoints[0]);
        }
        else{
            add_ok = urdf_doc->AddtoSystem(sim_system, ChVector<>(0,0,0));
        }

        if (!add_ok) {
            chrono::GetLog() << "Warning. Could not add urdf robot to ChSystem\n";
            return false;
        }
    }
    else {
        std::cerr << "Error: URDF file not set yet, call SetUrdfFile() first" << std::endl;
        return false;
    }

    // add waypoint markers
    auto wp_color = chrono_types::make_shared<ChColorAsset>();
    wp_color->SetColor(ChColor(0.8f, 0.0f, 0.0f));
    for(auto waypoint : waypoints){
        auto wp_marker = chrono_types::make_shared<ChBodyEasyBox>(0.5, 0.5, 0.2, 1.0, true, false);
        wp_marker->SetPos(waypoint);
        wp_marker->SetBodyFixed(true);
        wp_marker->AddAsset(wp_color);
        sim_system->AddBody(wp_marker);
    }

    // Add motors and extra weights to system
    for (auto payload : payloads) payload->AddtoSystem(sim_system);
    for (auto motor : motors) motor->AddtoSystem(*urdf_doc);

    // init controller
    if (urdf_doc->GetRobotName().find("manipulator") != std::string::npos) {
        controller = std::make_shared<ManipulatorController>(&motors, &waypoints);
    }
    else if (urdf_doc->GetRobotName().find("leg") != std::string::npos) {
        controller = std::make_shared<LeggedController>(&motors, &waypoints, urdf_doc->GetRootBody());
        if (urdf_doc->GetRobotName().find("2") != std::string::npos) {
            std::dynamic_pointer_cast<LeggedController>(controller)->model = LeggedController::M2;
        }
        else if (urdf_doc->GetRobotName().find("3") != std::string::npos) {
            std::dynamic_pointer_cast<LeggedController>(controller)->model = LeggedController::M3;
        }
        else{
            std::dynamic_pointer_cast<LeggedController>(controller)->model = LeggedController::M1;
        }

    }
    else if (urdf_doc->GetRobotName().find("wheel") != std::string::npos) {
        controller = std::make_shared<WheelController>(&motors, &waypoints, urdf_doc->GetRootBody());
    }
    else {
        std::cerr << "Need to specify controller type in robot name" << std::endl;
    }

    const std::shared_ptr<ChBody>& camera_body = urdf_doc->GetCameraBody();

    std::chrono::steady_clock::time_point tik;
    std::chrono::steady_clock::time_point tok;
    if(do_viz){
        using namespace chrono::irrlicht;
        using namespace irr::core;
        ChIrrApp vis_app(sim_system.get(),
                         L"Auto-Electronics Simulation",
                         dimension2d<irr::u32>(640, 480), false);

        vis_app.AddTypicalLogo();
        vis_app.AddTypicalSky();
        vis_app.AddTypicalLights(vector3df(0., 0., 50.), vector3df(0., 0., -50));
        vis_app.AddTypicalCamera(vector3df(0, -10, 10), vector3df(0, 0, 0));

        vis_app.AssetBindAll();
        vis_app.AssetUpdateAll();

        vis_app.SetTimestep(step_size);

        tik = std::chrono::steady_clock::now();
        while (sim_system->GetChTime() < timeout && !task_done && vis_app.GetDevice()->run()) {
            vis_app.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
            // vis_app.GetSceneManager()->getActiveCamera()->setTarget(vector3dfCH(camera_body->GetPos()));
            vis_app.DrawAll();
            vis_app.DoStep();
            vis_app.EndScene();

            task_done = controller->Update();
        }
        tok = std::chrono::steady_clock::now();

    }
    else{
        std::cout << "Simulating without visualization" << std::endl;

        tik = std::chrono::steady_clock::now();
        while(sim_system->GetChTime() < timeout && !task_done) {
            sim_system->DoStepDynamics(step_size);

            task_done = controller->Update();
        }
        tok = std::chrono::steady_clock::now();
    }

    std::cout << "Simulation time: " << std::chrono::duration_cast<std::chrono::milliseconds>(tok - tik).count() << "[ms]" << std::endl;
    std::cout << "Step count: " << sim_system->GetStepcount() << std::endl;

    return true;
}
