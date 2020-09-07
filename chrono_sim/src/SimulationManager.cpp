#include "chrono_irrlicht/ChIrrApp.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"

#include "ChUrdfDoc.h"
#include "SimulationManager.h"
#include "data_dir_path.h"

using namespace chrono;

SimulationManager::SimulationManager()
    :system_friction_k(1.9), system_friction_s(2.0), system_type(NSC),
     task_done(false), timeout(5), time_step(0.005)
{
    payloads.clear();
    motors.clear();
    waypoints.clear();
    SetChronoDataPath(CHRONO_DATA_DIR);
}

void SimulationManager::AddPayload(double mass, double size_x, double size_y, double size_z,
                                   double coord_x, double coord_y, double coord_z){
    payloads.push_back(std::make_shared<SimPayload>(mass, size_x, size_y, size_z,
                                                    coord_x, coord_y, coord_z));

}

void SimulationManager::AddMotor(const std::string& link_name, double mass,
                                 double size_x, double size_y, double size_z,
                                 double coord_x, double coord_y, double coord_z){
    motors.push_back(std::make_shared<SimMotor>(link_name, mass,
                                                size_x, size_y, size_z,
                                                coord_x, coord_y, coord_z));
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

    // Load URDF file and add to the system
    ChUrdfDoc urdf_doc(sim_system);

    if (!urdf_file.empty()){
        bool load_ok = urdf_doc.Load_URDF(urdf_file, waypoints[0]);

        if (!load_ok) {
            chrono::GetLog() << "Warning. Desired URDF file could not be opened/parsed \n";
            return false;
        }
    }
    else {
        return false;
    }

    // Add motors and extra weights to system
    for (auto payload : payloads) payload->AddtoSystem(sim_system);
    for (auto motor : motors) motor->AddtoSystem(sim_system, urdf_doc);

    // init controller
    if (urdf_doc.GetRobotName().find("manipulator") != std::string::npos) {
        controller = std::make_shared<ManipulatorController>(&motors, &waypoints);
    }
    else if (urdf_doc.GetRobotName().find("leg") != std::string::npos) {
        controller = std::make_shared<LeggedController>(&motors, &waypoints, urdf_doc.GetRootBody());
    }
    else if (urdf_doc.GetRobotName().find("wheel") != std::string::npos) {
        controller = std::make_shared<WheelController>(&motors, &waypoints, urdf_doc.GetRootBody());
    }
    else {
        std::cerr << "Need to specify controller type in robot name" << std::endl;
    }

    double timer = 0;
    if(do_viz){
        using namespace chrono::irrlicht;
        using namespace irr::core;
        ChIrrApp vis_app(sim_system.get(), L"Auto-Electronics Simulation", dimension2d<irr::u32>(640, 480), false);
        ChIrrWizard::add_typical_Logo(vis_app.GetDevice());
        ChIrrWizard::add_typical_Sky(vis_app.GetDevice());
        ChIrrWizard::add_typical_Lights(vis_app.GetDevice(), vector3df(0., 0., 50.), vector3df(0., 0., -50));
        ChIrrWizard::add_typical_Camera(vis_app.GetDevice(), vector3df(0, -5, 4), vector3df(0, 0, 0));

        vis_app.AssetBindAll();
        vis_app.AssetUpdateAll();

        vis_app.SetTimestep(time_step);
        while (timer < timeout && !task_done && vis_app.GetDevice()->run()) {
            vis_app.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
            vis_app.DrawAll();
            vis_app.DoStep();
            vis_app.EndScene();

            task_done = controller->Update();
            process_data();
            timer += time_step;
        }

    }
    else{
        while(timer < timeout && !task_done) {
            sim_system->DoStepDynamics(time_step);

            task_done = controller->Update();
            process_data();
            timer += time_step;
        }
    }

    return true;
}
