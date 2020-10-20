#include <chrono>
#include "chrono_irrlicht/ChIrrApp.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"

#include "SimulationManager.h"
#include "data_dir_path.h"
#include "traj_optimizer.h"

using namespace chrono;

SimulationManager::SimulationManager(double step_size,
                                     double timeout,
                                     double system_friction_k,
                                     double system_friction_s,
                                     SystemType system_type):
    step_size_(step_size), timeout_(timeout),
    k_friction_(system_friction_k),
    s_friction_(system_friction_s),
    system_type_(system_type)
{
    payloads_.clear();
    motors_.clear();
    ch_waypoints_.clear();
    SetChronoDataPath(CHRONO_DATA_DIR);
}

void SimulationManager::SetUrdfFile(std::string filename){
    urdf_doc_ = std::make_shared<ChUrdfDoc>(filename);
}

void SimulationManager::SetEnv(std::string filename, double env_x, double env_y, double env_z){
    env_file_ = filename;
    env_x_ = env_x;
    env_y_ = env_y;
    env_z_ = env_z;
}
void SimulationManager::SetEigenHeightmap(const std::shared_ptr<const Eigen::MatrixXd>& heightmap){
    heightmap_ = heightmap;
}

const std::string& SimulationManager::GetUrdfFileName(){
    if(!urdf_doc_){
        std::cerr << "Error: URDF file not set yet, call SetUrdfFile() first" << std::endl;
        exit(EXIT_FAILURE);
    }
    return urdf_doc_->GetUrdfFileName();
}

void SimulationManager::AddPayload(const std::string& body_name, double mass,
                                   double size_x, double size_y, double size_z,
                                   double pos_x, double pos_y, double pos_z){
    payloads_.push_back(std::make_shared<SimPayload>(body_name, mass,
                                                     size_x, size_y, size_z,
                                                     pos_x, pos_y, pos_z));
    auxrefs_.insert(body_name);
}

void SimulationManager::AddMotor(const std::string& link_name,
                                 double mass, double size_x, double size_y, double size_z,
                                 double pos_x, double pos_y, double pos_z){
    motors_.push_back(std::make_shared<SimMotor>(link_name, mass,
                                                 size_x, size_y, size_z,
                                                 pos_x, pos_y, pos_z));

    if (!urdf_doc_){
        std::cerr << "Error: URDF file not set yet, call SetUrdfFile() first" << std::endl;
        return;
    }
    auto& body_name = urdf_doc_->GetLinkBodyName(link_name, 2);
    auxrefs_.insert(body_name);
}

void SimulationManager::AddMotor(const std::string& body_name, const std::string& link_name,
                                 double mass, double size_x, double size_y, double size_z,
                                 double pos_x, double pos_y, double pos_z){
    motors_.push_back(std::make_shared<SimMotor>(body_name, link_name, mass,
                                                 size_x, size_y, size_z,
                                                 pos_x, pos_y, pos_z));
    auxrefs_.insert(body_name);
}

void SimulationManager::AddWaypoint(double x, double y, double z){
    ch_waypoints_.push_back(chrono::ChVector<>(x,y,z));
}

void SimulationManager::AddWaypoints(const std::shared_ptr<const Eigen::MatrixXd>& waypoints_ptr){
    eigen_waypoints_ = waypoints_ptr;
    auto& waypoints_mat = *waypoints_ptr;
    if (waypoints_mat.rows() != 3){
        std::cerr << "Error: waypoint matrix passed to SimulationManager::AddWaypoints should be 3xN matrix" << std::endl;
        exit(EXIT_FAILURE);
    }
    for (int i = 0; i < waypoints_mat.cols(); ++i){
        AddWaypoint(waypoints_mat(0,i), waypoints_mat(1,i), waypoints_mat(2,i));
    }
}

bool SimulationManager::RunSimulation(bool do_viz){
    if(!urdf_doc_){
        std::cerr << "Error: URDF file not set yet, call SetUrdfFile() first" << std::endl;
        exit(EXIT_FAILURE);
    }

    task_done_ = false;

    if (system_type_ == NSC){
        ch_system_ = chrono_types::make_shared<ChSystemNSC>();
    }
    else if (system_type_ == SMC){
        ch_system_ = chrono_types::make_shared<ChSystemSMC>();
    }
    else{
        std::cerr << "Wrong system type: " << system_type_ << std::endl;
        exit(EXIT_FAILURE);
    }

    ch_system_->Set_G_acc(ChVector<>(0, 0, -9.81));
    ch_system_->SetSolverMaxIterations(20);  // the higher, the easier to keep the constraints satisifed.

    auto ground_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    ground_mat->SetSfriction(s_friction_);
    ground_mat->SetKfriction(k_friction_);
    ground_mat->SetRestitution(0.01f);

    if (env_file_.empty()){
        std::cout << "Map file not initialized, building default ground" << std::endl;
        // ground body
        auto my_ground = chrono_types::make_shared<ChBodyEasyBox>(1, 1, 0.01, 1.0, true, true, ground_mat);
        // make sure (0,0) is at the corner of the environment
        my_ground->SetPos(ChVector<>(0.5, 0.5, -0.005));
        my_ground->SetRot(Q_ROTATE_X_TO_Y);
        my_ground->SetBodyFixed(true);
        auto ground_texture = chrono_types::make_shared<ChColorAsset>();
        ground_texture->SetColor(ChColor(0.2f, 0.2f, 0.2f));
        my_ground->AddAsset(ground_texture);
        ch_system_->AddBody(my_ground);
    }
    else if (env_file_.find(".bmp") != std::string::npos){
        vehicle::RigidTerrain terrain(ch_system_.get());
        auto patch = terrain.AddPatch(ground_mat, ChCoordsys<>(ChVector<>(env_x_ / 2, env_y_ / 2, -0.5), Q_ROTATE_X_TO_Y),
                                      env_file_, "ground_mesh", env_x_, env_y_, 0, env_z_);
        patch->SetColor(ChColor(0.2, 0.2, 0.2));
        terrain.Initialize();
    }
    else if (env_file_.find(".obj") != std::string::npos){
        vehicle::RigidTerrain terrain(ch_system_.get());
        // TODO: need to set map pos offset, to make (0,0) appear at corner
        auto patch = terrain.AddPatch(ground_mat, ChCoordsys<>(ChVector<>(0, 0, -0.5), Q_ROTATE_X_TO_Y),
                                      env_file_, "ground_mesh");
        patch->SetColor(ChColor(0.2, 0.2, 0.2));
        terrain.Initialize();

        // // For debug purposes
        // auto x_box = chrono_types::make_shared<ChBodyEasyBox>(20, 1, 1, 1.0, true, true, ground_mat);
        // x_box->SetPos(ChVector<>(10, 0, 0.5));
        // x_box->SetBodyFixed(true);
        // auto x_text = chrono_types::make_shared<ChColorAsset>();
        // x_text->SetColor(ChColor(0.9f, 0.0f, 0.0f)); // red
        // x_box->AddAsset(x_text);
        // ch_system_->AddBody(x_box);

        // auto y_box = chrono_types::make_shared<ChBodyEasyBox>(1, 20, 1, 1.0, true, true, ground_mat);
        // y_box->SetPos(ChVector<>(0, 10, 0.5));
        // y_box->SetBodyFixed(true);
        // auto y_text = chrono_types::make_shared<ChColorAsset>();
        // y_text->SetColor(ChColor(0.0f, 0.9f, 0.0f)); // green
        // y_box->AddAsset(y_text);
        // ch_system_->AddBody(y_box);

        // auto z_box = chrono_types::make_shared<ChBodyEasyBox>(1, 1, 20, 1.0, true, true, ground_mat);
        // z_box->SetPos(ChVector<>(0, 0, 10));
        // z_box->SetBodyFixed(true);
        // auto z_text = chrono_types::make_shared<ChColorAsset>();
        // z_text->SetColor(ChColor(0.0f, 0.0f, 0.9f)); // blue
        // z_box->AddAsset(z_text);
        // ch_system_->AddBody(z_box);
    }
    else {
        std::cerr << "Map file " << env_file_ << " not recognized, exiting" << std::endl;
        return 1;
    }

    urdf_doc_->SetAuxRef(auxrefs_);

    bool add_ok;
    if (!ch_waypoints_.empty()){
        add_ok = urdf_doc_->AddtoSystem(ch_system_, ch_waypoints_[0]);
    }
    else{
        add_ok = urdf_doc_->AddtoSystem(ch_system_, ChVector<>(0,0,0));
    }

    if (!add_ok) {
        chrono::GetLog() << "Warning. Could not add urdf robot to ChSystem\n";
        return false;
    }

    // add waypoint markers
    auto wp_color = chrono_types::make_shared<ChColorAsset>();
    wp_color->SetColor(ChColor(0.8f, 0.0f, 0.0f));
    // for(auto waypoint : ch_waypoints_){
    // TODO: put first waypoint marker on the ground
    for(auto waypoint = ch_waypoints_.begin() + 1; waypoint != ch_waypoints_.end(); ++waypoint){
        auto wp_marker = chrono_types::make_shared<ChBodyEasyBox>(0.01, 0.01, 0.005, 1.0, true, false);
        wp_marker->SetPos(*waypoint);
        wp_marker->SetBodyFixed(true);
        wp_marker->AddAsset(wp_color);
        ch_system_->AddBody(wp_marker);
    }

    // Add motors and extra weights to system
    for (auto payload : payloads_) payload->AddtoSystem(ch_system_);
    for (auto motor : motors_) motor->AddtoSystem(*urdf_doc_);

    // init controller
    if (urdf_doc_->GetRobotName().find("manipulator") != std::string::npos) {
        controller_ = std::make_shared<ManipulatorController>(&motors_, &ch_waypoints_);
    }
    else if (urdf_doc_->GetRobotName().find("leg") != std::string::npos) {
        controller_ = std::make_shared<LeggedController>(&motors_, &ch_waypoints_, urdf_doc_->GetRootBody());
        if (urdf_doc_->GetRobotName().find("2") != std::string::npos) {
            std::dynamic_pointer_cast<LeggedController>(controller_)->model = LeggedController::M2;
        }
        else if (urdf_doc_->GetRobotName().find("3") != std::string::npos) {
            std::dynamic_pointer_cast<LeggedController>(controller_)->model = LeggedController::M3;
            const auto& legged_controller = std::dynamic_pointer_cast<LeggedController>(controller_);
            // Calculate IK and feet constraint boxes
            // TODO: hard coded values
            // TODO: decide number of steps based on the trajectory length
            int num_ee = 4;
            int num_steps = 10;
            double env_x = 1;
            double env_y = 1;
            Eigen::Vector3d init_pos(ch_waypoints_[0].x(), ch_waypoints_[0].y(), ch_waypoints_[0].z());
            Eigen::Vector3d goal_pos(ch_waypoints_[1].x(), ch_waypoints_[1].y(), ch_waypoints_[1].z());

            TrajectoryOptimizer traj_opt(num_ee, urdf_doc_->GetRootMass(),
                                         num_steps, heightmap_, env_x, env_y,
                                         init_pos, Eigen::Vector3d::Zero(),
                                         goal_pos, Eigen::Vector3d::Zero());

            double l1 = 0.035;
            double l2 = 0.06;
            double l3 = 0.06;
            double offset_x = 0.065;
            double offset_y = 0.02;
            double offset_z = 0.12;
            // TODO: more accurate deviation
            double dev_x = 0.06;
            double dev_y = 0.02;
            double dev_z = 0.01;
            legged_controller->SetKinematics(l1, l2, l3, offset_x, offset_y);

            // world frame
            traj_opt.set_ee_pos(legged_controller->GetFK_W(towr::LF, 0, 0, 0),
                                legged_controller->GetFK_W(towr::LH, 0, 0, 0),
                                legged_controller->GetFK_W(towr::RF, 0, 0, 0),
                                legged_controller->GetFK_W(towr::RH, 0, 0, 0));

            traj_opt.set_kinematic_model(offset_x, offset_y, offset_z,
                                         dev_x, dev_y, dev_z);

            // TODO: automatic derive inertia matrix
            traj_opt.set_dynamic_model(1, 1, 1, 0, 0, 0);

            traj_opt.optimize();

            // pass the optimized trajectory to controller
            legged_controller->AddTrajectory();
        }
        else{
            std::dynamic_pointer_cast<LeggedController>(controller_)->model = LeggedController::M1;
        }

    }
    else if (urdf_doc_->GetRobotName().find("wheel") != std::string::npos) {
        controller_ = std::make_shared<WheelController>(&motors_, &ch_waypoints_, urdf_doc_->GetRootBody());
    }
    else {
        std::cerr << "Need to specify controller_ type in robot name" << std::endl;
    }

    const std::shared_ptr<ChBody>& camera_body = urdf_doc_->GetCameraBody();

    std::chrono::steady_clock::time_point tik;
    std::chrono::steady_clock::time_point tok;
    if(do_viz){
        using namespace chrono::irrlicht;
        using namespace irr::core;
        ChIrrApp vis_app(ch_system_.get(),
                         L"Auto-Electronics Simulation",
                         dimension2d<irr::u32>(640, 480), false);

        vis_app.AddTypicalLogo();
        vis_app.AddTypicalSky();
        vis_app.AddTypicalLights(vector3df(0., 0., 50.), vector3df(0., 0., -50));
        vis_app.AddTypicalCamera(vector3df(0, -1, 1), vector3df(0, 0, 0));

        vis_app.AssetBindAll();
        vis_app.AssetUpdateAll();

        vis_app.SetTimestep(step_size_);

        tik = std::chrono::steady_clock::now();
        while (ch_system_->GetChTime() < timeout_ && !task_done_ && vis_app.GetDevice()->run()) {
            vis_app.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
            // vis_app.GetSceneManager()->getActiveCamera()->setTarget(vector3dfCH(camera_body->GetPos()));
            vis_app.DrawAll();
            vis_app.DoStep();
            vis_app.EndScene();

            task_done_ = controller_->Update();
        }
        tok = std::chrono::steady_clock::now();

    }
    else{
        std::cout << "Simulating without visualization" << std::endl;

        tik = std::chrono::steady_clock::now();
        while(ch_system_->GetChTime() < timeout_ && !task_done_) {
            ch_system_->DoStepDynamics(step_size_);

            task_done_ = controller_->Update();
        }
        tok = std::chrono::steady_clock::now();
    }

    std::cout << "Simulation time: " << std::chrono::duration_cast<std::chrono::milliseconds>(tok - tik).count() << "[ms]" << std::endl;
    std::cout << "Step count: " << ch_system_->GetStepcount() << std::endl;

    return true;
}

void SimulationManager::GetComponentTypes(std::vector<std::string> &types_vec) const {
    if (types_vec.empty()){
        for (int i = 0; i < motors_.size(); ++i){
            types_vec.push_back("MOTOR");
        }
    }
    else{
        std::cerr << "Error: passing in a non-empty ComponentTypes vector. ComponentTypes vector can only be initialized once" << std::endl;
    }
}

void SimulationManager::
GetActuatorVels(std::vector<std::pair<double, double> > &vels_vec) const {
    if (vels_vec.empty()){
        vels_vec.resize(motors_.size());
    }
    else if (vels_vec.size() != motors_.size()){
        std::cerr << "Error, simulation motor number is not equal to generation motor number" << std::endl;
        exit(EXIT_FAILURE);
    }
    for (int i = 0; i < motors_.size(); ++i){
        vels_vec[i].second = motors_[i]->GetMaxVel();
    }
}

void SimulationManager::
GetActuatorTorques(std::vector<std::pair<double, double> > &torqs_vec) const {
    if (torqs_vec.empty()){
        torqs_vec.resize(motors_.size());
    }
    else if (torqs_vec.size() != motors_.size()){
        std::cerr << "Error, simulation motor number is not equal to generation motor number" << std::endl;
        exit(EXIT_FAILURE);
    }
    for (int i = 0; i < motors_.size(); ++i){
        torqs_vec[i].second = motors_[i]->GetMaxTorque();
    }
}

void SimulationManager::UpdateMassInfo(const std::vector<double>& mass_vec){
    if (mass_vec.size() != motors_.size()){
        std::cerr << "Error, simulation motor number is not equal to returned mass number" << std::endl;
        exit(EXIT_FAILURE);
    }
    for (int i = 0; i < mass_vec.size() - 1; ++i){
        motors_[i]->SetMass(mass_vec[i]);
    }
    // TODO: the final mass update apply to body
}
