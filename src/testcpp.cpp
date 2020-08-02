#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono_irrlicht/ChIrrApp.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"

#include "ChUrdfDoc.h"


using namespace chrono;

const std::string urdf_filename = "../robots/fourwheels.urdf";
const std::string env_filename = "";

const double s_friction = 2.0;
const double k_friction = 1.9;

int main(int argc, char* argv[]){

    std::shared_ptr<ChSystemNSC> my_system = chrono_types::make_shared<ChSystemNSC>();

    my_system->Set_G_acc(ChVector<>(0, 0, -9.81));
    my_system->SetSolverMaxIterations(20);  // the higher, the easier to keep the constraints satisifed.

    ChVector<> init_pos(0, 0, 2);
    ChVector<> goal_pos(5, 5, 0.);

    auto init_pos_body = chrono_types::make_shared<ChBodyEasyBox>(0.5, 0.5, 0.2, 1.0, true, false);
    init_pos_body->SetPos(init_pos);
    init_pos_body->SetBodyFixed(true);
    auto init_pos_color = chrono_types::make_shared<ChColorAsset>();
    init_pos_color->SetColor(ChColor(0.0f, 0.8f, 0.0f));
    init_pos_body->AddAsset(init_pos_color);
    my_system->AddBody(init_pos_body);

    auto goal_pos_body = chrono_types::make_shared<ChBodyEasyBox>(0.5, 0.5, 0.2, 1.0, true, false);
    goal_pos_body->SetPos(goal_pos);
    goal_pos_body->SetBodyFixed(true);
    auto goal_pos_color = chrono_types::make_shared<ChColorAsset>();
    goal_pos_color->SetColor(ChColor(0.0f, 0.0f, 0.8f));
    goal_pos_body->AddAsset(goal_pos_color);
    my_system->AddBody(goal_pos_body);

    auto ground_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    ground_mat->SetSfriction(s_friction);
    ground_mat->SetKfriction(k_friction);
    ground_mat->SetRestitution(0.01f);

    if (env_filename.find(".bmp") != std::string::npos){
        vehicle::RigidTerrain terrain(my_system.get());
        auto patch = terrain.AddPatch(ground_mat, ChCoordsys<>(ChVector<>(0, -0.5, 0), QUNIT),
                                      env_filename, "ground_mesh", 50, 50, 0, 2);
        patch->SetColor(ChColor(0.2, 0.2, 0.2));
        terrain.Initialize();
    }
    else if (env_filename.find(".obj") != std::string::npos){
        vehicle::RigidTerrain terrain(my_system.get());
        auto patch = terrain.AddPatch(ground_mat, ChCoordsys<>(ChVector<>(0, -0.5, 0), QUNIT),
                                      env_filename, "ground_mesh");
        patch->SetColor(ChColor(0.2, 0.2, 0.2));
        terrain.Initialize();
    }
    else {
        // ground body
        auto my_ground = chrono_types::make_shared<ChBodyEasyBox>(50, 50, 0.9, 1.0, true, true, ground_mat);
        my_ground->SetPos(ChVector<>(0, 0., -0.5));
        // my_ground->SetRot(Q_from_AngAxis(0.1, VECT_Y));
        my_ground->SetBodyFixed(true);
        auto ground_texture = chrono_types::make_shared<ChColorAsset>();
        ground_texture->SetColor(ChColor(0.2f, 0.2f, 0.2f));
        my_ground->AddAsset(ground_texture);
        my_system->AddBody(my_ground);
    }

    // Load URDF file and add to the system
    ChUrdfDoc urdf_doc(my_system);

    if (!urdf_filename.empty()){
        bool load_ok = urdf_doc.Load_URDF(urdf_filename, init_pos_body);

        if (!load_ok) {
            chrono::GetLog() << "Warning. Desired URDF file could not be opened/parsed \n";
        }
    }


    chrono::irrlicht::ChIrrApp application(my_system.get(), L"Auto-Electronics Simulation", irr::core::dimension2d<irr::u32>(640, 480), false);
    chrono::irrlicht::ChIrrWizard::add_typical_Logo(application.GetDevice());
    chrono::irrlicht::ChIrrWizard::add_typical_Sky(application.GetDevice());
    chrono::irrlicht::ChIrrWizard::add_typical_Lights(application.GetDevice(), irr::core::vector3df(0., 0., 50.), irr::core::vector3df(0., 0., -50));
    chrono::irrlicht::ChIrrWizard::add_typical_Camera(application.GetDevice(), irr::core::vector3df(0, -5, 4), irr::core::vector3df(0, 0, 0));

    // Bind visualization assets.
    application.AssetBindAll();
    application.AssetUpdateAll();

    // Timer for enforcing sodt real-time
    ChRealtimeStepTimer realtime_timer;
    double time_step = 0.005;

    while (application.GetDevice()->run()) {
        // Irrlicht must prepare frame to draw
        application.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));

        // Irrlicht now draws simple lines in 3D world representing a
        // skeleton of the mechanism, in this instant:
        //
        // .. draw solid 3D items (boxes, cylinders, shapes) belonging to Irrlicht scene, if any
        application.DrawAll();

        // .. draw a grid (rotated so that it's horizontal)
        chrono::irrlicht::ChIrrTools::drawGrid(application.GetVideoDriver(), 2, 2, 30, 30,
                             ChCoordsys<>(ChVector<>(0, 0.01, 0), QUNIT),
                             irr::video::SColor(255, 80, 130, 130), true);

        // .. draw GUI user interface items (sliders, buttons) belonging to Irrlicht screen, if any
        application.GetIGUIEnvironment()->drawAll();

        // ADVANCE SYSTEM STATE BY ONE STEP
        my_system->DoStepDynamics(time_step);
        // Enforce soft real-time
        realtime_timer.Spin(time_step);

        // Irrlicht must finish drawing the frame
        application.EndScene();
    }


    return 0;
}
