#include "ChUrdfDoc.h"

#include <iostream>
#include <filesystem>
#include "chrono/assets/ChBoxShape.h"
#include "chrono/assets/ChSphereShape.h"
#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChTriangleMeshShape.h"

namespace chrono {

std::string ChUrdfDoc::urdf_abs_path(const std::string& relative_path){
    std::filesystem::path abs_path(urdf_file_);
    abs_path.remove_filename();
    abs_path /= relative_path;
    return abs_path.string();
}

bool ChUrdfDoc::color_empty(const urdf::Color& test_color){
    if (test_color.r != 0.0f) return false;
    if (test_color.g != 0.0f) return false;
    if (test_color.b != 0.0f) return false;
    if (test_color.a != 1.0f) return false;
    return true;
}

void ChUrdfDoc::convert_materials(){
    for (auto u_mat_iter = urdf_robot_->materials_.begin();
         u_mat_iter != urdf_robot_->materials_.end();
         ++u_mat_iter){

        ChMatPair tmp_pair;
        if (!color_empty(u_mat_iter->second->color)){
            urdf::Color *u_color = &u_mat_iter->second->color;
            tmp_pair.color = chrono_types::make_shared<ChColor>(u_color->r, u_color->g, u_color->b, u_color->a);
        }
        if(!u_mat_iter->second->texture_filename.empty()){
            tmp_pair.texture = chrono_types::make_shared<ChTexture>();
            tmp_pair.texture->SetTextureFilename(urdf_abs_path(u_mat_iter->second->texture_filename));
        }
        ch_materials_.emplace(u_mat_iter->second->name, tmp_pair);
    }
}

std::shared_ptr<ChBody> ChUrdfDoc::convert_links(const urdf::LinkConstSharedPtr& u_link,
                                                 const std::shared_ptr<ChBody>& ch_parent_body){

    // first make the chbody for yourself
    urdf::JointSharedPtr u_parent_joint = u_link->parent_joint;

    std::shared_ptr<ChBody> ch_body;
    if (auxrefs_ && auxrefs_->find(u_link->name) != auxrefs_->end()){
        ch_body = chrono_types::make_shared<ChBodyAuxRef>();
    }
    else {
        ch_body = chrono_types::make_shared<ChBody>();
    }

    ch_body->SetIdentifier(link_idx_++);
    ch_body->SetNameString(u_link->name);
    if (u_link->name.find("fixed") != std::string::npos) ch_body->SetBodyFixed(true);
    ch_body->SetMass(u_link->inertial->mass);
    ch_body->SetInertia(ChMatrix33<>(ChVector<>(u_link->inertial->ixx,
                                                u_link->inertial->iyy,
                                                u_link->inertial->izz),
                                     ChVector<>(u_link->inertial->ixy,
                                                u_link->inertial->ixz,
                                                u_link->inertial->iyz)));

    if (u_parent_joint){
        ChCoordsys<> child_in_parent (ChVector<>(u_parent_joint->parent_to_joint_origin_transform.position.x,
                                                 u_parent_joint->parent_to_joint_origin_transform.position.y,
                                                 u_parent_joint->parent_to_joint_origin_transform.position.z),
                                      ChQuaternion<>(u_parent_joint->parent_to_joint_origin_transform.rotation.w,
                                                     u_parent_joint->parent_to_joint_origin_transform.rotation.x,
                                                     u_parent_joint->parent_to_joint_origin_transform.rotation.y,
                                                     u_parent_joint->parent_to_joint_origin_transform.rotation.z));
        ch_body->SetCoord(child_in_parent >> ch_parent_body->GetCoord());
    }
    else{
        ch_body->SetCoord(ch_parent_body->GetCoord());
    }


    // Visual
    if (u_link->visual){
        for (auto u_visual : u_link->visual_array){
            std::shared_ptr<ChVisualization> tmp_viasset;
            switch (u_visual->geometry->type){
                case urdf::Geometry::BOX:
                {
                    tmp_viasset = chrono_types::make_shared<ChBoxShape>();
                    urdf::BoxSharedPtr tmp_urdf_box_ptr = std::dynamic_pointer_cast<urdf::Box>(u_visual->geometry);
                    std::dynamic_pointer_cast<ChBoxShape>(tmp_viasset)->GetBoxGeometry().SetLengths(ChVector<>(tmp_urdf_box_ptr->dim.x,
                                                                                                               tmp_urdf_box_ptr->dim.y,
                                                                                                               tmp_urdf_box_ptr->dim.z));
                    break;
                }
                case urdf::Geometry::SPHERE:
                {
                    tmp_viasset = chrono_types::make_shared<ChSphereShape>();
                    std::dynamic_pointer_cast<ChSphereShape>(tmp_viasset)->GetSphereGeometry().rad = std::dynamic_pointer_cast<urdf::Sphere>(u_visual->geometry)->radius;
                    break;
                }
                case urdf::Geometry::CYLINDER:
                {
                    tmp_viasset = chrono_types::make_shared<ChCylinderShape>();
                    urdf::CylinderSharedPtr tmp_urdf_cylinder_ptr = std::dynamic_pointer_cast<urdf::Cylinder>(u_visual->geometry);
                    geometry::ChCylinder& tmp_geometry = std::dynamic_pointer_cast<ChCylinderShape>(tmp_viasset)->GetCylinderGeometry();
                    // urdf cylinder is along z axis (while chrono cylinder defaults to y axis - doesn't matter here)
                    tmp_geometry.p1 = ChVector<>(0, 0, -tmp_urdf_cylinder_ptr->length / 2);
                    tmp_geometry.p2 = ChVector<>(0, 0, tmp_urdf_cylinder_ptr->length / 2);
                    tmp_geometry.rad = tmp_urdf_cylinder_ptr->radius;
                    break;
                }
                case urdf::Geometry::MESH:
                {
                    urdf::MeshSharedPtr tmp_urdf_mesh_ptr = std::dynamic_pointer_cast<urdf::Mesh>(u_visual->geometry);
                    auto trimesh = chrono_types::make_shared<geometry::ChTriangleMeshConnected>();
                    trimesh->LoadWavefrontMesh(urdf_abs_path(tmp_urdf_mesh_ptr->filename));
                    trimesh->Transform(VNULL, ChMatrix33<>(ChVector<>(tmp_urdf_mesh_ptr->scale.x,
                                                                      tmp_urdf_mesh_ptr->scale.y,
                                                                      tmp_urdf_mesh_ptr->scale.z)));
                    auto trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
                    trimesh_shape->SetMesh(trimesh);
                    trimesh_shape->SetName(ch_body->GetNameString() + "_vis_mesh");
                    // for some reason this SetScale method doesn't work
                    // trimesh_shape->SetScale(ChVector<>(tmp_urdf_mesh_ptr->scale.x,
                                                       // tmp_urdf_mesh_ptr->scale.y,
                                                       // tmp_urdf_mesh_ptr->scale.z));
                    tmp_viasset = trimesh_shape;
                    break;
                }
            }
            tmp_viasset->Pos = ChVector<>(u_visual->origin.position.x,
                                          u_visual->origin.position.y,
                                          u_visual->origin.position.z);
            tmp_viasset->Rot = ChMatrix33<>(ChQuaternion<>(u_visual->origin.rotation.w,
                                                           u_visual->origin.rotation.x,
                                                           u_visual->origin.rotation.y,
                                                           u_visual->origin.rotation.z));

            if (u_visual->material){
                const ChMatPair& ch_vis_mat = GetMaterial(u_visual->material_name);
                if (ch_vis_mat.color) {
                    // currently the SetColor method doesn't work, waiting for response from chrono team
                    // the AddAsset method would change color for all vis shapes, so make sure only specify
                    // one color in urdf file
                    // tmp_viasset->SetColor(*(ch_vis_mat.color));
                    ch_body->AddAsset(chrono_types::make_shared<ChColorAsset>(*(ch_vis_mat.color)));
                }
                // undefined behavior if multiple textures have been defined for a single link
                // avoid that!
                if (ch_vis_mat.texture) ch_body->AddAsset(ch_vis_mat.texture);
            }

            ch_body->AddAsset(tmp_viasset);
        }
    }


    // Collision
    // The collsion geometry type could be different from visual, that's why we don't merge them together
    // TODO: Any way to specify collision material in urdf?
    if (u_link->collision){
        auto tmp_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
        ch_body->GetCollisionModel()->ClearModel();
        for (auto u_collision : u_link->collision_array){
            switch (u_collision->geometry->type){
                case urdf::Geometry::BOX:
                {
                    urdf::BoxSharedPtr tmp_urdf_box_ptr = std::dynamic_pointer_cast<urdf::Box>(u_collision->geometry);
                    ch_body->GetCollisionModel()->AddBox(tmp_mat,
                                                         tmp_urdf_box_ptr->dim.x / 2,
                                                         tmp_urdf_box_ptr->dim.y / 2,
                                                         tmp_urdf_box_ptr->dim.z / 2,
                                                         ChVector<>(u_collision->origin.position.x,
                                                                    u_collision->origin.position.y,
                                                                    u_collision->origin.position.z),
                                                         ChMatrix33<>(ChQuaternion<>(u_collision->origin.rotation.w,
                                                                                     u_collision->origin.rotation.x,
                                                                                     u_collision->origin.rotation.y,
                                                                                     u_collision->origin.rotation.z)));
                    break;
                }
                case urdf::Geometry::SPHERE:
                {
                    ch_body->GetCollisionModel()->AddSphere(tmp_mat,
                                                            std::dynamic_pointer_cast<urdf::Sphere>(u_collision->geometry)->radius,
                                                            ChVector<>(u_collision->origin.position.x,
                                                                       u_collision->origin.position.y,
                                                                       u_collision->origin.position.z));
                    break;
                }
                case urdf::Geometry::CYLINDER:
                {
                    urdf::CylinderSharedPtr tmp_urdf_cylinder_ptr = std::dynamic_pointer_cast<urdf::Cylinder>(u_collision->geometry);
                    // Chrono defaults cylinders to y axis, so we need to first rotate it to z axis (urdf default) then apply the rotation stored in urdf
                    ch_body->GetCollisionModel()->AddCylinder(tmp_mat,
                                                              tmp_urdf_cylinder_ptr->radius,
                                                              tmp_urdf_cylinder_ptr->radius,
                                                              tmp_urdf_cylinder_ptr->length / 2,
                                                              ChVector<>(u_collision->origin.position.x,
                                                                         u_collision->origin.position.y,
                                                                         u_collision->origin.position.z),
                                                              ChMatrix33<>(Q_ROTATE_Y_TO_Z >>
                                                                           ChQuaternion<>(u_collision->origin.rotation.w,
                                                                                          u_collision->origin.rotation.x,
                                                                                          u_collision->origin.rotation.y,
                                                                                          u_collision->origin.rotation.z)));
                    break;
                }
                case urdf::Geometry::MESH:
                {
                    // TODO: Let go for now
                    std::cout << "Chrono_urdf: MESH is not supported in urdf-link yet" << std::endl;
                    break;
                }
            }
        }
        ch_body->GetCollisionModel()->BuildModel();
        ch_body->SetCollide(true);
    }

    ch_system_->AddBody(ch_body);

    // then process the joint with parent link
    // TODO: Chrono supports different types of joints, blindly choose ChLinkLock family for now
    if (u_parent_joint){

        std::shared_ptr<ChLinkLock> ch_parent_link;
        switch (u_parent_joint->type){
            case urdf::Joint::REVOLUTE:
                ch_parent_link = chrono_types::make_shared<ChLinkLockRevolute>();
                ch_parent_link->Initialize(ch_body,
                                           ch_parent_body,
                                           ChCoordsys<>(ch_body->GetPos(),
                                                        Q_from_Vect_to_Vect(VECT_Z,
                                                                            ChVector<>(u_parent_joint->axis.x,
                                                                                       u_parent_joint->axis.y,
                                                                                       u_parent_joint->axis.z)).GetNormalized() >> ch_body->GetRot()));
                if (u_parent_joint->limits){
                    ch_parent_link->GetLimit_Rz().SetMin(u_parent_joint->limits->lower);
                    ch_parent_link->GetLimit_Rz().SetMax(u_parent_joint->limits->upper);
                    ch_parent_link->GetLimit_Rz().SetActive(true);
                    std::cout << "joint limits lower: " << u_parent_joint->limits->lower << " , upper: " << u_parent_joint->limits->upper << std::endl;
                    // u_parent_joint->limits->effort - max joint effort
                    // u_parent_joint->limits->velocity - max joint velocity
                }
                break;
            case urdf::Joint::PRISMATIC:
                ch_parent_link = chrono_types::make_shared<ChLinkLockPrismatic>();
                ch_parent_link->Initialize(ch_body,
                                           ch_parent_body,
                                           ChCoordsys<>(ch_body->GetPos(),
                                                        Q_from_Vect_to_Vect(VECT_Z,
                                                                            ChVector<>(u_parent_joint->axis.x,
                                                                                       u_parent_joint->axis.y,
                                                                                       u_parent_joint->axis.z)) >> ch_body->GetRot()));
                break;
            case urdf::Joint::CONTINUOUS:
                // the difference between continuous joint and revolute joint is that
                // continuous joint has no upper or lower limit
                // TODO:Currently using ChLinkLockRevolute, more efficient with ChLinkRevolute?
                ch_parent_link = chrono_types::make_shared<ChLinkLockRevolute>();
                ch_parent_link->Initialize(ch_body,
                                           ch_parent_body,
                                           ChCoordsys<>(ch_body->GetPos(),
                                                        Q_from_Vect_to_Vect(VECT_Z,
                                                                            ChVector<>(u_parent_joint->axis.x,
                                                                                       u_parent_joint->axis.y,
                                                                                       u_parent_joint->axis.z)).GetNormalized() >> ch_body->GetRot()));
                break;
            case urdf::Joint::FLOATING:
                // TODO: Let go for now
                std::cout << "Chrono_urdf: FLOATTING is not supported in urdf-joint yet" << std::endl;
                break;
            case urdf::Joint::PLANAR:
                // TODO: Let go for now
                std::cout << "Chrono_urdf: PLANAR is not supported in urdf-joint yet" << std::endl;
                break;
            case urdf::Joint::FIXED:
                ch_parent_link = chrono_types::make_shared<ChLinkLockLock>();
                ch_parent_link->Initialize(ch_body,
                                           ch_parent_body,
                                           ChCoordsys<>(ch_body->GetPos(),QUNIT));
                break;
            case urdf::Joint::UNKNOWN:
                // TODO: Let go for now
                std::cout << "Chrono_urdf: UNKNOWN is not supported in urdf-joint yet" << std::endl;
                break;
        }
        ch_parent_link->SetNameString(u_parent_joint->name);
        ch_system_->AddLink(ch_parent_link);

        ch_link_bodies_.emplace(u_parent_joint->name, ChLinkBodies{ch_body, ch_parent_body, ch_parent_link});
    }

    // finally process all child links
    for (auto link_iter = u_link->child_links.begin();
         link_iter != u_link->child_links.end();
         ++link_iter) {

        convert_links(*link_iter, ch_body);
    }
    return ch_body;
}

bool ChUrdfDoc::Load_URDF(const std::string& filename) {
    if (urdf_file_ == filename){
        return true;
    }
    urdf_file_ = filename;
    urdf_robot_ = urdf::parseURDFFile(filename);
    u_root_link_ = urdf_robot_->getRoot();

    if (!urdf_robot_){
        std::cerr << "ERROR: Model Parsing the xml failed" << std::endl;
        return false;
    }
    return true;
}

bool ChUrdfDoc::AddtoSystem(const std::shared_ptr<ChSystem>& sys, double x, double y, double z, double rx, double ry, double rz) {
    return AddtoSystem(sys, ChCoordsys<>(ChVector<>(x, y, z), Q_from_Euler123(ChVector<>(rx, ry, rz))));
}

bool ChUrdfDoc::AddtoSystem(const std::shared_ptr<ChSystem>& sys, const ChVector<>& init_pos) {
    return AddtoSystem(sys, ChCoordsys<>(init_pos, QUNIT));
}

bool ChUrdfDoc::AddtoSystem(const std::shared_ptr<ChSystem>& sys, const ChCoordsys<>& init_coord) {
    auto init_pos_body = chrono_types::make_shared<ChBody>();
    init_pos_body->SetCoord(init_coord);
    return AddtoSystem(sys, init_pos_body);
}

bool ChUrdfDoc::AddtoSystem(const std::shared_ptr<ChSystem>& sys, const std::shared_ptr<ChBody>& init_pos_body) {
    if (!urdf_robot_){
        std::cerr << "ERROR: No URDF loaded, call Load_URDF first" << std::endl;
        return false;
    }
    // clear chrono object containers in case this urdf file has been added to system before
    ch_materials_.clear();
    ch_link_bodies_.clear();

    std::cout << "robot name is: " << urdf_robot_->getName() << std::endl;
    ch_system_ = sys;

    convert_materials();

    if (u_root_link_){
        link_idx_ = 0;
        ch_root_body_ = convert_links(u_root_link_, init_pos_body);
    }
    else{
        std::cerr << "ERROR: Could not find root link in file " << urdf_file_ << std::endl;
        return false;
    }

    return true;
}

const ChLinkBodies& ChUrdfDoc::GetLinkBodies(const std::string& name) const {
    if (ch_link_bodies_.find(name) == ch_link_bodies_.end()){
        std::cerr << "Error: robot " << urdf_robot_->getName() << " doesn't contain link " << name << std::endl;
        exit(EXIT_FAILURE);
    }
    return ch_link_bodies_.find(name)->second;
}

void ChUrdfDoc::SetAuxRef(std::unordered_set<std::string>& new_auxrefs){
    auxrefs_ = &new_auxrefs;
}

const std::string& ChUrdfDoc::GetLinkBodyName(const std::string& link_name, int body_idx){
    if (!urdf_robot_){
        std::cerr << "ERROR: No URDF loaded, call Load_URDF first" << std::endl;
        exit(EXIT_FAILURE);
    }

    const urdf::JointConstSharedPtr& u_joint = urdf_robot_->getJoint(link_name);
    if (!u_joint){
        std::cerr << "ERROR: link " << link_name << " not found in " << GetRobotName() << std::endl;
        exit(EXIT_FAILURE);
    }

    if (body_idx == 1){
        return u_joint->child_link_name;
    }
    else if (body_idx == 2){
        return u_joint->parent_link_name;
    }
    else {
        std::cerr << "ERROR: Invalid body_idx, has to be 1 or 2" << std::endl;
        exit(EXIT_FAILURE);
    }
}

}  // END_OF_NAMESPACE____
