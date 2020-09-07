#ifndef CHURDFDOC_H
#define CHURDFDOC_H

#include "urdf_parser/urdf_parser.h"

#include <map>
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/assets/ChColorAsset.h"
#include "chrono/assets/ChTexture.h"

namespace chrono {

// ChLink stores only the raw pointer of ChBodyFrame
// causing issues when fetching bodies from links
struct ChLinkBodies{
    std::shared_ptr<ChBody> body1;
    std::shared_ptr<ChBody> body2;
    std::shared_ptr<ChLink> link;
};

class ChUrdfDoc {
  public:
    ChUrdfDoc(std::shared_ptr<ChSystem> sys){ robot_system = sys; }

    virtual ~ChUrdfDoc(){
        ch_materials_.clear();
        ch_link_bodies_.clear();
    };

    void SetSystem(std::shared_ptr<ChSystem>& sys){ robot_system = sys; }

    std::shared_ptr<chrono::ChSystem> GetSystem(){ return robot_system; }

    void SetUrdfFile(std::string& filepath){ urdf_file_ = filepath; }

    const std::string& GetUrdfFile(){ return urdf_file_; }

    urdf::ModelInterfaceSharedPtr GetUrdf(){ return urdf_robot; }

    bool Load_URDF(const std::string& filename, double x=0, double y=0, double z=0, double rx=0, double ry=0, double rz=0);
    bool Load_URDF(const std::string& filename, const ChVector<>& init_pos);
    bool Load_URDF(const std::string& filename, const ChCoordsys<>& init_coord);
    bool Load_URDF(const std::string& filename, const std::shared_ptr<ChBody>& init_pos_body);

    std::shared_ptr<chrono::ChSystem> robot_system;
    urdf::ModelInterfaceSharedPtr urdf_robot;

    const std::string& GetRobotName(){ return urdf_robot->getName(); }

    const ChLinkBodies& GetLinkBodies(const std::string& name);

    std::shared_ptr<ChBody> GetRootBody(){ return ch_root_body_; }

  private:
    struct ChMatPair{
        std::shared_ptr<ChColor> color;
        std::shared_ptr<ChTexture> texture;
    };

    const ChMatPair& GetMaterial(const std::string& name) {
        return ch_materials_.find(name)->second;
    }

    bool color_empty(const urdf::Color& test_color);
    std::string urdf_abs_path(const std::string& relative_path);
    std::shared_ptr<ChBody> convert_links(const urdf::LinkConstSharedPtr& u_link,
                                          const std::shared_ptr<ChBody>& ch_parent_body);
    void convert_materials();
    // concatenates the urdf flie path and the relative path to the urdf file
    int link_idx_;
    std::string urdf_file_;
    std::map<std::string, ChMatPair> ch_materials_;
    std::map<std::string, ChLinkBodies> ch_link_bodies_;
    std::shared_ptr<ChBody> ch_root_body_;

};

}  // END_OF_NAMESPACE____

#endif  // END of header
