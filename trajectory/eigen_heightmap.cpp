#include "eigen_heightmap.h"
#include <math.h>

#define EIGEN_HEIGHTMAP_LINEAR_INTERPOLATION
namespace towr {

EigenHeightMap::EigenHeightMap(const std::shared_ptr<const Eigen::MatrixXd>& heightmap,
                               double env_x, double env_y) :
    heightmap_ptr(heightmap), heightmap_(*heightmap_ptr),
    x_scale_(double(heightmap->rows()) / env_x),
    y_scale_(double(heightmap->cols()) / env_y) {}

double EigenHeightMap::GetHeight(double coord_x, double coord_y) const {
    double index_x = coord_x * x_scale_;
    double index_y = coord_y * y_scale_;

    // TODO: better interpolation of heightmap
    double coord_z = heightmap_(floor(index_x), floor(index_y));

    return coord_z;
}

#ifdef EIGEN_HEIGHTMAP_LINEAR_INTERPOLATION
double EigenHeightMap::GetHeightDerivWrtX (double coord_x, double coord_y) const {
    double index_x = coord_x * x_scale_;
    double index_y = coord_y * y_scale_;
    int fx = floor(index_x);
    int fy = floor(index_y);
    int cx = ceil(index_x);
    int cy = ceil(index_y);
    double z_min = (heightmap_(fx,cy) + heightmap_(fx,fy)) / (cy + fy) * index_y;
    double z_max = (heightmap_(cx,cy) + heightmap_(cx,fy)) / (cy + fy) * index_y;
    return (z_max - z_min) / (cx - fx);
}

double EigenHeightMap::GetHeightDerivWrtY (double coord_x, double coord_y) const {
    double index_x = coord_x * x_scale_;
    double index_y = coord_y * y_scale_;
    int fx = floor(index_x);
    int fy = floor(index_y);
    int cx = ceil(index_x);
    int cy = ceil(index_y);
    double z_min = (heightmap_(fx,fy) + heightmap_(cx,fy)) / (cx + fx) * index_x;
    double z_max = (heightmap_(fx,cy) + heightmap_(cx,cy)) / (cx + fx) * index_x;
    return (z_max - z_min) / (cy - fy);
}

// since we are using linear interpolation between points for the heightmap,
// all second order derivatives are zero
double EigenHeightMap::GetHeightDerivWrtXX(double x, double y) const { return 0.0; }
double EigenHeightMap::GetHeightDerivWrtXY(double x, double y) const { return 0.0; }
double EigenHeightMap::GetHeightDerivWrtYX(double x, double y) const { return 0.0; }
double EigenHeightMap::GetHeightDerivWrtYY(double x, double y) const { return 0.0; }
#endif /* EIGEN_HEIGHTMAP_LINEAR_INTERPOLATION */

#ifdef EIGEN_HEIGHTMAP_QUADRATIC_INTERPOLATION
// TODO
#endif /* EIGEN_HEIGHTMAP_QUADRATIC_INTERPOLATION */

} /* namespace towr */
