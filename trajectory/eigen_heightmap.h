#ifndef EIGEN_HEIGHTMAP_H_
#define EIGEN_HEIGHTMAP_H_

#include <Eigen/Core>
#include <towr/terrain/height_map.h>

namespace towr {

class EigenHeightMap : public HeightMap {
public:
  // env_x: length of environment on x direction
  // env_y: length of environment on y direction
  EigenHeightMap(const std::shared_ptr<const Eigen::MatrixXd>& heightmap, double env_x, double env_y);
  double GetHeight(double x, double y)  const override;

  double GetHeightDerivWrtX (double x, double y) const override;
  double GetHeightDerivWrtY (double x, double y) const override;
  double GetHeightDerivWrtXX(double x, double y) const override;
  double GetHeightDerivWrtXY(double x, double y) const override;
  double GetHeightDerivWrtYX(double x, double y) const override;
  double GetHeightDerivWrtYY(double x, double y) const override;

private:
  // keep a copy of shared_ptr to make sure the data is not destructed elsewhere
  std::shared_ptr<const Eigen::MatrixXd> heightmap_ptr;
  // make a reference for coding simplicity
  const Eigen::MatrixXd& heightmap_;
  double x_scale_;
  double y_scale_;
};


} /* namespace towr */

#endif /* EIGEN_HEIGHTMAP_H_ */
