
#ifndef _CARTOGRAPHER_MAPPING_INTERNAL_3D_GRAVITY_ESTIMATOR_H_
#define _CARTOGRAPHER_MAPPING_INTERNAL_3D_GRAVITY_ESTIMATOR_H_

#include <memory>
#include <deque>
#include "cartographer/mapping/internal/3d/initialization/rigid3d_with_preintegrator.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace mapping {

using namespace Eigen;
using Eigen::VectorXd;

class GravityEstimator {
 public:
  explicit GravityEstimator() {}
  bool Estimate(
    const std::deque<Rigid3dWithPreintegrator>& all_laser_transforms,
    const transform::Rigid3d &transform_lb,
    const std::deque<Vector3d> &Vs,
    double g_norm,
    Vector3d &g_approx);
private:
  MatrixXd TangentBasis(Vector3d &g0);
  bool ApproximateGravity(
    const std::deque<Rigid3dWithPreintegrator>&all_laser_transforms,
    const transform::Rigid3d &transform_lb, 
    const std::deque<Vector3d> &Vs,
    const double gnorm, Vector3d &g);
  void RefineGravity(
    const std::deque<Rigid3dWithPreintegrator>& all_laser_transforms,
    const transform::Rigid3d &transform_lb,
    const std::deque<Vector3d> &Vs,
    const double gnorm,
    Vector3d &g_approx);
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


} // namespace mapping 
} // namespace cartographer

#endif // _CARTOGRAPHER_MAPPING_INTERNAL_3D_GRAVITY_ESTIMATOR_H_
