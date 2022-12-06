
#ifndef _CARTOGRAPHER_MAPPING_INTERNAL_3D_RIGID3D_WITH_PREINTEGRATOR_H_
#define _CARTOGRAPHER_MAPPING_INTERNAL_3D_RIGID3D_WITH_PREINTEGRATOR_H_

#include <gtsam/navigation/ImuFactor.h>
#include "cartographer/mapping/internal/3d/initialization/integration_base.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace mapping {

using namespace Eigen;
using Eigen::VectorXd;

// template <typename T>
class Rigid3dWithPreintegrator{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  explicit Rigid3dWithPreintegrator(){
    transform = transform::Rigid3d::Identity();
    pre_integration = nullptr;
  };
  Rigid3dWithPreintegrator(
      const std::shared_ptr<gtsam::PreintegratedImuMeasurements>& imu_integrator, 
      const transform::Rigid3d& tsm){
    pre_integration = imu_integrator;
    transform = tsm;
  }
  transform::Rigid3d transform;
  std::shared_ptr<gtsam::PreintegratedImuMeasurements> pre_integration;
};

// more accurate in initialization via experiments.
class Rigid3dWithVINSPreintegrator{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  explicit Rigid3dWithVINSPreintegrator(){
    transform = transform::Rigid3d::Identity();
    pre_integration = nullptr;
  };
  Rigid3dWithVINSPreintegrator(
      const std::shared_ptr<IntegrationBase>& imu_integrator, 
      const transform::Rigid3d& tsm){
    pre_integration = imu_integrator;
    transform = tsm;
  }
  transform::Rigid3d transform;
  std::shared_ptr<IntegrationBase> pre_integration;
};

} // namespace mapping 
} // namespace cartographer

#endif // _CARTOGRAPHER_MAPPING_INTERNAL_3D_RIGID3D_WITH_PREINTEGRATOR_H_
