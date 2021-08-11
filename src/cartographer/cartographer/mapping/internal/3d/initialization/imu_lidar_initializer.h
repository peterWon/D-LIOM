//
// Created by wz on 05/29/2021. Adapted from VINS-Mono and LIO-Mapping.
//

#ifndef _CARTOGRAPHER_MAPPING_INTERNAL_3D_IMU_LIADR_INITIALIZER_H_
#define _CARTOGRAPHER_MAPPING_INTERNAL_3D_IMU_LIADR_INITIALIZER_H_

#include <deque>
#include <glog/logging.h>
#include "cartographer/mapping/internal/3d/initialization/rigid3d_with_preintegrator.h"
#include "cartographer/transform/rigid_transform.h"


namespace cartographer {
namespace mapping {

class Initializer {
 public:
 // return 'g' in the first frame's local coordinate.
  static bool Initialization(
    std::deque<Rigid3dWithVINSPreintegrator> &all_laser_transforms,
    const transform::Rigid3d &transform_lb,
    const double norm,
    std::deque<Vector3d> &Vs,
    std::deque<Vector3d> &Bgs,
    Vector3d &g);
};

}
}

#endif //_CARTOGRAPHER_MAPPING_INTERNAL_3D_IMU_LIADR_INITIALIZER_H_
