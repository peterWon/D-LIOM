#ifndef CARTOGRAPHER_MAPPING_INTERNAL_3D_SCAN_MATCHING_LOAM_FEATURE_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_3D_SCAN_MATCHING_LOAM_FEATURE_H_

#include <utility>
#include <vector>

#include "Eigen/Core"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/mapping/3d/hybrid_grid.h"
#include "cartographer/mapping/proto/scan_matching/loam_scan_matcher_options.pb.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {
/***************************************
 * 实现LOAM的特征提取
**************************************/
class LoamFeatureExtractor {
 public:
//   explicit LoamFeatureExtractor(const proto::LoamScanMatcherOptions& options);
  LoamFeatureExtractor();
  LoamFeatureExtractor(const LoamFeatureExtractor&) = delete;
  LoamFeatureExtractor& operator=(const LoamFeatureExtractor&) = delete;
  
  

 private:
};

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer

#endif