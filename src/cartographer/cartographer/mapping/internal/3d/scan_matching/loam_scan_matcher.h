#ifndef CARTOGRAPHER_MAPPING_INTERNAL_3D_SCAN_MATCHING_LOAM_SCAN_MATCHER_3D_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_3D_SCAN_MATCHING_LOAM_SCAN_MATCHER_3D_H_

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
 * 实现LOAM的帧间匹配，输入帧为去畸变后的数据
**************************************/

proto::LoamScanMatcherOptions CreateLoamScanMatcherOptions(
    common::LuaParameterDictionary* parameter_dictionary);
using PointCloudAndHybridGridPointers =
    std::pair<const sensor::PointCloud*, const HybridGrid*>;
// This scan matcher uses Ceres to align scans with an existing map.
class LoamScanMatcher {
 public:
  explicit LoamScanMatcher(const proto::LoamScanMatcherOptions& options);

  LoamScanMatcher(const LoamScanMatcher&) = delete;
  LoamScanMatcher& operator=(const LoamScanMatcher&) = delete;

  // Aligns 'point_clouds' within the 'hybrid_grids' given an
  // 'initial_pose_estimate' and returns a 'pose_estimate' and the solver
  // 'summary'.
  void Match(const Eigen::Vector3d& target_translation,
             const transform::Rigid3d& initial_pose_estimate,
             const std::vector<PointCloudAndHybridGridPointers>&
                 point_clouds_and_hybrid_grids,
             transform::Rigid3d* pose_estimate,
             ceres::Solver::Summary* summary);
  
 private:
  const proto::LoamScanMatcherOptions options_;
  ceres::Solver::Options ceres_solver_options_;
};

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer

#endif