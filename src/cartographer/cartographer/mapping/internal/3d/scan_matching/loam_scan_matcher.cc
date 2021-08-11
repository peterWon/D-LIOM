
#include "cartographer/mapping/internal/3d/scan_matching/loam_scan_matcher.h"

#include <string>
#include <utility>
#include <vector>

#include "cartographer/common/ceres_solver_options.h"
#include "cartographer/common/make_unique.h"
#include "cartographer/mapping/internal/3d/rotation_parameterization.h"
#include "cartographer/mapping/internal/3d/scan_matching/occupied_space_cost_function_3d.h"
#include "cartographer/mapping/internal/3d/scan_matching/rotation_delta_cost_functor_3d.h"
#include "cartographer/mapping/internal/3d/scan_matching/translation_delta_cost_functor_3d.h"
#include "cartographer/mapping/internal/optimization/ceres_pose.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"
#include "ceres/ceres.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {

proto::LoamScanMatcherOptions CreateLoamScanMatcherOptions(
    common::LuaParameterDictionary* const parameter_dictionary) {
  proto::LoamScanMatcherOptions options;
//   for (int i = 0;; ++i) {
//     const std::string lua_identifier =
//         "occupied_space_weight_" + std::to_string(i);
//     if (!parameter_dictionary->HasKey(lua_identifier)) {
//       break;
//     }
//     options.add_occupied_space_weight(
//         parameter_dictionary->GetDouble(lua_identifier));
//   }
//   options.set_translation_weight(
//       parameter_dictionary->GetDouble("translation_weight"));
//   options.set_rotation_weight(
//       parameter_dictionary->GetDouble("rotation_weight"));
//   options.set_only_optimize_yaw(
//       parameter_dictionary->GetBool("only_optimize_yaw"));
  *options.mutable_ceres_solver_options() =
      common::CreateCeresSolverOptionsProto(
          parameter_dictionary->GetDictionary("ceres_solver_options").get());
  return options;
}

LoamScanMatcher::LoamScanMatcher(
    const proto::LoamScanMatcherOptions& options)
    : options_(options),
      ceres_solver_options_(
          common::CreateCeresSolverOptions(options.ceres_solver_options())) {
  ceres_solver_options_.linear_solver_type = ceres::DENSE_QR;
}

void LoamScanMatcher::Match(
    const Eigen::Vector3d& target_translation,
    const transform::Rigid3d& initial_pose_estimate,
    const std::vector<PointCloudAndHybridGridPointers>&
        point_clouds_and_hybrid_grids,
    transform::Rigid3d* const pose_estimate,
    ceres::Solver::Summary* const summary) {
  
}

}
}
}