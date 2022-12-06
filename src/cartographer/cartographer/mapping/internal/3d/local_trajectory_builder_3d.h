/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_3D_LOCAL_TRAJECTORY_BUILDER_3D_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_3D_LOCAL_TRAJECTORY_BUILDER_3D_H_

#include <chrono>
#include <memory>

#include "cartographer/common/time.h"
#include "cartographer/mapping/3d/submap_3d.h"
#include "cartographer/mapping/internal/3d/scan_matching/ceres_scan_matcher_3d.h"
#include "cartographer/mapping/internal/3d/scan_matching/real_time_correlative_scan_matcher_3d.h"
#include "cartographer/mapping/internal/motion_filter.h"
#include "cartographer/mapping/internal/range_data_collator.h"
#include "cartographer/mapping/pose_extrapolator.h"
#include "cartographer/mapping/proto/3d/local_trajectory_builder_options_3d.pb.h"
#include "cartographer/metrics/family_factory.h"
#include "cartographer/sensor/imu_data.h"
#include "cartographer/sensor/internal/voxel_filter.h"
#include "cartographer/sensor/odometry_data.h"
#include "cartographer/sensor/range_data.h"
#include "cartographer/transform/rigid_transform.h"

#include "cartographer/mapping/internal/3d/range_data_synchronizer.h"
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h> 
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include "cartographer/mapping/internal/3d/initialization/rigid3d_with_preintegrator.h"
#include "cartographer/mapping/internal/3d/initialization/imu_lidar_initializer.h"
#include "cartographer/mapping/internal/3d/gravity_factor/gravity_estimator.h"
#include "cartographer/mapping/internal/3d/gravity_factor/gravity_factor.h"

namespace cartographer {
namespace mapping {

// Wires up the local SLAM stack (i.e. pose extrapolator, scan matching, etc.)
// without loop closure.
using gtsam::symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using gtsam::symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)
class LocalTrajectoryBuilder3D {
 public:
  struct InsertionResult {
    std::shared_ptr<const mapping::TrajectoryNode::Data> constant_data;
    std::vector<std::shared_ptr<const mapping::Submap3D>> insertion_submaps;
  };
  struct MatchingResult {
    common::Time time;
    transform::Rigid3d local_pose;
    sensor::RangeData range_data_in_local;
    // 'nullptr' if dropped by the motion filter.
    std::unique_ptr<const InsertionResult> insertion_result;
  };

  explicit LocalTrajectoryBuilder3D(
      const mapping::proto::LocalTrajectoryBuilderOptions3D& options,
      const std::vector<std::string>& expected_range_sensor_ids);
  ~LocalTrajectoryBuilder3D();

  LocalTrajectoryBuilder3D(const LocalTrajectoryBuilder3D&) = delete;
  LocalTrajectoryBuilder3D& operator=(const LocalTrajectoryBuilder3D&) = delete;

  void AddImuData(const sensor::ImuData& imu_data);
  // Returns 'MatchingResult' when range data accumulation completed,
  // otherwise 'nullptr'.  `TimedPointCloudData::time` is when the last point in
  // `range_data` was acquired, `TimedPointCloudData::ranges` contains the
  // relative time of point with respect to `TimedPointCloudData::time`.
  std::unique_ptr<MatchingResult> AddRangeData(
      const std::string& sensor_id,
      const sensor::TimedPointCloudData& range_data);
  void AddOdometryData(const sensor::OdometryData& odometry_data);

  static void RegisterMetrics(metrics::FamilyFactory* family_factory);

private:
  /*****************************************************************/
  void WindowOptimize(const transform::Rigid3d& matched_pose, bool is_drift);
  void InitializeIMU();
  void InitializeStatic();
  void ResetGTSAM();
  void FindIdxFromOdomQuene(
    const common::Time& time, size_t& former, size_t& later);
  transform::Rigid3d PoseFromGtsamNavState(const gtsam::NavState& pose_in);
  void InterpolatePose(const common::Time& t, 
    const common::Time& t1, const common::Time& t2, 
    const gtsam::NavState& pose1, const gtsam::NavState& pose2,
    transform::Rigid3d& pose_t);
  void InterpolatePose(const double timestamp_ratio, 
    const transform::Rigid3d& relative_transform,
    transform::Rigid3d& pose_t);
  void TrimStatesCache(const common::Time& time);
  void TrimOldImuData(const common::Time& time);
  bool FailureDetection(const gtsam::Vector3& velCur,
    const gtsam::imuBias::ConstantBias& biasCur);
  void ResetParams();
  void MatchByICP(
    pcl::PointCloud<pcl::PointXYZI>::Ptr pre_scan,
    pcl::PointCloud<pcl::PointXYZI>::Ptr cur_scan);
  void InitilizeByICP(const common::Time time,
    pcl::PointCloud<pcl::PointXYZI>::Ptr cur_scan);
  void MatchByNDT(
    pcl::PointCloud<pcl::PointXYZI>::Ptr pre_scan,
    pcl::PointCloud<pcl::PointXYZI>::Ptr cur_scan,
    const Eigen::Matrix4f& initial_guess,
    Eigen::Matrix3f& R, Eigen::Vector3f& t);
  void InitilizeByNDT(const common::Time& time, 
    const sensor::TimedPointCloudOriginData& synchronized_data
    /* pcl::PointCloud<pcl::PointXYZI>::Ptr cur_scan */);
  bool AlignWithWorld();
  void InitCircularBuffers();
  bool EstimateGravity();
  
 private:
  pcl::PointCloud<pcl::PointXYZI>::Ptr cvtPointCloud(
    const cartographer::sensor::TimedPointCloudOriginData& cloud);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cvtPointCloudAndDiscrew(
    const cartographer::sensor::TimedPointCloudOriginData& cloud,
    const transform::Rigid3d& rel_trans);

  std::unique_ptr<MatchingResult> AddAccumulatedRangeData(
      common::Time time,
      const transform::Rigid3d& pose_prediction,
      const sensor::RangeData& filtered_range_data_in_tracking);

  std::unique_ptr<InsertionResult> InsertIntoSubmap(
      common::Time time, const sensor::RangeData& filtered_range_data_in_local,
      const sensor::RangeData& filtered_range_data_in_tracking,
      const sensor::PointCloud& high_resolution_point_cloud_in_tracking,
      const sensor::PointCloud& low_resolution_point_cloud_in_tracking,
      const transform::Rigid3d& pose_estimate,
      const Eigen::Quaterniond& gravity_alignment);

  const mapping::proto::LocalTrajectoryBuilderOptions3D options_;
  mapping::ActiveSubmaps3D active_submaps_;

  mapping::MotionFilter motion_filter_;
  std::unique_ptr<scan_matching::RealTimeCorrelativeScanMatcher3D>
      real_time_correlative_scan_matcher_;
  std::unique_ptr<scan_matching::CeresScanMatcher3D> ceres_scan_matcher_;

  std::unique_ptr<mapping::PoseExtrapolator> extrapolator_;

  int num_accumulated_ = 0;
  sensor::RangeData accumulated_range_data_;
  std::chrono::steady_clock::time_point accumulation_started_;

  // RangeDataCollator range_data_collator_;
  RangeDataSynchronizer range_data_synchronizer_;

/**************************************************************/
  double scan_period_;
  bool eable_mannually_discrew_;
  int frames_for_static_initialization_ = 7;
  int accumulated_frame_num = 0;
  common::Time time_point_cloud_;
  double last_imu_time_opt_ = -1.;
  double last_imu_time_ini_ = -1.;
  int key_ = 1;
  double delta_t_ = 0;
  bool gtsam_initialized_ = false;
  bool done_first_opt_ = false;
  bool imu_initialized_ = false;
  bool first_scan_to_insert_ = true;

  gtsam::ISAM2 optimizer_;
  gtsam::NonlinearFactorGraph graph_factors_;
  gtsam::Values graph_values_;

  gtsam::noiseModel::Diagonal::shared_ptr prior_pose_noise_;
  gtsam::noiseModel::Diagonal::shared_ptr prior_vel_noise_;
  gtsam::noiseModel::Diagonal::shared_ptr prior_bias_noise_;
  gtsam::noiseModel::Diagonal::shared_ptr prior_gravity_noise_;
  gtsam::noiseModel::Diagonal::shared_ptr correction_noise_;
  gtsam::noiseModel::Diagonal::shared_ptr correction_noise_2_;
  gtsam::Vector noise_model_between_bias_;

  boost::shared_ptr<gtsam::PreintegrationParams> preint_param_ = nullptr;
  gtsam::PreintegratedImuMeasurements *imu_integrator_opt_ = nullptr;
  
  std::deque<sensor::ImuData> imu_que_opt_;
  std::deque<std::pair<common::Time, gtsam::NavState>> predicted_states_;

  gtsam::Pose3 prev_pose_;
  gtsam::Vector3 prev_vel_;
  gtsam::NavState prev_state_;
  gtsam::imuBias::ConstantBias prev_bias_;

  gtsam::NavState prev_state_odom_;
  gtsam::imuBias::ConstantBias prev_bias_odom_;
  
  //IMU initialization variables
  Eigen::Vector3d P_, V_, Ba_, Bg_;
  Eigen::Matrix3d R_;

  //For window optimization initialization
  int buffered_scan_count_ = 0;
  int frames_for_dynamic_initialization_ = 7;
  double last_stamp_ = -1.0;
  double dt_ = 0.1;
  pcl::PointCloud<pcl::PointXYZI>::Ptr last_scan_;
  Eigen::Vector3f linear_velocity_;
  Eigen::Vector3f angular_velocity_;
  
  std::deque<sensor::ImuData> init_imu_buffer_opt_;
  sensor::ImuData last_imu_;

  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_pre_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_cur_;
  pcl::ApproximateVoxelGrid<pcl::PointXYZI> voxel_filter_pre_;
  pcl::ApproximateVoxelGrid<pcl::PointXYZI> voxel_filter_cur_;
  pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt_;

  // Initilization by linear alignment
  std::shared_ptr<IntegrationBase> init_integrator_;
  // gtsam::imuBias::ConstantBias zero_imu_bias_;
  Eigen::Vector3d INIT_BA = Eigen::Vector3d::Zero();
  Eigen::Vector3d INIT_BW = Eigen::Vector3d::Zero();
  IMUNoise imu_noise_;
  
  // These variables should be with size of frames_for_dynamic_initialization_ + 1
  std::deque<Rigid3dWithVINSPreintegrator> all_laser_transforms_;
  std::deque<Eigen::Vector3d> Ps_;
  std::deque<Eigen::Matrix3d> Rs_;
  std::deque<Eigen::Vector3d> Vs_;
  std::deque<Eigen::Vector3d> Bas_;
  std::deque<Eigen::Vector3d> Bgs_;

  Eigen::Vector3d g_vec_; // always be ~(0,0,-9.8)
  Eigen::Vector3d g_vec_est_B_; // in body(IMU) frame
  Eigen::Vector3d g_vec_est_G_; // in Global(ENU) frame
  transform::Rigid3d transform_lb_ = transform::Rigid3d::Identity();

  // for gravity factor
  GravityEstimator g_estimator_;
  int g_est_win_size_ = frames_for_dynamic_initialization_;
  std::deque<Rigid3dWithPreintegrator> g_est_transforms_;
  std::deque<Rigid3dWithPreintegrator> g_est_transforms_tmp_;
  std::deque<Eigen::Vector3d> g_est_Vs_;
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_3D_LOCAL_TRAJECTORY_BUILDER_3D_H_
