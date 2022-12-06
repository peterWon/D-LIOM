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

#include "cartographer/mapping/internal/3d/local_trajectory_builder_3d.h"

#include <memory>

#include "cartographer/common/make_unique.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/internal/3d/scan_matching/rotational_scan_matcher.h"
#include "cartographer/mapping/proto/3d/local_trajectory_builder_options_3d.pb.h"
#include "cartographer/mapping/internal/3d/gravity_factor/gravity_factor.h"



#include <chrono>   
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
using namespace std;
using namespace chrono;
static auto* kLocalSlamLatencyMetric = metrics::Gauge::Null();
static auto* kRealTimeCorrelativeScanMatcherScoreMetric =
    metrics::Histogram::Null();
static auto* kCeresScanMatcherCostMetric = metrics::Histogram::Null();
static auto* kScanMatcherResidualDistanceMetric = metrics::Histogram::Null();
static auto* kScanMatcherResidualAngleMetric = metrics::Histogram::Null();


LocalTrajectoryBuilder3D::LocalTrajectoryBuilder3D(
    const mapping::proto::LocalTrajectoryBuilderOptions3D& options,
    const std::vector<std::string>& expected_range_sensor_ids)
    : options_(options),
      active_submaps_(options.submaps_options()),
      motion_filter_(options.motion_filter_options()),
      real_time_correlative_scan_matcher_(
          common::make_unique<scan_matching::RealTimeCorrelativeScanMatcher3D>(
              options_.real_time_correlative_scan_matcher_options())),
      ceres_scan_matcher_(
          common::make_unique<scan_matching::CeresScanMatcher3D>(
              options_.ceres_scan_matcher_options())),
      accumulated_range_data_{Eigen::Vector3f::Zero(), {}, {}},
      range_data_synchronizer_(expected_range_sensor_ids) {
  scan_period_ = options_.scan_period();
  eable_mannually_discrew_ = options_.eable_mannually_discrew();
  frames_for_static_initialization_ = options_.frames_for_static_initialization();
  frames_for_dynamic_initialization_ = options_.frames_for_dynamic_initialization();
  g_est_win_size_ = options_.frames_for_online_gravity_estimate();
  const float imuAccNoise = options_.imu_options().acc_noise();
  const float imuGyrNoise = options_.imu_options().gyr_noise();
  const float imuAccBiasN = options_.imu_options().acc_bias_noise();
  const float imuGyrBiasN = options_.imu_options().gyr_bias_noise();
  const float imuGravity  = options_.imu_options().gravity();
  const float prior_pose_n = options_.imu_options().prior_pose_noise();
  const float prior_gravity_noise = options_.imu_options().prior_gravity_noise();
  const float ceres_pose_n_t = options_.imu_options().ceres_pose_noise_t();
  const float ceres_pose_n_r = options_.imu_options().ceres_pose_noise_r();
  const float ceres_pose_n_t_1 = 
    options_.imu_options().ceres_pose_noise_t_drift();
  const float ceres_pose_n_r_1 = 
    options_.imu_options().ceres_pose_noise_r_drift();
  preint_param_ = gtsam::PreintegrationParams::MakeSharedU(imuGravity);
  preint_param_->accelerometerCovariance  = gtsam::Matrix33::Identity(3,3) 
    * pow(imuAccNoise, 2);
  preint_param_->gyroscopeCovariance = 
    gtsam::Matrix33::Identity(3,3) * pow(imuGyrNoise, 2);
  preint_param_->integrationCovariance = 
    gtsam::Matrix33::Identity(3,3) * pow(1e-4, 2);

  prior_pose_noise_  = gtsam::noiseModel::Diagonal::Sigmas(
    (gtsam::Vector(6) << prior_pose_n, prior_pose_n, prior_pose_n,
     prior_pose_n, prior_pose_n, prior_pose_n).finished());
  prior_vel_noise_   = gtsam::noiseModel::Isotropic::Sigma(3, 1e4);
  prior_bias_noise_  = gtsam::noiseModel::Isotropic::Sigma(6, 1e-2);
  
  //TODO:
  prior_gravity_noise_ = gtsam::noiseModel::Diagonal::Sigmas(
    (gtsam::Vector(2) << prior_gravity_noise, prior_gravity_noise).finished());

  correction_noise_ = gtsam::noiseModel::Diagonal::Sigmas(
    (gtsam::Vector(6) << 
      ceres_pose_n_t, ceres_pose_n_t, ceres_pose_n_t, 
      ceres_pose_n_r, ceres_pose_n_r, ceres_pose_n_r).finished()); 
  correction_noise_2_ = gtsam::noiseModel::Diagonal::Sigmas(
    (gtsam::Vector(6) << 
      ceres_pose_n_t_1, ceres_pose_n_t_1, ceres_pose_n_t_1, 
      ceres_pose_n_r_1, ceres_pose_n_r_1, ceres_pose_n_r_1).finished()); 
  noise_model_between_bias_ = (gtsam::Vector(6) << 
    imuAccBiasN, imuAccBiasN, imuAccBiasN,
    imuGyrBiasN, imuGyrBiasN, imuGyrBiasN).finished();
  
  // for vins initial integrator initialization
  imu_noise_.ACC_N = imuAccNoise;
  imu_noise_.ACC_W = imuAccBiasN;
  imu_noise_.GYR_N = imuGyrNoise;
  imu_noise_.GYR_W = imuGyrBiasN;
  init_integrator_.reset(new IntegrationBase(INIT_BA, INIT_BW, imu_noise_));
  
  InitCircularBuffers();
}

LocalTrajectoryBuilder3D::~LocalTrajectoryBuilder3D() {}

pcl::PointCloud<pcl::PointXYZI>::Ptr LocalTrajectoryBuilder3D::cvtPointCloud(
    const cartographer::sensor::TimedPointCloudOriginData& point_cloud){
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud;
  pcl_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
  pcl_cloud->resize(point_cloud.ranges.size());
  for (size_t i = 0; i < point_cloud.ranges.size(); ++i) {
    const auto& range = point_cloud.ranges.at(i);
    pcl_cloud->points[i].x = range.point_time[0];
    pcl_cloud->points[i].y = range.point_time[1];
    pcl_cloud->points[i].z = range.point_time[2];
    pcl_cloud->points[i].intensity =  range.point_time[3];
  }
  
  return pcl_cloud;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr LocalTrajectoryBuilder3D::
    cvtPointCloudAndDiscrew(
      const cartographer::sensor::TimedPointCloudOriginData& point_cloud,
      const transform::Rigid3d& rel_trans
      /*rel_trans is the transform from last frame to current frame*/){
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud;
  pcl_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
  pcl_cloud->resize(point_cloud.ranges.size());
  const double sample_period = scan_period_;
  transform::Rigid3d tracking_pose_in_start, tracking_pose_in_end;
  for (size_t i = 0; i < point_cloud.ranges.size(); ++i) {
    const auto& range = point_cloud.ranges.at(i);
    double s = (sample_period + range.point_time[3]) / sample_period;
    // pose in last frame
    InterpolatePose(s, rel_trans, tracking_pose_in_start);
    // transform to current frame
    tracking_pose_in_end = rel_trans.inverse() * tracking_pose_in_start;
    Eigen::Vector3d pt, pt_in_end;
    pt << range.point_time[0], range.point_time[1],range.point_time[2];
    pt_in_end = tracking_pose_in_end * pt;
    pcl_cloud->points[i].x = pt_in_end[0];
    pcl_cloud->points[i].y = pt_in_end[1];
    pcl_cloud->points[i].z = pt_in_end[2];
    pcl_cloud->points[i].intensity =  range.point_time[3];
  }
  
  return pcl_cloud;
}


void LocalTrajectoryBuilder3D::AddImuData(const sensor::ImuData& imu_data) {
  if(!imu_initialized_){
    if(options_.enable_ndt_initialization() && init_integrator_){
      double imu_time = common::ToSecondsStamp(imu_data.time);
      double dt = (last_imu_time_ini_ < 0) 
          ? (1.0 / 500.0) : (imu_time - last_imu_time_ini_);
      last_imu_time_ini_ = imu_time;
      init_integrator_->push_back(
        dt, imu_data.linear_acceleration, imu_data.angular_velocity);
    }else{
      init_imu_buffer_opt_.push_back(imu_data);
    }
  }

  //由于外层调用时需要加锁，因此IMU的添加与点云的添加其实是交替进行的
  if(imu_initialized_ && imu_integrator_opt_){
    // imu_que_opt_.push_back(imu_data);

    double imu_time = common::ToSecondsStamp(imu_data.time);
    double dt = (last_imu_time_opt_ < 0) 
        ? (1.0 / 500.0) : (imu_time - last_imu_time_opt_);
    last_imu_time_opt_ = imu_time;

    // integrate this single imu message
    imu_integrator_opt_->integrateMeasurement(
      gtsam::Vector3(imu_data.linear_acceleration.x(), 
                    imu_data.linear_acceleration.y(),
                    imu_data.linear_acceleration.z()),
      gtsam::Vector3(imu_data.angular_velocity.x(),
                    imu_data.angular_velocity.y(),
                    imu_data.angular_velocity.z()), dt);
    
    gtsam::NavState current_state = 
      imu_integrator_opt_->predict(prev_state_, prev_bias_);
    // 存储递推值用于纠正点云／用于scan matching的初始值
    predicted_states_.push_back({imu_data.time, current_state});
  }
}

void LocalTrajectoryBuilder3D::InitializeStatic(){
  Eigen::Vector3d accel_accum;
  Eigen::Vector3d gyro_accum;
  int num_readings = 0;

  accel_accum.setZero();
  gyro_accum.setZero();

  for(const auto& entry : init_imu_buffer_opt_){
    accel_accum += entry.linear_acceleration;
    gyro_accum += entry.angular_velocity;
    num_readings++;
  }

  Eigen::Vector3d accel_mean = accel_accum / num_readings;
  Eigen::Vector3d gyro_mean = gyro_accum / num_readings;
  
  g_vec_ << 0.0, 0.0, -options_.imu_options().gravity();
  P_.setZero();
  V_.setZero();
  //frame I to frame G
  R_ = Eigen::Quaternion<double>::FromTwoVectors(accel_mean, -g_vec_);
  
  Ba_ = R_.transpose() * g_vec_ + accel_mean;
  Bg_ = gyro_mean;
  InitializeIMU();
}

void LocalTrajectoryBuilder3D::InitilizeByNDT(
  const common::Time& time,
  const sensor::TimedPointCloudOriginData& synchronized_data){
  Rigid3dWithVINSPreintegrator plt;
  double stamp = common::ToSecondsStamp(time);
  
  if(!last_scan_){
    if(last_imu_time_ini_ < 0) return; // still no imu come in
    last_stamp_ = stamp;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cur_scan = cvtPointCloud(
        synchronized_data);
    last_scan_ = cur_scan;
    linear_velocity_ << 0., 0., 0.;
    angular_velocity_ << 0., 0., 0.;
    INIT_BA << 0,0,0;
    INIT_BW << 0,0,0;
    voxel_filter_pre_.setLeafSize(0.2, 0.2, 0.2);
    voxel_filter_cur_.setLeafSize(0.2, 0.2, 0.2);
  
    // Setting scale dependent NDT parameters
    // Setting minimum transformation difference for termination condition.
    ndt_.setTransformationEpsilon(0.01);
    // Setting maximum step size for More-Thuente line search.
    ndt_.setStepSize(0.1);
    //Setting Resolution of NDT grid structure (VoxelGridCovariance).
    ndt_.setResolution(1.0);

    // Setting max number of registration iterations.
    ndt_.setMaximumIterations(35);
    // plt.transform = transform::Rigid3d::Identity();
    // plt.pre_integration = nullptr;
    CHECK(buffered_scan_count_ == 0)
      << "buffered_scan_count_ must be zero here!";  
    all_laser_transforms_[buffered_scan_count_++] = plt;
    init_integrator_->resetIntegration(INIT_BA, INIT_BW, imu_noise_);
    return;
  }else{
    dt_ = stamp - last_stamp_;
    Eigen::Matrix3f R;
    Eigen::Vector3f t;
    // set initial alignment estimate
    // Eigen::Quaternionf init_rotation_q =
    //     transform::AngleAxisVectorToRotationQuaternion(
    //         Eigen::Vector3f(angular_velocity_ * dt_)).normalized();
    Eigen::Quaternionf init_q = init_integrator_->deltaQij().cast<float>();
    Eigen::Matrix3f init_R(init_q);
    Eigen::Vector3f init_t(linear_velocity_ * dt_);
    Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity();
    
    init_guess.block(0,0,3,3) = init_R;
    init_guess.block(0,3,3,1) = init_t;
    
    // discrewing points or not?
    // transform::Rigid3d rel_trans = transform::Rigid3d(
    //     linear_velocity_.cast<double>() * dt_, init_integrator_->delta_q);
    // pcl::PointCloud<pcl::PointXYZI>::Ptr cur_scan = cvtPointCloudAndDiscrew(
    //     synchronized_data, rel_trans);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cur_scan = cvtPointCloud(
        synchronized_data);
    MatchByNDT(last_scan_, cur_scan, init_guess, R, t);
    last_scan_ = cur_scan;
   
    // insert laser transform pair 
    plt.transform = transform::Rigid3d(t.cast<double>(),
      Eigen::Quaterniond(all_laser_transforms_[buffered_scan_count_ - 1]
        .transform.rotation().toRotationMatrix() 
        * R.cast<double>()));
    
    std::shared_ptr<IntegrationBase> imu_int_init_pt;
    imu_int_init_pt.reset(new IntegrationBase(*init_integrator_));
    plt.pre_integration = imu_int_init_pt;
  
    all_laser_transforms_[buffered_scan_count_++] = plt;
    init_integrator_->resetIntegration(INIT_BA, INIT_BW, imu_noise_);

    // update velocity and stamp
    linear_velocity_ = t / dt_;
    angular_velocity_ = transform::RotationQuaternionToAngleAxisVector(
          Eigen::Quaternionf(R)) / dt_;
    last_stamp_ = stamp;
  }

  if(buffered_scan_count_ == frames_for_dynamic_initialization_ + 1){
    if(!AlignWithWorld()){
      LOG(ERROR)<<"Initialization failed! Perform re-initialization...";
      
      buffered_scan_count_ = 1;
      InitCircularBuffers();
    }else{
      // init succeeded, initialize the reference frame's state
      P_ = Ps_[frames_for_dynamic_initialization_];
      V_ = Vs_[frames_for_dynamic_initialization_];
      R_ = Rs_[frames_for_dynamic_initialization_];
      Ba_ = Bas_[frames_for_dynamic_initialization_];
      Bg_ = Bgs_[frames_for_dynamic_initialization_];
      InitializeIMU();
    }
  }
}

void LocalTrajectoryBuilder3D::InitializeIMU(){
  // setting up the IMU integration for IMU prediction
  gtsam::imuBias::ConstantBias prior_imu_bias((gtsam::Vector(6) 
    << Ba_[0], Ba_[1], Ba_[2], Bg_[0], Bg_[1], Bg_[2]).finished());
  imu_integrator_opt_ = 
    new gtsam::PreintegratedImuMeasurements(preint_param_, prior_imu_bias);
  
  Eigen::Quaterniond q(R_);
  gtsam::Pose3 pose_start = gtsam::Pose3(
    gtsam::Rot3::Quaternion(q.w(), q.x(), q.y(), q.z()),
    gtsam::Point3(P_.x(), P_.y(), P_.z()));
  gtsam::Velocity3 v_start = gtsam::Velocity3(V_[0], V_[1], V_[2]);
  LOG(INFO)<<"Initialization paras: ";
  LOG(INFO)<<"P: "<<P_.x()<<","<<P_.y()<<","<<P_.z();
  LOG(INFO)<<"V: "<<V_[0]<<","<<V_[1]<<","<<V_[2];
  LOG(INFO)<<"Q: "<<q.w()<<","<<q.x()<<","<<q.y()<<","<<q.z();
  LOG(INFO)<<"Ba: "<<Ba_[0]<<","<<Ba_[1]<<","<<Ba_[2];
  LOG(INFO)<<"Bg: "<<Bg_[0]<<","<<Bg_[1]<<","<<Bg_[2];
  
  prev_state_ = gtsam::NavState(pose_start, v_start);
  prev_bias_ = prior_imu_bias;
  prev_state_odom_ = prev_state_;
  prev_bias_odom_ = prev_bias_;

  imu_initialized_ = true;
}

std::unique_ptr<LocalTrajectoryBuilder3D::MatchingResult>
LocalTrajectoryBuilder3D::AddRangeData(
    const std::string& sensor_id,
    const sensor::TimedPointCloudData& unsynchronized_data) {
  sensor::TimedPointCloudOriginData synchronized_data 
    = range_data_synchronizer_.AddRangeData(
      sensor_id, unsynchronized_data, eable_mannually_discrew_);
  if (synchronized_data.ranges.empty()) {
    // LOG(INFO) << "Range data collator filling buffer.";
    return nullptr;
  }
  const common::Time& time = synchronized_data.time;
  time_point_cloud_ = time;
  if(!imu_initialized_){
    if(options_.enable_ndt_initialization()){
      InitilizeByNDT(time, synchronized_data);
    }else{
      if(accumulated_frame_num++ > frames_for_static_initialization_){
        InitializeStatic();
      }
    }
    return nullptr;
  }
  
  CHECK(!synchronized_data.ranges.empty());
  CHECK_LE(synchronized_data.ranges.back().point_time[3], 0.1f);
  
  const common::Time time_first_point = time 
    + common::FromSeconds(synchronized_data.ranges.front().point_time[3]);
 
  if (num_accumulated_ == 0) {
    accumulation_started_ = std::chrono::steady_clock::now();
  }

  std::vector<sensor::TimedPointCloudOriginData::RangeMeasurement> hits =
      sensor::VoxelFilter(0.5f * options_.voxel_filter_size())
          .Filter(synchronized_data.ranges);
  
  std::vector<transform::Rigid3f> hits_poses;
  transform::Rigid3d tmp_pose;
  hits_poses.reserve(hits.size());
  bool warned = false;

  //即使是插入子地图的第一帧，也是经过初始化步骤的，同样可以进行相对运动的矫正
  first_scan_to_insert_ = false; 
  if(first_scan_to_insert_){
    hits_poses = std::vector<transform::Rigid3f>(
      hits.size(), PoseFromGtsamNavState(prev_state_).cast<float>());
    first_scan_to_insert_ = false;
  }else{
    size_t idx0, idx1;
    idx0 = idx1 = 0;
    transform::Rigid3d cur_state_pre, rel_trans;
    // FindIdxFromOdomQuene(time, idx0, idx1);
    
    // if(idx0 != idx1){
    //   InterpolatePose(time, predicted_states_.at(idx0).first, 
    //     predicted_states_.at(idx1).first, predicted_states_.at(idx0).second, 
    //     predicted_states_.at(idx1).second, cur_state_pre);
    // }else{
    //   if(idx0 == 0 ){
    //     LOG(WARNING)<<"Maybe IMU quene is empty?";
    //     return nullptr;
    //   }
    //   cur_state_pre = PoseFromGtsamNavState(predicted_states_.at(idx0).second);
    // }
    
    if(predicted_states_.empty()) return nullptr;
    cur_state_pre = PoseFromGtsamNavState(predicted_states_.back().second);
    rel_trans =  PoseFromGtsamNavState(prev_state_).inverse() * cur_state_pre;
    
    if(std::abs(hits.front().point_time[3]) < 1e-3){//没有单点的时间戳
      hits_poses = std::vector<transform::Rigid3f>(
        hits.size(), cur_state_pre.cast<float>());
      LOG(WARNING)<<"Not discrewing!";
    }else{
      const double sample_period = scan_period_;
      for (const auto& hit : hits) {
        common::Time time_point = time + common::FromSeconds(hit.point_time[3]);
        //注意，hit.point_time[3]是小于０的值
        //当前帧的最后一个点为０，帧的时间戳为最后一个点的采集时间
        double s = (sample_period + hit.point_time[3]) / sample_period;
        InterpolatePose(s, rel_trans, tmp_pose);
        hits_poses.push_back(
          (PoseFromGtsamNavState(prev_state_) * tmp_pose).cast<float>());
      }
    }
  }
  TrimStatesCache(time);

  if (num_accumulated_ == 0) {
    // 'accumulated_range_data_.origin' is not used.
    accumulated_range_data_ = sensor::RangeData{{}, {}, {}};
  }
          
  for (size_t i = 0; i < hits.size(); ++i) {
    const Eigen::Vector3f hit_in_local =
        hits_poses[i] * hits[i].point_time.head<3>();
    const Eigen::Vector3f origin_in_local =
        hits_poses[i] * synchronized_data.origins.at(hits[i].origin_index);
    const Eigen::Vector3f delta = hit_in_local - origin_in_local;
    const float range = delta.norm();
    if (range >= options_.min_range()) {
      if (range <= options_.max_range()) {
        accumulated_range_data_.returns.push_back(hit_in_local);
      } else {
        // We insert a ray cropped to 'max_range' as a miss for hits beyond the
        // maximum range. This way the free space up to the maximum range will
        // be updated.
        accumulated_range_data_.misses.push_back(
            origin_in_local + options_.max_range() / range * delta);
      }
    }
  }
  ++num_accumulated_;

  if (num_accumulated_ >= options_.num_accumulated_range_data()) {
    num_accumulated_ = 0;
    transform::Rigid3f current_pose = hits_poses.back();
    
    const sensor::RangeData filtered_range_data = {
        current_pose.translation(),
        sensor::VoxelFilter(options_.voxel_filter_size())
            .Filter(accumulated_range_data_.returns),
        sensor::VoxelFilter(options_.voxel_filter_size())
            .Filter(accumulated_range_data_.misses)};
    return AddAccumulatedRangeData(
      time, current_pose.cast<double>(), 
      sensor::TransformRangeData(filtered_range_data, current_pose.inverse()));
  }
  return nullptr;
}

std::unique_ptr<LocalTrajectoryBuilder3D::MatchingResult>
LocalTrajectoryBuilder3D::AddAccumulatedRangeData(
    const common::Time time,
    const transform::Rigid3d& pose_prediction,
    const sensor::RangeData& filtered_range_data_in_tracking) {
  if (filtered_range_data_in_tracking.returns.empty()) {
    LOG(WARNING) << "Dropped empty range data.";
    return nullptr;
  }

  std::shared_ptr<const mapping::Submap3D> matching_submap =
      active_submaps_.submaps().front();
  transform::Rigid3d initial_ceres_pose =
      matching_submap->local_pose().inverse() * pose_prediction;
  sensor::AdaptiveVoxelFilter adaptive_voxel_filter(
      options_.high_resolution_adaptive_voxel_filter_options());
  const sensor::PointCloud high_resolution_point_cloud_in_tracking =
      adaptive_voxel_filter.Filter(filtered_range_data_in_tracking.returns);
  if (high_resolution_point_cloud_in_tracking.empty()) {
    LOG(WARNING) << "Dropped empty high resolution point cloud data.";
    return nullptr;
  }
  if (options_.use_online_correlative_scan_matching()) {
    // We take a copy since we use 'initial_ceres_pose' as an output argument.
    const transform::Rigid3d initial_pose = initial_ceres_pose;
    double score = real_time_correlative_scan_matcher_->Match(
        initial_pose, high_resolution_point_cloud_in_tracking,
        matching_submap->high_resolution_hybrid_grid(), &initial_ceres_pose);
    kRealTimeCorrelativeScanMatcherScoreMetric->Observe(score);
  }

  transform::Rigid3d pose_observation_in_submap;
  ceres::Solver::Summary summary;

  sensor::AdaptiveVoxelFilter low_resolution_adaptive_voxel_filter(
      options_.low_resolution_adaptive_voxel_filter_options());
  const sensor::PointCloud low_resolution_point_cloud_in_tracking =
      low_resolution_adaptive_voxel_filter.Filter(
          filtered_range_data_in_tracking.returns);
  if (low_resolution_point_cloud_in_tracking.empty()) {
    LOG(WARNING) << "Dropped empty low resolution point cloud data.";
    return nullptr;
  }
  ceres_scan_matcher_->Match(
      (matching_submap->local_pose().inverse() * pose_prediction).translation(),
      initial_ceres_pose,
      {{&high_resolution_point_cloud_in_tracking,
        &matching_submap->high_resolution_hybrid_grid()},
       {&low_resolution_point_cloud_in_tracking,
        &matching_submap->low_resolution_hybrid_grid()}},
      &pose_observation_in_submap, &summary);
  kCeresScanMatcherCostMetric->Observe(summary.final_cost);
  double residual_distance = (pose_observation_in_submap.translation() -
                              initial_ceres_pose.translation())
                                 .norm();
  kScanMatcherResidualDistanceMetric->Observe(residual_distance);
  double residual_angle = pose_observation_in_submap.rotation().angularDistance(
      initial_ceres_pose.rotation());
  
  kScanMatcherResidualAngleMetric->Observe(residual_angle);
  transform::Rigid3d pose_estimate =
      matching_submap->local_pose() * pose_observation_in_submap;

  WindowOptimize(pose_estimate, false);
 
  auto opt_pose = PoseFromGtsamNavState(prev_state_);
  
  const Eigen::Quaterniond gravity_alignment = opt_pose.rotation();
  sensor::RangeData filtered_range_data_in_local = sensor::TransformRangeData(
      filtered_range_data_in_tracking, opt_pose.cast<float>());
  std::unique_ptr<InsertionResult> insertion_result = InsertIntoSubmap(
      time, filtered_range_data_in_local, filtered_range_data_in_tracking,
      high_resolution_point_cloud_in_tracking,
      low_resolution_point_cloud_in_tracking, opt_pose, gravity_alignment);
  auto duration = std::chrono::steady_clock::now() - accumulation_started_;
  kLocalSlamLatencyMetric->Set(
      std::chrono::duration_cast<std::chrono::seconds>(duration).count());
  return common::make_unique<MatchingResult>(MatchingResult{
      time, opt_pose, std::move(filtered_range_data_in_local),
      std::move(insertion_result)});
}

void LocalTrajectoryBuilder3D::AddOdometryData(
    const sensor::OdometryData& odometry_data) {
  if (extrapolator_ == nullptr) {
    // Until we've initialized the extrapolator we cannot add odometry data.
    LOG(INFO) << "Extrapolator not yet initialized.";
    return;
  }
  extrapolator_->AddOdometryData(odometry_data);
}

std::unique_ptr<LocalTrajectoryBuilder3D::InsertionResult>
LocalTrajectoryBuilder3D::InsertIntoSubmap(
    const common::Time time,
    const sensor::RangeData& filtered_range_data_in_local,
    const sensor::RangeData& filtered_range_data_in_tracking,
    const sensor::PointCloud& high_resolution_point_cloud_in_tracking,
    const sensor::PointCloud& low_resolution_point_cloud_in_tracking,
    const transform::Rigid3d& pose_estimate,
    const Eigen::Quaterniond& gravity_alignment) {
  if (motion_filter_.IsSimilar(time, pose_estimate)) {
    return nullptr;
  }
  // Querying the active submaps must be done here before calling
  // InsertRangeData() since the queried values are valid for next insertion.
  std::vector<std::shared_ptr<const mapping::Submap3D>> insertion_submaps;
  for (const std::shared_ptr<mapping::Submap3D>& submap :
       active_submaps_.submaps()) {
    insertion_submaps.push_back(submap);
  }
  active_submaps_.InsertRangeData(filtered_range_data_in_local,
                                  gravity_alignment);
  const Eigen::VectorXf rotational_scan_matcher_histogram =
      scan_matching::RotationalScanMatcher::ComputeHistogram(
          sensor::TransformPointCloud(
              filtered_range_data_in_tracking.returns,
              transform::Rigid3f::Rotation(gravity_alignment.cast<float>())),
          options_.rotational_histogram_size());
  return common::make_unique<InsertionResult>(
      InsertionResult{std::make_shared<const mapping::TrajectoryNode::Data>(
                          mapping::TrajectoryNode::Data{
                              time,
                              gravity_alignment,
                              {},  // 'filtered_point_cloud' is only used in 2D.
                              high_resolution_point_cloud_in_tracking,
                              low_resolution_point_cloud_in_tracking,
                              rotational_scan_matcher_histogram,
                              pose_estimate}),
                      std::move(insertion_submaps)});
}

void LocalTrajectoryBuilder3D::RegisterMetrics(
    metrics::FamilyFactory* family_factory) {
  auto* latency = family_factory->NewGaugeFamily(
      "mapping_internal_3d_local_trajectory_builder_latency",
      "Duration from first incoming point cloud in accumulation to local slam "
      "result");
  kLocalSlamLatencyMetric = latency->Add({});
  auto score_boundaries = metrics::Histogram::FixedWidth(0.05, 20);
  auto* scores = family_factory->NewHistogramFamily(
      "mapping_internal_3d_local_trajectory_builder_scores",
      "Local scan matcher scores", score_boundaries);
  kRealTimeCorrelativeScanMatcherScoreMetric =
      scores->Add({{"scan_matcher", "real_time_correlative"}});
  auto cost_boundaries = metrics::Histogram::ScaledPowersOf(2, 0.01, 100);
  auto* costs = family_factory->NewHistogramFamily(
      "mapping_internal_3d_local_trajectory_builder_costs",
      "Local scan matcher costs", cost_boundaries);
  kCeresScanMatcherCostMetric = costs->Add({{"scan_matcher", "ceres"}});
  auto distance_boundaries = metrics::Histogram::ScaledPowersOf(2, 0.01, 10);
  auto* residuals = family_factory->NewHistogramFamily(
      "mapping_internal_3d_local_trajectory_builder_residuals",
      "Local scan matcher residuals", distance_boundaries);
  kScanMatcherResidualDistanceMetric =
      residuals->Add({{"component", "distance"}});
  kScanMatcherResidualAngleMetric = residuals->Add({{"component", "angle"}});
}

/******************************************************************************/
void LocalTrajectoryBuilder3D::FindIdxFromOdomQuene(
    const common::Time& time, size_t& former, size_t& later){
  //Maybe a simple implementation is ok.
  size_t size = predicted_states_.size();
  
  size_t idx = size - 1;
  for(size_t i = size - 1; i >=0; --i){
    if(common::ToSeconds(predicted_states_.at(i).first - time) < 0){
      break;
    }else{
      idx = i;
    }
  }
  if(idx == (size - 1)){
    former = later = idx;
  }else if(idx == 0){
    former = later = 0;
  }else{
    former = idx - 1;
    later = idx;
  }
}

void LocalTrajectoryBuilder3D::ResetGTSAM(){
    gtsam::ISAM2Params optParameters;
    optParameters.relinearizeThreshold = 0.1;
    optParameters.relinearizeSkip = 1;
    optimizer_ = gtsam::ISAM2(optParameters);

    gtsam::NonlinearFactorGraph new_graph_factors_;
    graph_factors_ = new_graph_factors_;

    gtsam::Values new_graph_values_;
    graph_values_ = new_graph_values_;
}

void LocalTrajectoryBuilder3D::ResetParams(){
  done_first_opt_ = false;
  gtsam_initialized_ = false;
}

void LocalTrajectoryBuilder3D::WindowOptimize(
    const transform::Rigid3d& matched_pose, bool is_drift){
  double currentCorrectionTime = common::ToSecondsStamp(time_point_cloud_);

  float p_x = matched_pose.translation().x();
  float p_y = matched_pose.translation().y();
  float p_z = matched_pose.translation().z();
  float r_x = matched_pose.rotation().x();
  float r_y = matched_pose.rotation().y();
  float r_z = matched_pose.rotation().z();
  float r_w = matched_pose.rotation().w();
  //TODO(wz): check this
  bool degenerate = is_drift;
  
  // bool degenerate = (int)odomMsg->pose.covariance[0] == 1 ? true : false;
  gtsam::Pose3 lidarPose = gtsam::Pose3(
    gtsam::Rot3::Quaternion(r_w, r_x, r_y, r_z), gtsam::Point3(p_x, p_y, p_z));

  // 0. initialize system
  if (gtsam_initialized_ == false){
    ResetGTSAM();
    
    // pop old IMU message
    TrimOldImuData(time_point_cloud_);
    
    // initial pose
    // prev_state_ and prev_bias_ were initialized in 'InitializeIMU'
    prev_pose_ = prev_state_.pose();
    gtsam::PriorFactor<gtsam::Pose3> priorPose(
      X(0), prev_pose_, prior_pose_noise_);
    graph_factors_.add(priorPose);
    // initial velocity
    prev_vel_ = prev_state_.velocity();
    gtsam::PriorFactor<gtsam::Vector3> priorVel(
      V(0), prev_vel_, prior_vel_noise_);
    graph_factors_.add(priorVel);
    // initial bias
    gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(
      B(0), prev_bias_, prior_bias_noise_);
    graph_factors_.add(priorBias);
    // add values
    graph_values_.insert(X(0), prev_pose_);
    graph_values_.insert(V(0), prev_vel_);
    graph_values_.insert(B(0), prev_bias_);
    // optimize once
    optimizer_.update(graph_factors_, graph_values_);
    graph_factors_.resize(0);
    graph_values_.clear();

    imu_integrator_opt_->resetIntegrationAndSetBias(prev_bias_);
    
    key_ = 1;
    gtsam_initialized_ = true;
    return;
  }

  // reset graph for speed
  if (key_ == options_.submaps_options().num_range_data()) {
      // get updated noise before reset
      gtsam::noiseModel::Gaussian::shared_ptr updatedPoseNoise = 
        gtsam::noiseModel::Gaussian::Covariance(
          optimizer_.marginalCovariance(X(key_-1)));
      gtsam::noiseModel::Gaussian::shared_ptr updatedVelNoise  = 
        gtsam::noiseModel::Gaussian::Covariance(
          optimizer_.marginalCovariance(V(key_-1)));
      gtsam::noiseModel::Gaussian::shared_ptr updatedBiasNoise = 
        gtsam::noiseModel::Gaussian::Covariance(
          optimizer_.marginalCovariance(B(key_-1)));
      // reset graph
      ResetGTSAM();
      // add pose
      gtsam::PriorFactor<gtsam::Pose3> priorPose(
        X(0), prev_pose_, updatedPoseNoise);
      graph_factors_.add(priorPose);
      // add velocity
      gtsam::PriorFactor<gtsam::Vector3> priorVel(
        V(0), prev_vel_, updatedVelNoise);
      graph_factors_.add(priorVel);
      // add bias
      gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(
        B(0), prev_bias_, updatedBiasNoise);
      graph_factors_.add(priorBias);
      // gravity factor for the last pose
      if(options_.enable_gravity_factor()){
        if(EstimateGravity()){
          auto ng_G = g_vec_est_G_.normalized();
          gtsam::Unit3 g_z(ng_G[0], ng_G[1], ng_G[2]);
          gtsam::Unit3 g_ref_B(0., 0., -1.);
        
          gtsam::Pose3GravityFactor g_factor(
            X(0), g_z, prior_gravity_noise_, g_ref_B);
          graph_factors_.add(g_factor);
        }
      }
      // add values
      graph_values_.insert(X(0), prev_pose_);
      graph_values_.insert(V(0), prev_vel_);
      graph_values_.insert(B(0), prev_bias_);
      // optimize once
      optimizer_.update(graph_factors_, graph_values_);
      graph_factors_.resize(0);
      graph_values_.clear();

      key_ = 1;
  }

  // integrate imu data and optimize（Done in AddImuData）
  // add imu factor to graph
  const gtsam::PreintegratedImuMeasurements& preint_imu = dynamic_cast<
    const gtsam::PreintegratedImuMeasurements&>(*imu_integrator_opt_);
  gtsam::ImuFactor imu_factor(
    X(key_ - 1), V(key_ - 1), X(key_), V(key_), B(key_ - 1), preint_imu);
  graph_factors_.add(imu_factor);
  
  // add imu bias between factor
  graph_factors_.add(
    gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(
      B(key_ - 1), B(key_), gtsam::imuBias::ConstantBias(),
      gtsam::noiseModel::Diagonal::Sigmas(
        sqrt(imu_integrator_opt_->deltaTij()) * noise_model_between_bias_)));

  // add pose factor
  gtsam::Pose3 curPose = lidarPose;
  gtsam::PriorFactor<gtsam::Pose3> pose_factor(X(key_), curPose, 
    degenerate ? correction_noise_2_ : correction_noise_);
  graph_factors_.add(pose_factor);
  
  // gravity factor for the last pose
  if(options_.enable_gravity_factor()){
    if(EstimateGravity() && (key_ - g_est_win_size_) >= 0){      
      auto ng_G = g_vec_est_G_.normalized();
      gtsam::Unit3 g_z(ng_G[0], ng_G[1], ng_G[2]);
      gtsam::Unit3 g_ref_B(0., 0., -1.);
     
      gtsam::Pose3GravityFactor g_factor(
        X(key_ - g_est_win_size_), g_z, prior_gravity_noise_, g_ref_B);
      graph_factors_.add(g_factor);
    }
  }
  
  // insert predicted values
  gtsam::NavState propState_ = imu_integrator_opt_->predict(
    prev_state_, prev_bias_);
  graph_values_.insert(X(key_), propState_.pose());
  graph_values_.insert(V(key_), propState_.v());
  graph_values_.insert(B(key_), prev_bias_);
  
  // optimize
  optimizer_.update(graph_factors_, graph_values_);
  optimizer_.update();
  graph_factors_.resize(0);
  graph_values_.clear();
  // Overwrite the beginning of the preintegration for the next step.
  gtsam::Values result = optimizer_.calculateEstimate();
  prev_pose_  = result.at<gtsam::Pose3>(X(key_));
  prev_vel_   = result.at<gtsam::Vector3>(V(key_));
  prev_state_ = gtsam::NavState(prev_pose_, prev_vel_);
  prev_bias_  = result.at<gtsam::imuBias::ConstantBias>(B(key_));
  // Reset the optimization preintegration object.
  imu_integrator_opt_->resetIntegrationAndSetBias(prev_bias_);
  TrimOldImuData(time_point_cloud_);
  
  // check optimization
  if (FailureDetection(prev_vel_, prev_bias_)){
    ResetParams();
    return;
  }
  
  ++key_;
  done_first_opt_ = true;
}


transform::Rigid3d LocalTrajectoryBuilder3D::PoseFromGtsamNavState(
    const gtsam::NavState& pose_in){
  return transform::Rigid3d(pose_in.position(), pose_in.quaternion());
}

void LocalTrajectoryBuilder3D::InterpolatePose(
    const double s, 
    const transform::Rigid3d& relative_transform,
    transform::Rigid3d& pose_t){
  Eigen::Quaterniond q = Eigen::Quaterniond::Identity().slerp(
    s, relative_transform.rotation());
  Eigen::Vector3d t = s * relative_transform.translation();
  pose_t = transform::Rigid3d(t, q);
}

void LocalTrajectoryBuilder3D::InterpolatePose(
    const common::Time& t, 
    const common::Time& t1, const common::Time& t2, 
    const gtsam::NavState& pose1, const gtsam::NavState& pose2,
    transform::Rigid3d& pose_t){
  CHECK_LE(t1, t2)<<"t1 must be smaller than t2!";
  double s = common::ToSeconds(t - t1) / common::ToSeconds(t2 - t1);
  transform::Rigid3d trans1 = PoseFromGtsamNavState(pose1);
  transform::Rigid3d trans2 = PoseFromGtsamNavState(pose2);
  transform::Rigid3d relative_transform = trans1.inverse()*trans2;
  transform::Rigid3d inter_pose_in_1;
  InterpolatePose(s, relative_transform, inter_pose_in_1);
  pose_t = trans1 * inter_pose_in_1;
}

bool LocalTrajectoryBuilder3D::FailureDetection(
  const gtsam::Vector3& velCur, const gtsam::imuBias::ConstantBias& biasCur){
  Eigen::Vector3f vel(velCur.x(), velCur.y(), velCur.z());
  if (vel.norm() > 30){
    LOG(WARNING)<<"Large velocity, reset IMU-preintegration!";
    return true;
  }

  Eigen::Vector3f ba(biasCur.accelerometer().x(),
    biasCur.accelerometer().y(), biasCur.accelerometer().z());
  Eigen::Vector3f bg(biasCur.gyroscope().x(), 
    biasCur.gyroscope().y(), biasCur.gyroscope().z());
  if (ba.norm() > 1.0 || bg.norm() > 1.0){
      LOG(WARNING)<<"Large bias, reset IMU-preintegration!";
      return true;
  }
  return false;
}

void LocalTrajectoryBuilder3D::TrimStatesCache(const common::Time& time){
  while(1){
    if(predicted_states_.empty()) break;
    if(common::ToSeconds(predicted_states_.front().first - time) < 0){
      predicted_states_.pop_front();
    }else{
      break;
    }
  }
}

void LocalTrajectoryBuilder3D::TrimOldImuData(const common::Time& time){
  while (!imu_que_opt_.empty()){
    if (common::ToSeconds(imu_que_opt_.front().time - time) < 0) {
      last_imu_time_opt_ = common::ToSecondsStamp(imu_que_opt_.front().time);
      imu_que_opt_.pop_front();
    }
    else
      break;
  }
}        

void LocalTrajectoryBuilder3D::MatchByICP(
  pcl::PointCloud<pcl::PointXYZI>::Ptr pre_scan,
   pcl::PointCloud<pcl::PointXYZI>::Ptr cur_scan){
  // ICP Settings
  static pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
  icp.setMaxCorrespondenceDistance(3);
  icp.setMaximumIterations(100);
  icp.setTransformationEpsilon(1e-6);
  icp.setEuclideanFitnessEpsilon(1e-6);
  icp.setRANSACIterations(0);

  // Align clouds
  auto start = system_clock::now();
  icp.setInputSource(cur_scan);
  icp.setInputTarget(pre_scan);
  pcl::PointCloud<pcl::PointXYZI>::Ptr unused_result(
    new pcl::PointCloud<pcl::PointXYZI>());
  icp.align(*unused_result);
  auto end = system_clock::now();
  auto duration = duration_cast<microseconds>(end - start);
  LOG(WARNING) <<  "ICP Cost " << double(duration.count()) 
      * microseconds::period::num / microseconds::period::den << "s.";
  LOG(INFO)<<icp.hasConverged()<<", "<<icp.getFitnessScore();
  if (icp.hasConverged() == false || icp.getFitnessScore() > 0.3)
      return;
  // Get pose transformation
  float x, y, z, roll, pitch, yaw;
  Eigen::Affine3f correctionLidarFrame;
  correctionLidarFrame = icp.getFinalTransformation();
  float noiseScore = icp.getFitnessScore();
}

void LocalTrajectoryBuilder3D::MatchByNDT(
    pcl::PointCloud<pcl::PointXYZI>::Ptr pre_scan,
    pcl::PointCloud<pcl::PointXYZI>::Ptr cur_scan,
    const Eigen::Matrix4f& init_guess,
    Eigen::Matrix3f& R, Eigen::Vector3f& t){
   // Filtering input scan to roughly 10% of original size 
   // to increase speed of registration.
  filtered_cloud_pre_.reset(new pcl::PointCloud<pcl::PointXYZI>);
  voxel_filter_pre_.setInputCloud(pre_scan);
  voxel_filter_pre_.filter(*filtered_cloud_pre_);
  
  auto start = system_clock::now();
  filtered_cloud_cur_.reset(new pcl::PointCloud<pcl::PointXYZI>);
  voxel_filter_cur_.setInputCloud (cur_scan);
  voxel_filter_cur_.filter (*filtered_cloud_cur_);

  // Setting point cloud to be aligned.
  ndt_.setInputSource(filtered_cloud_cur_);
  // Setting point cloud to be aligned to.
  ndt_.setInputTarget(filtered_cloud_pre_);

  // Calculating required rigid transform to align the input cloud 
  // to the target cloud.
  pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(
    new pcl::PointCloud<pcl::PointXYZI>);
  ndt_.align(*output_cloud, init_guess);

  auto trans = ndt_.getFinalTransformation();
  R = trans.block(0,0,3,3).cast<float>();
  t = trans.block(0,3,3,1).cast<float>();
  
  // lins::Transform transform_to_start;
  // transform_to_start = all_laser_transforms_[
  //     buffered_scan_count_-1].second.transform.rot.toRotationMatrix() * R;
  
  // *filtered_cloud_pre_ += *TransformPointCloudToStart(
  //   filtered_cloud_cur_, transform_to_start);
  auto end = system_clock::now();
  auto duration = duration_cast<microseconds>(end - start);
}

bool LocalTrajectoryBuilder3D::AlignWithWorld() {
  // Check IMU observibility, adapted from VINS-mono
{
  Rigid3dWithVINSPreintegrator laser_trans_j;
  Eigen::Vector3d sum_g;

  for (size_t i = 0; i < frames_for_dynamic_initialization_; ++i) {
    laser_trans_j = all_laser_transforms_[i + 1];
    if(laser_trans_j.pre_integration == nullptr) continue;// first frame maybe nullptr
    double dt = laser_trans_j.pre_integration->deltaTij();
    Eigen::Vector3d tmp_g = laser_trans_j.pre_integration->deltaVij() / dt;
    sum_g += tmp_g;
  }

  Vector3d aver_g;
  aver_g = sum_g * 1.0 / (frames_for_dynamic_initialization_);
  double var = 0;

  for (size_t i = 0; i < frames_for_dynamic_initialization_; ++i) {
    laser_trans_j = all_laser_transforms_[i + 1];
    if(laser_trans_j.pre_integration == nullptr) continue;
    double dt = laser_trans_j.pre_integration->deltaTij();
    Eigen::Vector3d tmp_g = laser_trans_j.pre_integration->deltaVij() / dt;
    var += (tmp_g - aver_g).transpose() * (tmp_g - aver_g);
  }

  var = sqrt(var / (frames_for_dynamic_initialization_));
  
  if (var < 0.25) {
    LOG(WARNING) << "IMU variation: " 
      << var << " is not sufficient, maybe using InitializeStatic instead.";
    return false;
  }
}
  //所有的Vs都在Laser坐标系下，此处的g所采用的坐标系参考为北东地
  Eigen::Vector3d g_vec_in_laser, g_vec_in_base;
  bool init_result = Initializer::Initialization(
    all_laser_transforms_, transform_lb_, 
    options_.imu_options().gravity(), 
    Vs_, Bgs_, g_vec_in_laser);
  if (!init_result) {
    return false;
  }
  
  // update Position and Rotation
  for (size_t i = 0; i < frames_for_dynamic_initialization_ + 1; ++i) {
    const auto& trans_li = all_laser_transforms_[i].transform;
    auto trans_bi = trans_li * transform_lb_;
    Ps_[i] = trans_bi.translation();
    Rs_[i] = trans_bi.rotation().normalized().toRotationMatrix();
    Vs_[i] = Rs_[i] * Vs_[i];
  }
  
  g_vec_ << 0.0, 0.0, -options_.imu_options().gravity();
  
  // frame I to frame G
  // 对照Foster的预积分，感觉上VINS论文中（1）式中的重力项差了一个负号，
  // 否则静止时该式不成立（IMU是敏感不到重力的）
  // 所以这里所求出来的重力向量要取反
  g_vec_in_base = -g_vec_in_laser;
  R_ = Eigen::Quaternion<double>::FromTwoVectors(
    g_vec_in_base.normalized(), g_vec_.normalized());
  Eigen::Vector3d g_est = R_ * g_vec_in_base;
  LOG(WARNING) << "g_est: " << g_est[0] << "," << g_est[1] << "," << g_est[2];
  LOG(WARNING) << "g_vec_in_base: " << g_vec_in_base[0] <<","
               << g_vec_in_base[1] << "," << g_vec_in_base[2];

  // Align with world frame
  for (int i = 0; i < frames_for_dynamic_initialization_ + 1; i++) {
    Ps_[i] = (R_ * Ps_[i]).eval();
    Rs_[i] = (R_ * Rs_[i]).eval();
    Vs_[i] = (R_ * Vs_[i]).eval();
  }

  LOG(WARNING) << "Imu initialization successful!";
  return true;
}

void LocalTrajectoryBuilder3D::InitCircularBuffers(){
  Eigen::Vector3d zero_vec(0., 0., 0.);
  all_laser_transforms_.clear();
  Ps_.clear();
  Rs_.clear();
  Vs_.clear();
  Bas_.clear();
  Bgs_.clear();
  for(size_t i = 0; i < frames_for_dynamic_initialization_+1; ++i){
    all_laser_transforms_.push_back(Rigid3dWithVINSPreintegrator());
    Ps_.push_back(zero_vec);
    Rs_.push_back(Eigen::Matrix3d::Identity());
    Vs_.push_back(zero_vec);
    Bas_.push_back(zero_vec);
    Bgs_.push_back(zero_vec);
  }
}

bool LocalTrajectoryBuilder3D::EstimateGravity(){
  Eigen::Quaterniond rot = Eigen::Quaterniond(
    prev_pose_.rotation().quaternion().w(),
    prev_pose_.rotation().quaternion().x(),
    prev_pose_.rotation().quaternion().y(),
    prev_pose_.rotation().quaternion().z());
  Eigen::Vector3d pos = Eigen::Vector3d(
    prev_pose_.x(), prev_pose_.y(), prev_pose_.z());
  transform::Rigid3d tsf(pos, rot);

  std::shared_ptr<gtsam::PreintegratedImuMeasurements> cur_intgrator;
  cur_intgrator.reset(new gtsam::PreintegratedImuMeasurements(
    *imu_integrator_opt_));
  Rigid3dWithPreintegrator tsf_pre_int(cur_intgrator, tsf);
  
  g_est_transforms_.push_back(tsf_pre_int);
  g_est_Vs_.push_back(
    Eigen::Vector3d(prev_vel_.x(), prev_vel_.y(), prev_vel_.z()));
  if(g_est_transforms_.size() > g_est_win_size_ + 1){
    g_est_transforms_.pop_front();
    g_est_Vs_.pop_front();
    g_est_transforms_tmp_.clear();
    g_est_transforms_tmp_ = g_est_transforms_;
    const auto& T_w_inv = g_est_transforms_.front().transform.inverse();
    for(int i = 0; i < g_est_transforms_.size(); ++i){
      auto& tsf = g_est_transforms_tmp_[i];
      // transform to start frame of window
      tsf.transform = T_w_inv * tsf.transform;
      // transform to cur frame
      const auto& R_w_inv = g_est_transforms_[i].transform.rotation().inverse();
      g_est_Vs_[i] =  R_w_inv * g_est_Vs_[i];
    }
    //注意这里的g_vec_est_B_是窗口里面第一帧的IMU坐标系下的
    if(g_estimator_.Estimate(
      g_est_transforms_tmp_, transform_lb_, g_est_Vs_, 
      options_.imu_options().gravity(), g_vec_est_B_)){
      g_vec_est_G_ = g_est_transforms_.front().transform
          .rotation().toRotationMatrix() * (-g_vec_est_B_);
      
      if(g_vec_est_G_[2] + options_.imu_options().gravity() < 0.5){
        // LOG(INFO) << g_vec_est_G_[0]<<","<<g_vec_est_G_[1]<<","<<g_vec_est_G_[2];
        return true;
      }else{
        return false;
      }
    }
  }
  return false;
}
/* 
pcl::PointCloud<pcl::PointXYZI>::Ptr LocalTrajectoryBuilder3D::
    TransformPointCloudToStart(pcl::PointCloud<pcl::PointXYZI>::Ptr cloudIn, 
                               const lins::Transform& tsf) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudOut(
    new pcl::PointCloud<pcl::PointXYZI>());

  int cloudSize = cloudIn->size();
  cloudOut->resize(cloudSize);

  // Eigen::Affine3f transCur = pcl::getTransformation(transformIn->x, transformIn->y, transformIn->z, transformIn->roll, transformIn->pitch, transformIn->yaw);
  Eigen::Affine3f transCur = tsf.transform();
  #pragma omp parallel for num_threads(4)
  for (int i = 0; i < cloudSize; ++i){
    const auto &pointFrom = cloudIn->points[i];
    cloudOut->points[i].x = transCur(0,0) * pointFrom.x + transCur(0,1) 
        * pointFrom.y + transCur(0,2) * pointFrom.z + transCur(0,3);
    cloudOut->points[i].y = transCur(1,0) * pointFrom.x + transCur(1,1) 
        * pointFrom.y + transCur(1,2) * pointFrom.z + transCur(1,3);
    cloudOut->points[i].z = transCur(2,0) * pointFrom.x + transCur(2,1) 
        * pointFrom.y + transCur(2,2) * pointFrom.z + transCur(2,3);
    cloudOut->points[i].intensity = pointFrom.intensity;
  }
  return cloudOut;
} */

}  // namespace mapping
}  // namespace cartographer
