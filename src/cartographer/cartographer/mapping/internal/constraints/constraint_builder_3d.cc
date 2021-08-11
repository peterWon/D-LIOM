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

#include "cartographer/mapping/internal/constraints/constraint_builder_3d.h"

#include <cmath>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>

#include "Eigen/Eigenvalues"
#include "cartographer/common/make_unique.h"
#include "cartographer/common/math.h"
#include "cartographer/common/thread_pool.h"
#include "cartographer/mapping/proto/scan_matching//ceres_scan_matcher_options_3d.pb.h"
#include "cartographer/mapping/proto/scan_matching//fast_correlative_scan_matcher_options_3d.pb.h"
#include "cartographer/metrics/counter.h"
#include "cartographer/metrics/gauge.h"
#include "cartographer/metrics/histogram.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
namespace constraints {

static auto* kConstraintsSearchedMetric = metrics::Counter::Null();
static auto* kConstraintsFoundMetric = metrics::Counter::Null();
static auto* kGlobalConstraintsSearchedMetric = metrics::Counter::Null();
static auto* kGlobalConstraintsFoundMetric = metrics::Counter::Null();
static auto* kQueueLengthMetric = metrics::Gauge::Null();
static auto* kConstraintScoresMetric = metrics::Histogram::Null();
static auto* kConstraintRotationalScoresMetric = metrics::Histogram::Null();
static auto* kConstraintLowResolutionScoresMetric = metrics::Histogram::Null();
static auto* kGlobalConstraintScoresMetric = metrics::Histogram::Null();
static auto* kGlobalConstraintRotationalScoresMetric =
    metrics::Histogram::Null();
static auto* kGlobalConstraintLowResolutionScoresMetric =
    metrics::Histogram::Null();

ConstraintBuilder3D::ConstraintBuilder3D(
    const proto::ConstraintBuilderOptions& options,
    common::ThreadPoolInterface* const thread_pool)
    : options_(options),
      thread_pool_(thread_pool),
      finish_node_task_(common::make_unique<common::Task>()),
      when_done_task_(common::make_unique<common::Task>()),
      sampler_(options.sampling_ratio()),
      ceres_scan_matcher_(options.ceres_scan_matcher_options_3d()) {}

ConstraintBuilder3D::~ConstraintBuilder3D() {
  common::MutexLocker locker(&mutex_);
  CHECK_EQ(finish_node_task_->GetState(), common::Task::NEW);
  CHECK_EQ(when_done_task_->GetState(), common::Task::NEW);
  // CHECK_EQ(constraints_.size(), 0) << "WhenDone() was not called";
  CHECK_EQ(num_started_nodes_, num_finished_nodes_);
  CHECK(when_done_ == nullptr);
}

void ConstraintBuilder3D::NotifyEndOfNode() {
  common::MutexLocker locker(&mutex_);
  CHECK(finish_node_task_ != nullptr);
  finish_node_task_->SetWorkItem([this] {
    common::MutexLocker locker(&mutex_);
    ++num_finished_nodes_;
  });
  auto finish_node_task_handle =
      thread_pool_->Schedule(std::move(finish_node_task_));
  finish_node_task_ = common::make_unique<common::Task>();
  // when_done_task_->AddDependency(finish_node_task_handle);
  ++num_started_nodes_;
}


// called by PoseGraph3D's optimization timer
void ConstraintBuilder3D::WhenDone(
    const std::function<void(const ConstraintBuilder3D::Result&)>& callback) {
  common::MutexLocker locker(&mutex_);
  CHECK(when_done_ == nullptr);
  // TODO(gaschler): Consider using just std::function, it can also be empty.
  when_done_ =
      common::make_unique<std::function<void(const Result&)>>(callback);
  CHECK(when_done_task_ != nullptr);
  when_done_task_->SetWorkItem([this] { RunWhenDoneCallback(); });
  thread_pool_->Schedule(std::move(when_done_task_));
  when_done_task_ = common::make_unique<common::Task>();
}

void ConstraintBuilder3D::DispatchScanMatcherConstruction(
    const SubmapId& submap_id, 
    const transform::Rigid3d& global_submap_pose,
    const std::vector<std::pair<NodeId, TrajectoryNode>>& submap_nodes,
    const Submap3D* submap) {
  common::MutexLocker locker(&mutex_);
  if (when_done_) {
    LOG(WARNING) << "DispatchScanMatcherConstruction was called"
                 << " while WhenDone was scheduled.";
  }
  if (submap_scan_matchers_.count(submap_id) != 0) {
    return;
  }
  auto& submap_scan_matcher = submap_scan_matchers_[submap_id];
  submap_scan_matcher.high_resolution_hybrid_grid =
      &submap->high_resolution_hybrid_grid();
  submap_scan_matcher.low_resolution_hybrid_grid =
      &submap->low_resolution_hybrid_grid();
  auto& scan_matcher_options =
      options_.fast_correlative_scan_matcher_options_3d();
  std::vector<TrajectoryNode> nodes_wiouout_id={};
  for(const auto& node: submap_nodes){
    nodes_wiouout_id.emplace_back(node.second);
  }
  auto scan_matcher_task = common::make_unique<common::Task>();
  //捕获列表临时变量要以值拷贝的形式传入
  scan_matcher_task->SetWorkItem(
      [=,  &submap_scan_matcher, &scan_matcher_options]() {
        cartographer::common::TicToc tic_toc;
        tic_toc.Tic();
        submap_scan_matcher.fast_correlative_scan_matcher =
            common::make_unique<scan_matching::FastCorrelativeScanMatcher3D>(
              *submap_scan_matcher.high_resolution_hybrid_grid,
              submap_scan_matcher.low_resolution_hybrid_grid, nodes_wiouout_id,
              scan_matcher_options);
        sum_t_cost_ += tic_toc.Toc();
        submap_scan_matcher.global_submap_pose = global_submap_pose;
        submap_scan_matcher.nodes_in_submap = submap_nodes;
        ExtractFeaturesForSubmap(submap_id);
      });
  submap_scan_matcher.creation_task_handle =
      thread_pool_->Schedule(std::move(scan_matcher_task));
  
  auto submap_constraints_task = common::make_unique<common::Task>();
  submap_constraints_task->SetWorkItem([=]() EXCLUDES(mutex_) { 
    ComputeConstraintsBetweenSubmaps(submap_id);
  });
  submap_constraints_task->AddDependency(
    submap_scan_matcher.creation_task_handle);
  auto submap_constraints_task_handle =
      thread_pool_->Schedule(std::move(submap_constraints_task));
  tasks_tracker_.push_back(submap_constraints_task_handle);
  // finish_node_task_->AddDependency(submap_constraints_task_handle);
  when_done_task_->AddDependency(submap_constraints_task_handle);
}

void ConstraintBuilder3D::ComputeConstraintsBetweenSubmaps(
      const SubmapId& submap_id_from) EXCLUDES(mutex_){
  const auto& scan_matcher_from = submap_scan_matchers_.at(submap_id_from);
  // no constraints for this submap
  if(scan_matcher_from.matched_submaps.empty()) return;
  
  for(const auto& submap_id_to: scan_matcher_from.matched_submaps){
    if(submap_scan_matchers_.find(submap_id_to.first) 
        == submap_scan_matchers_.end()){
      LOG(WARNING)<<"This should not happen!";
      continue;
    };
    
    const auto& scan_matcher_to = submap_scan_matchers_.at(submap_id_to.first);
    int j = 0;
    for(const auto& node: scan_matcher_from.nodes_in_submap){
      if(j++ % options_.every_nodes_to_find_constraint() != 0) continue;
      // constraint between the node and the submap has been added before.
      const auto& it = computed_constraints_.find(submap_id_to.first);
      if(it != computed_constraints_.end() 
        && it->second.find(node.first) != it->second.end()){
        continue;
      }
      constraints_.emplace_back();
      kQueueLengthMetric->Set(constraints_.size());
      auto* const constraint = &constraints_.back();
      
      auto constraint_task = common::make_unique<common::Task>();
      constraint_task->SetWorkItem([=]() 
        EXCLUDES(mutex_) {
        ComputeConstraint(
          submap_id_to.first, node.first, submap_id_from, constraint);
      });
      
      auto constraint_task_handle =
          thread_pool_->Schedule(std::move(constraint_task));
    } 
  }
}

void ConstraintBuilder3D::ComputeConstraint(
    const SubmapId& submap_id, const NodeId& node_id, 
    const SubmapId& node_submap_id /*submap_id the node belongs to*/,
    std::unique_ptr<Constraint>* constraint) {
  cartographer::common::TicToc tic_toc;
  tic_toc.Tic();
  // The 'constraint_transform' (submap i <- node j) is computed from:
  // - a 'high_resolution_point_cloud' in node j and
  // - the initial guess 'initial_pose' (submap i <- node j).
  std::unique_ptr<scan_matching::FastCorrelativeScanMatcher3D::Result>
      match_result;
  const SubmapScanMatcher& submap_to_matcher 
      = submap_scan_matchers_.at(submap_id);
  const SubmapScanMatcher& submap_from_matcher 
      = submap_scan_matchers_.at(node_submap_id);

  
  const auto& iter = submap_from_matcher.matched_submaps.find(submap_id);
  if(iter == submap_from_matcher.matched_submaps.end()) {
    LOG(WARNING) << "No matching between submaps, should not get into this function.";
    return;
  }
  // LOG(WARNING) << "Matching " <<node_submap_id.submap_index
  //              <<" to "<< submap_id.submap_index;
  transform::Rigid3d submap_to_submap_2D = iter->second;
  transform::Rigid3d node_pose_in_submap_from;
  

  std::shared_ptr<const TrajectoryNode::Data> constant_data = nullptr;
  const auto nodes = submap_from_matcher.nodes_in_submap;
  for(int i = 0; i < nodes.size(); ++i){
    if(nodes.at(i).first == node_id){
      constant_data = nodes.at(i).second.constant_data;
      node_pose_in_submap_from = nodes.at(i).second.global_pose;
      break;
    }
  } 
  CHECK(constant_data != nullptr) << "Invalid constant_data!";
  // P in submap to match
  // T_G1_S1 * T_G2_G1 * T_S2_G2 * T_N_S2 * P
  transform::Rigid3d gravity_aligned_from = transform::Rigid3d::Rotation(
      submap_from_matcher.global_submap_pose.rotation());
  transform::Rigid3d gravity_aligned_to = transform::Rigid3d::Rotation(
      submap_to_matcher.global_submap_pose.rotation());
  // Remove yaw in map frame
  double yaw_from = transform::GetYaw(gravity_aligned_from);
  auto inv_yaw_from_rot = transform::Embed3D(
      transform::Rigid2d::Rotation(-yaw_from));
  gravity_aligned_from = inv_yaw_from_rot * gravity_aligned_from;

  double yaw_to = transform::GetYaw(gravity_aligned_to);
  auto inv_yaw_to = transform::Embed3D(transform::Rigid2d::Rotation(-yaw_to));
  gravity_aligned_to = inv_yaw_to * gravity_aligned_to;

  const auto& T_G1_S1 = gravity_aligned_to.inverse();
  const auto& T_S2_G2 = gravity_aligned_from;
  transform::Rigid3d node_pose_in_submap_to = T_G1_S1 * submap_to_submap_2D 
      * T_S2_G2 * node_pose_in_submap_from;
  

  // Compute 'pose_estimate' in three stages:
  // 1. Fast estimate using the fast correlative scan matcher.
  // 2. Prune if the score is too low.
  // 3. Refine.
  {
    kConstraintsSearchedMetric->Increment();
    // Match full submap.
    /* match_result =
        submap_to_matcher.fast_correlative_scan_matcher->MatchFullSubmap(
            global_node_pose.rotation(), 
            submap_to_matcher.global_submap_pose.rotation(),
            *constant_data, options_.global_localization_min_score()); */
    // Match with initial guess.
    match_result = submap_to_matcher.fast_correlative_scan_matcher
        ->MatchWith3DofInitial(node_pose_in_submap_to,  
          *constant_data, options_.min_score());
    if (match_result != nullptr) {
      /* auto relative_submap_to_submap = match_result->pose_estimate 
        * (node_pose_in_submap_from.inverse());
      LOG(WARNING)<<node_pose_in_submap_to.translation().x()
                  <<","<<node_pose_in_submap_to.translation().y()
                  <<","<<node_pose_in_submap_to.translation().z();
      LOG(WARNING)<<match_result->pose_estimate.translation().x()
                  <<","<<match_result->pose_estimate.translation().y()
                  <<","<<match_result->pose_estimate.translation().z();
      LOG(WARNING)<<submap_to_submap_2D.translation().x()
                  <<","<<submap_to_submap_2D.translation().y()
                  <<","<<submap_to_submap_2D.translation().z()
                  <<","<<transform::GetYaw(submap_to_submap_2D);
      LOG(WARNING)<<relative_submap_to_submap.translation().x()
                  <<","<<relative_submap_to_submap.translation().y()
                  <<","<<relative_submap_to_submap.translation().z()
                  <<","<<transform::GetYaw(relative_submap_to_submap); */
      
      // We've reported a successful local match.
      CHECK_GT(match_result->score, options_.min_score());
      kConstraintsFoundMetric->Increment();
      kConstraintScoresMetric->Observe(match_result->score);
      kConstraintRotationalScoresMetric->Observe(
          match_result->rotational_score);
      kConstraintLowResolutionScoresMetric->Observe(
          match_result->low_resolution_score);
    } else {
      return;
    }
  }
  {
    common::MutexLocker locker(&mutex_);
    score_histogram_.Add(match_result->score);
    rotational_score_histogram_.Add(match_result->rotational_score);
    low_resolution_score_histogram_.Add(match_result->low_resolution_score);
  }

  // Use the CSM estimate as both the initial and previous pose. This has the
  // effect that, in the absence of better information, we prefer the original
  // CSM estimate.
  ceres::Solver::Summary unused_summary;
  transform::Rigid3d constraint_transform;
  ceres_scan_matcher_.Match(match_result->pose_estimate.translation(),
                            match_result->pose_estimate,
                            {{&constant_data->high_resolution_point_cloud,
                              submap_to_matcher.high_resolution_hybrid_grid},
                             {&constant_data->low_resolution_point_cloud,
                              submap_to_matcher.low_resolution_hybrid_grid}},
                            &constraint_transform, &unused_summary);

  constraint->reset(new Constraint{
      submap_id,
      node_id,
      {constraint_transform, options_.loop_closure_translation_weight(),
       options_.loop_closure_rotation_weight()},
      Constraint::INTER_SUBMAP});
  computed_constraints_[submap_id].insert(node_id);

  if (options_.log_matches()) {
    std::ostringstream info;
    info << "Node " << node_id << " with "
         << constant_data->high_resolution_point_cloud.size()
         << " points on submap " << submap_id << std::fixed;
    info << " matches";
    info << " with score " << std::setprecision(1) << 100. * match_result->score
         << "%.";
    LOG(INFO) << info.str();
  }
  sum_t_cost_ += tic_toc.Toc();
}

void ConstraintBuilder3D::RunWhenDoneCallback() {
  Result result;
  std::unique_ptr<std::function<void(const Result&)>> callback;
  {
    common::MutexLocker locker(&mutex_);
    CHECK(when_done_ != nullptr);
    for (const std::unique_ptr<Constraint>& constraint : constraints_) {
      if (constraint == nullptr) continue;
      result.push_back(*constraint);
    }
    if (options_.log_matches()) {
      LOG(INFO) << constraints_.size() << " computations resulted in "
                << result.size() << " additional constraints.\n"
                << "Score histogram:\n"
                << score_histogram_.ToString(10) << "\n"
                << "Rotational score histogram:\n"
                << rotational_score_histogram_.ToString(10) << "\n"
                << "Low resolution score histogram:\n"
                << low_resolution_score_histogram_.ToString(10);
    }
    constraints_.clear();
    callback = std::move(when_done_);
    when_done_.reset();
    kQueueLengthMetric->Set(constraints_.size());
  }
  (*callback)(result);
}

int ConstraintBuilder3D::GetNumFinishedNodes() {
  common::MutexLocker locker(&mutex_);
  return num_finished_nodes_;
}

double ConstraintBuilder3D::GetCostTime() {
  common::MutexLocker locker(&mutex_);
  return sum_t_cost_;
}

void ConstraintBuilder3D::DeleteScanMatcher(const SubmapId& submap_id) {
  common::MutexLocker locker(&mutex_);
  if (when_done_) {
    LOG(WARNING)
        << "DeleteScanMatcher was called while WhenDone was scheduled.";
  }
  submap_scan_matchers_.erase(submap_id);
  for(auto& constrait: constraints_){
    if(constrait && constrait->submap_id == submap_id){
      constrait = nullptr;
    }
  }
  computed_constraints_.erase(submap_id);
}

void ConstraintBuilder3D::RegisterMetrics(metrics::FamilyFactory* factory) {
  auto* counts = factory->NewCounterFamily(
      "mapping_internal_constraints_constraint_builder_3d_constraints",
      "Constraints computed");
  kConstraintsSearchedMetric =
      counts->Add({{"search_region", "local"}, {"matcher", "searched"}});
  kConstraintsFoundMetric =
      counts->Add({{"search_region", "local"}, {"matcher", "found"}});
  kGlobalConstraintsSearchedMetric =
      counts->Add({{"search_region", "global"}, {"matcher", "searched"}});
  kGlobalConstraintsFoundMetric =
      counts->Add({{"search_region", "global"}, {"matcher", "found"}});
  auto* queue_length = factory->NewGaugeFamily(
      "mapping_internal_constraints_constraint_builder_3d_queue_length",
      "Queue length");
  kQueueLengthMetric = queue_length->Add({});
  auto boundaries = metrics::Histogram::FixedWidth(0.05, 20);
  auto* scores = factory->NewHistogramFamily(
      "mapping_internal_constraints_constraint_builder_3d_scores",
      "Constraint scores built", boundaries);
  kConstraintScoresMetric =
      scores->Add({{"search_region", "local"}, {"kind", "score"}});
  kConstraintRotationalScoresMetric =
      scores->Add({{"search_region", "local"}, {"kind", "rotational_score"}});
  kConstraintLowResolutionScoresMetric = scores->Add(
      {{"search_region", "local"}, {"kind", "low_resolution_score"}});
  kGlobalConstraintScoresMetric =
      scores->Add({{"search_region", "global"}, {"kind", "score"}});
  kGlobalConstraintRotationalScoresMetric =
      scores->Add({{"search_region", "global"}, {"kind", "rotational_score"}});
  kGlobalConstraintLowResolutionScoresMetric = scores->Add(
      {{"search_region", "global"}, {"kind", "low_resolution_score"}});
}

void ConstraintBuilder3D::ExtractFeaturesForSubmap(
    const SubmapId& submap_id){
  cartographer::common::TicToc tic_toc;
  tic_toc.Tic();
  const double resolution = submap_scan_matchers_.at(
    submap_id).high_resolution_hybrid_grid->resolution();
  if(submap_scan_matchers_.at(submap_id).prj_grid.empty()){
    // generate cv Mat for imcomming submap
    submap_scan_matchers_.at(submap_id).prj_grid = ProjectToCvMat(
        submap_scan_matchers_.at(submap_id).high_resolution_hybrid_grid,
        submap_scan_matchers_.at(submap_id).global_submap_pose,
        submap_scan_matchers_.at(submap_id).ox, 
        submap_scan_matchers_.at(submap_id).oy, 
        submap_scan_matchers_.at(submap_id).resolution); 
    cv::Mat& grid = submap_scan_matchers_.at(submap_id).prj_grid;
    cv::threshold(
        grid, grid, options_.cv_binary_threshold(), 255, CV_THRESH_BINARY);
    int se = options_.cv_structure_element_size();  
    auto ele = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(se, se));
    cv::erode(grid, grid, ele);
    surf_detector_->detectAndCompute(grid, cv::noArray(), 
        submap_scan_matchers_.at(submap_id).key_points, 
        submap_scan_matchers_.at(submap_id).descriptors);
    // compute constraints between submaps.
    cv::Point2f tl(submap_scan_matchers_.at(submap_id).ox,
                   submap_scan_matchers_.at(submap_id).oy);
    for(const auto& entry: submap_scan_matchers_){
      const SubmapId& id = entry.first;
      const SubmapScanMatcher& scan_matcher = entry.second;
      
      if(id == submap_id) continue;
      if(scan_matcher.prj_grid.empty()) continue;
      // skip adjacent submaps for they are sharing most of scans and 
      // intra-constraints have been added.
      if(submap_id.trajectory_id == id.trajectory_id 
        && std::abs(submap_id.submap_index - id.submap_index) <= 2){
        continue;
      }
      
      std::vector< std::vector<cv::DMatch> > knn_matches;
      if(submap_scan_matchers_.at(submap_id).key_points.size() >= 2 
          && submap_scan_matchers_.at(id).key_points.size() >= 2){
          surf_matcher_->knnMatch(
            submap_scan_matchers_.at(submap_id).descriptors, 
            submap_scan_matchers_.at(id).descriptors, knn_matches, 2);
      }else{
        continue;
      }
    
      const double r = options_.good_match_ratio_of_distance();
      std::vector<cv::DMatch> good_matches = {};
      for (size_t i = 0; i < knn_matches.size(); i++){
        if (knn_matches[i][0].distance < r * knn_matches[i][1].distance){
          good_matches.push_back(knn_matches[i][0]);
        }
      }

      if(good_matches.size() >= options_.minimum_good_match_num()){ 
        std::vector<cv::Point2f> from_pts={};
        std::vector<cv::Point2f> to_pts={};
        for(int i = 0; i < good_matches.size(); ++i){
          auto gmt = good_matches[i];
          cv::Point2f tl_to(submap_scan_matchers_.at(id).ox,
                            submap_scan_matchers_.at(id).oy);
          from_pts.push_back(tl + submap_scan_matchers_.at(
            submap_id).key_points.at(gmt.queryIdx).pt * resolution);
          to_pts.push_back(tl_to + submap_scan_matchers_.at(
            id).key_points.at(gmt.trainIdx).pt * resolution);
        }
        std::vector<uchar> inliers;
        cv::Mat transform = cv::estimateAffinePartial2D(
            from_pts, to_pts, inliers, 
            cv::RANSAC, options_.ransac_thresh_of_2d_transform_estimate());
        if(!transform.empty()){
          double scale = std::sqrt(
            transform.at<double>(0, 0) * transform.at<double>(0, 0) 
            + transform.at<double>(0, 1) * transform.at<double>(0, 1));
          if(std::abs(scale - 1) > options_.scale_estimated_tolerance()) {
            continue; //no constraint exists
          }else{
            double theta = std::atan2(
              transform.at<double>(1,0), transform.at<double>(1,1));
           
            transform::Rigid2d::Vector pos;
            transform::Rigid2d::Rotation2D rot(theta);
            pos << transform.at<double>(0,2), transform.at<double>(1,2);
            transform::Rigid2d rt2(pos, rot);
            // insert submap-to-submap constraint
            submap_scan_matchers_.at(submap_id).matched_submaps.insert(
              {id, transform::Embed3D(rt2)});
          }
        }
      }
    }
  }
  sum_t_cost_ += tic_toc.Toc();
}

}  // namespace constraints
}  // namespace mapping
}  // namespace cartographer
