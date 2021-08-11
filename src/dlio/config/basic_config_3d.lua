-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  -- tracking_frame = "imu",
  -- tracking_frame = "imu_enu_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = false,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 0.1,
  trajectory_publish_period_sec = 0.1,
  -- TODO(wz): remove this
  full_map_cloud_publish_period_sec = 5.,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}
-- ============================================
--        TRAJECTORY_BUILDER_3D params (local SLAM)
-- ============================================
TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_3D.imu_gravity_time_constant = .1
TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 1

-- online real-time scan matcher
TRAJECTORY_BUILDER_3D.use_online_correlative_scan_matching = false
TRAJECTORY_BUILDER_3D.real_time_correlative_scan_matcher.linear_search_window = 0.1
TRAJECTORY_BUILDER_3D.real_time_correlative_scan_matcher.angular_search_window = math.rad(3)
TRAJECTORY_BUILDER_3D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 0.1
TRAJECTORY_BUILDER_3D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 0.3

-- submap size
TRAJECTORY_BUILDER_3D.voxel_filter_size = 0.3
TRAJECTORY_BUILDER_3D.submaps.high_resolution = 0.2
TRAJECTORY_BUILDER_3D.submaps.high_resolution_max_range = 60.
-- 150建图效果明显变差
TRAJECTORY_BUILDER_3D.submaps.num_range_data= 100.

-- Manully stamp pointcloud if no timestamp for each point.
TRAJECTORY_BUILDER_3D.eable_mannually_discrew = false

-- Initialization approach, if enable_ndt_initialization is set to 'true',
-- then, window optimization based method will be used to perform initializaiton.
-- Otherwise, static initializaitn will take effect.
TRAJECTORY_BUILDER_3D.frames_for_static_initialization = 7
TRAJECTORY_BUILDER_3D.frames_for_dynamic_initialization = 7
TRAJECTORY_BUILDER_3D.frames_for_online_gravity_estimate = 7
TRAJECTORY_BUILDER_3D.enable_ndt_initialization = false

TRAJECTORY_BUILDER_3D.enable_gravity_factor = true
TRAJECTORY_BUILDER_3D.imu.prior_gravity_noise = 0.01

TRAJECTORY_BUILDER_3D.imu.ceres_pose_noise_t = 0.05
TRAJECTORY_BUILDER_3D.imu.ceres_pose_noise_r = 0.05
TRAJECTORY_BUILDER_3D.imu.ceres_pose_noise_t_drift = 0.3
TRAJECTORY_BUILDER_3D.imu.ceres_pose_noise_r_drift = 0.1

-- No point of trying to SLAM over the points on your car
TRAJECTORY_BUILDER_3D.min_range = 0.5
TRAJECTORY_BUILDER_3D.max_range = 100.
TRAJECTORY_BUILDER_3D.motion_filter.max_time_seconds = 0.5
TRAJECTORY_BUILDER_3D.motion_filter.max_distance_meters = 0.2
TRAJECTORY_BUILDER_3D.motion_filter.max_angle_radians = math.rad(5.)
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.rotation_weight = 4.5e1
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.translation_weight = 6

-- ============================================
--        MAP_BUILDER params (trivial thing that switching 2D or 3D)
-- ============================================
MAP_BUILDER.use_trajectory_builder_3d = true
MAP_BUILDER.num_background_threads = 8

-- ============================================
--        POSE_GRAPH params (global SLAM)
-- ============================================
-- The bigger the Huber scale, the higher is the impact of (potential) outliers.
-- high huber scale allows more outliers with more noisy samples
POSE_GRAPH.optimization_problem.huber_scale = 1e2

--POSE_GRAPH.optimization_problem.rotation_weight = 6e5
POSE_GRAPH.optimize_every_n_nodes = 0
-- POSE_GRAPH.global_constraint_search_after_n_seconds = 3.
POSE_GRAPH.global_sampling_ratio = 0.1

--POSE_GRAPH.constraint_builder.sampling_ratio = 0.03
POSE_GRAPH.constraint_builder.sampling_ratio = 0.05

POSE_GRAPH.constraint_builder.every_nodes_to_find_constraint = 3

POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 10
POSE_GRAPH.constraint_builder.min_score = 0.45

POSE_GRAPH.constraint_builder.global_localization_min_score = 0.45
POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 1e2
POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 1e4

POSE_GRAPH.constraint_builder.max_constraint_distance = 50.
POSE_GRAPH.max_num_final_iterations = 400
-- 局部搜索匹配的窗口,不必太大
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.min_rotational_score = 0.6
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_xy_search_window = 15.
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_z_search_window = 8.
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.angular_search_window = math.rad(45.)
POSE_GRAPH.constraint_builder.ceres_scan_matcher.rotation_weight = 10
POSE_GRAPH.optimization_problem.log_solver_summary = false
POSE_GRAPH.log_residual_histograms = false

POSE_GRAPH.max_radius_eable_loop_detection = 10.
POSE_GRAPH.num_close_submaps_loop_with_initial_value = 5
POSE_GRAPH.nodes_space_to_perform_loop_detection = 1.0



-- return optionss