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
  tracking_frame = "base_link", -- good map recorded with base link
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = false,
  use_pose_extrapolator = true,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.5,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

TRAJECTORY_BUILDER_3D.min_range = 0.5
TRAJECTORY_BUILDER_3D.max_range = 30.
TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_3D.voxel_filter_size = 0.1
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.max_length = 2.
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.min_num_points = 100
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.max_range = 10.
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.max_length = 10.
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.min_num_points = 200
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.max_range = 30.
TRAJECTORY_BUILDER_3D.use_online_correlative_scan_matching = false
TRAJECTORY_BUILDER_3D.real_time_correlative_scan_matcher.linear_search_window = 0.15
TRAJECTORY_BUILDER_3D.real_time_correlative_scan_matcher.angular_search_window = math.rad(1.)
TRAJECTORY_BUILDER_3D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 1e-1
TRAJECTORY_BUILDER_3D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.occupied_space_weight_0 = 1
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.occupied_space_weight_1 = 4.
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.translation_weight = 15
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.rotation_weight = 10
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.only_optimize_yaw = false
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps = false
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 20
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.ceres_solver_options.num_threads = 1

TRAJECTORY_BUILDER_3D.motion_filter.max_time_seconds = 0.5
TRAJECTORY_BUILDER_3D.motion_filter.max_distance_meters = 0.1
TRAJECTORY_BUILDER_3D.motion_filter.max_angle_radians = 0.01
TRAJECTORY_BUILDER_3D.imu_gravity_time_constant = 10.
TRAJECTORY_BUILDER_3D.rotational_histogram_size = 120
TRAJECTORY_BUILDER_3D.submaps.high_resolution = 0.05
TRAJECTORY_BUILDER_3D.submaps.high_resolution_max_range = 15.
TRAJECTORY_BUILDER_3D.submaps.low_resolution = 0.45
TRAJECTORY_BUILDER_3D.submaps.num_range_data = 140
TRAJECTORY_BUILDER_3D.submaps.range_data_inserter.hit_probability = 0.55
TRAJECTORY_BUILDER_3D.submaps.range_data_inserter.miss_probability = 0.49
TRAJECTORY_BUILDER_3D.submaps.range_data_inserter.num_free_space_voxels = 2

-- POSE_GRAPH.optimize_every_n_nodes = 0
-- POSE_GRAPH.constraint_builder.sampling_ratio = 1.0
-- POSE_GRAPH.constraint_builder.max_constraint_distance = 15.
-- POSE_GRAPH.constraint_builder.min_score = 0.55
-- POSE_GRAPH.constraint_builder.global_localization_min_score = 1.0
-- POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 1.1e4
-- POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 1e5
-- POSE_GRAPH.constraint_builder.log_matches = true

-- POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 7.
-- POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(30.)
-- POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth = 7

-- POSE_GRAPH.constraint_builder.ceres_scan_matcher.occupied_space_weight = 20.
-- POSE_GRAPH.constraint_builder.ceres_scan_matcher.translation_weight = 10.
-- POSE_GRAPH.constraint_builder.ceres_scan_matcher.rotation_weight = 1.
-- POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps = true
-- POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 10
-- POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.num_threads = 1

-- POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.branch_and_bound_depth = 8
-- POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.full_resolution_depth = 3
-- POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.min_rotational_score = 0.55 --\\
-- POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.min_low_resolution_score = 0.55 
-- POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_xy_search_window = 10. --\\
-- POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_z_search_window = 1.
-- POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.angular_search_window = math.rad(15.)

-- POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.occupied_space_weight_0 = 5.
-- POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.occupied_space_weight_1 = 30.
-- POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.translation_weight = 10.
-- POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.rotation_weight = 1.
-- POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.only_optimize_yaw = false
-- POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.ceres_solver_options.use_nonmonotonic_steps = false
-- POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.ceres_solver_options.max_num_iterations = 10
-- POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.ceres_solver_options.num_threads = 1

-- POSE_GRAPH.matcher_translation_weight = 5e2
-- POSE_GRAPH.matcher_rotation_weight = 1.6e3

-- POSE_GRAPH.optimization_problem.huber_scale = 1e1
-- POSE_GRAPH.optimization_problem.acceleration_weight = 1e3
-- POSE_GRAPH.optimization_problem.rotation_weight = 3e5
-- POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1e5
-- POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1e5
-- POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e5
-- POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e5
-- POSE_GRAPH.optimization_problem.fixed_frame_pose_translation_weight = 1e1
-- POSE_GRAPH.optimization_problem.fixed_frame_pose_rotation_weight = 1e2
-- POSE_GRAPH.optimization_problem.log_solver_summary = false
-- POSE_GRAPH.optimization_problem.use_online_imu_extrinsics_in_3d = true
-- POSE_GRAPH.optimization_problem.ceres_solver_options.use_nonmonotonic_steps = false
-- POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 50
-- POSE_GRAPH.optimization_problem.ceres_solver_options.num_threads = 7

-- POSE_GRAPH.max_num_final_iterations = 50
-- POSE_GRAPH.global_sampling_ratio = 0.003
-- POSE_GRAPH.log_residual_histograms = true
-- POSE_GRAPH.global_constraint_search_after_n_seconds = 10.

MAP_BUILDER.use_trajectory_builder_2d = false
MAP_BUILDER.use_trajectory_builder_3d = true
MAP_BUILDER.num_background_threads = 16
-- MAP_BUILDER.pose_graph = POSE_GRAPH
return options