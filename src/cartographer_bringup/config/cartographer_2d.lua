include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  use_odometry = true,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.5,
  pose_publish_period_sec = 0.02,
  trajectory_publish_period_sec = 0.05,
}

MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 4

local tb = TRAJECTORY_BUILDER_2D
tb.min_range = 0.3
tb.max_range = 12.0
tb.missing_data_ray_length = 12.0
tb.use_imu_data = false
tb.voxel_filter_size = 0.025
tb.use_online_correlative_scan_matching = true
tb.real_time_correlative_scan_matcher.linear_search_window = 0.1
tb.real_time_correlative_scan_matcher.angular_search_window = 0.1
tb.ceres_scan_matcher.translation_weight = 10.0
tb.ceres_scan_matcher.rotation_weight = 1.0
tb.submaps.num_range_data = 90
tb.submaps.grid_options_2d.resolution = 0.05

POSE_GRAPH.optimize_every_n_nodes = 90
POSE_GRAPH.constraint_builder.min_score = 0.55
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.6

return options
