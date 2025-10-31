include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,

  -- Frames
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,          -- cartographer publishes map->odom
  publish_frame_projected_to_2d = false,

  -- Sensors
  use_odometry = true,                -- set to false if you don't have /odom
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_point_clouds = 0,
  num_subdivisions_per_laser_scan = 1,   -- REQUIRED by your Cartographer build

  -- Timing
  lookup_transform_timeout_sec = 0.2
  ,submap_publish_period_sec = 0.5
  ,pose_publish_period_sec = 0.02
  ,trajectory_publish_period_sec = 0.05
}

-- Map builder: 2D
MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 4

-- Trajectory builder (front-end)
local tb = TRAJECTORY_BUILDER_2D
tb.min_range = 0.30
tb.max_range = 12.0
tb.missing_data_ray_length = 12.0
tb.use_imu_data = false                 -- set true only if you have IMU in TF
tb.voxel_filter_size = 0.025            -- downsampling

-- Correlative scan matching helps initial alignment
tb.use_online_correlative_scan_matching = true
tb.real_time_correlative_scan_matcher.linear_search_window = 0.1
tb.real_time_correlative_scan_matcher.angular_search_window = 0.1

-- Ceres scan matcher weights (tune if drift or wobble)
tb.ceres_scan_matcher.translation_weight = 10.0
tb.ceres_scan_matcher.rotation_weight = 1.0
tb.ceres_scan_matcher.occupied_space_weight = 1.0

-- Motion filter (reduces redundant scans)
tb.motion_filter.max_time_seconds = 0.5
tb.motion_filter.max_distance_meters = 0.1
tb.motion_filter.max_angle_radians = 0.004

-- Submap / grid settings
tb.submaps.num_range_data = 90                -- scans per submap
tb.submaps.grid_options_2d.resolution = 0.05  -- meters per cell
tb.submaps.grid_options_2d.grid_type = "PROBABILITY_GRID"
-- DO NOT set tb.submaps.range_data_inserter.insert_free_space here (not used in this build)

-- Pose graph (back-end / loop closure)
POSE_GRAPH.optimize_every_n_nodes = 90
POSE_GRAPH.optimization_problem.huber_scale = 1e1
POSE_GRAPH.constraint_builder.min_score = 0.55
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.6
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3
POSE_GRAPH.global_sampling_ratio = 0.003
POSE_GRAPH.max_num_final_iterations = 200

return options
