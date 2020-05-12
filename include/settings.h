#pragma once

//#include <atomic>

struct Settings {
  Settings() {}

  double cam_z_threshold;
  int num_features_per_image;
  bool rotate_features;
  double match_max_dist_2d;
  double feature_match_max_dist;
  double feature_match_test_next_best;
  double reprojection_error_pnp_inlier_threshold_pixel;
  int new_kf_min_inliers;
  float loop_dist_thresh;
  float loop_angle_thresh;
  unsigned int loop_min_inliers;
  bool ba_optimize_intrinsics;
  int ba_verbose;
  bool ba_use_huber;
  double localBA_huber_parameter;
  double globalBA_huber_parameter;
  int localBA_max_iterations;
  int globalBA_max_iterations;
};
