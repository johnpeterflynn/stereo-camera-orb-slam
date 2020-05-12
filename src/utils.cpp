#include "utils.h"

#include <opengv/absolute_pose/CentralAbsoluteAdapter.hpp>
#include <opengv/absolute_pose/methods.hpp>
#include <opengv/relative_pose/CentralRelativeAdapter.hpp>
#include <opengv/relative_pose/methods.hpp>
#include <opengv/sac/Ransac.hpp>
#include <opengv/sac_problems/absolute_pose/AbsolutePoseSacProblem.hpp>
#include <opengv/sac_problems/relative_pose/CentralRelativePoseSacProblem.hpp>
#include <opengv/triangulation/methods.hpp>

void computeEssential(const Sophus::SE3d& T_0_1, Eigen::Matrix3d& E) {
  const Eigen::Vector3d t_0_1 = T_0_1.translation().normalized();
  const Eigen::Matrix3d R_0_1 = T_0_1.rotationMatrix();

  Eigen::Matrix3d t_hat;
  t_hat << 0, -t_0_1(2), t_0_1(1), t_0_1(2), 0, -t_0_1(0), -t_0_1(1), t_0_1(0),
      0;

  E = t_hat * R_0_1;
}

void findInliersEssential(const Keypoints& kd1, const Keypoints& kd2,
                          const std::shared_ptr<AbstractCamera<double>>& cam1,
                          const std::shared_ptr<AbstractCamera<double>>& cam2,
                          const Eigen::Matrix3d& E,
                          double epipolar_error_threshold, MatchData& md) {
  md.inliers.clear();

  for (size_t j = 0; j < md.matches.size(); j++) {
    const Eigen::Vector2d p0_2d = kd1.corners[md.matches[j].first];
    const Eigen::Vector2d p1_2d = kd2.corners[md.matches[j].second];

    Eigen::Vector3d p0_3d = cam1->unproject(p0_2d);
    Eigen::Vector3d p1_3d = cam2->unproject(p1_2d);

    double epipolar = p0_3d.transpose() * E * p1_3d;

    if (std::abs(epipolar) < epipolar_error_threshold) {
      md.inliers.push_back(md.matches[j]);
    }
  }
}

void project_landmarks(
    const Sophus::SE3d& current_pose,
    const std::shared_ptr<AbstractCamera<double>>& cam,
    const std::vector<Landmark*> landmarks, const double cam_z_threshold,
    std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>&
        projected_points,
    std::vector<Landmark*>& projected_landmarks) {
  static const int IMAGE_WIDTH = 752;
  static const int IMAGE_HEIGHT = 480;

  projected_points.clear();

  projected_landmarks.clear();

  for (const auto& landmark : landmarks) {
    Eigen::Vector3d p_3d_c = current_pose.inverse() * landmark->getWorldPos();

    if (p_3d_c[2] >= cam_z_threshold) {
      Eigen::Vector2d point = cam->project(p_3d_c);

      if (point[0] >= 0 && point[0] <= IMAGE_WIDTH && point[1] >= 0 &&
          point[1] <= IMAGE_HEIGHT) {
        projected_points.push_back(point);
        projected_landmarks.push_back(landmark);
      }
    }
  }
}

int findBestLandmarkDescriptorDist(const std::bitset<256>& corner_desc,
                                   Landmark* landmark,
                                   double feature_match_max_dist) {
  int dist_min = std::numeric_limits<int>::max();
  int dist_valid = -1;

  for (const auto& obs : landmark->getObservations()) {
    std::bitset<256> landmark_obs_desc =
        obs.first->getKeypoints().corner_descriptors[obs.second];

    int dist = (corner_desc ^ landmark_obs_desc).count();

    // Keep track of the smallest distance
    if (dist < dist_min) {
      dist_min = dist;
    }
  }

  if (dist_min < feature_match_max_dist) {
    dist_valid = dist_min;
  }

  return dist_valid;
}

void find_matches_landmarks(
    const Keypoints& kdl,
    const std::vector<Eigen::Vector2d,
                      Eigen::aligned_allocator<Eigen::Vector2d>>&
        projected_points,
    const std::vector<Landmark*>& projected_landmarks,
    const double match_max_dist_2d, const double feature_match_max_dist,
    const double feature_match_test_next_best,
    std::vector<std::pair<Landmark*, FeatureId>>& md) {
  md.clear();

  for (FeatureId corner_fid = 0; corner_fid < kdl.corners.size();
       corner_fid++) {
    std::bitset<256> corner_desc = kdl.corner_descriptors.at(corner_fid);
    int dist_min = std::numeric_limits<int>::max();
    int dist_2ndmin = std::numeric_limits<int>::max();
    Landmark* best_landmark = nullptr;
    Landmark* landmark_min = nullptr;

    for (size_t i = 0; i < projected_points.size(); i++) {
      Landmark* projected_lm = projected_landmarks[i];
      double dist_l2 =
          (kdl.corners.at(corner_fid) - projected_points[i]).norm();

      // If the distance between the corner and projected landmark is smalled
      // than a threshold
      if (dist_l2 <= match_max_dist_2d) {
        int dist = findBestLandmarkDescriptorDist(corner_desc, projected_lm,
                                                  feature_match_max_dist);

        // If there is a valid distance between the corner descriptor and the
        // closest of the landmark descriptors
        if (dist != -1) {
          // Keep track of the two smallest distances
          if (dist < dist_min) {
            landmark_min = projected_lm;
            dist_2ndmin = dist_min;
            dist_min = dist;
          } else if (dist < dist_2ndmin) {
            dist_2ndmin = dist;
          }
        }
      }
    }

    if (dist_min < feature_match_max_dist &&
        dist_2ndmin >= dist_min * feature_match_test_next_best) {
      best_landmark = landmark_min;
    }

    if (best_landmark != nullptr) {
      md.push_back(std::pair<Landmark*, FeatureId>(best_landmark, corner_fid));
    }
  }
}

void localize_camera(const std::shared_ptr<AbstractCamera<double>>& cam,
                     const Keypoints& kdl,
                     const double reprojection_error_pnp_inlier_threshold_pixel,
                     const std::vector<std::pair<Landmark*, FeatureId>>& md,
                     Sophus::SE3d& T_w_c, std::vector<int>& inliers) {
  inliers.clear();

  if (md.size() == 0) {
    T_w_c = Sophus::SE3d();
    return;
  }

  opengv::bearingVectors_t bearingVectors;
  opengv::points_t points;

  for (const auto& lm_fid_entity : md) {
    Landmark* landmark = lm_fid_entity.first;
    FeatureId corner_fid = lm_fid_entity.second;

    Eigen::Vector2d feature_corner = kdl.corners.at(corner_fid);
    Eigen::Vector3d p0_3d = cam->unproject(feature_corner).normalized();

    bearingVectors.push_back(p0_3d);
    points.push_back(landmark->getWorldPos());
  }

  opengv::absolute_pose::CentralAbsoluteAdapter adapter(bearingVectors, points);

  // create a Ransac object
  opengv::sac::Ransac<
      opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem>
      ransac;
  // create an AbsolutePoseSacProblem
  // (algorithm is selectable: KNEIP, GAO, or EPNP)
  std::shared_ptr<opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem>
      absposeproblem_ptr(
          new opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem(
              adapter, opengv::sac_problems::absolute_pose::
                           AbsolutePoseSacProblem::KNEIP));

  double f = 500;
  double threshold =
      1 - cos(atan(reprojection_error_pnp_inlier_threshold_pixel / f));

  // run ransac
  ransac.sac_model_ = absposeproblem_ptr;
  ransac.threshold_ = threshold;
  ransac.max_iterations_ = 500;

  ransac.computeModel();

  // Hypothesis for rotation and translation from RANSAC
  Eigen::Matrix3d initial_rotation =
      ransac.model_coefficients_.block<3,3>(0, 0);
  Eigen::Array3d initial_translation =
      ransac.model_coefficients_.block<3,1>(0, 3);

  // non-linear optimization (using all correspondences)
  // NOTE: Setting the initial rotation/transformation to the RANSAC hypothesis
  // is critical because if there aren't enough inliers to execute
  // optimize_nonlinear() then it will serve as a best guess.
  adapter.sett(initial_translation);
  adapter.setR(initial_rotation);
  opengv::transformation_t nonlinear_transformation =
      opengv::absolute_pose::optimize_nonlinear(adapter, ransac.inliers_);

  // Select and save inliers
  ransac.sac_model_->selectWithinDistance(nonlinear_transformation,
                                          ransac.threshold_, inliers);

  Eigen::Matrix4d best_transformation = Eigen::Matrix4d::Identity();
  best_transformation.block<3, 4>(0, 0) = nonlinear_transformation;

  // Update MatchData with the camera transformation
  T_w_c = Sophus::SE3d(best_transformation);
}
