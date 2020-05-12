#pragma once
#include "calibration.h"
#include "common_types.h"

#include "keyframe.h"
#include "keypoints.h"
#include "local_mapping.h"
#include "loop_closing.h"
#include "map.h"
#include "settings.h"

class LocalMapping;
class LoopClosing;

class Tracking {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Tracking(Map* map, const Calibration& calib_cam, const Settings& settings);

  void setLocalMapper(LocalMapping* p);

  void setLoopCloser(LoopClosing* p);

  bool run(std::string imgLeft, std::string imgRight);

  Sophus::SE3d getCurrentPose();
  std::vector<Landmark*> getProjectedLandmarks();

  Frame currentFrame;
  Frame mInitialFrame;
  Keyframe* mLastKeyframe;

 protected:
  // Initialization
  void initialize();

  void findKeypointLandmarkInliers();
  void addMatchedLandmarkFeatureMatches(Keyframe* kf);
  bool checkNeedKeyframe();
  void createNewKeyframe();
  std::vector<Landmark*> getCovisibleLandmarks();

  enum State { NOT_INITIALIZED, TRACKING };

  State mState;

  // Variables required for tracking current frame
  std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>
      projected_points;
  std::vector<Landmark*> projected_landmarks;
  std::vector<std::pair<Landmark*, FeatureId>> lm_feature_matches;
  std::vector<int> landmark_inlier_ids;

  Sophus::SE3d currentPose;

  Map* map;
  const Settings& mSettings;
  const Calibration& mCalibCam;

  LocalMapping* localMapper;
  LoopClosing* loopCloser;
};
