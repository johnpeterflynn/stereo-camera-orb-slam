#include "tracking.h"
#include "utils.h"

Tracking::Tracking(Map* m, const Calibration& calibCam,
                   const Settings& settings)
    : mLastKeyframe(nullptr),
      mState(State::NOT_INITIALIZED),
      map(m),
      mSettings(settings),
      mCalibCam(calibCam) {}

void Tracking::setLocalMapper(LocalMapping* p) { localMapper = p; }

void Tracking::setLoopCloser(LoopClosing* p) { loopCloser = p; }

Sophus::SE3d Tracking::getCurrentPose() { return currentPose; }

// API to be called by main function to track given stereo pair
bool Tracking::run(std::string imgLeft, std::string imgRight) {
  currentFrame = Frame(imgLeft, imgRight, mSettings.num_features_per_image,
                       mSettings.rotate_features);

  TimeCamId tcidl(currentFrame.mFrameId, 0);
  map->feature_corners[tcidl] = currentFrame.getLeftKeypoints();

  if (mState == NOT_INITIALIZED) {
    initialize();
    return true;
  } else {
    findKeypointLandmarkInliers();

    Sophus::SE3d T_w_c;

    localize_camera(mCalibCam.intrinsics[0], currentFrame.getLeftKeypoints(),
                    mSettings.reprojection_error_pnp_inlier_threshold_pixel,
                    lm_feature_matches, T_w_c, landmark_inlier_ids);

    // TODO: Maybe we should use the pose from last frame in case
    // localize_camera fails to find a good T_w_c
    currentPose = T_w_c;
    currentFrame.setCamToWorld(currentPose);

    if (checkNeedKeyframe()) {
      createNewKeyframe();
      return true;
    }
  }
  return false;
}

void Tracking::initialize() {
  // Q: Should we use the pose stored in currentFrame in place of currentPose in
  // this class?
  currentPose = Sophus::SE3d();

  // TODO: Should test here whether or not enough keypoints exist to create the
  // first keyframe. If not, don't leave inltialization state

  findKeypointLandmarkInliers();

  createNewKeyframe();

  // TODO: Check if finding matches and creating keyframe succeeds/fails before
  // changing state to TRACKING
  mState = State::TRACKING;
}

bool Tracking::checkNeedKeyframe() {
  bool needKeyframe = false;
  if (int(landmark_inlier_ids.size()) < mSettings.new_kf_min_inliers) {
    needKeyframe = true;
  }

  return needKeyframe;
}

void Tracking::createNewKeyframe() {
  Keyframe* kf = new Keyframe(currentFrame, map);
  Sophus::SE3d stereo = mCalibCam.T_i_c[0].inverse() * mCalibCam.T_i_c[1];
  kf->setStereoTransform(stereo);
  kf->setCamToWorld(currentPose);
  addMatchedLandmarkFeatureMatches(kf);

  // TODO: Local Mapping should queue this keyframe instead of Tracking
  // throwing it away
  if (localMapper->isIdle()) {
    std::cout << "INFO: Tracking::createNewKeyframe(): Created new Keyframe "
              << kf->mkfId << std::endl;
    localMapper->insertKF(kf);
  }

  mLastKeyframe = kf;
}

std::vector<Landmark*> Tracking::getCovisibleLandmarks() {
  if (!mLastKeyframe) {
    std::cout << "INFO: Tracking (getCovisibleLandmarks) : there is no last "
                 "keyframe, returning empty vector\n";
    return std::vector<Landmark*>();
  }

  std::vector<Keyframe*> neighbor_kfs =
      mLastKeyframe->getSortedCovisibleKeyframes();

  std::vector<Landmark*> this_kf_landmarks = mLastKeyframe->getLandmarks();
  std::set<Landmark*> landmarks(this_kf_landmarks.begin(),
                                this_kf_landmarks.end());

  for (const auto& neighbor : neighbor_kfs) {
    std::vector<Landmark*> neighbor_landmarks = neighbor->getLandmarks();
    std::copy(neighbor_landmarks.begin(), neighbor_landmarks.end(),
              std::inserter(landmarks, landmarks.end()));
  }

  return std::vector<Landmark*>(landmarks.begin(), landmarks.end());
}

std::vector<Landmark*> Tracking::getProjectedLandmarks() {
  return projected_landmarks;
}

void Tracking::findKeypointLandmarkInliers() {
  // Set pose of current frame to identity
  currentFrame.setCamToWorld(currentPose);

  std::vector<Landmark*> neighbor_landmarks = getCovisibleLandmarks();

  // Project landmarks into the keyframe

  project_landmarks(currentPose, mCalibCam.intrinsics[0], neighbor_landmarks,
                    mSettings.cam_z_threshold, projected_points,
                    projected_landmarks);

  // Find matches between projected landmarks and keypoints

  find_matches_landmarks(
      currentFrame.getLeftKeypoints(), projected_points, projected_landmarks,
      mSettings.match_max_dist_2d, mSettings.feature_match_max_dist,
      mSettings.feature_match_test_next_best, lm_feature_matches);
}

void Tracking::addMatchedLandmarkFeatureMatches(Keyframe* kf) {
  for (auto index : landmark_inlier_ids) {
    Landmark* lm = lm_feature_matches[index].first;
    FeatureId fId = lm_feature_matches[index].second;
    kf->addLandmark(lm);
    lm->addObservation(kf->leftView, fId);
  }
}
