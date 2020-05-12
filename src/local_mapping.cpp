#include "local_mapping.h"

#include <ceres/ceres.h>

#include <opengv/absolute_pose/methods.hpp>
#include <opengv/relative_pose/CentralRelativeAdapter.hpp>
#include <opengv/sac/Ransac.hpp>
#include <opengv/sac_problems/absolute_pose/AbsolutePoseSacProblem.hpp>
#include <opengv/triangulation/methods.hpp>

#include "local_parameterization_se3.hpp"

#include "reprojection.h"
#include "utils.h"

LocalMapping::LocalMapping(Map* m, Calibration& calib, const Settings& settings)
    : mSettings(settings),
      calib_cam(calib),
      bFinished(true),
      bFinishRequested(false),
      bStop(false),
      bStopRequested(false),
      bIdle(true),
      bAbortBA(false) {
  map = m;
}

void LocalMapping::setTracker(Tracking* p) { tracker = p; }

void LocalMapping::setLoopCloser(LoopClosing* p) { loopCloser = p; }

void LocalMapping::run() {
  while (1) {
    // Shows that local mapping is busy
    setIdle(false);

    // Check if there are new frames to be processed
    if (checkNewKFs()) {
      // Process the new keyframe
      processNewKF();

      // Check recent Map points and remove the bad ones
      mapPointsCulling();

      // Create new map points by triangulation
      createMapPoints();

      bAbortBA = false;

      if (!checkNewKFs() && !checkStopRequested()) {
        if (map->noOfKeyframes() > 2) {
          // Perform local bundle adjustment
          std::cout << "Perform local bundle adjustment\n";
          localBA();
        }

        // Remove redundant keyframes
        KFCulling();
        loopCloser->insertKeyFrame(currentKF);
      }
    } else if (stop()) {
      while (isStopped() && !checkFinishRequested()) {
        usleep(3000);
      }

      if (checkFinishRequested()) break;
    }

    // set local mapping to idle
    setIdle(true);
    requestFinish(
        true);  // Putting this here for now so that it runs sequentially

    if (checkFinishRequested()) break;

    usleep(3000);
  }

  // finish local mapping
  finish();
}

void LocalMapping::insertKF(Keyframe* pKF) {
  std::unique_lock<std::mutex> lock(mutexNewKF);
  newKFs.push_back(pKF);
  bAbortBA = true;
}

Keyframe* LocalMapping::getCurrentKeyframe() {
  std::unique_lock<std::mutex> lock(mutexNewKF);

  return currentKF;
}

bool LocalMapping::checkNewKFs() {
  std::unique_lock<std::mutex> lock(mutexNewKF);
  return (!newKFs.empty());
}

void LocalMapping::processNewKF() {
  {
    // Get first KF to be processed
    std::unique_lock<std::mutex> lock(mutexNewKF);
    currentKF = newKFs.front();
    newKFs.pop_front();
  }

  currentKF->updateConnections();
  map->addKeyframe(currentKF);
}

void LocalMapping::mapPointsCulling() {
  std::vector<Landmark*> all_landmarks = map->getAllLandmarks();
  for (auto& lm : all_landmarks) {

    if(lm->isBad()) continue;

    if (lm->getNoOfObservations() <= 2) {
      lm->setBadFlag();
    }
  }
}

void LocalMapping::createMapPoints() {
  // Get neighboring keyframe from covisibility graph
  std::vector<Keyframe*> neighKFs = currentKF->getSortedCovisibleKeyframes();
  std::vector<Landmark*> addedLandmarks;

  MatchData md_stereo;
  md_stereo.T_i_j = currentKF->getStereoTransform();

  Eigen::Matrix3d E;
  computeEssential(md_stereo.T_i_j, E);

  // Match the ORB descriptors for current keyframe stereo pair
  matchDescriptors(currentKF->getLeftKeypoints().corner_descriptors,
                   currentKF->getRightKeypoints().corner_descriptors,
                   md_stereo.matches, mSettings.feature_match_max_dist,
                   mSettings.feature_match_test_next_best);

  // Find inlier matches that fulfill epipolar constraint
  findInliersEssential(currentKF->getLeftKeypoints(),
                       currentKF->getRightKeypoints(),
                       calib_cam.intrinsics[currentKF->leftView->id().second],
                       calib_cam.intrinsics[currentKF->rightView->id().second],
                       E, 1e-3, md_stereo);

  TimeCamId tcidl(currentKF->mframeid, 0);
  TimeCamId tcidr(currentKF->mframeid, 1);

  // Entry for left frame already added in tracking
  map->feature_corners[tcidr] = currentKF->getRightKeypoints();
  map->feature_matches[std::make_pair(tcidl, tcidr)] = md_stereo;

  // Triangulate new landmarks
  triangulate_landmarks(md_stereo, addedLandmarks);

  // Project landmarks in neighboring keyframes, find the matches and add
  // corresponding entries in keyframes and landmarks
  for (auto& kf : neighKFs) {
    std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>
        projected_points;
    std::vector<Landmark*> projected_landmarks;

    project_landmarks(kf->getCamToWorld(), calib_cam.intrinsics[0],
                      addedLandmarks, mSettings.cam_z_threshold,
                      projected_points, projected_landmarks);

    MatchData stereoMatch;

    stereoMatch.T_i_j = kf->getStereoTransform();

    Eigen::Matrix3d E;
    computeEssential(stereoMatch.T_i_j, E);

    matchDescriptors(kf->getLeftKeypoints().corner_descriptors,
                     kf->getRightKeypoints().corner_descriptors,
                     stereoMatch.matches, mSettings.feature_match_max_dist,
                     mSettings.feature_match_test_next_best);

    findInliersEssential(kf->getLeftKeypoints(), kf->getRightKeypoints(),
                         calib_cam.intrinsics[0], calib_cam.intrinsics[1], E,
                         1e-3, stereoMatch);

    std::vector<std::pair<Landmark*, FeatureId>> md;

    find_matches_landmarks(kf->getLeftKeypoints(), projected_points,
                           projected_landmarks, mSettings.match_max_dist_2d,
                           mSettings.feature_match_max_dist,
                           mSettings.feature_match_test_next_best, md);

    // convert vector of stereo matches to a map
    std::map<FeatureId, FeatureId> stereo_matches;

    for (const auto& stereo_match : stereoMatch.inliers) {
      stereo_matches[stereo_match.first] = stereo_match.second;
    }

    for (const auto& lm_kd_match : md) {
      Landmark* lm = lm_kd_match.first;
      FeatureId lfId = lm_kd_match.second;

      lm->addObservation(kf->leftView, lfId);

      if (stereo_matches.find(lfId) != stereo_matches.end()) {
        lm->addObservation(kf->rightView, stereo_matches[lfId]);
        stereo_matches.erase(lfId);
      }

      kf->addLandmark(lm);
    }
  }
}

void LocalMapping::triangulate_landmarks(
    MatchData md_stereo, std::vector<Landmark*>& addedLandmarks) {
  const Sophus::SE3d T_0_1 = calib_cam.T_i_c[0].inverse() * calib_cam.T_i_c[1];
  const Eigen::Vector3d t_0_1 = T_0_1.translation();
  const Eigen::Matrix3d R_0_1 = T_0_1.rotationMatrix();

  std::map<FeatureId, FeatureId> md_stereo_unused;

  // Convert vector of stereo match pairs to map
  for (const auto& entity : md_stereo.inliers) {
    md_stereo_unused[entity.first] = entity.second;
  }

  // Landmark observations of current KF
  std::vector<Landmark*> landmarkObs = currentKF->getLandmarks();

  // For the stereo matches which have already been matched to landmarks,
  // add the entry for right frame and erase the entry
  for (auto& obs : landmarkObs) {
    FeatureId lfId = obs->getIndexInObservations(currentKF->leftView);
    if (md_stereo_unused.find(lfId) != md_stereo_unused.end()) {
      FeatureId rfId = md_stereo_unused[lfId];
      obs->addObservation(currentKF->rightView, rfId);
      md_stereo_unused.erase(lfId);
    }
  }

  // Triangulate the unmatched stereo matches
  opengv::bearingVectors_t bearingVectors0;
  opengv::bearingVectors_t bearingVectors1;

  // Triangulate the landmark for all stereo matches whose feature ids were not
  // previously associated with a landmark id
  for (const auto& lfid_rfid_pair : md_stereo_unused) {
    FeatureId left_corner_fid = lfid_rfid_pair.first;
    FeatureId right_corner_fid = lfid_rfid_pair.second;

    // Get the 2D projected coordinates of the feature in each camera
    Eigen::Vector2d p0_2d =
        currentKF->leftView->getKeypoints().corners.at(left_corner_fid);
    Eigen::Vector2d p1_2d =
        currentKF->rightView->getKeypoints().corners.at(right_corner_fid);

    // Unproject to 3D coordinates with respect to each camera
    Eigen::Vector3d p0_3d =
        calib_cam.intrinsics[0]->unproject(p0_2d).normalized();
    Eigen::Vector3d p1_3d =
        calib_cam.intrinsics[1]->unproject(p1_2d).normalized();

    bearingVectors0.push_back(p0_3d);
    bearingVectors1.push_back(p1_3d);
  }

  // Create the adapter with the bearing vectors and the transformation from the
  // second camera to the first
  opengv::relative_pose::CentralRelativeAdapter adapter(
      bearingVectors0, bearingVectors1, t_0_1, R_0_1);

  unsigned int index = 0;
  for (const auto& lfid_rfid_pair : md_stereo_unused) {
    FeatureId left_corner_fid = lfid_rfid_pair.first;
    FeatureId right_corner_fid = lfid_rfid_pair.second;

    // Calculate the landmark location in 3D world coordinates
    Eigen::Vector3d Pos = currentKF->getCamToWorld() *
                          opengv::triangulation::triangulate(adapter, index);

    Landmark* landmark = new Landmark(Pos, map, currentKF);

    landmark->addObservation(currentKF->leftView, left_corner_fid);
    landmark->addObservation(currentKF->rightView, right_corner_fid);

    // Add new landmark in current keyframe observation
    currentKF->addLandmark(landmark);

    // Add new landmark in map
    map->addLandmark(landmark);

    addedLandmarks.push_back(landmark);

    index++;
  }
}

void LocalMapping::localBA() {
  std::vector<Keyframe*> localKFs;

  std::vector<Keyframe*> neighKFs = currentKF->getSortedCovisibleKeyframes();

  std::vector<Keyframe*> fixedKFs;

  std::vector<Landmark*> localLandmarks;

  localKFs.push_back(currentKF);

  for (auto& kf : neighKFs) {
    if (kf->isBad()) continue;
    localKFs.push_back(kf);
  }

  // Add the Map points observed in all local KeyFrames
  for (auto& kf : localKFs) {
    for (auto& lm : kf->getLandmarks()) {
      if (lm->isBad()) continue;
      if (std::find(localLandmarks.begin(), localLandmarks.end(), lm) ==
          localLandmarks.end()) {
        localLandmarks.push_back(lm);
      }
    }
  }

  // Find KFs that see the map points but are not neighbors of current KF
  for (auto& lm : localLandmarks) {
    std::map<View*, size_t> landmarkObs = lm->getObservations();
    std::map<View*, size_t>::iterator obsItr;
    Keyframe* kf;
    for (obsItr = landmarkObs.begin(); obsItr != landmarkObs.end(); obsItr++) {
      kf = obsItr->first->getParentKeyframe();

      if (kf->isBad()) continue;

      if (std::find(localKFs.begin(), localKFs.end(), kf) == localKFs.end()) {
        if (std::find(fixedKFs.begin(), fixedKFs.end(), kf) == fixedKFs.end()) {
          fixedKFs.push_back(kf);
        }
      }
    }
  }

  ceres::Problem problem;

  // Add camera transformation and intrinsics parameter blocks
  for (auto& kf : localKFs) {
    problem.AddParameterBlock(kf->getCamToWorld().data(),
                              Sophus::SE3d::num_parameters,
                              new Sophus::test::LocalParameterizationSE3);
  }

  for (auto& kf : fixedKFs) {
    problem.AddParameterBlock(kf->getCamToWorld().data(),
                              Sophus::SE3d::num_parameters,
                              new Sophus::test::LocalParameterizationSE3);
  }

  for (auto& intrinsic : calib_cam.intrinsics) {
    problem.AddParameterBlock(intrinsic->data(), 8);
  }

  // Set camera transformation blocks to constant if they are flagged as fixed
  for (auto& kf : fixedKFs) {
    problem.SetParameterBlockConstant(kf->getCamToWorld().data());
  }

  if (!mSettings.ba_optimize_intrinsics) {
    for (auto& intrinsic : calib_cam.intrinsics) {
      problem.SetParameterBlockConstant(intrinsic->data());
    }
  }

  // Identity transformation
  const Sophus::SE3d Identity = Sophus::SE3d(Eigen::Matrix4d::Identity());

  // Add landmarks parameter blocks
  for (auto& landmark : localLandmarks) {
    std::set<Keyframe*> kf_observations = landmark->getKeyframeObservations();

    for (auto& kf : kf_observations) {
      View* leftView = kf->leftView;
      View* rightView = kf->rightView;

      addLocalBAResidualBlock(problem, kf, leftView, landmark, Identity);
      addLocalBAResidualBlock(problem, kf, rightView, landmark,
                              kf->getStereoTransform());
    }
  }

  // Solve
  ceres::Solver::Options ceres_options;
  ceres_options.max_num_iterations = mSettings.localBA_max_iterations;
  ceres_options.linear_solver_type = ceres::SPARSE_SCHUR;
  ceres::Solver::Summary summary;
  Solve(ceres_options, &problem, &summary);
  switch (mSettings.ba_verbose) {
    // 0: silent
    case 1:
      std::cout << summary.BriefReport() << std::endl;
      break;
    case 2:
      std::cout << summary.FullReport() << std::endl;
      break;
  }
}

void LocalMapping::addLocalBAResidualBlock(ceres::Problem& problem,
                                           Keyframe* keyframe, View* view,
                                           Landmark* landmark,
                                           const Sophus::SE3d& T_0_c) {
  int f_index = landmark->getIndexInObservations(view);

  if (f_index >= 0) {
    BundleAdjustmentReprojectionCostFunctor* cost_functor =
        new BundleAdjustmentReprojectionCostFunctor(
            view->getKeypoints().corners[f_index],
            calib_cam.intrinsics[view->id().second]->name(), T_0_c);

    ceres::CostFunction* cost_function =
        new ceres::AutoDiffCostFunction<BundleAdjustmentReprojectionCostFunctor,
                                        2, Sophus::SE3d::num_parameters, 3, 8>(
            cost_functor);

    ceres::LossFunctionWrapper* loss = nullptr;
    if (mSettings.ba_use_huber) {
      loss = new ceres::LossFunctionWrapper(
          new ceres::HuberLoss(mSettings.localBA_huber_parameter),
          ceres::TAKE_OWNERSHIP);
    }

    // Notice here that kf->getCamToWorld() can be combined with T_0_c to define
    // the pose for the left and right views
    problem.AddResidualBlock(
        cost_function, loss, keyframe->getCamToWorld().data(),
        landmark->p.data(), calib_cam.intrinsics[view->id().second]->data());
  } else {
    // Invalid index
  }
}

void LocalMapping::KFCulling() {
  // Check redundant local keyframes
  // A keyframe is considered redundant if the 90% of the MapPoints it sees are
  // seen by atleast three other keyframes

  std::vector<Keyframe*> localKFs = currentKF->getSortedCovisibleKeyframes();

  for (auto& kf : localKFs) {
    if(kf->mkfId == 0) continue;

    std::vector<Landmark*> landmarks = kf->getLandmarks();

    // +1 because all landmarks of kf will observe kf and that doesn't count!
    const size_t num_obs_threshold = MIN_NUM_LANDMARK_OBS_FOR_CULLING + 1;
    size_t nMPs = landmarks.size();
    size_t redundantObs = 0;

    for (auto& landmark : landmarks) {
      if (landmark->isBad()) continue;
      if (landmark->getNumKeyframeObservations() >= num_obs_threshold)
        redundantObs++;
    }

    if (redundantObs >= 0.9 * nMPs) {
      std::cout << "Info: LocalMapping::KFCulling(): Culling Keyframe "
                << kf->mkfId << "! num points: " << redundantObs
                << ", max points: " << 0.9 * nMPs << std::endl;
      kf->setBadFlag();
    }
  }
}

void LocalMapping::Release() {
  std::unique_lock<std::mutex> lock2(mutexFinish);
  std::unique_lock<std::mutex> lock(mutexStop);
  if (bFinished) return;

  bStop = false;
  bStopRequested = false;

  newKFs.clear();

  std::cout << "Local Mapping Released\n";
}

void LocalMapping::requestFinish(bool value) {
  std::unique_lock<std::mutex> lock(mutexFinish);
  bFinishRequested = value;
}

bool LocalMapping::isFinished() {
  std::unique_lock<std::mutex> lock(mutexFinish);
  return bFinished;
}

void LocalMapping::requestStop() {
  std::unique_lock<std::mutex> lock(mutexStop);
  bStopRequested = true;
  bAbortBA = true;
}

bool LocalMapping::isStopped() {
  std::unique_lock<std::mutex> lock(mutexStop);
  return bStop;
}

bool LocalMapping::checkFinishRequested() {
  std::unique_lock<std::mutex> lock(mutexFinish);
  return bFinishRequested;
}

bool LocalMapping::checkStopRequested() {
  std::unique_lock<std::mutex> lock(mutexStop);
  return bStopRequested;
}

void LocalMapping::finish() {
  std::unique_lock<std::mutex> lock(mutexFinish);
  bFinished = true;
  std::unique_lock<std::mutex> lock2(mutexStop);
  bStop = true;
}

bool LocalMapping::stop() {
  std::unique_lock<std::mutex> lock(mutexStop);
  if (bStopRequested) {
    bStop = true;
    std::cout << "Local Mapping STOP ! \n";
    return true;
  }
  return false;
}

void LocalMapping::InterruptBA() { bAbortBA = true; }

bool LocalMapping::isIdle() {
  std::unique_lock<std::mutex> lock(mutexIdle);
  return bIdle;
}

void LocalMapping::setIdle(bool value) {
  std::unique_lock<std::mutex> lock(mutexIdle);
  bIdle = value;
}
