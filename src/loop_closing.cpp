#include "loop_closing.h"
#include <opengv/absolute_pose/CentralAbsoluteAdapter.hpp>
#include <opengv/absolute_pose/methods.hpp>
#include <opengv/sac/Ransac.hpp>
#include <opengv/sac_problems/absolute_pose/AbsolutePoseSacProblem.hpp>
#include <opengv/triangulation/methods.hpp>
#include "local_parameterization_se3.hpp"
#include "math.h"
#include "utils.h"

LoopClosing::LoopClosing(Map* m, Calibration& calib, const Settings& settings)
    : map(m),
      matchedKF(nullptr),
      mLastLoopKFid(0),
      mSettings(settings),
      mCalibCam(calib),
      bRunningGBA(false),
      bFinishedGBA(true),
      bStopGBA(false),
      mFullBAIdx(0),
      pthreadGBA(nullptr),
      bFinished(false),
      bFinishRequested(false) {}

void LoopClosing::setTracker(Tracking* p) { tracker = p; }

void LoopClosing::setLocalMapper(LocalMapping* p) { localMapper = p; }

void LoopClosing::run() {
  setFinish(false);

  while (1) {
    // Check if there are new keyframes
    if (checkNewKeyframes()) {
      // Check if there is any loop candidate, if not return false
      // If true, Compute similarity transformation
      // If similarity transformation is supported by enough inliers return true
      // else return false

      if (detectLoopCandidate()) {
        // Fuse map points
        fuseLoop();

        poseGraphOptimization();

        globalBA();
      }
    }

    requestFinish();  // Request finish here for sequential run
    // ToDo: After multithreading, remove this from here

    if (checkFinishRequested()) break;

    usleep(5000);
  }

  setFinish(true);
}

void LoopClosing::insertKeyFrame(Keyframe* kf) {
  std::unique_lock<std::mutex> lock(mutexKFQueue);
  if (kf->mkfId != 0) KFQueue.push_back(kf);
}

bool LoopClosing::checkNewKeyframes() {
  std::unique_lock<std::mutex> lock(mutexKFQueue);
  return (!KFQueue.empty());
}

bool LoopClosing::detectLoopCandidate() {
  {
    std::unique_lock<std::mutex> lock(mutexKFQueue);
    currentKF = KFQueue.front();
    KFQueue.pop_front();
    // Avoid that a keyframe can be erased while it is being process by this
    // thread
    currentKF->setNotErase();
    matchedKFs.clear();
    matchedKF = nullptr;
    best_inliers.clear();
  }

  //If the map contains less than 10 kf or less than 10 kf have passed from last loop detection
  if(currentKF->mkfId<mLastLoopKFid+10){
      currentKF->setErase();
      return false;
  }

  std::vector<Keyframe*> loopCandidates = map->getAllKeyframes();
  std::vector<Keyframe*>::iterator itrloop = loopCandidates.begin();

  // Remove current keyframe from loop candidate
  loopCandidates.erase(
      std::find(loopCandidates.begin(), loopCandidates.end(), currentKF));

  // Remove neighbor keyframes and neighbors of neighbor keyframe from loop candidates
  std::vector<Keyframe*> neighKFs = currentKF->getSortedCovisibleKeyframes();

  for (const auto& neighKF : neighKFs){
      itrloop = std::find(loopCandidates.begin(), loopCandidates.end(), neighKF);
      if(itrloop != loopCandidates.end()) {
          loopCandidates.erase(itrloop);
      }
    std::vector<Keyframe*> neighsOfneighKF = neighKF->getSortedCovisibleKeyframes();

    for (const auto& neighOfneighKF : neighsOfneighKF) {
        itrloop = std::find(loopCandidates.begin(), loopCandidates.end(), neighOfneighKF);
        if(itrloop != loopCandidates.end()) {
            loopCandidates.erase(itrloop);
        }
    }
  }

  if (loopCandidates.empty()) {
        currentKF->setErase();
        return false;
  }

  // Remove loop candidate frames which are not within a threshold radius of
  // current frame
  {
    Eigen::Vector3d currentFrameTrans =
        currentKF->getCamToWorld().translation();
    Eigen::Matrix3d currentFrameRotation =
        currentKF->getCamToWorld().rotationMatrix();

    std::vector<Keyframe*> newLoopCandidates;
    for (const auto& loopCandidate : loopCandidates) {
      Eigen::Vector3d candidateTrans =
          loopCandidate->getCamToWorld().translation();

      Eigen::Matrix3d candidateRotation =
          loopCandidate->getCamToWorld().rotationMatrix();

      Eigen::Matrix3d Rab =
          currentFrameRotation.transpose() * candidateRotation;

      // Euclidean Distance
      float euclideanDist = (currentFrameTrans - candidateTrans).norm();

      float angleDiff = acos((Rab.trace() - 1) / 2);

      if (euclideanDist > mSettings.loop_dist_thresh) continue;

      if (angleDiff > mSettings.loop_angle_thresh) continue;

      newLoopCandidates.push_back(loopCandidate);
    }
    loopCandidates.clear();
    loopCandidates = newLoopCandidates;

    if (loopCandidates.empty()) {
        currentKF->setErase();
        return false;
    }
  }

  // Discard the loop candidates which do not have enough inlier matches
  {
    for (const auto& loopCandidate : loopCandidates) {
      std::vector<std::pair<FeatureId, FeatureId>> matches;

      // Match the ORB descriptors for current keyframe stereo pair
      matchDescriptors(currentKF->getLeftKeypoints().corner_descriptors,
                       loopCandidate->getLeftKeypoints().corner_descriptors,
                       matches, mSettings.feature_match_max_dist,
                       mSettings.feature_match_test_next_best);

      if (matches.size() < mSettings.loop_min_inliers) continue;

      matchedKFs[loopCandidate] = matches;
    }
  }

  // Return true or false depending on whether we have a loop candidate or not
  if (!matchedKFs.empty())
    return computeSimilarityTransformation();
  else{
      currentKF->setErase();
      return false;
  }
}

bool LoopClosing::computeSimilarityTransformation() {
  size_t best_match = 0;
  Eigen::Matrix4d best_transformation = Eigen::Matrix4d::Identity();

  for (auto& kf_match : matchedKFs) {
    // stop local mapper from erasing candidate keyframe while it is being
    // processed
    kf_match.first->setNotErase();

    std::vector<int> inliers;

    const Keypoints& currentFramekd = currentKF->getLeftKeypoints();
    std::map<FeatureId, Landmark*> fId_lm;
    for (const auto& lm : kf_match.first->getLandmarks()) {
      fId_lm[lm->getIndexInObservations(kf_match.first->leftView)] = lm;
    }

    opengv::bearingVectors_t bearingVectors;
    opengv::points_t points;
    std::vector<std::pair<FeatureId, FeatureId>>::iterator itr =
        kf_match.second.begin();

    for (; itr != kf_match.second.end();) {
      if (fId_lm.find(itr->second) == fId_lm.end()) {
        itr = kf_match.second.erase(itr);
        continue;
      }

      Eigen::Vector3d p3d_matched = fId_lm[itr->second]->getWorldPos();
      Eigen::Vector3d p3d_transformed =
          kf_match.first->getCamToWorld().inverse() * p3d_matched;
      points.push_back(p3d_transformed);

      Eigen::Vector2d feature_curr = currentFramekd.corners.at(itr->first);
      Eigen::Vector3d p3d_curr =
          mCalibCam.intrinsics[0]->unproject(feature_curr).normalized();
      bearingVectors.push_back(p3d_curr);

      itr++;
    }

    opengv::absolute_pose::CentralAbsoluteAdapter adapter(bearingVectors,
                                                          points);

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

    double f = mCalibCam.intrinsics[0]->data()[0];
    double threshold =
        1 -
        cos(atan(mSettings.reprojection_error_pnp_inlier_threshold_pixel / f));

    // run ransac
    ransac.sac_model_ = absposeproblem_ptr;
    ransac.threshold_ = threshold;
    ransac.max_iterations_ = 500;

    ransac.computeModel();

    // Hypothesis for rotation and translation from RANSAC
    Eigen::Matrix3d initial_rotation =
        ransac.model_coefficients_.block<3, 3>(0, 0);
    Eigen::Array3d initial_translation =
        ransac.model_coefficients_.block<3, 1>(0, 3);

    // non-linear optimization (using all correspondences)
    // NOTE: Setting the initial rotation/transformation to the RANSAC
    // hypothesis is critical because if there aren't enough inliers to execute
    // optimize_nonlinear() then it will serve as a best guess.
    adapter.sett(initial_translation);
    adapter.setR(initial_rotation);
    opengv::transformation_t nonlinear_transformation =
        opengv::absolute_pose::optimize_nonlinear(adapter, ransac.inliers_);

    // Select and save inliers
    ransac.sac_model_->selectWithinDistance(nonlinear_transformation,
                                            ransac.threshold_, inliers);

    if (inliers.size() < mSettings.loop_min_inliers) continue;

    if (inliers.size() < best_match) continue;

    best_match = inliers.size();
    best_inliers = inliers;
    matchedKF = kf_match.first;
    best_transformation.block<3, 4>(0, 0) = nonlinear_transformation;
  }

  if (!matchedKF) {
    for (const auto& kf_match : matchedKFs) {
      kf_match.first->setErase();
    }
    currentKF->setErase();
    return false;
  }

  for (const auto& kf_match : matchedKFs) {
    if (kf_match.first != matchedKF) {
      kf_match.first->setErase();
    }
  }

  // This gives transformation from current keyframe to candidate keyframe
  similarityTransformation = Sophus::SE3d(best_transformation);
  return true;
}

void LoopClosing::correctCurrentCovisiblesWithSimTrans() {
  mUncorrectedPoses.clear();

  std::vector<Keyframe*> current_neighbors =
      currentKF->getSortedCovisibleKeyframes();

  // Save the current keyframe's old pose for later
  mUncorrectedPoses[currentKF] = currentKF->getCamToWorld();

  // Once similarityTransformation is determined we wish to propogate it through
  // the covisible keyframes. Matched keyframe pose and the relative pose
  // between covisible keyframes are assumed to be correct.

  // Old pose of the current keyframe
  Sophus::SE3d old_currentKF_T_w_c = currentKF->getCamToWorld();

  // New pose of the current keyframe, calculated by transforming the
  // matched keyframe pose with the similarity matrix
  Sophus::SE3d adjusted_currentKF_T_w_c =
      matchedKF->getCamToWorld() * similarityTransformation;

  // Set the new keyframe pose
  currentKF->setCamToWorld(adjusted_currentKF_T_w_c);

  // Set the new pose of each neighbor
  for (auto& neighbor : current_neighbors) {

    // Save the old poses of each neighbor
    mUncorrectedPoses[neighbor] = neighbor->getCamToWorld();

    // New pose of the neighbor is equal to the new pose of the current keyframe
    // multiplied by old the transform between the current keyframe and its
    // neighbor
    Sophus::SE3d adjusted_neighbor_T_w_c = adjusted_currentKF_T_w_c *
                                           old_currentKF_T_w_c.inverse() *
                                           neighbor->getCamToWorld();

    neighbor->setCamToWorld(adjusted_neighbor_T_w_c);
  }
}

void LoopClosing::fuseLoop() {
  // send a stop signal to local mapping
  // avoid adding of new keyframes while correcting the loop
  // localMapper->requestStop();   // Uncomment it after multithreading

  // If a global bundle adjustment is running, stop it
  if (isRunningGBA()) {
    std::unique_lock<std::mutex> lock(mutexGBA);
    bStopGBA = true;
    mFullBAIdx++;
    if (pthreadGBA) {
      pthreadGBA->detach();
      delete pthreadGBA;
    }
  }

  // Wait until local mapper has fully stopped
  /*while (!localMapper->isStopped()) {
      usleep(1000);
  }*/

  correctCurrentCovisiblesWithSimTrans();

  std::vector<Landmark*> currentLandmarks = currentKF->getLandmarks();
  std::map<FeatureId, Landmark*> curr_fId_lm;

  for (const auto& lm : currentLandmarks) {
    curr_fId_lm[lm->getIndexInObservations(currentKF->leftView)] = lm;
  }

  std::vector<Landmark*> matchedLandmarks = matchedKF->getLandmarks();
  std::map<FeatureId, Landmark*> matched_fId_lm;

  for (const auto& lm : matchedLandmarks) {
    matched_fId_lm[lm->getIndexInObservations(matchedKF->leftView)] = lm;
  }

  std::vector<std::pair<FeatureId, FeatureId>> fId_fId_match =
      matchedKFs[matchedKF];

  // Fuse all the matched inlier landmarks in ransac
  for (size_t i = 0; i < best_inliers.size(); i++) {
    Landmark* currlm = curr_fId_lm[fId_fId_match[i].first];
    Landmark* matchedlm = matched_fId_lm[fId_fId_match[i].second];

    if (currlm && matchedlm) {
      currentLandmarks.erase(
          std::find(currentLandmarks.begin(), currentLandmarks.end(), currlm));
      matchedLandmarks.erase(std::find(matchedLandmarks.begin(),
                                       matchedLandmarks.end(), matchedlm));
      currlm->replace(matchedlm);
    } else if (currlm) {
      currentLandmarks.erase(
          std::find(currentLandmarks.begin(), currentLandmarks.end(), currlm));
      currlm->addObservation(matchedKF->leftView, fId_fId_match[i].second);
      matchedKF->addLandmark(currlm);
    } else if (matchedlm) {
      matchedLandmarks.erase(std::find(matchedLandmarks.begin(),
                                       matchedLandmarks.end(), matchedlm));
      matchedlm->addObservation(currentKF->leftView, fId_fId_match[i].first);
      currentKF->addLandmark(matchedlm);
    }
  }

  // ToDo: Add projections of unmatched landmarks which could increase the
  // number of observations

  // Update connections of current and matched keyframe
  currentKF->updateConnections();
  matchedKF->updateConnections();

  // Add loop edge
  currentKF->addLoopEdge(matchedKF);
  matchedKF->addLoopEdge(currentKF);

  // Launch a new thread to perform global bundle adjustment
  bRunningGBA = true;
  bFinishedGBA = false;
  bStopGBA = false;
  // pthreadGBA = new std::thread(&LoopClosing::globalBA, this);

  // Loop closed. Release local mapping
  // localMapper->Release();  // Uncomment it after multithreading

  //matchedKF->setErase();
  mLastLoopKFid = currentKF->mkfId;
}

void LoopClosing::globalBA() {
  // local mapping will be working at the same time global BA is working
  // map may be inconsistent with what we get after global BA

  std::cout << "Parika: Optimizing map with " << map->noOfKeyframes()
            << "keyframes and " << map->noOfLandmarks() << " landmarks. \n ";

  std::vector<Keyframe*> vKeyframes = map->getAllKeyframes();
  std::vector<Landmark*> vLandmarks = map->getAllLandmarks();

  ceres::Problem problem;

  // Add camera transformation and intrinsics parameter blocks
  for (auto& kf : vKeyframes) {
    problem.AddParameterBlock(kf->getCamToWorld().data(),
                              Sophus::SE3d::num_parameters,
                              new Sophus::test::LocalParameterizationSE3);
  }

  for (auto& intrinsic : mCalibCam.intrinsics) {
    problem.AddParameterBlock(intrinsic->data(), 8);
  }

  // Fix first keyframe
  problem.SetParameterBlockConstant(
      map->getFirstKeyframe()->getCamToWorld().data());

  // Fix camera intrinsics if not to be optimized
  if (!mSettings.ba_optimize_intrinsics) {
    for (auto& intrinsic : mCalibCam.intrinsics) {
      problem.SetParameterBlockConstant(intrinsic->data());
    }
  }

  // Identity transformation
  const Sophus::SE3d Identity = Sophus::SE3d(Eigen::Matrix4d::Identity());

  // Add landmarks parameter blocks
  for (auto& landmark : vLandmarks) {
    std::set<Keyframe*> kf_observations = landmark->getKeyframeObservations();

    for (auto& kf : kf_observations) {
      View* leftView = kf->leftView;
      View* rightView = kf->rightView;

      addGlobalBAResidualBlock(problem, kf, leftView, landmark, Identity);
      addGlobalBAResidualBlock(problem, kf, rightView, landmark,
                               kf->getStereoTransform());
    }
  }

  // Solve
  ceres::Solver::Options ceres_options;
  ceres_options.max_num_iterations = mSettings.globalBA_max_iterations;
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

  bFinishedGBA = true;
  bRunningGBA = false;

  std::cout << "Parika: Global BA finished\n";
}

void LoopClosing::addGlobalBAResidualBlock(ceres::Problem& problem,
                                           Keyframe* keyframe, View* view,
                                           Landmark* landmark,
                                           const Sophus::SE3d& T_0_c) {
  int f_index = landmark->getIndexInObservations(view);

  if (f_index >= 0) {
    BundleAdjustmentReprojectionCostFunctor* cost_functor =
        new BundleAdjustmentReprojectionCostFunctor(
            view->getKeypoints().corners[f_index],
            mCalibCam.intrinsics[view->id().second]->name(), T_0_c);

    ceres::CostFunction* cost_function =
        new ceres::AutoDiffCostFunction<BundleAdjustmentReprojectionCostFunctor,
                                        2, Sophus::SE3d::num_parameters, 3, 8>(
            cost_functor);

    ceres::LossFunctionWrapper* loss = nullptr;
    if (mSettings.ba_use_huber) {
      loss = new ceres::LossFunctionWrapper(
          new ceres::HuberLoss(mSettings.globalBA_huber_parameter),
          ceres::TAKE_OWNERSHIP);
    }

    // Notice here that kf->getCamToWorld() can be combined with T_0_c to define
    // the pose for the left and right views
    problem.AddResidualBlock(
        cost_function, loss, keyframe->getCamToWorld().data(),
        landmark->p.data(), mCalibCam.intrinsics[view->id().second]->data());
  } else {
    // Invalid index
  }
}

void LoopClosing::poseGraphOptimization() {
  mOldPoses.clear();

  ceres::Problem problem;

  // Could also iterate the Essential Graph but it too contains all keyframes
  std::vector<Keyframe*> all_keyframes = map->getAllKeyframes();

  // TODO: Handle the newest loop closure (give more weight to oldest residual)

  // Add the parameter block for all keyframes. Q: Is it necessary to do this
  // before creating the residual blocks?
  for (auto& kf : all_keyframes) {

    mOldPoses[kf] = kf->getCamToWorld();

    problem.AddParameterBlock(kf->getCamToWorld().data(),
                              Sophus::SE3d::num_parameters,
                              new Sophus::test::LocalParameterizationSE3);

    // Fix the matched (loop) keyframe
    // Q: Does it make sense to set both the first keyframe and loop keyframe to
    // constant?
    if (kf->mkfId == matchedKF->mkfId) {
      problem.SetParameterBlockConstant(kf->getCamToWorld().data());
    }
  }

  // Fix the first keyframe
  problem.SetParameterBlockConstant(
      map->getFirstKeyframe()->getCamToWorld().data());

  // Algorithm: iterate over all keyframes and create residuals between them and
  // their spanning tree children, loop closures and strong connections (the
  // three create the Essential Graph). A residual is bidirectional (adding one
  // for A to B is the same as B to A) so keep track of which keyframes have
  // been processed and don't add them multiple times.

  std::set<Keyframe*> processed_kfs;

  // Add the residual blocks for every connection in the Essential Graph
  for (auto& kf : all_keyframes) {
    std::set<Keyframe*> loop_keyframes = kf->getLoopEdges();

    // Add loop closure connections. Add first because we want to give them
    // extra weight in the optimization if they exist
    for (auto& loop_kf : loop_keyframes) {
      // If loop_kf was not previously processed as a kf
      if (processed_kfs.find(loop_kf) == processed_kfs.end()) {
        addPoseGraphOptResidualBlock(problem, kf, loop_kf, 1000.0);
      }
    }

    std::set<Keyframe*> child_keyframes = kf->getChildren();

    // Add connections from spanning tree
    for (auto& child_kf : child_keyframes) {

      // If child_kf was not previously processed as a kf or a loop closer
      if (processed_kfs.find(child_kf) == processed_kfs.end() &&
          loop_keyframes.find(child_kf) == loop_keyframes.end()) {
        addPoseGraphOptResidualBlock(problem, kf, child_kf);
      }
    }

    std::vector<Keyframe*> strong_cov_keyframes =
        kf->getSortedCovisibleKeyframesWithMinWeight(
            Keyframe::MIN_ESSENTIAL_WEIGHT_THRESH);

    // Add strong connections (at least Keyframe::MIN_ESSENTIAL_WEIGHT_THRESH
    // shared landmarks)
    for (auto& strong_kf : strong_cov_keyframes) {

      // If strong_kf was not previously processed as a kf, child in the
      // spanning tree or loop closer
      if (processed_kfs.find(strong_kf) == processed_kfs.end() &&
          child_keyframes.find(strong_kf) == child_keyframes.end() &&
          loop_keyframes.find(strong_kf) == loop_keyframes.end()) {
        addPoseGraphOptResidualBlock(problem, kf, strong_kf);
      }
    }

    // Flag keyframe as processed
    processed_kfs.insert(kf);
  }

  // Solve
  ceres::Solver::Options ceres_options;
  ceres_options.max_num_iterations = mSettings.globalBA_max_iterations;
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

  correctAllLandmarksWithOldKFPoses();
}

void LoopClosing::correctAllLandmarksWithOldKFPoses() {
  std::vector<Landmark*> vLandmarks = map->getAllLandmarks();

  for (auto& landmark : vLandmarks) {
    for (auto& kf : landmark->getKeyframeObservations()) {
      if (!kf->isBad()) {
        Eigen::Vector3d adjusted_lmk_T_w_l =
            (kf->getCamToWorld() * mOldPoses[kf].inverse()).translation() +
            landmark->getWorldPos();

        landmark->setWorldPos(adjusted_lmk_T_w_l);
        break;
      }
    }
  }
}

void LoopClosing::addPoseGraphOptResidualBlock(ceres::Problem& problem,
                                               Keyframe* from_kf,
                                               Keyframe* to_kf, int weight) {
  Sophus::SE3d T_to_from;

  // If this connection is the loop closure connection, weigh it a lot more!
  if ((from_kf->mkfId == currentKF->mkfId &&
       to_kf->mkfId == matchedKF->mkfId)) {
    T_to_from = similarityTransformation;
  } else if ((from_kf->mkfId == matchedKF->mkfId &&
              to_kf->mkfId == currentKF->mkfId)) {
    T_to_from = similarityTransformation.inverse();
  } else {
    Sophus::SE3d T_w_from;
    Sophus::SE3d T_w_to;

    auto from_iter = mUncorrectedPoses.find(from_kf);
    if (from_iter != mUncorrectedPoses.end()) {
      T_w_from = from_iter->second;
    } else {
      T_w_from = from_kf->getCamToWorld();
    }

    auto to_iter = mUncorrectedPoses.find(to_kf);
    if (to_iter != mUncorrectedPoses.end()) {
      T_w_to = to_iter->second;
    } else {
      T_w_to = to_kf->getCamToWorld();
    }

    T_to_from = T_w_to.inverse() * T_w_from;
  }

  PoseGraphOptimizationCostFunctor* cost_functor =
      new PoseGraphOptimizationCostFunctor(T_to_from, weight);

  ceres::CostFunction* cost_function =
      new ceres::AutoDiffCostFunction<PoseGraphOptimizationCostFunctor, 6,
                                      Sophus::SE3d::num_parameters,
                                      Sophus::SE3d::num_parameters>(
          cost_functor);

  ceres::LossFunctionWrapper* loss = nullptr;
  if (mSettings.ba_use_huber) {
    loss = new ceres::LossFunctionWrapper(
        new ceres::HuberLoss(mSettings.globalBA_huber_parameter),
        ceres::TAKE_OWNERSHIP);
  }

  problem.AddResidualBlock(cost_function, loss, to_kf->getCamToWorld().data(),
                           from_kf->getCamToWorld().data());
}

bool LoopClosing::isRunningGBA() {
  std::unique_lock<std::mutex> lock(mutexGBA);
  return bRunningGBA;
}

bool LoopClosing::isFinishedGBA() {
  std::unique_lock<std::mutex> lock(mutexGBA);
  return bFinishedGBA;
}

void LoopClosing::requestFinish() {
  std::unique_lock<std::mutex> lock(mutexFinish);
  bFinishRequested = true;
}

bool LoopClosing::isFinished() {
  std::unique_lock<std::mutex> lock(mutexFinish);
  return bFinished;
}

bool LoopClosing::checkFinishRequested() {
  std::unique_lock<std::mutex> lock(mutexFinish);
  return bFinishRequested;
}

void LoopClosing::setFinish(bool value) {
  std::unique_lock<std::mutex> lock(mutexFinish);
  bFinished = value;
}
