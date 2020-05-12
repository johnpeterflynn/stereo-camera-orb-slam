#ifndef LOOP_CLOSING_H
#define LOOP_CLOSING_H

#include <thread>
#include "ceres/ceres.h"
#include "local_mapping.h"
#include "map.h"
#include "settings.h"
#include "tracking.h"

class Tracking;
class LocalMapping;

class LoopClosing {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef std::pair<std::vector<Keyframe*>, int> ConsistentGroup;

  LoopClosing(Map* map, Calibration& calib, const Settings& settings);

  void setTracker(Tracking* p);
  void setLocalMapper(LocalMapping* p);

  // Main function
  void run();

  void insertKeyFrame(Keyframe* kf);

  void requestFinish();
  bool isFinished();

  // Functions related to global bundle adjustment will run in a separate thread
  void globalBA();
  bool isRunningGBA();
  bool isFinishedGBA();
  void addGlobalBAResidualBlock(ceres::Problem& problem, Keyframe* keyframe,
                                View* view, Landmark* landmark,
                                const Sophus::SE3d& T_0_c);
  void poseGraphOptimization();
  void addPoseGraphOptResidualBlock(ceres::Problem& problem, Keyframe* from_kf,
                                    Keyframe* to_kf, int weight = 1.0);

 protected:
  bool checkNewKeyframes();
  bool detectLoopCandidate();
  bool computeSimilarityTransformation();
  void correctCurrentCovisiblesWithSimTrans();
  void correctAllLandmarksWithOldKFPoses();
  void fuseLoop();
  void setFinish(bool value);
  bool checkFinishRequested();

 public:
  std::map<Keyframe*, Sophus::SE3d/*, std::less<Keyframe*>,
           Eigen::aligned_allocator<std::pair<const Keyframe*, Sophus::SE3d>>*/>
      mUncorrectedPoses;  // Poses of keyframes before they are updated in
                          // fuseLoop()
  std::map<Keyframe*, Sophus::SE3d/*, std::less<Keyframe*>,
           Eigen::aligned_allocator<std::pair<const Keyframe*, Sophus::SE3d>>*/>
      mOldPoses;  // Poses of all keyframes right before pose optimization
                  // starts

 protected:
  Map* map;

  Tracking* tracker;
  LocalMapping* localMapper;

  std::list<Keyframe*> KFQueue;

  // Loop Detector variables
  Keyframe* currentKF;
  Keyframe* matchedKF;
  std::map<Keyframe*, std::vector<std::pair<FeatureId, FeatureId>>> matchedKFs;
  std::vector<int> best_inliers;
  Sophus::SE3d similarityTransformation;

  long unsigned int mLastLoopKFid;

  const Settings& mSettings;
  Calibration& mCalibCam;

  // Members related to global bundle adjustment
  bool bRunningGBA;
  bool bFinishedGBA;
  bool bStopGBA;
  int mFullBAIdx;
  std::mutex mutexGBA;
  std::thread* pthreadGBA;

  // Boolean flags
  bool bFinished;
  bool bFinishRequested;

  // Mutex variables
  std::mutex mutexKFQueue;
  std::mutex mutexFinish;
};

#endif  // LOOP_CLOSING_H
