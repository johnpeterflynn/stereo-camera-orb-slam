#ifndef LOCAL_MAPPING_H
#define LOCAL_MAPPING_H

#include "calibration.h"
#include "common_types.h"
#include "keyframe.h"
#include "landmark.h"
#include "list"
#include "loop_closing.h"
#include "map.h"
#include "reprojection.h"
#include "settings.h"
#include "tracking.h"

#include <mutex>

class Tracking;
class LoopClosing;

class LocalMapping {
 public:
  LocalMapping(Map* m, Calibration& calib, const Settings& settings);

  // main function
  void run();

  void setTracker(Tracking* p);
  void setLoopCloser(LoopClosing* p);

  void insertKF(Keyframe* pKF);
  Keyframe* getCurrentKeyframe();

  void Release();

  // publically available setters and getters for booleans
  void requestFinish(bool value);
  bool isFinished();
  void requestStop();
  bool isStopped();
  void InterruptBA();
  bool isIdle();

 public:
  static const int MIN_NUM_LANDMARK_OBS_FOR_CULLING = 3;

 protected:
  bool checkNewKFs();
  void processNewKF();
  void mapPointsCulling();
  void createMapPoints();
  void triangulate_landmarks(MatchData md_stereo,
                             std::vector<Landmark*>& addedLandmarks);
  void localBA();
  void KFCulling();
  void addLocalBAResidualBlock(ceres::Problem& problem, Keyframe* keyframe,
                               View* view, Landmark* landmark,
                               const Sophus::SE3d& T_0_c);

  // setters and getters for boolean
  bool checkFinishRequested();
  bool checkStopRequested();
  void finish();
  bool stop();
  void setIdle(bool value);

  // member variables
  Keyframe* currentKF;
  std::list<Keyframe*> newKFs;

  Map* map;
  const Settings& mSettings;

  Tracking* tracker;
  LoopClosing* loopCloser;

  Calibration&
      calib_cam;  // ToDo: Mutex guarding the access of calib_cam if required

  // mutex variables
  std::mutex mutexNewKF;
  std::mutex mutexFinish;
  std::mutex mutexStop;
  std::mutex mutexIdle;

  // boolean variables
  bool bFinished;
  bool bFinishRequested;
  bool bStop;
  bool bStopRequested;
  bool bIdle;
  bool bAbortBA;

  // setters and getters for booleans
  void setFinish(bool value);
  bool getFinish();
};

#endif  // LOCAL_MAPPING_H
