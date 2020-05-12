#ifndef LANDMARK_H
#define LANDMARK_H

#include "common_types.h"
#include "keyframe.h"
#include "map.h"
#include "mutex"

class View;
class Keyframe;
class Landmark;
class Map;

class Landmark {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  Landmark(const Eigen::Vector3d& pos, Map* m, Keyframe* kf);

  void setWorldPos(const Eigen::Vector3d& Pos);
  Eigen::Vector3d getWorldPos();

  // APIs to add, delete and access Views

  void addObservation(View* view, size_t idx);
  void addOutlierObservation(View* view, size_t idx);
  void deleteObservation(Keyframe* kf);

  size_t getNoOfObservations();
  int getIndexInObservations(View* view);
  std::map<View*, size_t> getObservations();
  std::map<View*, size_t> getOutlierObservations();

  bool isInObservations(View* view);               // Look up by View.
  bool isInObservations(FrameId frame_id);         // Look up by FrameId.
  bool isInOutlierObservations(View* view);        // Look up by View.
  bool isInOutlierObservations(FrameId frame_id);  // Look up by FrameId.

  Keyframe* getRefKeyframe();

  void replace(Landmark* lm);

  void setBadFlag();
  bool isBad();

  // Sort of a hack for now. Use this to get the number of observing keyframes.
  size_t getNumKeyframeObservations();
  std::set<Keyframe*> getKeyframeObservations();

  long unsigned int mId;
  static long unsigned int mNextId;

 public:
  // Keep member variables below public for now so that Ceres can optimize
  // over them. We can't use our get() functions because they only make a copy
  // of these values. It is also dangerous to pass references of member
  // variables via a get() function outside of the class.
  // TODO: Q: What is the solution? Should the Ceres optimizer make a copy
  // of the landmark class, optimize it and then replace the original class with
  // the copy?

  // 3d position in world coordinates
  Eigen::Vector3d p;

  // Keyframes observing the point and corresponding feature index
  std::map<View*, size_t> mObs;

  // Outliers observing the point and corresponding feature index
  std::map<View*, size_t> mOutObs;

 protected:

  std::set<Keyframe*> mObservingKeyframes;  // WARNING: Kind of a hack. Make sure this class stays consistent with mObs.
  Map* map;

  Keyframe* mRefKf;

  bool bBad;

  std::mutex mutexPos;
  std::mutex mutexObs;
  std::mutex mutexOutObs;
};

#endif  // LANDMARK_H
