#ifndef MAP_H
#define MAP_H

#include "keyframe.h"
#include "landmark.h"

class Keyframe;
class Landmark;

class Map {
 public:
  Map();

  std::vector<Landmark*> getAllLandmarks();
  void addLandmark(Landmark* lm);
  void addLandmarks(std::vector<Landmark*> lm);
  void eraseLandmark(Landmark* lm);
  void eraseAllLandmarks();
  size_t noOfLandmarks();

  std::vector<Keyframe*> getAllKeyframes();
  void addKeyframe(Keyframe* kf);
  void eraseKeyframe(Keyframe* kf);
  size_t noOfKeyframes();
  Keyframe* getFirstKeyframe();

  /// detected feature locations and descriptors
  Corners feature_corners;

  /// pairwise feature matches
  Matches feature_matches;

 protected:
  std::vector<Landmark*> mLandmarks;
  std::vector<Keyframe*> mKeyframes;

  std::vector<Landmark*> oldLandmarks;
  std::vector<Keyframe*> oldKeyframes;

  std::mutex mutexMap;
};

#endif  // MAP_H
