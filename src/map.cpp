#include "map.h"

Map::Map() {}

std::vector<Landmark*> Map::getAllLandmarks() {
  std::unique_lock<std::mutex>(mutexMap);
  return std::vector<Landmark*>(mLandmarks.begin(), mLandmarks.end());
}

void Map::addLandmark(Landmark* lm) {
  std::unique_lock<std::mutex>(mutexMap);
  mLandmarks.push_back(lm);
}

void Map::addLandmarks(std::vector<Landmark*> landmarks) {
  std::unique_lock<std::mutex>(mutexMap);
  for (auto& lm : landmarks) {
    mLandmarks.push_back(lm);
  }
}

void Map::eraseLandmark(Landmark* lm) {
  std::unique_lock<std::mutex>(mutexMap);
  oldLandmarks.push_back(lm);
  std::vector<Landmark*>::iterator index =
      std::find(mLandmarks.begin(), mLandmarks.end(), lm);
  if (index != mLandmarks.end()) {
    mLandmarks.erase(index);
  }
}

void Map::eraseAllLandmarks() {
  std::unique_lock<std::mutex>(mutexMap);
  mLandmarks.clear();
}

size_t Map::noOfLandmarks() {
  std::unique_lock<std::mutex>(mutexMap);
  return mLandmarks.size();
}

std::vector<Keyframe*> Map::getAllKeyframes() {
  std::unique_lock<std::mutex>(mutexMap);
  return std::vector<Keyframe*>(mKeyframes.begin(), mKeyframes.end());
}

void Map::addKeyframe(Keyframe* kf) {
  std::unique_lock<std::mutex>(mutexMap);
  mKeyframes.push_back(kf);
}

void Map::eraseKeyframe(Keyframe* kf) {
  std::unique_lock<std::mutex>(mutexMap);
  oldKeyframes.push_back(kf);
  std::vector<Keyframe*>::iterator index =
      std::find(mKeyframes.begin(), mKeyframes.end(), kf);
  if (index != mKeyframes.end()) {
    mKeyframes.erase(index);
  }
}

size_t Map::noOfKeyframes() {
  std::unique_lock<std::mutex>(mutexMap);
  return mKeyframes.size();
}

// Assumption: The first keyframe is never culled.
Keyframe* Map::getFirstKeyframe() {
  std::unique_lock<std::mutex>(mutexMap);
  return mKeyframes[0];
}
