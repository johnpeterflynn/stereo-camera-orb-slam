#include "landmark.h"

long unsigned int Landmark::mNextId = 0;

Landmark::Landmark(const Eigen::Vector3d &pos, Map *m, Keyframe* kf) : map(m), mRefKf(kf), bBad(false) {
  mId = mNextId++;
  p = pos;
}

void Landmark::setWorldPos(const Eigen::Vector3d &pos) {
  std::unique_lock<std::mutex> lock(mutexPos);
  p = pos;
}

Eigen::Vector3d Landmark::getWorldPos() {
  std::unique_lock<std::mutex> lock(mutexPos);
  return p;
}

size_t Landmark::getNoOfObservations() {
  std::unique_lock<std::mutex> lock(mutexObs);
  return mObs.size();
}

std::map<View *, size_t> Landmark::getObservations() {
  std::unique_lock<std::mutex> lock(mutexObs);
  return mObs;
}

void Landmark::addObservation(View *view, size_t idx) {
  std::unique_lock<std::mutex> lock(mutexObs);
  if (mObs.count(view)) return;

  mObservingKeyframes.insert(view->getParentKeyframe());

  mObs[view] = idx;
}

void Landmark::addOutlierObservation(View *view, size_t idx) {
  std::unique_lock<std::mutex> lock(mutexOutObs);
  if (mOutObs.count(view)) return;

  mOutObs[view] = idx;
}

void Landmark::deleteObservation(Keyframe* kf) {
  bool bad = false;
  {
      std::unique_lock<std::mutex> lock(mutexObs);

      if (mObs.count(kf->leftView)) {
          mObs.erase(kf->leftView);
      }

      if (mObs.count(kf->rightView)) {
          mObs.erase(kf->rightView);
      }

      mObservingKeyframes.erase(kf);

      if (mRefKf == kf){
          mRefKf = *mObservingKeyframes.begin();
      }

      if (mObs.size() <= 2)  // If no of observations less than 2, discard the point
          bad = true;
  }

  if (bad) setBadFlag();
}

int Landmark::getIndexInObservations(View *view) {
  std::unique_lock<std::mutex> lock(mutexObs);

  if (mObs.count(view)) {
    return mObs[view];
  }
  return -1;
}

std::map<View *, size_t> Landmark::getOutlierObservations() {
  std::unique_lock<std::mutex> lock(mutexOutObs);
  return mOutObs;
}

bool Landmark::isInObservations(View *view) {
  std::unique_lock<std::mutex> lock(mutexObs);
  return (mObs.count(view));
}

bool Landmark::isInObservations(FrameId frame_id) {
  std::unique_lock<std::mutex> lock(mutexObs);
  // This functions could be improved by having a way to look
  // up a View/FeatureId pair by FrameId directly.
  for (const auto &iter : mObs) {
    if (iter.first->id().first == frame_id) {
      return true;
    }
  }

  return false;
}

bool Landmark::isInOutlierObservations(View *view) {
  std::unique_lock<std::mutex> lock(mutexOutObs);
  return (mOutObs.count(view));
}

bool Landmark::isInOutlierObservations(FrameId frame_id) {
  std::unique_lock<std::mutex> lock(mutexOutObs);
  // This functions could be improved by having a way to look
  // up a View/FeatureId pair by FrameId directly.
  for (const auto &iter : mOutObs) {
    if (iter.first->id().first == frame_id) {
      return true;
    }
  }

  return false;
}

Keyframe* Landmark::getRefKeyframe() {
    std::unique_lock<std::mutex> lock(mutexObs);
    return mRefKf;
}

void Landmark::replace(Landmark *lm) {
  if (this->mId == lm->mId) return;

  std::map<View *, size_t> obs;
  {
    std::unique_lock<std::mutex> lock(mutexObs);
    obs = mObs;
    mObs.clear();
    mObservingKeyframes.clear();
    bBad = true;
  }

  std::vector<Keyframe *> replacedKeyframes;

  for (auto &observation : obs) {
    View *view = observation.first;
    FeatureId fId = observation.second;
    Keyframe *kf = view->getParentKeyframe();
    if (!kf->observes(lm)) {
      replacedKeyframes.push_back(kf);
      kf->addLandmark(lm);
      lm->addObservation(view, fId);
    } else {
      if (std::find(replacedKeyframes.begin(), replacedKeyframes.end(), kf) !=
          replacedKeyframes.end()) {
        lm->addObservation(view, fId);
      }
    }
    kf->eraseLandmark(this);
  }

  map->eraseLandmark(this);
}

// CONTRACT: Since setBadFlag triggers Landmark to notify other Landmarks and
// Keyframes to delete itself from those other Landmarks/Keyframes
// (deleteObservation() and eraseConnection(), respectively), it should only
// notify Landmarks/Keyframes that aren't also bad. The assumption is that a
// Keyframe or Landmark that is set to bad has already erased its contents.
void Landmark::setBadFlag() {
  {
    // Only execute if Landmark isn't already bad
    if (bBad) {
      return;
    }

    std::unique_lock<std::mutex> lock(mutexPos);
    std::unique_lock<std::mutex> lock2(mutexObs);
    bBad = true;
    std::map<View *, size_t>::iterator itrObs;
    Keyframe *parentKf;
    for (itrObs = mObs.begin(); itrObs != mObs.end(); itrObs++) {
      parentKf = itrObs->first->getParentKeyframe();

      // Delete this landmark from parent keyframe only if the keyframe isn't
      // bad
      if (!parentKf->isBad()) {
        parentKf->eraseLandmark(this);
      }
    }

    mObs.clear();
    mObservingKeyframes.clear();
  }
  {
    std::unique_lock<std::mutex> lock(mutexOutObs);
    mOutObs.clear();
  }
  map->eraseLandmark(this);
}

bool Landmark::isBad() {
  std::unique_lock<std::mutex> lock(mutexPos);
  std::unique_lock<std::mutex> lock2(mutexObs);
  return bBad;
}

size_t Landmark::getNumKeyframeObservations() {
  std::unique_lock<std::mutex> lock(mutexObs);
  return mObservingKeyframes.size();
}

std::set<Keyframe *> Landmark::getKeyframeObservations() {
  std::unique_lock<std::mutex> lock(mutexObs);
  return mObservingKeyframes;
}
