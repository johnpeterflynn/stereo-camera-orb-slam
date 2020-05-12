#include "keyframe.h"
#include "keypoints.h"

// View Class : Start

View::View(TimeCamId tcid, std::string imgPath, int nfeatures,
           bool rotateFeatures)
    : mTimeCamId(tcid), parentKF(nullptr) {
  mKeypoints = Keypoints(imgPath, nfeatures, rotateFeatures);
}

TimeCamId View::id() const { return mTimeCamId; }

void View::setCamToWorld(const Sophus::SE3d Twc) { T_w_c = Twc; }

Sophus::SE3d& View::getCamToWorld() { return T_w_c; }

const Keypoints& View::getKeypoints() {
  if (!mKeypoints.computedDescriptors()) {
    mKeypoints.detectKeypointsAndDescriptors();
  }

  return mKeypoints;
}

Keyframe* View::getParentKeyframe() { return parentKF; }

void View::setParentKeyframe(Keyframe* pKF) { parentKF = pKF; }

// View Class : End

// Frame Class : Start

unsigned long int Frame::mNextId = 0;

Frame::Frame() {}

Frame::Frame(const Frame& frame)
    : mFrameId(frame.mFrameId), mRefKeyframe(frame.mRefKeyframe) {
  leftView = frame.leftView;
  rightView = frame.rightView;
}

Frame::Frame(std::string imglPath, std::string imgrPath, int nfeatures,
             bool rotateFeatures) {
  mFrameId = mNextId++;
  leftView =
      new View(TimeCamId(mFrameId, 0), imglPath, nfeatures, rotateFeatures);
  rightView =
      new View(TimeCamId(mFrameId, 1), imgrPath, nfeatures, rotateFeatures);
}

void Frame::setCamToWorld(const Sophus::SE3d Twc) {
  leftView->setCamToWorld(Twc);
}

Sophus::SE3d& Frame::getCamToWorld() { return leftView->getCamToWorld(); }

const Keypoints& Frame::getLeftKeypoints() { return leftView->getKeypoints(); }

const Keypoints& Frame::getRightKeypoints() {
  return rightView->getKeypoints();
}

Keyframe* Frame::getRefKeyframe() { return mRefKeyframe; }

void Frame::setRefKeyframe(Keyframe* keyframe) { mRefKeyframe = keyframe; }

// Frame Class : End

// Keyframe Class : Start

long unsigned int Keyframe::mNextId = 0;

Keyframe::Keyframe(Frame& frame, Map* m)
    : mframeid(frame.mFrameId),
      map(m),
      bNotErase(false),
      bToBeErased(false),
      bBad(false),
      mbFirstUpdateConnection(true) {
  mkfId = mNextId++;
  leftView = frame.leftView;
  rightView = frame.rightView;
  leftView->setParentKeyframe(this);
  rightView->setParentKeyframe(this);
}

bool operator<(const Keyframe& kf1, const Keyframe& kf2){
    return (kf1.mkfId < kf2.mkfId);
}

// Pose APIs

void Keyframe::setCamToWorld(const Sophus::SE3d Twc) {
  std::unique_lock<std::mutex> lock(mutexPose);
  leftView->setCamToWorld(Twc);
  rightView->setCamToWorld(Twc * T_0_1);
}

Sophus::SE3d& Keyframe::getCamToWorld() { return leftView->getCamToWorld(); }

void Keyframe::setStereoTransform(const Sophus::SE3d& T01) {
  std::unique_lock<std::mutex> lock(mutexPose);
  T_0_1 = T01;
}

Sophus::SE3d Keyframe::getStereoTransform() {
  std::unique_lock<std::mutex> lock(mutexPose);
  return T_0_1;
}

// Keypoints APIs

const Keypoints& Keyframe::getLeftKeypoints() {
  return leftView->getKeypoints();
}

const Keypoints& Keyframe::getRightKeypoints() {
  return rightView->getKeypoints();
}

void Keyframe::addLandmark(Landmark* pLM) {
  std::unique_lock<std::mutex> lock(mutexLandmarks);
  vpLandmarks.push_back(pLM);
}

void Keyframe::eraseLandmark(Landmark* pLM) {
  std::unique_lock<std::mutex> lock(mutexLandmarks);
  std::vector<Landmark*>::iterator itr =
      std::find(vpLandmarks.begin(), vpLandmarks.end(), pLM);
  if (itr != vpLandmarks.end()) vpLandmarks.erase(itr);
}

std::vector<Landmark*> Keyframe::getLandmarks() {
  std::unique_lock<std::mutex> lock(mutexLandmarks);
  return vpLandmarks;
}

bool Keyframe::observes(Landmark* plm) {
  std::unique_lock<std::mutex> lock(mutexLandmarks);
  if (std::find(vpLandmarks.begin(), vpLandmarks.end(), plm) !=
      vpLandmarks.end())
    return true;
  else
    return false;
}

std::vector<Keyframe*> Keyframe::getSortedCovisibleKeyframes() {
  std::unique_lock<std::mutex> lock(mutexConnections);
  return mSortedConnectedNeighborKFs;
}

std::vector<Keyframe*> Keyframe::getSortedCovisibleKeyframesWithMinWeight(
    int weight) {
  std::unique_lock<std::mutex> lock(mutexConnections);
  std::vector<Keyframe*> minWeightNeighborKFs;

  // Add all keyframes with shared number of landmarks >= weight
  for (size_t i = 0; i < mSortedConnectedNeighborWeights.size(); i++) {
    if (mSortedConnectedNeighborWeights[i] >= weight) {
      minWeightNeighborKFs.push_back(mSortedConnectedNeighborKFs[i]);
    } else {
      break;
    }
  }

  return minWeightNeighborKFs;
}

std::vector<int> Keyframe::getSortedNeighborWeights() {
  std::unique_lock<std::mutex> lock(mutexConnections);
  return mSortedConnectedNeighborWeights;
}

bool Keyframe::hasConnectedNeighbor(Keyframe* keyframe) const {
  auto kf_iter = mNeighborKFsWeights.find(keyframe);

  bool connected = false;

  if ((kf_iter != mNeighborKFsWeights.end()) &
      (kf_iter->second >= MIN_WEIGHT_THRESH)) {
    connected = true;
  }

  return connected;
}

void Keyframe::addConnection(Keyframe* neighbor, int weight) {
  {
    std::unique_lock<std::mutex> lock(mutexConnections);

    // Return from function if neighbor already exists and its
    // weight is the same.
    auto neighbor_iter = mNeighborKFsWeights.find(neighbor);
    if (neighbor_iter != mNeighborKFsWeights.end() &&
        neighbor_iter->second == weight) {
      return;
    } else {
      mNeighborKFsWeights[neighbor] = weight;
    }
  }

  // Weight updated, so update neighbors with largest weights
  updateBestNeighbors();
}

void Keyframe::eraseConnection(Keyframe* pKF) {
  bool bUpdate = false;
  {
    std::unique_lock<std::mutex> lock(mutexConnections);
    if (mNeighborKFsWeights.count(pKF)) {
      mNeighborKFsWeights.erase(pKF);
      bUpdate = true;
    }
  }

  if (bUpdate) updateBestNeighbors();
}

void Keyframe::eraseAllConnections() {
  for (auto& kf_weight_iter : mNeighborKFsWeights) {
    kf_weight_iter.first->eraseConnection(this);
  }
}

void Keyframe::updateBestNeighbors() {
  std::map<Keyframe*, int> keyframe_weights;
  {
    std::unique_lock<std::mutex> lock(mutexConnections);
    keyframe_weights = mNeighborKFsWeights;
  }

  std::vector<std::pair<int, Keyframe*>> weight_keyframes;
  weight_keyframes.reserve(keyframe_weights.size());

  // Build a sortable list of weight-keyframe pairs
  for (const auto& kf_weight_iter : keyframe_weights) {
    Keyframe* keyframe = kf_weight_iter.first;
    int weight = kf_weight_iter.second;

    weight_keyframes.push_back(std::make_pair(weight, keyframe));
  }

  // Sort in descending order aka largest weights first
  std::sort(weight_keyframes.begin(), weight_keyframes.end(),
            std::greater<std::pair<int, Keyframe*>>());

  std::vector<Keyframe*> neighbor_kfs;
  std::vector<int> neighbor_weights;
  neighbor_kfs.reserve(keyframe_weights.size());
  neighbor_weights.reserve(keyframe_weights.size());

  // Build sorted vectors of neighbor keyframes and their weights
  for (const auto& weight_kf_iter : weight_keyframes) {
    Keyframe* keyframe = weight_kf_iter.second;
    int weight = weight_kf_iter.first;

    neighbor_kfs.push_back(keyframe);
    neighbor_weights.push_back(weight);
  }

  // Save updated neighboring keyframes and weights (nodes and edges in
  // covisibility graph)
  {
    std::unique_lock<std::mutex> lock(mutexConnections);
    mNeighborKFsWeights = keyframe_weights;
    mSortedConnectedNeighborKFs = neighbor_kfs;
    mSortedConnectedNeighborWeights = neighbor_weights;
  }
}

void Keyframe::updateConnections() {
  std::map<Keyframe*, int> keyframe_weights;
  std::vector<Landmark*> landmarks = getLandmarks();

  std::cout << "updateConnections : landmarks size " << landmarks.size()
            << "\n";
  // Overview: Get a list of all neighboring keyframes and the
  // number of landmarks they each share with this keyframe (weights). Then
  // determine which keyframes have a minimum number of weights, notify them
  // of the updated number of weights and then build a sorted vector of
  // neighboring keyframes that have at least the minimum weight.

  // Hash all observed keyframes and the number of landmarks they share with
  // this keyframe
  for (const auto& landmark_iter : landmarks) {
    if (landmark_iter->isBad()) continue;
    for (auto& kf : landmark_iter->getKeyframeObservations()) {
      if (keyframe_weights.find(kf) != keyframe_weights.end())
        keyframe_weights[kf] += 1;
      else
        keyframe_weights[kf] = 1;
    }
  }

  // Remove reference to this keyframe (can't be its own neighbor)
  keyframe_weights.erase(this);

  std::vector<Keyframe*> neighbor_kfs;
  std::vector<int> neighbor_weights;

  if (keyframe_weights.empty()) {
    neighbor_kfs.clear();
    neighbor_weights.clear();

    std::cout << "WARNING : Keyframe " << mkfId
              << " does not share landmarks with any other keyframe\n";
  } else {
    std::vector<std::pair<int, Keyframe*>> weight_keyframes;
    weight_keyframes.reserve(keyframe_weights.size());

    int max_weight = -1;
    Keyframe* max_keyframe = nullptr;

    // Build a sortable vector of weight-keyframe pairs that meet the weight
    // threshold
    for (const auto& kf_weight_iter : keyframe_weights) {
      Keyframe* keyframe = kf_weight_iter.first;
      int weight = kf_weight_iter.second;

      if (weight > max_weight) {
        max_weight = weight;
        max_keyframe = keyframe;
      }

      // Only connect in cv graph if the weight is above a minimum threshold
      if (weight >= MIN_WEIGHT_THRESH) {
        weight_keyframes.push_back(std::make_pair(weight, keyframe));

        // Notify the other keyframe of the updated connection
        keyframe->addConnection(this, weight);
      }
    }

    // Necessary because datasets have cases where the only connections between
    // some keyframes are less than the MAX_WEIGHT_THRESH. It's more important
    // to keep the covisibility graph connected, so connect with the best
    // keyframe
    if (weight_keyframes.empty()) {
      weight_keyframes.push_back(std::make_pair(max_weight, max_keyframe));

      // Notify the other keyframe of the updated connection
      max_keyframe->addConnection(this, max_weight);
    }

    // Sort in descending order aka largest weights first
    std::sort(weight_keyframes.begin(), weight_keyframes.end(),
              std::greater<std::pair<int, Keyframe*>>());

    neighbor_kfs.reserve(keyframe_weights.size());
    neighbor_weights.reserve(keyframe_weights.size());

    // Build sorted vectors of neighbor keyframes and their weights
    for (const auto& weight_kf_iter : weight_keyframes) {
      Keyframe* keyframe = weight_kf_iter.second;
      int weight = weight_kf_iter.first;

      neighbor_kfs.push_back(keyframe);
      neighbor_weights.push_back(weight);
    }
  }

  // Save updated neighboring keyframes and weights (nodes and edges in
  // covisibility graph)
  {
    std::unique_lock<std::mutex> lock(mutexConnections);
    mNeighborKFsWeights = keyframe_weights;
    mSortedConnectedNeighborKFs = neighbor_kfs;
    mSortedConnectedNeighborWeights = neighbor_weights;

    // Find parent and assign this class as the child on the first update. The
    // very first keyframe (mframeid = 0) doesn't have a parent.
    if (mbFirstUpdateConnection && mframeid != 0 &&
        !mSortedConnectedNeighborKFs.empty()) {
      mSpanTreeParent = mSortedConnectedNeighborKFs.front();
      mSpanTreeParent->addChild(this);
      mbFirstUpdateConnection = false;
    }
  }
}

int Keyframe::getNeighborWeight(Keyframe* neighbor) {
  std::unique_lock<std::mutex> lock(mutexConnections);

  int weight = 0;
  auto iter = mNeighborKFsWeights.find(neighbor);

  if (iter != mNeighborKFsWeights.end()) {
    weight = iter->second;
  }

  return weight;
}

void Keyframe::addChild(Keyframe* child) {
  std::unique_lock<std::mutex> lock(mutexConnections);
  mSpanTreeChildren.insert(child);
}
void Keyframe::eraseChild(Keyframe* child) {
  std::unique_lock<std::mutex> lock(mutexConnections);
  mSpanTreeChildren.erase(child);
}
void Keyframe::changeParent(Keyframe* parent) {
  std::unique_lock<std::mutex> lock(mutexConnections);
  // mSpanTreeParent->eraseChild(this);
  mSpanTreeParent = parent;
  mSpanTreeParent->addChild(this);
}

std::set<Keyframe*> Keyframe::getChildren() {
  std::unique_lock<std::mutex> lock(mutexConnections);
  return mSpanTreeChildren;
}
Keyframe* Keyframe::getParent() {
  std::unique_lock<std::mutex> lock(mutexConnections);
  return mSpanTreeParent;
}

void Keyframe::detachFromSpanningTree() {
  std::unique_lock<std::mutex> lock(mutexConnections);

  // When this keyframe is removed from the spanning tree it needs to connect
  // its parent to its children in a way that preserves the spanning tree
  // (connects all children with no loops) and connectes each child to its
  // strongest neighbor. We do this by searching for neighbors of each child
  // that are either the original parent (mSpanTreeParent) or children of it,
  // starting with the strongest connections. If there are any children that are
  // orphaned (no parents found in their neighbors) after this search then they
  // are assigned directly to mSpanTreeParent.

  // parentCandidates is the list of Keyframes to which children must be
  // connected to preserve the connectivity of the spanning tree
  std::set<Keyframe*> parentCandidates;
  parentCandidates.insert(mSpanTreeParent);

  // Iterate until all children are removed from this Keyframe
  while (!mSpanTreeChildren.empty()) {
    bool bValidEdge = false;
    int max_weight = -1;
    Keyframe* child_with_valid_parent;
    Keyframe* valid_parent;

    // For each child of this Keyframe in the spanning tree..
    for (const auto& child : mSpanTreeChildren) {
      if (child->isBad()) {
        continue;
      }

      std::vector<Keyframe*> child_neighbors =
          child->getSortedCovisibleKeyframes();

      // Search each child's visible neighbors and see if any of those neighbors
      // are also a parent candidate. If so, keep track of the candidate with
      // the most shared weights and accept it as a valid parent
      for (const auto& child_neighbor : child_neighbors) {
        for (const auto& parent_candidate : parentCandidates) {
          if (child_neighbor->mkfId == parent_candidate->mkfId) {
            int weight = child->getNeighborWeight(child_neighbor);

            if (weight >= max_weight) {
              max_weight = weight;
              child_with_valid_parent = child;
              valid_parent = child_neighbor;
              bValidEdge = true;
            }
          }
        }
      }
    }

    // If a valid parent was found for a child, assign the parent to that child
    // and allow that child to be a parent candidate for another child
    if (bValidEdge) {
      child_with_valid_parent->changeParent(valid_parent);
      parentCandidates.insert(child_with_valid_parent);
      mSpanTreeChildren.erase(child_with_valid_parent);
    } else {
      // If no children matched with any parents then end loop
      break;
    }
  }

  // If there are any children without visible neighbors that are also parent
  // candidates then assign them to original parent of this keyframe.
  for (auto orphan_child : mSpanTreeChildren) {
    orphan_child->changeParent(mSpanTreeParent);
    mSpanTreeParent->addChild(orphan_child);
  }

  // Finally, remove this keyframe as a child.
  mSpanTreeParent->eraseChild(this);
}

void Keyframe::addLoopEdge(Keyframe* kf) {
  std::unique_lock<std::mutex> lock(mutexConnections);
  bNotErase = true;
  mLoopEdges.insert(kf);
}

std::set<Keyframe*> Keyframe::getLoopEdges() {
  std::unique_lock<std::mutex> lock(mutexConnections);
  return mLoopEdges;
}

void Keyframe::setErase() {
  bool bErase = false;
  {
    std::unique_lock<std::mutex> lock(mutexConnections);
    bNotErase = false;
    bErase = bToBeErased;
  }

  if (bErase) {
    setBadFlag();
  }
}

void Keyframe::setNotErase() {
  std::unique_lock<std::mutex> lock(mutexConnections);
  bNotErase = true;
}

bool Keyframe::isBad() {
  std::unique_lock<std::mutex> lock(mutexConnections);
  return bBad;
}

// CONTRACT: Since setBadFlag triggers Keyframe to notify other Landmarks and
// Keyframes to delete itself from those other Landmarks/Keyframes
// (deleteObservation() and eraseConnection(), respectively), it should only
// notify Landmarks/Keyframes that aren't also bad. The assumption is that a
// Keyframe or Landmark that is set to bad has already erased its contents.
void Keyframe::setBadFlag() {
  // This function is called when this keyframe needs to be removed from the
  // system Need to remove this keyframe from all landmarks Need to update
  // covisibility graph Need to remove from map Then it should self destroy
  {
    std::unique_lock<std::mutex> lock(mutexConnections);
    if (bNotErase) {
      bToBeErased = true;

      std::cout << "Info: Keyframe::setBadFlag(): Attempted to erase Keyframe "
                   "with bNotErase = true"
                << std::endl;
      return;
    }
  }

  // Only execute if Keyframe isn't already bad
  if (bBad) {
    return;
  }

  bBad = true;

  detachFromSpanningTree();

  {
    // Removing keyframe from all landmarks it observes
    std::unique_lock<std::mutex> lock2(mutexLandmarks);

    for (size_t i = 0; i < vpLandmarks.size(); i++) {
      Landmark* lm = vpLandmarks[i];

      // Delete this keyframe from landmark obs only if the landmark isn't bad
      if (!lm->isBad()) {
        lm->deleteObservation(this);
      }
    }

    vpLandmarks.clear();
  }

  // Removing current keyframe from all neighboring keyframes
  {
    for (std::map<Keyframe*, int>::iterator itr = mNeighborKFsWeights.begin();
         itr != mNeighborKFsWeights.end(); itr++)
      if (!itr->first->isBad()) {
        itr->first->eraseConnection(this);
      }
  }

  // Clearing neighbor list of current keyframe
  {
    std::unique_lock<std::mutex> lock(mutexConnections);
    mNeighborKFsWeights.clear();
    mSortedConnectedNeighborKFs.clear();
    mSortedConnectedNeighborWeights.clear();
  }

  map->eraseKeyframe(this);
}

// Keyframe Class : End
