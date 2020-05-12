#pragma once

#include <pangolin/image/image.h>
#include <pangolin/image/image_io.h>
#include <pangolin/pangolin.h>
#include "calibration.h"
#include "common_types.h"
#include "landmark.h"
#include "map.h"

#include <set>

class Landmark;
class Keypoints;
class Keyframe;
class Map;

class View {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  View(TimeCamId tcid, std::string imgPath, int nfeatures, bool rotateFeatures);

  TimeCamId id() const;

  // Pose functions
  void setCamToWorld(const Sophus::SE3d Twc);
  Sophus::SE3d& getCamToWorld();

  // Keypoints APIs
  const Keypoints& getKeypoints();

  // Get parent keyframe
  Keyframe* getParentKeyframe();
  void setParentKeyframe(Keyframe* pKF);

 protected:
  TimeCamId mTimeCamId;

  // Camera pose: Transforms from camera to world
  Sophus::SE3d T_w_c;

  // Keypoint Data of all detected points in an image, indexed by feature ID
  Keypoints mKeypoints;

  // Pointer to parent keyframe
  Keyframe* parentKF;
};

class Frame {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Frame();

  Frame(const Frame& frame);

  Frame(std::string imglPath, std::string imgrPath, int nfeatures,
        bool rotateFeatures);

  // Pose functions
  void setCamToWorld(const Sophus::SE3d Twc);
  Sophus::SE3d& getCamToWorld();

  const Keypoints& getLeftKeypoints();
  const Keypoints& getRightKeypoints();

  Keyframe* getRefKeyframe();
  void setRefKeyframe(Keyframe* keyframe);

  View* leftView;
  View* rightView;

  long unsigned int mFrameId;
  static long unsigned int mNextId;

 protected:
  Keyframe* mRefKeyframe;
};

class Keyframe {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Keyframe(Frame& frame, Map* m);

  // comparison function to sort the keyframes
  friend bool operator<(const Keyframe& kf1, const Keyframe& kf2);

  // Pose functions
  void setCamToWorld(const Sophus::SE3d Twc);
  Sophus::SE3d& getCamToWorld();
  void setStereoTransform(const Sophus::SE3d& T01);
  Sophus::SE3d getStereoTransform();

  // Keypoints APIs
  const Keypoints& getLeftKeypoints();
  const Keypoints& getRightKeypoints();

  // Landmark APIs
  void addLandmark(Landmark* plm);
  void eraseLandmark(Landmark* plm);
  std::vector<Landmark*> getLandmarks();
  bool observes(Landmark* plm);

  // Enable/Disable flags
  void setNotErase();
  void setErase();

  // API to remove the keyframe from the system
  void setBadFlag();
  bool isBad();

  // Covisibility Graph APIs
  std::vector<Keyframe*> getSortedCovisibleKeyframes();
  std::vector<Keyframe*> getSortedCovisibleKeyframesWithMinWeight(int weight);
  std::vector<int> getSortedNeighborWeights();
  bool hasConnectedNeighbor(Keyframe* keyframe) const;
  void updateConnections();
  void eraseConnection(Keyframe* pKF);
  void eraseAllConnections();
  void addConnection(Keyframe* neighbor, int weight);
  int getNeighborWeight(Keyframe* neighbor);

  // Essential Graph API
  void addChild(Keyframe* child);
  void eraseChild(Keyframe* child);
  void changeParent(Keyframe* parent);
  std::set<Keyframe*> getChildren();
  Keyframe* getParent();

  // Loop Edges
  void addLoopEdge(Keyframe* kf);
  std::set<Keyframe*> getLoopEdges();

 protected:
  void detachFromSpanningTree();

 public:
  View* leftView;
  View* rightView;

  // Identifier for number of keyframes
  static long unsigned int mNextId;
  long unsigned int mkfId;
  // Identifier for left and right frames in keyframe
  FrameId mframeid;

  // Minimum weight threshold between keyframes for a connection in
  // covisibility graph
  static const int MIN_WEIGHT_THRESH = 15;
  static const int MIN_ESSENTIAL_WEIGHT_THRESH = 100;

 protected:
  // Update the connected neighbors with the best weights
  void updateBestNeighbors();

  // Transformation between stereo pair
  Sophus::SE3d T_0_1;

  // Map Points observed by the frame
  std::vector<Landmark*> vpLandmarks;

  Map* map;

  // Flags
  bool bNotErase;
  bool bToBeErased;
  bool bBad;

  // Mutex variables
  std::mutex mutexPose;         // Mutex for accessing camera pose
  std::mutex mutexConnections;  // Mutex for accessing covisibility graph
  std::mutex mutexLandmarks;    // Mutex for accessing landmark data

  // Essential Graph  and Loop Edge variables
  bool mbFirstUpdateConnection;
  Keyframe* mSpanTreeParent;
  std::set<Keyframe*> mSpanTreeChildren;
  std::set<Keyframe*> mLoopEdges;

  // All neighbors in the covisibility greaph
  std::map<Keyframe*, int> mNeighborKFsWeights;

  // Sorted (descending), connected (> MIN_WEIGHT_THRESH weights) neighbors
  std::vector<Keyframe*> mSortedConnectedNeighborKFs;
  std::vector<int> mSortedConnectedNeighborWeights;
};
