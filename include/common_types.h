/**
BSD 3-Clause License

Copyright (c) 2018, Vladyslav Usenko and Nikolaus Demmel.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#pragma once

#include <bitset>
#include <cstdint>
#include <map>
#include <unordered_map>
#include <vector>

#include <tbb/concurrent_unordered_map.h>

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <sophus/se3.hpp>

#include "keypoints.h"

/// identifies a frame of multiple images (stereo pair)
using FrameId = int64_t;

/// identifies the camera (left or right)
using CamId = std::size_t;

/// pair of image timestamp and camera id identifies an image (imageId)
typedef std::pair<FrameId, CamId> TimeCamId;

inline std::ostream& operator<<(std::ostream& os, const TimeCamId& tcid) {
  os << tcid.first << "_" << tcid.second;
  return os;
}

/// ids for 2D features detected in images
using FeatureId = unsigned int;

class Keypoints;

/// feature corners is a collection of { iamgeId => KeypointsData }
using Corners = tbb::concurrent_unordered_map<TimeCamId, Keypoints>;

/// feature matches for an image pair
struct MatchData {
  /// estimated transformation (based on inliers or calibration) from the second
  /// image's coordinate system to the first image's corrdinate system
  Sophus::SE3d T_i_j;
  /// collection of {featureId_i, featureId_j} pairs of all matches
  std::vector<std::pair<FeatureId, FeatureId>> matches;
  /// collection of {featureId_i, featureId_j} pairs of inlier matches
  std::vector<std::pair<FeatureId, FeatureId>> inliers;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/// feature matches is a collection of { (imageId, imageId) => MatchData }
using Matches = tbb::concurrent_unordered_map<
    std::pair<TimeCamId, TimeCamId>, MatchData,
    tbb::tbb_hash<std::pair<TimeCamId, TimeCamId>>,
    std::equal_to<std::pair<TimeCamId, TimeCamId>>,
    Eigen::aligned_allocator<
        std::pair<const std::pair<TimeCamId, TimeCamId>, MatchData>>>;

/// pair of image and feature indices
using ImageFeaturePair = std::pair<TimeCamId, FeatureId>;

/// cameras in the map
struct Camera {
  // camera pose (transforms from camera to world)
  Sophus::SE3d T_w_c;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/// collection {imageId => Camera} for all cameras in the map
using Cameras =
    std::map<TimeCamId, Camera, std::less<TimeCamId>,
             Eigen::aligned_allocator<std::pair<const TimeCamId, Camera>>>;

/// Flags for different landmark outlier criteria
enum OutlierFlags {
  OutlierNone = 0,
  // reprojection error much too large
  OutlierReprojectionErrorHuge = 1 << 0,
  // reprojection error too large
  OutlierReprojectionErrorNormal = 1 << 1,
  // distance to a camera too small
  OutlierCameraDistance = 1 << 2,
  // z-coord in some camera frame too small
  OutlierZCoordinate = 1 << 3
};

/// info on a single projected landmark
struct ProjectedLandmark {
  Eigen::Vector2d point_measured;            //!< detected feature location
  Eigen::Vector2d point_reprojected;         //!< landmark projected into image
  Eigen::Vector3d point_3d_c;                //!< 3d point in camera coordinates
  double reprojection_error = 0;             //!< current reprojection error
  unsigned int outlier_flags = OutlierNone;  //!< flags for outlier
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

using ProjectedLandmarkPtr = std::shared_ptr<ProjectedLandmark>;
using ProjectedLandmarkConstPtr = std::shared_ptr<const ProjectedLandmark>;

/// all landmark projections for inlier and outlier observations for a single
/// image
struct ImageProjection {
  std::vector<ProjectedLandmarkConstPtr> obs;
  std::vector<ProjectedLandmarkConstPtr> outlier_obs;
};

/// projections for all images
using ImageProjections = std::map<TimeCamId, ImageProjection>;
