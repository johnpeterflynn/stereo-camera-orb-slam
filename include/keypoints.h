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
#include <set>

#include <Eigen/Dense>
#include <sophus/se3.hpp>

#include <pangolin/image/image.h>
#include <pangolin/image/image_io.h>
#include <pangolin/pangolin.h>

#include <opencv2/imgproc/imgproc.hpp>

#include "common_types.h"

const int PATCH_SIZE = 31;
const int HALF_PATCH_SIZE = 15;
const int EDGE_THRESHOLD = 19;

typedef std::bitset<256> Descriptor;

int findValidDescriptorMatchIndex(
    const std::bitset<256>& fa,
    const std::vector<std::bitset<256>>& corner_descriptors_b, double threshold,
    double dist_2_best);
void matchDescriptors(
    const std::vector<std::bitset<256>>& corner_descriptors_1,
    const std::vector<std::bitset<256>>& corner_descriptors_2,
    std::vector<std::pair<unsigned int, unsigned int>>& matches,
    double threshold, double dist_2_best);

class Keypoints {
 public:
  // APIs

  Keypoints() {}
  Keypoints(const std::string img, int nfeatures, bool rotateFeatures);

  void detectKeypointsAndDescriptors();

  bool computedDescriptors();

  // members

  /// collection of 2d corner points (indexed by FeatureId)
  std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>
      corners;
  /// collection of feature orientation (in radian) with same index as `corners`
  /// (indexed by FeatureId)
  std::vector<double> corner_angles;
  /// collection of feature descriptors with same index as `corners` (indexed by
  /// FeatureId)
  std::vector<std::bitset<256>> corner_descriptors;

 protected:
  void detectKeypoints(const pangolin::ManagedImage<uint8_t>& img_raw);
  double computeMoment(const pangolin::ManagedImage<uint8_t>& img_raw, int p,
                       int q, int u, int v);
  void computeAngles(const pangolin::ManagedImage<uint8_t>& img_raw);
  void computeDescriptors(const pangolin::ManagedImage<uint8_t>& img_raw);

  std::string mImage;

  int num_features;
  bool rotate_features;

  bool mbComputedDescriptors;
};
