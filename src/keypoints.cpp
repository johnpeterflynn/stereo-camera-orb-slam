#include "keypoints.h"

char pattern_31_x_a[256] = {
    8,   4,   -11, 7,   2,   1,   -2,  -13, -13, 10,  -13, -11, 7,   -4,  -13,
    -9,  12,  -3,  -6,  11,  4,   5,   3,   -8,  -2,  -13, -7,  -4,  -10, 5,
    5,   1,   9,   4,   2,   -4,  -8,  4,   0,   -13, -3,  -6,  8,   0,   7,
    -13, 10,  -6,  10,  -13, -13, 3,   5,   -1,  3,   2,   -13, -13, -13, -7,
    6,   -9,  -2,  -12, 3,   -7,  -3,  2,   -11, -1,  5,   -4,  -9,  -12, 10,
    7,   -7,  -4,  7,   -7,  -13, -3,  7,   -13, 1,   2,   -4,  -1,  7,   1,
    9,   -1,  -13, 7,   12,  6,   5,   2,   3,   2,   9,   -8,  -11, 1,   6,
    2,   6,   3,   7,   -11, -10, -5,  -10, 8,   4,   -10, 4,   -2,  -5,  7,
    -9,  -5,  8,   -9,  1,   7,   -2,  11,  -12, 3,   5,   0,   -9,  0,   -1,
    5,   3,   -13, -5,  -4,  6,   -7,  -13, 1,   4,   -2,  2,   -2,  4,   -6,
    -3,  7,   4,   -13, 7,   7,   -7,  -8,  -13, 2,   10,  -6,  8,   2,   -11,
    -12, -11, 5,   -2,  -1,  -13, -10, -3,  2,   -9,  -4,  -4,  -6,  6,   -13,
    11,  7,   -1,  -4,  -7,  -13, -7,  -8,  -5,  -13, 1,   1,   9,   5,   -1,
    -9,  -1,  -13, 8,   2,   7,   -10, -10, 4,   3,   -4,  5,   4,   -9,  0,
    -12, 3,   -10, 8,   -8,  2,   10,  6,   -7,  -3,  -1,  -3,  -8,  4,   2,
    6,   3,   11,  -3,  4,   2,   -10, -13, -13, 6,   0,   -13, -9,  -13, 5,
    2,   -1,  9,   11,  3,   -1,  3,   -13, 5,   8,   7,   -10, 7,   9,   7,
    -1};

char pattern_31_y_a[256] = {
    -3,  2,   9,   -12, -13, -7,  -10, -13, -3,  4,   -8,  7,   7,   -5,  2,
    0,   -6,  6,   -13, -13, 7,   -3,  -7,  -7,  11,  12,  3,   2,   -12, -12,
    -6,  0,   11,  7,   -1,  -12, -5,  11,  -8,  -2,  -2,  9,   12,  9,   -5,
    -6,  7,   -3,  -9,  8,   0,   3,   7,   7,   -10, -4,  0,   -7,  3,   12,
    -10, -1,  -5,  5,   -10, -7,  -2,  9,   -13, 6,   -3,  -13, -6,  -10, 2,
    12,  -13, 9,   -1,  6,   11,  7,   -8,  -7,  -3,  -6,  3,   -13, 1,   -1,
    1,   -9,  -13, 7,   -5,  3,   -13, -12, 8,   6,   -12, 4,   12,  12,  -9,
    3,   3,   -3,  8,   -5,  11,  -8,  5,   -1,  -6,  12,  -2,  0,   -8,  -6,
    -13, -13, -8,  -11, -8,  -4,  1,   -6,  -9,  7,   5,   -4,  12,  7,   2,
    11,  5,   -4,  9,   -7,  5,   6,   6,   -10, 1,   -2,  -12, -13, 1,   -10,
    -13, 5,   -2,  9,   1,   -8,  -4,  11,  6,   4,   -5,  -5,  -3,  -12, -2,
    -13, 0,   -3,  -13, -8,  -11, -2,  9,   -3,  -13, 6,   12,  -11, -3,  11,
    11,  -5,  12,  -8,  1,   -12, -2,  5,   -1,  7,   5,   0,   12,  -8,  11,
    -3,  -10, 1,   -11, -13, -13, -10, -8,  -6,  12,  2,   -13, -13, 9,   3,
    1,   2,   -10, -13, -12, 2,   6,   8,   10,  -9,  -13, -7,  -2,  2,   -5,
    -9,  -1,  -1,  0,   -11, -4,  -6,  7,   12,  0,   -1,  3,   8,   -6,  -9,
    7,   -6,  5,   -3,  0,   4,   -6,  0,   8,   9,   -4,  4,   3,   -7,  0,
    -6};

char pattern_31_x_b[256] = {
    9,   7,  -8, 12,  2,   1,  -2,  -11, -12, 11,  -8,  -9,  12,  -3,  -12, -7,
    12,  -2, -4, 12,  5,   10, 6,   -6,  -1,  -8,  -5,  -3,  -6,  6,   7,   4,
    11,  4,  4,  -2,  -7,  9,  1,   -8,  -2,  -4,  10,  1,   11,  -11, 12,  -6,
    12,  -8, -8, 7,   10,  1,  5,   3,   -13, -12, -11, -4,  12,  -7,  0,   -7,
    8,   -4, -1, 5,   -5,  0,  5,   -4,  -9,  -8,  12,  12,  -6,  -3,  12,  -5,
    -12, -2, 12, -11, 12,  3,  -2,  1,   8,   3,   12,  -1,  -10, 10,  12,  7,
    6,   2,  4,  12,  10,  -7, -4,  2,   7,   3,   11,  8,   9,   -6,  -5,  -3,
    -9,  12, 6,  -8,  6,   -2, -5,  10,  -8,  -5,  9,   -9,  1,   9,   -1,  12,
    -6,  7,  10, 2,   -5,  2,  1,   7,   6,   -8,  -3,  -3,  8,   -6,  -5,  3,
    8,   2,  12, 0,   9,   -3, -1,  12,  5,   -9,  8,   7,   -7,  -7,  -12, 3,
    12,  -6, 9,  2,   -10, -7, -10, 11,  -1,  0,   -12, -10, -2,  3,   -4,  -3,
    -2,  -4, 6,  -5,  12,  12, 0,   -3,  -6,  -8,  -6,  -6,  -4,  -8,  5,   10,
    10,  10, 1,  -6,  1,   -8, 10,  3,   12,  -5,  -8,  8,   8,   -3,  10,  5,
    -4,  3,  -6, 4,   -10, 12, -6,  3,   11,  8,   -6,  -3,  -1,  -3,  -8,  12,
    3,   11, 7,  12,  -3,  4,  2,   -8,  -11, -11, 11,  1,   -9,  -6,  -8,  8,
    3,   -1, 11, 12,  3,   0,  4,   -10, 12,  9,   8,   -10, 12,  10,  12,  0};

char pattern_31_y_b[256] = {
    5,   -12, 2,   -13, 12,  6,   -4,  -8,  -9,  9,   -9,  12,  6,   0,  -3,
    5,   -1,  12,  -8,  -8,  1,   -3,  12,  -2,  -10, 10,  -3,  7,   11, -7,
    -1,  -5,  -13, 12,  4,   7,   -10, 12,  -13, 2,   3,   -9,  7,   3,  -10,
    0,   1,   12,  -4,  -12, -4,  8,   -7,  -12, 6,   -10, 5,   12,  8,  7,
    8,   -6,  12,  5,   -13, 5,   -7,  -11, -13, -1,  2,   12,  6,   -4, -3,
    12,  5,   4,   2,   1,   5,   -6,  -7,  -12, 12,  0,   -13, 9,   -6, 12,
    6,   3,   5,   12,  9,   11,  10,  3,   -6,  -13, 3,   9,   -6,  -8, -4,
    -2,  0,   -8,  3,   -4,  10,  12,  0,   -6,  -11, 7,   7,   12,  2,  12,
    -8,  -2,  -13, 0,   -2,  1,   -4,  -11, 4,   12,  8,   8,   -13, 12, 7,
    -9,  -8,  9,   -3,  -12, 0,   12,  -2,  10,  -4,  -13, 12,  -6,  3,  -5,
    1,   -11, -7,  -5,  6,   6,   1,   -8,  -8,  9,   3,   7,   -8,  8,  3,
    -9,  -5,  8,   12,  9,   -5,  11,  -13, 2,   0,   -10, -7,  9,   11, 5,
    6,   -2,  7,   -2,  7,   -13, -8,  -9,  5,   10,  -13, -13, -1,  -9, -13,
    2,   12,  -10, -6,  -6,  -9,  -7,  -13, 5,   -13, -3,  -12, -1,  3,  -9,
    1,   -8,  9,   12,  -5,  7,   -8,  -12, 5,   9,   5,   4,   3,   12, 11,
    -13, 12,  4,   6,   12,  1,   1,   1,   -13, -13, 4,   -2,  -3,  -2, 10,
    -9,  -1,  -2,  -8,  5,   10,  5,   5,   11,  -6,  -12, 9,   4,   -2, -2,
    -11};

Keypoints::Keypoints(const std::string img, int nfeatures, bool rotateFeatures)
    : mImage(img),
      num_features(nfeatures),
      rotate_features(rotateFeatures),
      mbComputedDescriptors(false) {}

void Keypoints::detectKeypoints(
    const pangolin::ManagedImage<uint8_t>& img_raw) {
  cv::Mat image(img_raw.h, img_raw.w, CV_8U, img_raw.ptr);

  std::vector<cv::Point2f> points;
  goodFeaturesToTrack(image, points, num_features, 0.01, 8);

  corners.clear();
  corner_angles.clear();
  corner_descriptors.clear();

  for (size_t i = 0; i < points.size(); i++) {
    if (img_raw.InBounds(points[i].x, points[i].y, EDGE_THRESHOLD)) {
      corners.emplace_back(points[i].x, points[i].y);
    }
  }
}

bool Keypoints::computedDescriptors() { return mbComputedDescriptors; }

double Keypoints::computeMoment(const pangolin::ManagedImage<uint8_t>& img_raw,
                                int p, int q, int u, int v) {
  double m = 0;

  for (int x = -HALF_PATCH_SIZE; x <= HALF_PATCH_SIZE; x++) {
    const int x_sq = x * x;
    const int x_p = pow(x, p);

    for (int y = -HALF_PATCH_SIZE; y <= HALF_PATCH_SIZE; y++) {
      if (x_sq + y * y <= HALF_PATCH_SIZE * HALF_PATCH_SIZE) {
        // Q: What is a reasonable way to avoid using pow() here?
        m += x_p * pow(y, q) * img_raw(u + x, v + y);
      }
    }
  }

  return m;
}

void Keypoints::computeAngles(const pangolin::ManagedImage<uint8_t>& img_raw) {
  corner_angles.resize(corners.size());

  for (size_t i = 0; i < corners.size(); i++) {
    const Eigen::Vector2d& p = corners[i];
    double angle = 0;

    if (rotate_features) {
      int cx = p[0];
      int cy = p[1];
      double m_01 = computeMoment(img_raw, 0, 1, cx, cy);
      double m_10 = computeMoment(img_raw, 1, 0, cx, cy);
      angle = atan2(m_01, m_10);
    }

    corner_angles[i] = angle;
  }
}

void Keypoints::computeDescriptors(
    const pangolin::ManagedImage<uint8_t>& img_raw) {
  corner_descriptors.resize(corners.size());

  for (size_t i = 0; i < corners.size(); i++) {
    std::bitset<256> descriptor;

    const Eigen::Vector2d& p = corners[i];
    double angle = corner_angles[i];

    Eigen::Rotation2D<double> rot2(angle);
    for (unsigned int j = 0; j < descriptor.size(); j++) {
      Eigen::Vector2d Pa(pattern_31_x_a[j], pattern_31_y_a[j]);
      Eigen::Vector2d Pb(pattern_31_x_b[j], pattern_31_y_b[j]);

      Pa = p + rot2 * Pa;
      Pb = p + rot2 * Pb;

      int x_a = lround(Pa[0]);
      int y_a = lround(Pa[1]);
      int x_b = lround(Pb[0]);
      int y_b = lround(Pb[1]);

      if (img_raw(x_a, y_a) < img_raw(x_b, y_b)) {
        descriptor.set(j, 1);
      } else {
        descriptor.set(j, 0);
      }
    }

    corner_descriptors[i] = descriptor;
  }
}

void Keypoints::detectKeypointsAndDescriptors() {
  pangolin::ManagedImage<uint8_t> img_raw = pangolin::LoadImage(mImage);

  detectKeypoints(img_raw);
  computeAngles(img_raw);
  computeDescriptors(img_raw);

  mbComputedDescriptors = true;
}

int findValidDescriptorMatchIndex(
    const std::bitset<256>& fa,
    const std::vector<std::bitset<256>>& corner_descriptors_b, double threshold,
    double dist_2_best) {
  int index_valid = -1;
  int index_min = -1;
  int dist_min = std::numeric_limits<int>::max();
  int dist_2ndmin = std::numeric_limits<int>::max();

  for (size_t index = 0; index < corner_descriptors_b.size(); index++) {
    const std::bitset<256>& fb = corner_descriptors_b[index];

    // Compute the sum of squared differences between fa and fb. Equivalent to
    // computing the XOR between fa and fb and counting the 1's.
    int dist = int((fa ^ fb).count());

    // Keep track of the two smallest distances
    if (dist < dist_min) {
      index_min = int(index);
      dist_2ndmin = dist_min;
      dist_min = dist;
    } else if (dist < dist_2ndmin) {
      dist_2ndmin = dist;
    }
  }

  // Index for descriptor match in corner_descriptors_b is considered valid
  // if certain conditions on the two smallest distances are met
  if (dist_min < threshold && dist_2ndmin >= dist_min * dist_2_best) {
    index_valid = index_min;
  }

  return index_valid;
}

void matchDescriptors(
    const std::vector<std::bitset<256>>& corner_descriptors_1,
    const std::vector<std::bitset<256>>& corner_descriptors_2,
    std::vector<std::pair<unsigned int, unsigned int>>& matches,
    double threshold, double dist_2_best) {
  matches.clear();

  // For each descriptor f1..
  for (size_t i = 0; i < corner_descriptors_1.size(); i++) {
    const std::bitset<256>& f1 = corner_descriptors_1[i];

    // Find the best match for descriptor f1 in corner_descriptors_2
    int j = findValidDescriptorMatchIndex(f1, corner_descriptors_2, threshold,
                                          dist_2_best);

    // If a match f2 for f1 exists in corner_descriptors_2, then search the best
    // match for descriptor f2 in corner_descriptors_1.
    if (j != -1) {
      const std::bitset<256>& f2 = corner_descriptors_2[j];
      int k = findValidDescriptorMatchIndex(f2, corner_descriptors_1, threshold,
                                            dist_2_best);

      // If the the match for f1 is f2 and the match for f2 is f1 then the match
      // is considered valid.
      if (k != -1 && k == int(i)) {
        matches.push_back(std::pair<int, int>(i, j));
      }
    }
  }
}
