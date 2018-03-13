/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cartographer/mapping/internal/3d/scan_matching/rotational_scan_matcher.h"

#include <map>
#include <vector>

#include "cartographer/common/math.h"
#include "cartographer/common/port.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {

namespace {

constexpr float kMinDistance = 0.2f;
constexpr float kMaxDistance = 0.9f;
constexpr float kSliceHeight = 0.2f;

void AddValueToHistogram(float angle, const float value,
                         Eigen::VectorXf* histogram) {
  // Map the angle to [0, pi), i.e. a vector and its inverse are considered to
  // represent the same angle.
  while (angle > static_cast<float>(M_PI)) {
    angle -= static_cast<float>(M_PI);
  }
  while (angle < 0.f) {
    angle += static_cast<float>(M_PI);
  }
  const float zero_to_one = angle / static_cast<float>(M_PI);
  const int bucket = common::Clamp<int>(
      common::RoundToInt(histogram->size() * zero_to_one - 0.5f), 0,
      histogram->size() - 1);
  (*histogram)(bucket) += value;
}

Eigen::Vector3f ComputeCentroid(const sensor::PointCloud& slice) {
  CHECK(!slice.empty());
  Eigen::Vector3f sum = Eigen::Vector3f::Zero();
  for (const Eigen::Vector3f& point : slice) {
    sum += point;
  }
  return sum / static_cast<float>(slice.size());
}

void AddPointCloudSliceToHistogram(const sensor::PointCloud& slice,
                                   Eigen::VectorXf* const histogram) {
  if (slice.empty()) {
    return;
  }
  // We compute the angle of the ray from a point to the centroid of the whole
  // point cloud. If it is orthogonal to the angle we compute between points, we
  // will add the angle between points to the histogram with the maximum weight.
  // This is to reject, e.g., the angles observed on the ceiling and floor.
  const Eigen::Vector3f centroid = ComputeCentroid(slice);
  Eigen::Vector3f last_point = slice.front();
  for (const Eigen::Vector3f& point : slice) {
    const Eigen::Vector2f delta = (point - last_point).head<2>();
    const Eigen::Vector2f direction = (point - centroid).head<2>();
    const float distance = delta.norm();
    if (distance < kMinDistance || direction.norm() < kMinDistance) {
      continue;
    }
    if (distance > kMaxDistance) {
      last_point = point;
      continue;
    }
    const float angle = common::atan2(delta);
    const float value = std::max(
        0.f, 1.f - std::abs(delta.normalized().dot(direction.normalized())));
    AddValueToHistogram(angle, value, histogram);
  }
}

// A function to sort the points in each slice by angle around the centroid.
// This is because the returns from different rangefinders are interleaved in
// the data.
sensor::PointCloud SortSlice(const sensor::PointCloud& slice) {
  struct SortableAnglePointPair {
    bool operator<(const SortableAnglePointPair& rhs) const {
      return angle < rhs.angle;
    }

    float angle;
    Eigen::Vector3f point;
  };
  const Eigen::Vector3f centroid = ComputeCentroid(slice);
  std::vector<SortableAnglePointPair> by_angle;
  by_angle.reserve(slice.size());
  for (const Eigen::Vector3f& point : slice) {
    const Eigen::Vector2f delta = (point - centroid).head<2>();
    if (delta.norm() < kMinDistance) {
      continue;
    }
    by_angle.push_back(SortableAnglePointPair{common::atan2(delta), point});
  }
  std::sort(by_angle.begin(), by_angle.end());
  sensor::PointCloud result;
  for (const auto& pair : by_angle) {
    result.push_back(pair.point);
  }
  return result;
}

// Rotates the given 'histogram' by the given 'angle'. This might lead to
// rotations of a fractional bucket which is handled by linearly interpolating.
Eigen::VectorXf RotateHistogram(const Eigen::VectorXf& histogram,
                                const float angle) {
  const float rotate_by_buckets = -angle * histogram.size() / M_PI;
  int full_buckets = common::RoundToInt(rotate_by_buckets - 0.5f);
  const float fraction = rotate_by_buckets - full_buckets;
  while (full_buckets < 0) {
    full_buckets += histogram.size();
  }
  Eigen::VectorXf rotated_histogram_0 = Eigen::VectorXf::Zero(histogram.size());
  Eigen::VectorXf rotated_histogram_1 = Eigen::VectorXf::Zero(histogram.size());
  for (int i = 0; i != histogram.size(); ++i) {
    rotated_histogram_0[i] = histogram[(i + full_buckets) % histogram.size()];
    rotated_histogram_1[i] =
        histogram[(i + 1 + full_buckets) % histogram.size()];
  }
  return fraction * rotated_histogram_1 +
         (1.f - fraction) * rotated_histogram_0;
}

float MatchHistograms(const Eigen::VectorXf& submap_histogram,
                      const Eigen::VectorXf& scan_histogram) {
  // We compute the dot product of normalized histograms as a measure of
  // similarity.
  const float scan_histogram_norm = scan_histogram.norm();
  const float submap_histogram_norm = submap_histogram.norm();
  const float normalization = scan_histogram_norm * submap_histogram_norm;
  if (normalization < 1e-3f) {
    return 1.f;
  }
  return submap_histogram.dot(scan_histogram) / normalization;
}

}  // namespace

Eigen::VectorXf RotationalScanMatcher::ComputeHistogram(
    const sensor::PointCloud& point_cloud, const int histogram_size) {
  Eigen::VectorXf histogram = Eigen::VectorXf::Zero(histogram_size);
  std::map<int, sensor::PointCloud> slices;
  for (const Eigen::Vector3f& point : point_cloud) {
    slices[common::RoundToInt(point.z() / kSliceHeight)].push_back(point);
  }
  for (const auto& slice : slices) {
    AddPointCloudSliceToHistogram(SortSlice(slice.second), &histogram);
  }
  return histogram;
}

RotationalScanMatcher::RotationalScanMatcher(
    const std::vector<std::pair<Eigen::VectorXf, float>>& histograms_at_angles)
    : histogram_(
          Eigen::VectorXf::Zero(histograms_at_angles.at(0).first.size())) {
  for (const auto& histogram_at_angle : histograms_at_angles) {
    histogram_ +=
        RotateHistogram(histogram_at_angle.first, histogram_at_angle.second);
  }
}

std::vector<float> RotationalScanMatcher::Match(
    const Eigen::VectorXf& histogram, const float initial_angle,
    const std::vector<float>& angles) const {
  std::vector<float> result;
  result.reserve(angles.size());
  for (const float angle : angles) {
    const Eigen::VectorXf scan_histogram =
        RotateHistogram(histogram, initial_angle + angle);
    result.push_back(MatchHistograms(histogram_, scan_histogram));
  }
  return result;
}

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer
