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

#include "cartographer/mapping_3d/scan_matching/rotational_scan_matcher.h"

#include <map>
#include <vector>

#include "cartographer/common/math.h"

namespace cartographer {
namespace mapping_3d {
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

struct AngleValuePair {
  float angle;
  float value;
};

void AddPointCloudSliceToValueVector(
    const sensor::PointCloud& slice,
    std::vector<AngleValuePair>* value_vector) {
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
    value_vector->push_back(AngleValuePair{angle, value});
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

std::vector<AngleValuePair> GetValuesForHistogram(
    const sensor::PointCloud& point_cloud) {
  std::map<int, sensor::PointCloud> slices;
  for (const Eigen::Vector3f& point : point_cloud) {
    slices[common::RoundToInt(point.z() / kSliceHeight)].push_back(point);
  }
  std::vector<AngleValuePair> result;
  for (const auto& slice : slices) {
    AddPointCloudSliceToValueVector(SortSlice(slice.second), &result);
  }
  return result;
}

void AddValuesToHistogram(const std::vector<AngleValuePair>& value_vector,
                          const float rotation, Eigen::VectorXf* histogram) {
  for (const AngleValuePair& pair : value_vector) {
    AddValueToHistogram(pair.angle + rotation, pair.value, histogram);
  }
}

}  // namespace

RotationalScanMatcher::RotationalScanMatcher(
    const std::vector<mapping::TrajectoryNode>& nodes, const int histogram_size)
    : histogram_(Eigen::VectorXf::Zero(histogram_size)) {
  for (const mapping::TrajectoryNode& node : nodes) {
    AddValuesToHistogram(
        GetValuesForHistogram(sensor::TransformPointCloud(
            node.constant_data->range_data_3d.returns.Decompress(),
            node.pose.cast<float>())),
        0.f, &histogram_);
  }
}

std::vector<float> RotationalScanMatcher::Match(
    const sensor::PointCloud& point_cloud,
    const std::vector<float>& angles) const {
  std::vector<float> result;
  result.reserve(angles.size());
  const std::vector<AngleValuePair> value_vector =
      GetValuesForHistogram(point_cloud);
  for (const float angle : angles) {
    Eigen::VectorXf scan_histogram = Eigen::VectorXf::Zero(histogram_.size());
    AddValuesToHistogram(value_vector, angle, &scan_histogram);
    result.push_back(MatchHistogram(scan_histogram));
  }
  return result;
}

float RotationalScanMatcher::MatchHistogram(
    const Eigen::VectorXf& scan_histogram) const {
  // We compute the dot product of normalized histograms as a measure of
  // similarity.
  const float scan_histogram_norm = scan_histogram.norm();
  const float histogram_norm = histogram_.norm();
  const float normalization = scan_histogram_norm * histogram_norm;
  if (normalization < 1e-3f) {
    return 1.f;
  }
  return histogram_.dot(scan_histogram) / normalization;
}

}  // namespace scan_matching
}  // namespace mapping_3d
}  // namespace cartographer
