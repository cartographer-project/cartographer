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

#include "cartographer/sensor/internal/voxel_filter.h"

#include <cmath>
#include <random>
#include <utility>

#include "absl/container/flat_hash_map.h"
#include "cartographer/common/math.h"

namespace cartographer {
namespace sensor {

namespace {

PointCloud FilterByMaxRange(const PointCloud& point_cloud,
                            const float max_range) {
  PointCloud result;
  for (const RangefinderPoint& point : point_cloud) {
    if (point.position.norm() <= max_range) {
      result.push_back(point);
    }
  }
  return result;
}

PointCloud AdaptivelyVoxelFiltered(
    const proto::AdaptiveVoxelFilterOptions& options,
    const PointCloud& point_cloud) {
  if (point_cloud.size() <= options.min_num_points()) {
    // 'point_cloud' is already sparse enough.
    return point_cloud;
  }
  PointCloud result = VoxelFilter(point_cloud, options.max_length());
  if (result.size() >= options.min_num_points()) {
    // Filtering with 'max_length' resulted in a sufficiently dense point cloud.
    return result;
  }
  // Search for a 'low_length' that is known to result in a sufficiently
  // dense point cloud. We give up and use the full 'point_cloud' if reducing
  // the edge length by a factor of 1e-2 is not enough.
  for (float high_length = options.max_length();
       high_length > 1e-2f * options.max_length(); high_length /= 2.f) {
    float low_length = high_length / 2.f;
    result = VoxelFilter(point_cloud, low_length);
    if (result.size() >= options.min_num_points()) {
      // Binary search to find the right amount of filtering. 'low_length' gave
      // a sufficiently dense 'result', 'high_length' did not. We stop when the
      // edge length is at most 10% off.
      while ((high_length - low_length) / low_length > 1e-1f) {
        const float mid_length = (low_length + high_length) / 2.f;
        const PointCloud candidate = VoxelFilter(point_cloud, mid_length);
        if (candidate.size() >= options.min_num_points()) {
          low_length = mid_length;
          result = candidate;
        } else {
          high_length = mid_length;
        }
      }
      return result;
    }
  }
  return result;
}

using VoxelKeyType = uint64_t;

VoxelKeyType GetVoxelCellIndex(const Eigen::Vector3f& point,
                               const float resolution) {
  const Eigen::Array3f index = point.array() / resolution;
  const uint64_t x = common::RoundToInt(index.x());
  const uint64_t y = common::RoundToInt(index.y());
  const uint64_t z = common::RoundToInt(index.z());
  return (x << 42) + (y << 21) + z;
}

template <class T, class PointFunction>
std::vector<T> RandomizedVoxelFilter(const std::vector<T>& point_cloud,
                                     const float resolution,
                                     PointFunction&& point_function) {
  // According to https://en.wikipedia.org/wiki/Reservoir_sampling
  std::minstd_rand0 generator;
  absl::flat_hash_map<VoxelKeyType, std::pair<int, int>>
      voxel_count_and_point_index;
  for (size_t i = 0; i < point_cloud.size(); i++) {
    auto& voxel = voxel_count_and_point_index[GetVoxelCellIndex(
        point_function(point_cloud[i]), resolution)];
    voxel.first++;
    if (voxel.first == 1) {
      voxel.second = i;
    } else {
      std::uniform_int_distribution<> distribution(1, voxel.first);
      if (distribution(generator) == voxel.first) {
        voxel.second = i;
      }
    }
  }
  std::vector<bool> points_used(point_cloud.size(), false);
  for (const auto& voxel_and_index : voxel_count_and_point_index) {
    points_used[voxel_and_index.second.second] = true;
  }

  std::vector<T> results;
  for (size_t i = 0; i < point_cloud.size(); i++) {
    if (points_used[i]) {
      results.push_back(point_cloud[i]);
    }
  }
  return results;
}

}  // namespace

PointCloud VoxelFilter(const PointCloud& point_cloud, const float resolution) {
  return RandomizedVoxelFilter(
      point_cloud, resolution,
      [](const RangefinderPoint& point) { return point.position; });
}

TimedPointCloud VoxelFilter(const TimedPointCloud& timed_point_cloud,
                            const float resolution) {
  return RandomizedVoxelFilter(
      timed_point_cloud, resolution,
      [](const TimedRangefinderPoint& point) { return point.position; });
}

std::vector<sensor::TimedPointCloudOriginData::RangeMeasurement> VoxelFilter(
    const std::vector<sensor::TimedPointCloudOriginData::RangeMeasurement>&
        range_measurements,
    const float resolution) {
  return RandomizedVoxelFilter(
      range_measurements, resolution,
      [](const sensor::TimedPointCloudOriginData::RangeMeasurement&
             range_measurement) {
        return range_measurement.point_time.position;
      });
}

proto::AdaptiveVoxelFilterOptions CreateAdaptiveVoxelFilterOptions(
    common::LuaParameterDictionary* const parameter_dictionary) {
  proto::AdaptiveVoxelFilterOptions options;
  options.set_max_length(parameter_dictionary->GetDouble("max_length"));
  options.set_min_num_points(
      parameter_dictionary->GetNonNegativeInt("min_num_points"));
  options.set_max_range(parameter_dictionary->GetDouble("max_range"));
  return options;
}

PointCloud AdaptiveVoxelFilter(
    const PointCloud& point_cloud,
    const proto::AdaptiveVoxelFilterOptions& options) {
  return AdaptivelyVoxelFiltered(
      options, FilterByMaxRange(point_cloud, options.max_range()));
}

}  // namespace sensor
}  // namespace cartographer
