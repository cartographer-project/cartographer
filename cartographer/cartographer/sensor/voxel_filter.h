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

#ifndef CARTOGRAPHER_SENSOR_VOXEL_FILTER_H_
#define CARTOGRAPHER_SENSOR_VOXEL_FILTER_H_

#include <tuple>
#include <unordered_set>
#include <utility>

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/mapping_3d/hybrid_grid.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/sensor/proto/adaptive_voxel_filter_options.pb.h"

namespace cartographer {
namespace sensor {

// Returns a voxel filtered copy of 'point_cloud' where 'size' is the length
// a voxel edge.
PointCloud2D VoxelFiltered(const PointCloud2D& point_cloud, float size);

// Returns a voxel filtered copy of 'point_cloud' where 'size' is the length
// a voxel edge.
PointCloud VoxelFiltered(const PointCloud& point_cloud, float size);

// Voxel filter for point clouds. For each voxel, the assembled point cloud
// contains the first point that fell into it from any of the inserted point
// clouds.
class VoxelFilter {
 public:
  // 'size' is the length of a voxel edge.
  explicit VoxelFilter(float size);

  VoxelFilter(const VoxelFilter&) = delete;
  VoxelFilter& operator=(const VoxelFilter&) = delete;

  // Inserts a point cloud into the voxel filter.
  void InsertPointCloud(const PointCloud2D& point_cloud);

  // Returns the filtered point cloud representing the occupied voxels.
  const PointCloud2D& point_cloud() const;

 private:
  struct IntegerPairHash {
    size_t operator()(const std::pair<int64, int64>& x) const {
      const uint64 first = x.first;
      const uint64 second = x.second;
      return first ^ (first + 0x9e3779b9u + (second << 6) + (second >> 2));
    }
  };

  const float size_;
  std::unordered_set<std::pair<int64, int64>, IntegerPairHash> voxels_;
  PointCloud2D point_cloud_;
};

// The same as VoxelFilter but for 3D PointClouds.
class VoxelFilter3D {
 public:
  // 'size' is the length of a voxel edge.
  explicit VoxelFilter3D(float size);

  VoxelFilter3D(const VoxelFilter3D&) = delete;
  VoxelFilter3D& operator=(const VoxelFilter3D&) = delete;

  // Inserts a point cloud into the voxel filter.
  void InsertPointCloud(const PointCloud& point_cloud);

  // Returns the filtered point cloud representing the occupied voxels.
  const PointCloud& point_cloud() const;

 private:
  mapping_3d::HybridGridBase<uint8> voxels_;
  PointCloud point_cloud_;
};

proto::AdaptiveVoxelFilterOptions CreateAdaptiveVoxelFilterOptions(
    common::LuaParameterDictionary* const parameter_dictionary);

class AdaptiveVoxelFilter {
 public:
  explicit AdaptiveVoxelFilter(
      const proto::AdaptiveVoxelFilterOptions& options);

  AdaptiveVoxelFilter(const AdaptiveVoxelFilter&) = delete;
  AdaptiveVoxelFilter& operator=(const AdaptiveVoxelFilter&) = delete;

  PointCloud2D Filter(const PointCloud2D& point_cloud) const;
  PointCloud Filter(const PointCloud& point_cloud) const;

 private:
  const proto::AdaptiveVoxelFilterOptions options_;
};

}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER_SENSOR_VOXEL_FILTER_H_
