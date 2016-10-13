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

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/mapping_3d/hybrid_grid.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/sensor/proto/adaptive_voxel_filter_options.pb.h"

namespace cartographer {
namespace sensor {

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

  PointCloud Filter(const PointCloud& point_cloud) const;

 private:
  const proto::AdaptiveVoxelFilterOptions options_;
};

}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER_SENSOR_VOXEL_FILTER_H_
