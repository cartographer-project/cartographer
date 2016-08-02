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

#ifndef CARTOGRAPHER_MAPPING_3D_SUBMAPS_H_
#define CARTOGRAPHER_MAPPING_3D_SUBMAPS_H_

#include <memory>
#include <string>
#include <vector>

#include "Eigen/Geometry"
#include "cartographer/mapping/sparse_pose_graph.h"
#include "cartographer/mapping/submaps.h"
#include "cartographer/mapping_2d/laser_fan_inserter.h"
#include "cartographer/mapping_2d/probability_grid.h"
#include "cartographer/mapping_3d/hybrid_grid.h"
#include "cartographer/mapping_3d/laser_fan_inserter.h"
#include "cartographer/mapping_3d/proto/submaps_options.pb.h"
#include "cartographer/sensor/laser.h"
#include "cartographer/transform/transform.h"

namespace cartographer {
namespace mapping_3d {

void InsertIntoProbabilityGrid(
    const sensor::LaserFan3D& laser_fan_3d, const transform::Rigid3f& pose,
    const float slice_z, const mapping_2d::LaserFanInserter& laser_fan_inserter,
    mapping_2d::ProbabilityGrid* result);

proto::SubmapsOptions CreateSubmapsOptions(
    common::LuaParameterDictionary* parameter_dictionary);

struct Submap : public mapping::Submap {
  Submap(float high_resolution, float low_resolution,
         const Eigen::Vector3f& origin, int begin_laser_fan_index);

  HybridGrid high_resolution_hybrid_grid;
  HybridGrid low_resolution_hybrid_grid;
  bool finished = false;
  std::vector<int> trajectory_node_indices;
};

// A container of Submaps.
class Submaps : public mapping::Submaps {
 public:
  explicit Submaps(const proto::SubmapsOptions& options);

  Submaps(const Submaps&) = delete;
  Submaps& operator=(const Submaps&) = delete;

  const Submap* Get(int index) const override;
  int size() const override;
  void SubmapToProto(
      int index, const std::vector<mapping::TrajectoryNode>& trajectory_nodes,
      const transform::Rigid3d& global_submap_pose,
      mapping::proto::SubmapQuery::Response* response) override;

  // Inserts 'laser_fan' into the Submap collection.
  void InsertLaserFan(const sensor::LaserFan3D& laser_fan);

  // Returns the 'high_resolution' HybridGrid to be used for matching.
  const HybridGrid& high_resolution_matching_grid() const;

  // Returns the 'low_resolution' HybridGrid to be used for matching.
  const HybridGrid& low_resolution_matching_grid() const;

  // Adds a node to be used when visualizing the submap.
  void AddTrajectoryNodeIndex(int trajectory_node_index);

 private:
  struct PixelData {
    int min_z = INT_MAX;
    int max_z = INT_MIN;
    int count = 0;
    float probability_sum = 0.f;
    float max_probability = 0.5f;
  };

  void AddSubmap(const Eigen::Vector3f& origin);
  void AccumulatePixelData(const int width, const int height,
                           const Eigen::Array2i& min_index,
                           const Eigen::Array2i& max_index);
  void ExtractVoxelData(const HybridGrid& hybrid_grid,
                        const transform::Rigid3f& transform,
                        Eigen::Array2i* min_index, Eigen::Array2i* max_index);
  // Builds texture data containing interleaved value and alpha for the
  // visualization from 'accumulated_pixel_data_' into 'celldata_'.
  void ComputePixelValues(const int width, const int height);

  const proto::SubmapsOptions options_;

  std::vector<std::unique_ptr<Submap>> submaps_;
  LaserFanInserter laser_fan_inserter_;

  // Number of LaserFans inserted.
  int num_laser_fans_ = 0;

  // Number of LaserFans inserted since the last Submap was added.
  int num_laser_fans_in_last_submap_ = 0;

  // The following members are used for visualization and kept around for
  // performance reasons (mainly to avoid reallocations).
  std::vector<PixelData> accumulated_pixel_data_;
  string celldata_;
  // The first three entries of this is are a cell_index and the last is the
  // corresponding probability value. We batch them together like this to only
  // have one vector and have better cache locality.
  std::vector<Eigen::Array4i> voxel_indices_and_probabilities_;
};

}  // namespace mapping_3d
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_3D_SUBMAPS_H_
