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

#ifndef CARTOGRAPHER_MAPPING_2D_SUBMAPS_H_
#define CARTOGRAPHER_MAPPING_2D_SUBMAPS_H_

#include <memory>
#include <vector>

#include "Eigen/Core"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/mapping/proto/submaps.pb.h"
#include "cartographer/mapping/submaps.h"
#include "cartographer/mapping/trajectory_node.h"
#include "cartographer/mapping_2d/laser_fan_inserter.h"
#include "cartographer/mapping_2d/map_limits.h"
#include "cartographer/mapping_2d/probability_grid.h"
#include "cartographer/mapping_2d/proto/submaps_options.pb.h"
#include "cartographer/sensor/laser.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace mapping_2d {

ProbabilityGrid ComputeCroppedProbabilityGrid(
    const ProbabilityGrid& probability_grid);

proto::SubmapsOptions CreateSubmapsOptions(
    common::LuaParameterDictionary* parameter_dictionary);

struct Submap : public mapping::Submap {
  Submap(const MapLimits& limits, const Eigen::Vector2f& origin,
         int begin_laser_fan_index);

  ProbabilityGrid probability_grid;
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
  void InsertLaserFan(const sensor::LaserFan& laser_fan);

 private:
  void FinishSubmap(int index);
  void AddSubmap(const Eigen::Vector2f& origin);

  const proto::SubmapsOptions options_;

  std::vector<std::unique_ptr<Submap>> submaps_;
  LaserFanInserter laser_fan_inserter_;

  // Number of LaserFans inserted.
  int num_laser_fans_ = 0;

  // Number of LaserFans inserted since the last Submap was added.
  int num_laser_fans_in_last_submap_ = 0;
};

}  // namespace mapping_2d
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_2D_SUBMAPS_H_
