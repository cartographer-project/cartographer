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
#include "cartographer/mapping/proto/serialization.pb.h"
#include "cartographer/mapping/proto/submap_visualization.pb.h"
#include "cartographer/mapping/submaps.h"
#include "cartographer/mapping/trajectory_node.h"
#include "cartographer/mapping_2d/map_limits.h"
#include "cartographer/mapping_2d/probability_grid.h"
#include "cartographer/mapping_2d/proto/submaps_options.pb.h"
#include "cartographer/mapping_2d/range_data_inserter.h"
#include "cartographer/sensor/range_data.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace mapping_2d {

ProbabilityGrid ComputeCroppedProbabilityGrid(
    const ProbabilityGrid& probability_grid);

proto::SubmapsOptions CreateSubmapsOptions(
    common::LuaParameterDictionary* parameter_dictionary);

class Submap : public mapping::Submap {
 public:
  Submap(const MapLimits& limits, const Eigen::Vector2f& origin);
  explicit Submap(const mapping::proto::Submap2D& proto);

  void ToProto(mapping::proto::Submap* proto) const override;

  const ProbabilityGrid& probability_grid() const { return probability_grid_; }
  bool finished() const { return finished_; }

  void ToResponseProto(
      const transform::Rigid3d& global_submap_pose,
      mapping::proto::SubmapQuery::Response* response) const override;

  // Insert 'range_data' into this submap using 'range_data_inserter'. The
  // submap must not be finished yet.
  void InsertRangeData(const sensor::RangeData& range_data,
                       const RangeDataInserter& range_data_inserter);
  void Finish();

 private:
  ProbabilityGrid probability_grid_;
  bool finished_ = false;
};

// Except during initialization when only a single submap exists, there are
// always two submaps into which range data is inserted: an old submap that is
// used for matching, and a new one, which will be used for matching next, that
// is being initialized.
//
// Once a certain number of range data have been inserted, the new submap is
// considered initialized: the old submap is no longer changed, the "new" submap
// is now the "old" submap and is used for scan-to-map matching. Moreover, a
// "new" submap gets created. The "old" submap is forgotten by this object.
class ActiveSubmaps {
 public:
  explicit ActiveSubmaps(const proto::SubmapsOptions& options);

  ActiveSubmaps(const ActiveSubmaps&) = delete;
  ActiveSubmaps& operator=(const ActiveSubmaps&) = delete;

  // Returns the index of the newest initialized Submap which can be
  // used for scan-to-map matching.
  int matching_index() const;

  // Inserts 'range_data' into the Submap collection.
  void InsertRangeData(const sensor::RangeData& range_data);

  std::vector<std::shared_ptr<Submap>> submaps() const;

 private:
  void FinishSubmap();
  void AddSubmap(const Eigen::Vector2f& origin);

  const proto::SubmapsOptions options_;
  int matching_submap_index_ = 0;
  std::vector<std::shared_ptr<Submap>> submaps_;
  RangeDataInserter range_data_inserter_;
};

}  // namespace mapping_2d
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_2D_SUBMAPS_H_
