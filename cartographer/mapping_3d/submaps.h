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
#include "cartographer/common/port.h"
#include "cartographer/mapping/id.h"
#include "cartographer/mapping/proto/submap_visualization.pb.h"
#include "cartographer/mapping/submaps.h"
#include "cartographer/mapping_2d/probability_grid.h"
#include "cartographer/mapping_2d/range_data_inserter.h"
#include "cartographer/mapping_3d/hybrid_grid.h"
#include "cartographer/mapping_3d/proto/submaps_options.pb.h"
#include "cartographer/mapping_3d/range_data_inserter.h"
#include "cartographer/sensor/range_data.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"

namespace cartographer {
namespace mapping_3d {

void InsertIntoProbabilityGrid(
    const sensor::RangeData& range_data, const transform::Rigid3f& pose,
    const float slice_z,
    const mapping_2d::RangeDataInserter& range_data_inserter,
    mapping_2d::ProbabilityGrid* result);

proto::SubmapsOptions CreateSubmapsOptions(
    common::LuaParameterDictionary* parameter_dictionary);

class Submap : public mapping::Submap {
 public:
  Submap(float high_resolution, float low_resolution,
         const transform::Rigid3d& local_pose);

  const HybridGrid& high_resolution_hybrid_grid() const {
    return high_resolution_hybrid_grid_;
  }
  const HybridGrid& low_resolution_hybrid_grid() const {
    return low_resolution_hybrid_grid_;
  }
  const bool finished() const { return finished_; }

  void ToResponseProto(
      const transform::Rigid3d& global_submap_pose,
      mapping::proto::SubmapQuery::Response* response) const override;

 private:
  // TODO(hrapp): Remove friend declaration.
  friend class Submaps;

  HybridGrid high_resolution_hybrid_grid_;
  HybridGrid low_resolution_hybrid_grid_;
  bool finished_ = false;
};

// Submaps is a sequence of maps to which scans are matched and into which scans
// are inserted.
//
// Except during initialization when only a single submap exists, there are
// always two submaps into which scans are inserted: an old submap that is used
// for matching, and a new one, which will be used for matching next, that is
// being initialized.
//
// Once a certain number of scans have been inserted, the new submap is
// considered initialized: the old submap is no longer changed, the "new" submap
// is now the "old" submap and is used for scan-to-map matching. Moreover,
// a "new" submap gets inserted.
class Submaps {
 public:
  explicit Submaps(const proto::SubmapsOptions& options);

  Submaps(const Submaps&) = delete;
  Submaps& operator=(const Submaps&) = delete;

  std::shared_ptr<const Submap> Get(int index) const;
  int size() const;

  // Returns the index of the newest initialized Submap which can be
  // used for scan-to-map matching.
  int matching_index() const;

  // Returns the indices of the Submap into which point clouds will
  // be inserted.
  std::vector<int> insertion_indices() const;

  // Inserts 'range_data' into the Submap collection. 'gravity_alignment' is
  // used for the orientation of new submaps so that the z axis approximately
  // aligns with gravity.
  void InsertRangeData(const sensor::RangeData& range_data,
                       const Eigen::Quaterniond& gravity_alignment);

 private:
  void AddSubmap(const transform::Rigid3d& local_pose);

  const proto::SubmapsOptions options_;
  std::vector<std::shared_ptr<Submap>> submaps_;
  RangeDataInserter range_data_inserter_;
};

}  // namespace mapping_3d
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_3D_SUBMAPS_H_
