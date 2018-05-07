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

#ifndef CARTOGRAPHER_MAPPING_3D_SUBMAP_3D_H_
#define CARTOGRAPHER_MAPPING_3D_SUBMAP_3D_H_

#include <memory>
#include <string>
#include <vector>

#include "Eigen/Geometry"
#include "cartographer/common/port.h"
#include "cartographer/mapping/3d/hybrid_grid.h"
#include "cartographer/mapping/3d/range_data_inserter_3d.h"
#include "cartographer/mapping/id.h"
#include "cartographer/mapping/proto/3d/submaps_options_3d.pb.h"
#include "cartographer/mapping/proto/serialization.pb.h"
#include "cartographer/mapping/proto/submap_visualization.pb.h"
#include "cartographer/mapping/submaps.h"
#include "cartographer/sensor/range_data.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"

namespace cartographer {
namespace mapping {

proto::SubmapsOptions3D CreateSubmapsOptions3D(
    common::LuaParameterDictionary* parameter_dictionary);

class Submap3D : public Submap {
 public:
  Submap3D(float high_resolution, float low_resolution,
           const transform::Rigid3d& local_submap_pose);
  explicit Submap3D(const proto::Submap3D& proto);

  void ToProto(proto::Submap* proto,
               bool include_probability_grid_data) const override;
  void UpdateFromProto(const proto::Submap& proto) override;

  void ToResponseProto(const transform::Rigid3d& global_submap_pose,
                       proto::SubmapQuery::Response* response) const override;

  const HybridGrid& high_resolution_hybrid_grid() const {
    return *high_resolution_hybrid_grid_;
  }
  const HybridGrid& low_resolution_hybrid_grid() const {
    return *low_resolution_hybrid_grid_;
  }

  // Insert 'range_data' into this submap using 'range_data_inserter'. The
  // submap must not be finished yet.
  void InsertRangeData(const sensor::RangeData& range_data,
                       const RangeDataInserter3D& range_data_inserter,
                       int high_resolution_max_range);
  void Finish();

 private:
  std::unique_ptr<HybridGrid> high_resolution_hybrid_grid_;
  std::unique_ptr<HybridGrid> low_resolution_hybrid_grid_;
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
class ActiveSubmaps3D {
 public:
  explicit ActiveSubmaps3D(const proto::SubmapsOptions3D& options);

  ActiveSubmaps3D(const ActiveSubmaps3D&) = delete;
  ActiveSubmaps3D& operator=(const ActiveSubmaps3D&) = delete;

  // Returns the index of the newest initialized Submap which can be
  // used for scan-to-map matching.
  int matching_index() const;

  // Inserts 'range_data' into the Submap collection. 'gravity_alignment' is
  // used for the orientation of new submaps so that the z axis approximately
  // aligns with gravity.
  void InsertRangeData(const sensor::RangeData& range_data,
                       const Eigen::Quaterniond& gravity_alignment);

  std::vector<std::shared_ptr<Submap3D>> submaps() const;

 private:
  void AddSubmap(const transform::Rigid3d& local_submap_pose);

  const proto::SubmapsOptions3D options_;
  int matching_submap_index_ = 0;
  std::vector<std::shared_ptr<Submap3D>> submaps_;
  RangeDataInserter3D range_data_inserter_;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_3D_SUBMAP_3D_H_
