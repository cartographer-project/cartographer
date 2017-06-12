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

struct Submap : public mapping::Submap {
  Submap(float high_resolution, float low_resolution,
         const transform::Rigid3d& local_pose);

  HybridGrid high_resolution_hybrid_grid;
  HybridGrid low_resolution_hybrid_grid;
  bool finished = false;

  void ToResponseProto(
      const transform::Rigid3d& global_submap_pose,
      mapping::proto::SubmapQuery::Response* response) const override;
};

// A container of Submaps.
class Submaps : public mapping::Submaps {
 public:
  explicit Submaps(const proto::SubmapsOptions& options);

  Submaps(const Submaps&) = delete;
  Submaps& operator=(const Submaps&) = delete;

  const Submap* Get(int index) const override;
  int size() const override;

  // Inserts 'range_data' into the Submap collection. 'gravity_alignment' is
  // used for the orientation of new submaps so that the z axis approximately
  // aligns with gravity.
  void InsertRangeData(const sensor::RangeData& range_data,
                       const Eigen::Quaterniond& gravity_alignment);

 private:
  void AddSubmap(const transform::Rigid3d& local_pose);

  const proto::SubmapsOptions options_;

  // 'submaps_' contains pointers, so that resizing the vector does not
  // invalidate handed out Submap* pointers.
  std::vector<std::unique_ptr<Submap>> submaps_;
  RangeDataInserter range_data_inserter_;
};

}  // namespace mapping_3d
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_3D_SUBMAPS_H_
