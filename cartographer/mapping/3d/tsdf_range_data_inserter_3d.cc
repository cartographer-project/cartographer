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

#include "cartographer/mapping/3d/tsdf_range_data_inserter_3d.h"

#include "Eigen/Core"
#include "cartographer/mapping/probability_values.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
namespace {
}  // namespace


TSDFRangeDataInserter3D::TSDFRangeDataInserter3D(
    const proto::RangeDataInserterOptions3D& options)
    : options_(options) {}

void TSDFRangeDataInserter3D::Insert(const sensor::RangeData& range_data,
                                              GridInterface* grid) const {
  CHECK(grid != nullptr);
  HybridGridTSDF* tsdf = static_cast<HybridGridTSDF*>(grid);
  LOG(FATAL)<<"TODO Implement";

  for (const sensor::RangefinderPoint& hit : range_data.returns) {
//    const Eigen::Array3i hit_cell = occupancy_grid->GetCellIndex(hit.position);
//    occupancy_grid->ApplyLookupTable(hit_cell, hit_table_);
  }

  tsdf->FinishUpdate();
}

}  // namespace mapping
}  // namespace cartographer
