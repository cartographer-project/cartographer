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

#include "cartographer/mapping/3d/range_data_inserter_3d.h"

#include "Eigen/Core"
#include "cartographer/mapping/probability_values.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
namespace {

void InsertMissesIntoGrid(const std::vector<uint16>& miss_table,
                          const Eigen::Vector3f& origin,
                          const sensor::PointCloud& returns,
                          HybridGrid* hybrid_grid,
                          const int num_free_space_voxels) {
  const Eigen::Array3i origin_cell = hybrid_grid->GetCellIndex(origin);
  for (const sensor::RangefinderPoint& hit : returns) {
    const Eigen::Array3i hit_cell = hybrid_grid->GetCellIndex(hit.position);

    const Eigen::Array3i delta = hit_cell - origin_cell;
    const int num_samples = delta.cwiseAbs().maxCoeff();
    CHECK_LT(num_samples, 1 << 15);
    // 'num_samples' is the number of samples we equi-distantly place on the
    // line between 'origin' and 'hit'. (including a fractional part for sub-
    // voxels) It is chosen so that between two samples we change from one voxel
    // to the next on the fastest changing dimension.
    //
    // Only the last 'num_free_space_voxels' are updated for performance.
    for (int position = std::max(0, num_samples - num_free_space_voxels);
         position < num_samples; ++position) {
      const Eigen::Array3i miss_cell =
          origin_cell + delta * position / num_samples;
      hybrid_grid->ApplyLookupTable(miss_cell, miss_table);
    }
  }
}

void InsertIntensitiesIntoGrid(const sensor::PointCloud& returns,
                               IntensityHybridGrid* intensity_hybrid_grid,
                               const float intensity_threshold) {
  if (returns.intensities().size() > 0) {
    for (size_t i = 0; i < returns.size(); ++i) {
      if (returns.intensities()[i] > intensity_threshold) {
        continue;
      }
      const Eigen::Array3i hit_cell =
          intensity_hybrid_grid->GetCellIndex(returns[i].position);
      intensity_hybrid_grid->AddIntensity(hit_cell, returns.intensities()[i]);
    }
  }
}

}  // namespace

proto::RangeDataInserterOptions3D CreateRangeDataInserterOptions3D(
    common::LuaParameterDictionary* parameter_dictionary) {
  proto::RangeDataInserterOptions3D options;
  options.set_hit_probability(
      parameter_dictionary->GetDouble("hit_probability"));
  options.set_miss_probability(
      parameter_dictionary->GetDouble("miss_probability"));
  options.set_num_free_space_voxels(
      parameter_dictionary->GetInt("num_free_space_voxels"));
  options.set_intensity_threshold(
      parameter_dictionary->GetDouble("intensity_threshold"));
  CHECK_GT(options.hit_probability(), 0.5);
  CHECK_LT(options.miss_probability(), 0.5);
  return options;
}

RangeDataInserter3D::RangeDataInserter3D(
    const proto::RangeDataInserterOptions3D& options)
    : options_(options),
      hit_table_(
          ComputeLookupTableToApplyOdds(Odds(options_.hit_probability()))),
      miss_table_(
          ComputeLookupTableToApplyOdds(Odds(options_.miss_probability()))) {}

void RangeDataInserter3D::Insert(
    const sensor::RangeData& range_data, HybridGrid* hybrid_grid,
    IntensityHybridGrid* intensity_hybrid_grid) const {
  CHECK_NOTNULL(hybrid_grid);

  for (const sensor::RangefinderPoint& hit : range_data.returns) {
    const Eigen::Array3i hit_cell = hybrid_grid->GetCellIndex(hit.position);
    hybrid_grid->ApplyLookupTable(hit_cell, hit_table_);
  }

  // By not starting a new update after hits are inserted, we give hits priority
  // (i.e. no hits will be ignored because of a miss in the same cell).
  InsertMissesIntoGrid(miss_table_, range_data.origin, range_data.returns,
                       hybrid_grid, options_.num_free_space_voxels());
  if (intensity_hybrid_grid != nullptr) {
    InsertIntensitiesIntoGrid(range_data.returns, intensity_hybrid_grid,
                              options_.intensity_threshold());
  }
  hybrid_grid->FinishUpdate();
}

}  // namespace mapping
}  // namespace cartographer
