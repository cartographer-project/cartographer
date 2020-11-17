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

#ifndef CARTOGRAPHER_MAPPING_3D_HYBRID_GRID_H_
#define CARTOGRAPHER_MAPPING_3D_HYBRID_GRID_H_

#include <array>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include "Eigen/Core"
#include "absl/memory/memory.h"
#include "cartographer/common/math.h"
#include "cartographer/common/port.h"
#include "cartographer/mapping/3d/hybrid_grid_base.h"
#include "cartographer/mapping/grid_interface.h"
#include "cartographer/mapping/probability_values.h"
#include "cartographer/mapping/proto/3d/hybrid_grid.pb.h"
#include "cartographer/mapping/value_conversion_tables.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
// A grid containing probability values stored using 15 bits, and an update
// marker per voxel.
// Points are expected to be close to the origin. Points far from the origin
// require the grid to grow dynamically. For centimeter resolution, points
// can only be tens of meters from the origin.
// The hard limit of cell indexes is +/- 8192 around the origin.
class HybridGrid : public GridInterface, public HybridGridBase<uint16> {
 public:
  explicit HybridGrid(const float resolution,
                      ValueConversionTables* conversion_tables)
      : HybridGridBase<uint16>(resolution),
        value_to_correspondence_cost_table_(
            conversion_tables->GetConversionTable(kMaxCorrespondenceCost,
                                                  kMinCorrespondenceCost,
                                                  kMaxCorrespondenceCost)) {}

  virtual GridType GetGridType() const override {
    return GridType::PROBABILITY_GRID;
  };


  explicit HybridGrid(const proto::HybridGrid& proto,
                         ValueConversionTables* conversion_tables)
      : HybridGrid(proto.resolution(), conversion_tables) {
    CHECK_EQ(proto.values_size(), proto.x_indices_size());
    CHECK_EQ(proto.values_size(), proto.y_indices_size());
    CHECK_EQ(proto.values_size(), proto.z_indices_size());
    for (int i = 0; i < proto.values_size(); ++i) {
      // SetProbability does some error checking for us.
      SetProbability(Eigen::Vector3i(proto.x_indices(i), proto.y_indices(i),
                                     proto.z_indices(i)),
                     ValueToProbability(proto.values(i)));
    }
  }

  // Finishes the update sequence.
  void FinishUpdate() {
    while (!update_indices_.empty()) {
      DCHECK_GE(*update_indices_.back(), kUpdateMarker);
      *update_indices_.back() -= kUpdateMarker;
      update_indices_.pop_back();
    }
  }

  // Returns true if the probability at the specified 'index' is known.
  bool IsKnown(const Eigen::Array3i& index) const { return value(index) != 0; }

  proto::HybridGrid ToProto() const {
    CHECK(update_indices_.empty()) << "Serializing a grid during an update is "
                                      "not supported. Finish the update first.";
    proto::HybridGrid result;
    result.set_resolution(resolution());
    for (const auto it : *this) {
      result.add_x_indices(it.first.x());
      result.add_y_indices(it.first.y());
      result.add_z_indices(it.first.z());
      result.add_values(it.second);
    }
    return result;
  }

  std::vector<ValueType*>* UpdateIndices() { return &update_indices_; }

  float GetCorrespondenceCost(const Eigen::Array3i& index) const {
    return (*value_to_correspondence_cost_table_)[value(index)];
  }

  // Sets the probability of the cell at 'index' to the given 'probability'.
  void SetProbability(const Eigen::Array3i& index, const float probability) {
    *mutable_value(index) = ProbabilityToValue(probability);
  }

  // Applies the 'odds' specified when calling ComputeLookupTableToApplyOdds()
  // to the probability of the cell at 'index' if the cell has not already been
  // updated. Multiple updates of the same cell will be ignored until
  // FinishUpdate() is called. Returns true if the cell was updated.
  //
  // If this is the first call to ApplyOdds() for the specified cell, its value
  // will be set to probability corresponding to 'odds'.
  bool ApplyLookupTable(const Eigen::Array3i& index,
                        const std::vector<uint16>& table) {
    DCHECK_EQ(table.size(), kUpdateMarker);
    uint16* const cell = mutable_value(index);
    if (*cell >= kUpdateMarker) {
      return false;
    }
    UpdateIndices()->push_back(cell);
    *cell = table[*cell];
    DCHECK_GE(*cell, kUpdateMarker);
    return true;
  }

  // Returns the probability of the cell with 'index'.
  float GetProbability(const Eigen::Array3i& index) const {
    return ValueToProbability(value(index));
  }

 private:
  // Markers at changed cells.
  std::vector<ValueType*> update_indices_;
  const std::vector<float>* value_to_correspondence_cost_table_;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_3D_HYBRID_GRID_H_
