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
#include "cartographer/mapping/2d/probability_grid.h"

#include <limits>

#include "cartographer/mapping/probability_values.h"

namespace cartographer {
namespace mapping {
namespace {

// Converts a 'cell_index' into an index into 'cells_'.
int ToFlatIndex(const Eigen::Array2i& cell_index, const MapLimits& limits) {
  CHECK(limits.Contains(cell_index)) << cell_index;
  return limits.cell_limits().num_x_cells * cell_index.y() + cell_index.x();
}

}  // namespace

ProbabilityGrid::ProbabilityGrid(const MapLimits& limits) : Grid2D(limits) {}

ProbabilityGrid::ProbabilityGrid(const proto::Grid2D& proto)
    : Grid2D(MapLimits(proto.limits())) {
  CHECK(proto.has_probability_grid_2d());
  if (proto.has_known_cells_box()) {
    const auto& box = proto.known_cells_box();
    known_cells_box_ =
        Eigen::AlignedBox2i(Eigen::Vector2i(box.min_x(), box.min_y()),
                            Eigen::Vector2i(box.max_x(), box.max_y()));
  }
  correspondence_cost_cells_.reserve(proto.cells_size());
  for (const auto& cell : proto.cells()) {
    CHECK_LE(cell, std::numeric_limits<uint16>::max());
    correspondence_cost_cells_.push_back(cell);
  }
}

// Sets the probability of the cell at 'cell_index' to the given
// 'probability'. Only allowed if the cell was unknown before.
void ProbabilityGrid::SetProbability(const Eigen::Array2i& cell_index,
                                     const float probability) {
  uint16& cell = correspondence_cost_cells_[ToFlatIndex(cell_index, limits_)];
  CHECK_EQ(cell, kUnknownProbabilityValue);
  cell = ProbabilityToValue(probability);
  known_cells_box_.extend(cell_index.matrix());
}

// Applies the 'odds' specified when calling ComputeLookupTableToApplyOdds()
// to the probability of the cell at 'cell_index' if the cell has not already
// been updated. Multiple updates of the same cell will be ignored until
// FinishUpdate() is called. Returns true if the cell was updated.
//
// If this is the first call to ApplyOdds() for the specified cell, its value
// will be set to probability corresponding to 'odds'.
bool ProbabilityGrid::ApplyLookupTable(const Eigen::Array2i& cell_index,
                                       const std::vector<uint16>& table) {
  DCHECK_EQ(table.size(), kUpdateMarker);
  const int flat_index = ToFlatIndex(cell_index, limits_);
  uint16* cell = &correspondence_cost_cells_[flat_index];
  if (*cell >= kUpdateMarker) {
    return false;
  }
  update_indices_.push_back(flat_index);
  *cell = table[*cell];
  DCHECK_GE(*cell, kUpdateMarker);
  known_cells_box_.extend(cell_index.matrix());
  return true;
}

// Returns the probability of the cell with 'cell_index'.
float ProbabilityGrid::GetProbability(const Eigen::Array2i& cell_index) const {
  if (!limits_.Contains(cell_index)) return kMinProbability;
  return ValueToProbability(
      correspondence_cost_cells_[ToFlatIndex(cell_index, limits_)]);
}
proto::Grid2D ProbabilityGrid::ToProto() const {
  proto::Grid2D result;
  *result.mutable_limits() = mapping::ToProto(limits_);
  result.mutable_cells()->Reserve(correspondence_cost_cells_.size());
  for (const auto& cell : correspondence_cost_cells_) {
    result.mutable_cells()->Add(cell);
  }
  CHECK(update_indices_.empty()) << "Serializing a grid during an update is "
                                    "not supported. Finish the update first.";
  if (!known_cells_box_.isEmpty()) {
    auto* const box = result.mutable_known_cells_box();
    box->set_max_x(known_cells_box_.max().x());
    box->set_max_y(known_cells_box_.max().y());
    box->set_min_x(known_cells_box_.min().x());
    box->set_min_y(known_cells_box_.min().y());
  }
  result.mutable_probability_grid_2d();
  return result;
}

}  // namespace mapping
}  // namespace cartographer
