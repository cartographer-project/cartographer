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

ProbabilityGrid::ProbabilityGrid(const MapLimits& limits)
    : Grid2D(limits) {}

ProbabilityGrid::ProbabilityGrid(const proto::Grid2D& proto)
    : Grid2D(proto) {}

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
  if (limits_.Contains(cell_index)) {
    return ValueToProbability(correspondence_cost_cells_[ToFlatIndex(cell_index, limits_)]);
  }
  return kMinProbability;
}

}  // namespace mapping
}  // namespace cartographer
