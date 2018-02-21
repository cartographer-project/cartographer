#include "cartographer/mapping_2d/probability_grid.h"

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
    : limits_(limits),
      cells_(
          limits_.cell_limits().num_x_cells * limits_.cell_limits().num_y_cells,
          kUnknownProbabilityValue) {}

ProbabilityGrid::ProbabilityGrid(const proto::ProbabilityGrid& proto)
    : limits_(proto.limits()), cells_() {
  if (proto.has_known_cells_box()) {
    const auto& box = proto.known_cells_box();
    known_cells_box_ =
        Eigen::AlignedBox2i(Eigen::Vector2i(box.min_x(), box.min_y()),
                            Eigen::Vector2i(box.max_x(), box.max_y()));
  }
  cells_.reserve(proto.cells_size());
  for (const auto& cell : proto.cells()) {
    CHECK_LE(cell, std::numeric_limits<uint16>::max());
    cells_.push_back(cell);
  }
}

// Finishes the update sequence.
void ProbabilityGrid::FinishUpdate() {
  while (!update_indices_.empty()) {
    DCHECK_GE(cells_[update_indices_.back()], kUpdateMarker);
    cells_[update_indices_.back()] -= kUpdateMarker;
    update_indices_.pop_back();
  }
}

// Sets the probability of the cell at 'cell_index' to the given
// 'probability'. Only allowed if the cell was unknown before.
void ProbabilityGrid::SetProbability(const Eigen::Array2i& cell_index,
                                     const float probability) {
  uint16& cell = cells_[ToFlatIndex(cell_index, limits_)];
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
  uint16* cell = &cells_[flat_index];
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
    return ValueToProbability(cells_[ToFlatIndex(cell_index, limits_)]);
  }
  return kMinProbability;
}

// Returns true if the probability at the specified index is known.
bool ProbabilityGrid::IsKnown(const Eigen::Array2i& cell_index) const {
  return limits_.Contains(cell_index) &&
         cells_[ToFlatIndex(cell_index, limits_)] != kUnknownProbabilityValue;
}

// Fills in 'offset' and 'limits' to define a subregion of that contains all
// known cells.
void ProbabilityGrid::ComputeCroppedLimits(Eigen::Array2i* const offset,
                                           CellLimits* const limits) const {
  if (known_cells_box_.isEmpty()) {
    *offset = Eigen::Array2i::Zero();
    *limits = CellLimits(1, 1);
    return;
  }
  *offset = known_cells_box_.min().array();
  *limits = CellLimits(known_cells_box_.sizes().x() + 1,
                       known_cells_box_.sizes().y() + 1);
}

// Grows the map as necessary to include 'point'. This changes the meaning of
// these coordinates going forward. This method must be called immediately
// after 'FinishUpdate', before any calls to 'ApplyLookupTable'.
void ProbabilityGrid::GrowLimits(const Eigen::Vector2f& point) {
  CHECK(update_indices_.empty());
  while (!limits_.Contains(limits_.GetCellIndex(point))) {
    const int x_offset = limits_.cell_limits().num_x_cells / 2;
    const int y_offset = limits_.cell_limits().num_y_cells / 2;
    const MapLimits new_limits(
        limits_.resolution(),
        limits_.max() +
            limits_.resolution() * Eigen::Vector2d(y_offset, x_offset),
        CellLimits(2 * limits_.cell_limits().num_x_cells,
                   2 * limits_.cell_limits().num_y_cells));
    const int stride = new_limits.cell_limits().num_x_cells;
    const int offset = x_offset + stride * y_offset;
    const int new_size = new_limits.cell_limits().num_x_cells *
                         new_limits.cell_limits().num_y_cells;
    std::vector<uint16> new_cells(new_size, kUnknownProbabilityValue);
    for (int i = 0; i < limits_.cell_limits().num_y_cells; ++i) {
      for (int j = 0; j < limits_.cell_limits().num_x_cells; ++j) {
        new_cells[offset + j + i * stride] =
            cells_[j + i * limits_.cell_limits().num_x_cells];
      }
    }
    cells_ = new_cells;
    limits_ = new_limits;
    if (!known_cells_box_.isEmpty()) {
      known_cells_box_.translate(Eigen::Vector2i(x_offset, y_offset));
    }
  }
}

proto::ProbabilityGrid ProbabilityGrid::ToProto() const {
  proto::ProbabilityGrid result;
  *result.mutable_limits() = mapping::ToProto(limits_);
  result.mutable_cells()->Reserve(cells_.size());
  for (const auto& cell : cells_) {
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
  return result;
}

}  // namespace mapping
}  // namespace cartographer
