#include "cartographer/io/probability_grid_utils.h"

#include "cartographer/common/make_unique.h"
#include "cartographer/mapping/probability_values.h"

namespace cartographer {
namespace io {
namespace {

uint8 ProbabilityToColor(float probability_from_grid) {
  const float probability = 1.f - probability_from_grid;
  return ::cartographer::common::RoundToInt(
      255 * ((probability - mapping::kMinProbability) /
             (mapping::kMaxProbability - mapping::kMinProbability)));
}

}  // namespace

std::unique_ptr<Image> DrawProbabilityGrid(
    const mapping::ProbabilityGrid& probability_grid, Eigen::Array2i* offset) {
  mapping::CellLimits cell_limits;
  probability_grid.ComputeCroppedLimits(offset, &cell_limits);
  if (cell_limits.num_x_cells == 0 || cell_limits.num_y_cells == 0) {
    LOG(WARNING) << "Not writing output: empty probability grid";
    return nullptr;
  }
  auto image = common::make_unique<Image>(cell_limits.num_x_cells,
                                          cell_limits.num_y_cells);
  for (const Eigen::Array2i& xy_index :
       mapping::XYIndexRangeIterator(cell_limits)) {
    const Eigen::Array2i index = xy_index + *offset;
    constexpr uint8 kUnknownValue = 128;
    const uint8 value =
        probability_grid.IsKnown(index)
            ? ProbabilityToColor(probability_grid.GetProbability(index))
            : kUnknownValue;
    ;
    image->SetPixel(xy_index.x(), xy_index.y(), {{value, value, value}});
  }
  return image;
}

mapping::ProbabilityGrid CreateProbabilityGrid(const double resolution) {
  constexpr int kInitialProbabilityGridSize = 100;
  Eigen::Vector2d max =
      0.5 * kInitialProbabilityGridSize * resolution * Eigen::Vector2d::Ones();
  return mapping::ProbabilityGrid(
      mapping::MapLimits(resolution, max,
                         mapping::CellLimits(kInitialProbabilityGridSize,
                                             kInitialProbabilityGridSize)));
}

}  // namespace io
}  // namespace cartographer
