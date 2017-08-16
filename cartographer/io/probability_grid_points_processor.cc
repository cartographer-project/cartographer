#include "cartographer/io/probability_grid_points_processor.h"

#include "Eigen/Core"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/make_unique.h"
#include "cartographer/common/math.h"
#include "cartographer/io/draw_trajectories.h"
#include "cartographer/io/image.h"
#include "cartographer/io/points_batch.h"

namespace cartographer {
namespace io {
namespace {

void WriteGrid(
    const mapping_2d::ProbabilityGrid& probability_grid,
    const ProbabilityGridPointsProcessor::DrawTrajectories& draw_trajectories,
    const std::vector<mapping::proto::Trajectory>& trajectories,
    FileWriter* const file_writer) {
  Eigen::Array2i offset;
  mapping_2d::CellLimits cell_limits;
  probability_grid.ComputeCroppedLimits(&offset, &cell_limits);
  if (cell_limits.num_x_cells == 0 || cell_limits.num_y_cells == 0) {
    LOG(WARNING) << "Not writing output: empty probability grid";
    return;
  }
  const auto compute_color_value = [&probability_grid](
                                       const Eigen::Array2i& index) {
    if (probability_grid.IsKnown(index)) {
      const float probability = 1.f - probability_grid.GetProbability(index);
      return static_cast<uint8>(
          255 * ((probability - mapping::kMinProbability) /
                 (mapping::kMaxProbability - mapping::kMinProbability)));
    } else {
      constexpr uint8 kUnknownValue = 128;
      return kUnknownValue;
    }
  };
  Image image(cell_limits.num_x_cells, cell_limits.num_y_cells);
  for (const auto& xy_index :
       cartographer::mapping_2d::XYIndexRangeIterator(cell_limits)) {
    const auto index = xy_index + offset;
    uint8 value = compute_color_value(index);
    image.SetPixel(xy_index.x(), xy_index.y(), {{value, value, value}});
  }

  if (draw_trajectories ==
      ProbabilityGridPointsProcessor::DrawTrajectories::kYes) {
    for (size_t i = 0; i < trajectories.size(); ++i) {
      DrawTrajectory(trajectories[i], GetColor(i),
                     [&probability_grid, &offset](
                         const transform::Rigid3d& pose) -> Eigen::Array2i {
                       return probability_grid.limits().GetCellIndex(
                                  pose.cast<float>().translation().head<2>()) -
                              offset;
                     },
                     image.GetCairoSurface().get());
    }
  }
  image.WritePng(file_writer);
  CHECK(file_writer->Close());
}

mapping_2d::ProbabilityGrid CreateProbabilityGrid(const double resolution) {
  constexpr int kInitialProbabilityGridSize = 100;
  Eigen::Vector2d max =
      0.5 * kInitialProbabilityGridSize * resolution * Eigen::Vector2d::Ones();
  return mapping_2d::ProbabilityGrid(cartographer::mapping_2d::MapLimits(
      resolution, max,
      mapping_2d::CellLimits(kInitialProbabilityGridSize,
                             kInitialProbabilityGridSize)));
}

}  // namespace

ProbabilityGridPointsProcessor::ProbabilityGridPointsProcessor(
    const double resolution,
    const mapping_2d::proto::RangeDataInserterOptions&
        range_data_inserter_options,
    const DrawTrajectories& draw_trajectories,
    std::unique_ptr<FileWriter> file_writer,
    const std::vector<mapping::proto::Trajectory>& trajectories,
    PointsProcessor* const next)
    : draw_trajectories_(draw_trajectories),
      trajectories_(trajectories),
      file_writer_(std::move(file_writer)),
      next_(next),
      range_data_inserter_(range_data_inserter_options),
      probability_grid_(CreateProbabilityGrid(resolution)) {}

std::unique_ptr<ProbabilityGridPointsProcessor>
ProbabilityGridPointsProcessor::FromDictionary(
    const std::vector<mapping::proto::Trajectory>& trajectories,
    const FileWriterFactory& file_writer_factory,
    common::LuaParameterDictionary* const dictionary,
    PointsProcessor* const next) {
  const auto draw_trajectories = (!dictionary->HasKey("draw_trajectories") ||
                                  dictionary->GetBool("draw_trajectories"))
                                     ? DrawTrajectories::kYes
                                     : DrawTrajectories::kNo;
  return common::make_unique<ProbabilityGridPointsProcessor>(
      dictionary->GetDouble("resolution"),
      mapping_2d::CreateRangeDataInserterOptions(
          dictionary->GetDictionary("range_data_inserter").get()),
      draw_trajectories,
      file_writer_factory(dictionary->GetString("filename") + ".png"),
      trajectories, next);
}

void ProbabilityGridPointsProcessor::Process(
    std::unique_ptr<PointsBatch> batch) {
  range_data_inserter_.Insert({batch->origin, batch->points, {}},
                              &probability_grid_);
  next_->Process(std::move(batch));
}

PointsProcessor::FlushResult ProbabilityGridPointsProcessor::Flush() {
  WriteGrid(probability_grid_, draw_trajectories_, trajectories_,
            file_writer_.get());
  switch (next_->Flush()) {
    case FlushResult::kRestartStream:
      LOG(FATAL) << "ProbabilityGrid generation must be configured to occur "
                    "after any stages that require multiple passes.";

    case FlushResult::kFinished:
      return FlushResult::kFinished;
  }
  LOG(FATAL);
}

}  // namespace io
}  // namespace cartographer
