/*
 * Copyright 2017 The Cartographer Authors
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

#include "cartographer/io/probability_grid_points_processor.h"

#include "Eigen/Core"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/make_unique.h"
#include "cartographer/common/math.h"
#include "cartographer/io/draw_trajectories.h"
#include "cartographer/io/image.h"
#include "cartographer/io/points_batch.h"
#include "cartographer/mapping/probability_values.h"

namespace cartographer {
namespace io {
namespace {

void DrawTrajectoriesIntoImage(
    const mapping::ProbabilityGrid& probability_grid,
    const Eigen::Array2i& offset,
    const std::vector<mapping::proto::Trajectory>& trajectories,
    cairo_surface_t* cairo_surface) {
  for (size_t i = 0; i < trajectories.size(); ++i) {
    DrawTrajectory(trajectories[i], GetColor(i),
                   [&probability_grid,
                    &offset](const transform::Rigid3d& pose) -> Eigen::Array2i {
                     return probability_grid.limits().GetCellIndex(
                                pose.cast<float>().translation().head<2>()) -
                            offset;
                   },
                   cairo_surface);
  }
}

uint8 ProbabilityToColor(float probability_from_grid) {
  const float probability = 1.f - probability_from_grid;
  return ::cartographer::common::RoundToInt(
      255 * ((probability - mapping::kMinProbability) /
             (mapping::kMaxProbability - mapping::kMinProbability)));
}

}  // namespace

ProbabilityGridPointsProcessor::ProbabilityGridPointsProcessor(
    const double resolution,
    const mapping::proto::ProbabilityGridRangeDataInserterOptions2D&
        probability_grid_range_data_inserter_options,
    const DrawTrajectories& draw_trajectories,
    std::unique_ptr<FileWriter> file_writer,
    const std::vector<mapping::proto::Trajectory>& trajectories,
    PointsProcessor* const next)
    : draw_trajectories_(draw_trajectories),
      trajectories_(trajectories),
      file_writer_(std::move(file_writer)),
      next_(next),
      range_data_inserter_(probability_grid_range_data_inserter_options),
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
      mapping::CreateProbabilityGridRangeDataInserterOptions2D(
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
  Eigen::Array2i offset;
  std::unique_ptr<Image> image =
      DrawProbabilityGrid(probability_grid_, &offset);
  if (image != nullptr) {
    if (draw_trajectories_ ==
        ProbabilityGridPointsProcessor::DrawTrajectories::kYes) {
      DrawTrajectoriesIntoImage(probability_grid_, offset, trajectories_,
                                image->GetCairoSurface().get());
    }
    image->WritePng(file_writer_.get());
    CHECK(file_writer_->Close());
  }

  switch (next_->Flush()) {
    case FlushResult::kRestartStream:
      LOG(FATAL) << "ProbabilityGrid generation must be configured to occur "
                    "after any stages that require multiple passes.";

    case FlushResult::kFinished:
      return FlushResult::kFinished;
  }
  LOG(FATAL);
  // The following unreachable return statement is needed to avoid a GCC bug
  // described at https://gcc.gnu.org/bugzilla/show_bug.cgi?id=81508
  return FlushResult::kFinished;
}

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
