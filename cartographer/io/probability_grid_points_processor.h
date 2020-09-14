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

#ifndef CARTOGRAPHER_IO_PROBABILITY_GRID_POINTS_PROCESSOR_H_
#define CARTOGRAPHER_IO_PROBABILITY_GRID_POINTS_PROCESSOR_H_

#include <memory>

#include "cartographer/io/file_writer.h"
#include "cartographer/io/image.h"
#include "cartographer/io/points_batch.h"
#include "cartographer/io/points_processor.h"
#include "cartographer/mapping/2d/probability_grid.h"
#include "cartographer/mapping/2d/probability_grid_range_data_inserter_2d.h"
#include "cartographer/mapping/proto/probability_grid_range_data_inserter_options_2d.pb.h"
#include "cartographer/mapping/proto/trajectory.pb.h"
#include "cartographer/mapping/value_conversion_tables.h"

namespace cartographer {
namespace io {

// Creates a probability grid with the specified 'resolution'. As all points are
// projected into the x-y plane the z component of the data is ignored.
// 'range_data_inserter' options are used to configure the range data ray
// tracing through the probability grid.
class ProbabilityGridPointsProcessor : public PointsProcessor {
 public:
  constexpr static const char* kConfigurationFileActionName =
      "write_probability_grid";
  enum class DrawTrajectories { kNo, kYes };
  enum class OutputType { kPng, kPb };
  ProbabilityGridPointsProcessor(
      double resolution,
      const mapping::proto::ProbabilityGridRangeDataInserterOptions2D&
          probability_grid_range_data_inserter_options,
      const DrawTrajectories& draw_trajectories, const OutputType& output_type,
      std::unique_ptr<FileWriter> file_writer,
      const std::vector<mapping::proto::Trajectory>& trajectories,
      PointsProcessor* next);
  ProbabilityGridPointsProcessor(const ProbabilityGridPointsProcessor&) =
      delete;
  ProbabilityGridPointsProcessor& operator=(
      const ProbabilityGridPointsProcessor&) = delete;

  static std::unique_ptr<ProbabilityGridPointsProcessor> FromDictionary(
      const std::vector<mapping::proto::Trajectory>& trajectories,
      const FileWriterFactory& file_writer_factory,
      common::LuaParameterDictionary* dictionary, PointsProcessor* next);

  ~ProbabilityGridPointsProcessor() override {}

  void Process(std::unique_ptr<PointsBatch> batch) override;
  FlushResult Flush() override;

 private:
  const DrawTrajectories draw_trajectories_;
  const OutputType output_type_;
  const std::vector<mapping::proto::Trajectory> trajectories_;
  std::unique_ptr<FileWriter> file_writer_;
  PointsProcessor* const next_;
  mapping::ProbabilityGridRangeDataInserter2D range_data_inserter_;
  mapping::ValueConversionTables conversion_tables_;
  mapping::ProbabilityGrid probability_grid_;
};

// Draws 'probability_grid' into an image and fills in 'offset' with the cropped
// map limits. Returns 'nullptr' if probability_grid was empty.
std::unique_ptr<Image> DrawProbabilityGrid(
    const mapping::ProbabilityGrid& probability_grid, Eigen::Array2i* offset);

// Create an initially empty probability grid with 'resolution' and a small
// size, suitable for a PointsBatchProcessor.
mapping::ProbabilityGrid CreateProbabilityGrid(
    const double resolution, mapping::ValueConversionTables* conversion_tables);

}  // namespace io
}  // namespace cartographer

#endif  // CARTOGRAPHER_IO_PROBABILITY_GRID_POINTS_PROCESSOR_H_
