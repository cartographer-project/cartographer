#include "cartographer/io/probability_grid_points_processor.h"

#include <memory>
#include <string>

#include "Eigen/Core"
#include "cartographer/common/make_unique.h"
#include "cartographer/sensor/range_data.h"
#include "glog/logging.h"

namespace cartographer {
namespace io {
namespace {

static mapping_2d::ProbabilityGrid Create(
  double resolution) {
  double half_length = 200;
  const int num_cells_per_dimension =
    cartographer::common::RoundToInt(2. * half_length / resolution) + 1;
  Eigen::Vector2d max = half_length * Eigen::Vector2d::Ones();
  return mapping_2d::ProbabilityGrid(
    cartographer::mapping_2d::MapLimits(
      resolution, max,
      mapping_2d::CellLimits(num_cells_per_dimension, num_cells_per_dimension)));
}

} // namespace

ProbabilityGridPointsProcessor::ProbabilityGridPointsProcessor(
    const double resolution,
    const cartographer::mapping_2d::proto::RangeDataInserterOptions&
        range_data_inserter_options,
    std::unique_ptr<FileWriter> file_writer, PointsProcessor* const next)
    : next_(next),
      range_data_inserter_(range_data_inserter_options),
      probability_grid_(Create(resolution)),
      file_writer_(std::move(file_writer)) {}

std::unique_ptr<ProbabilityGridPointsProcessor>
ProbabilityGridPointsProcessor::FromDictionary(
    FileWriterFactory file_writer_factory,
    common::LuaParameterDictionary* const dictionary,
    PointsProcessor* const next) {
  return common::make_unique<ProbabilityGridPointsProcessor>(
      dictionary->GetDouble("resolution"),
      mapping_2d::CreateRangeDataInserterOptions(
          dictionary->GetDictionary("range_data_inserter").get()),
      file_writer_factory(dictionary->GetString("filename")), next);
}

void ProbabilityGridPointsProcessor::Process(std::unique_ptr<PointsBatch> batch) {
  range_data_inserter_.Insert({batch->origin, batch->points, {}},
                              &probability_grid_);
  next_->Process(std::move(batch));
}

PointsProcessor::FlushResult ProbabilityGridPointsProcessor::Flush() {
  const mapping_2d::proto::ProbabilityGrid probability_grid_proto =
    probability_grid_.ToProto();
  string serialized;
  probability_grid_proto.SerializeToString(&serialized);
  file_writer_->Write(serialized.data(), serialized.size());
  CHECK(file_writer_->Close());

  switch (next_->Flush()) {
    case FlushResult::kRestartStream:
      LOG(FATAL) << "Probability grid generation must be configured to occur after "
                    "any stages that require multiple passes.";

    case FlushResult::kFinished:
      return FlushResult::kFinished;
  }
  LOG(FATAL) << "Failed to receive FlushResult::kFinished";
}

}  // namespace io
}  // namespace cartographer
