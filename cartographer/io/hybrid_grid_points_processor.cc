#include "cartographer/io/hybrid_grid_points_processor.h"

#include <memory>
#include <string>
#include <unordered_set>

#include "Eigen/Core"
#include "cartographer/common/make_unique.h"
#include "cartographer/io/file_writer.h"
#include "cartographer/io/points_batch.h"
#include "cartographer/io/points_processor.h"
#include "cartographer/mapping_3d/hybrid_grid.h"
#include "cartographer/mapping_3d/range_data_inserter.h"
#include "cartographer/sensor/range_data.h"
#include "glog/logging.h"

namespace cartographer {
namespace io {

namespace {

std::unique_ptr<sensor::RangeData> RangeDataFromPointsBatch(
    const PointsBatch& points_batch) {
  auto range_data = common::make_unique<sensor::RangeData>();
  range_data->origin = points_batch.origin;
  for (const Eigen::Vector3f& point : points_batch.points) {
    range_data->returns.push_back(point);
  }
  return range_data;
}

}  // namespace

HybridGridPointsProcessor::HybridGridPointsProcessor(
    const double voxel_size,
    const mapping_3d::proto::RangeDataInserterOptions&
        range_data_inserter_options,
    std::unique_ptr<FileWriter> file_writer, PointsProcessor* const next)
    : range_data_inserter_(range_data_inserter_options),
      file_writer_(std::move(file_writer)),
      next_(next) {
  hybrid_grid_ = common::make_unique<mapping_3d::HybridGrid>(voxel_size);
}

std::unique_ptr<HybridGridPointsProcessor>
HybridGridPointsProcessor::FromDictionary(
    FileWriterFactory file_writer_factory,
    common::LuaParameterDictionary* const dictionary,
    PointsProcessor* const next) {
  return common::make_unique<HybridGridPointsProcessor>(
      dictionary->GetDouble("voxel_size"),
      mapping_3d::CreateRangeDataInserterOptions(
          dictionary->GetDictionary("range_data_inserter").get()),
      file_writer_factory(dictionary->GetString("filename")), next);
}

void HybridGridPointsProcessor::Process(std::unique_ptr<PointsBatch> batch) {
  std::unique_ptr<sensor::RangeData> range_data_local =
      RangeDataFromPointsBatch(*batch);
  range_data_inserter_.Insert(*range_data_local, hybrid_grid_.get());

  next_->Process(std::move(batch));
}

PointsProcessor::FlushResult HybridGridPointsProcessor::Flush() {
  mapping_3d::proto::HybridGrid hybrid_grid_proto =
      mapping_3d::ToProto(*hybrid_grid_);
  string serialized;
  hybrid_grid_proto.SerializeToString(&serialized);
  file_writer_->Write(serialized.data(), serialized.size());
  CHECK(file_writer_->Close());

  switch (next_->Flush()) {
    case FlushResult::kRestartStream:
      LOG(FATAL) << "Hybrid grid generation must be configured to occur after "
                    "any stages that require multiple passes.";

    case FlushResult::kFinished:
      return FlushResult::kFinished;
  }
  LOG(FATAL) << "Failed to receive FlushResult::kFinished";
}

}  // namespace io
}  // namespace cartographer
