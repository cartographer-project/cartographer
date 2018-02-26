#include "cartographer/io/hybrid_grid_points_processor.h"

#include <memory>
#include <string>

#include "Eigen/Core"
#include "cartographer/common/make_unique.h"
#include "cartographer/io/file_writer.h"
#include "cartographer/io/points_batch.h"
#include "cartographer/io/points_processor.h"
#include "cartographer/mapping/3d/hybrid_grid.h"
#include "cartographer/mapping/3d/range_data_inserter_3d.h"
#include "cartographer/sensor/range_data.h"
#include "glog/logging.h"

namespace cartographer {
namespace io {

HybridGridPointsProcessor::HybridGridPointsProcessor(
    const double voxel_size,
    const mapping::proto::RangeDataInserterOptions3D&
        range_data_inserter_options,
    std::unique_ptr<FileWriter> file_writer, PointsProcessor* const next)
    : next_(next),
      range_data_inserter_(range_data_inserter_options),
      hybrid_grid_(voxel_size),
      file_writer_(std::move(file_writer)) {}

std::unique_ptr<HybridGridPointsProcessor>
HybridGridPointsProcessor::FromDictionary(
    const FileWriterFactory& file_writer_factory,
    common::LuaParameterDictionary* const dictionary,
    PointsProcessor* const next) {
  return common::make_unique<HybridGridPointsProcessor>(
      dictionary->GetDouble("voxel_size"),
      mapping::CreateRangeDataInserterOptions3D(
          dictionary->GetDictionary("range_data_inserter").get()),
      file_writer_factory(dictionary->GetString("filename")), next);
}

void HybridGridPointsProcessor::Process(std::unique_ptr<PointsBatch> batch) {
  range_data_inserter_.Insert({batch->origin, batch->points, {}},
                              &hybrid_grid_);
  next_->Process(std::move(batch));
}

PointsProcessor::FlushResult HybridGridPointsProcessor::Flush() {
  const mapping::proto::HybridGrid hybrid_grid_proto = hybrid_grid_.ToProto();
  std::string serialized;
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
  // The following unreachable return statement is needed to avoid a GCC bug
  // described at https://gcc.gnu.org/bugzilla/show_bug.cgi?id=81508
  return FlushResult::kFinished;
}

}  // namespace io
}  // namespace cartographer
