#ifndef CARTOGRAPHER_IO_HYBRID_GRID_POINTS_PROCESSOR_H_
#define CARTOGRAPHER_IO_HYBRID_GRID_POINTS_PROCESSOR_H_

// Library used for processing points processor pipeline.

#include <memory>
#include <string>

#include "cartographer/io/file_writer.h"
#include "cartographer/io/points_batch.h"
#include "cartographer/io/points_processor.h"
#include "cartographer/mapping_3d/hybrid_grid.h"
#include "cartographer/mapping_3d/range_data_inserter.h"

namespace cartographer {
namespace io {

// Creates a geolocated hybrid grid of the points with voxels being
// 'voxel_size' big.
class HybridGridPointsProcessor : public PointsProcessor {
 public:
  constexpr static const char* kConfigurationFileActionName =
      "write_geolocated_hybrid_grid";
  HybridGridPointsProcessor(
      double voxel_size,
      std::unique_ptr<mapping_3d::RangeDataInserter>
      range_data_inserter,
      const string& output_filename,
      io::FileWriterFactory file_writer_factory,
      io::PointsProcessor* next);

  static std::unique_ptr<HybridGridPointsProcessor> FromDictionary(
      io::FileWriterFactory file_writer_factory,
      common::LuaParameterDictionary* dictionary,
      io::PointsProcessor* next);

  ~HybridGridPointsProcessor() override = default;

  void Process(std::unique_ptr<PointsBatch> batch) override;
  FlushResult Flush() override;

 private:
  std::unique_ptr<mapping_3d::RangeDataInserter> range_data_inserter_;
  std::unique_ptr<mapping_3d::HybridGrid> hybrid_grid_;
  const string output_filename_;
  FileWriterFactory file_writer_factory_;
  PointsProcessor* const next_;

};

}  // namespace io
}  // namespace cartographer

#endif  // CARTOGRAPHER_IO_HYBRID_GRID_POINTS_PROCESSOR_H_
