#ifndef CARTOGRAPHER_IO_PROBABILITY_GRID_POINTS_PROCESSOR_H_
#define CARTOGRAPHER_IO_PROBABILITY_GRID_POINTS_PROCESSOR_H_

#include <memory>
#include <string>

#include "cartographer/io/file_writer.h"
#include "cartographer/io/points_batch.h"
#include "cartographer/io/points_processor.h"
#include "cartographer/mapping_2d/probability_grid.h"
#include "cartographer/mapping_2d/proto/range_data_inserter_options.pb.h"
#include "cartographer/mapping_2d/range_data_inserter.h"

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
  ProbabilityGridPointsProcessor(
      double resolution,
      const mapping_2d::proto::RangeDataInserterOptions&
          range_data_inserter_options,
      std::unique_ptr<FileWriter> file_writer, PointsProcessor* next);
  ProbabilityGridPointsProcessor(const ProbabilityGridPointsProcessor&) =
      delete;
  ProbabilityGridPointsProcessor& operator=(
      const ProbabilityGridPointsProcessor&) = delete;

  static std::unique_ptr<ProbabilityGridPointsProcessor> FromDictionary(
      FileWriterFactory file_writer_factory,
      common::LuaParameterDictionary* dictionary, PointsProcessor* next);

  ~ProbabilityGridPointsProcessor() override {}

  void Process(std::unique_ptr<PointsBatch> batch) override;
  FlushResult Flush() override;

 private:
  PointsProcessor* const next_;
  std::unique_ptr<FileWriter> file_writer_;
  mapping_2d::RangeDataInserter range_data_inserter_;
  mapping_2d::ProbabilityGrid probability_grid_;
};

}  // namespace io
}  // namespace cartographer

#endif  // CARTOGRAPHER_IO_PROBABILITY_GRID_POINTS_PROCESSOR_H_
