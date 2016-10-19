#ifndef CARTOGRAPHER_IO_MIN_MAX_RANGE_FILTERING_POINTS_PROCESSOR_H_
#define CARTOGRAPHER_IO_MIN_MAX_RANGE_FILTERING_POINTS_PROCESSOR_H_

#include <memory>

#include "cartographer/io/points_processor.h"

namespace cartographer {
namespace io {

// Filters all points that are farther away from their 'origin' as 'max_range'
// or closer than 'min_range'.
class MinMaxRangeFiteringPointsProcessor : public PointsProcessor {
 public:
  MinMaxRangeFiteringPointsProcessor(double min_range, double max_range,
                                     PointsProcessor* next);
  virtual ~MinMaxRangeFiteringPointsProcessor() {}

  MinMaxRangeFiteringPointsProcessor(const MinMaxRangeFiteringPointsProcessor&) = delete;
  MinMaxRangeFiteringPointsProcessor& operator=(const MinMaxRangeFiteringPointsProcessor&) = delete;

  void Process(std::unique_ptr<PointsBatch> batch) override;
  FlushResult Flush() override;

 private:
  const double min_range_;
  const double max_range_;
  PointsProcessor* const next_;
};

}  // namespace io
}  // namespace cartographer

#endif  // CARTOGRAPHER_IO_MIN_MAX_RANGE_FILTERING_POINTS_PROCESSOR_H_
