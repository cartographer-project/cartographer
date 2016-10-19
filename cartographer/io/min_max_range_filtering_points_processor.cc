#include "cartographer/io/min_max_range_filtering_points_processor.h"

#include "cartographer/io/points_batch.h"

namespace cartographer {
namespace io {

MinMaxRangeFiteringPointsProcessor::MinMaxRangeFiteringPointsProcessor(
    const double min_range, const double max_range, PointsProcessor* next)
    : min_range_(min_range), max_range_(max_range), next_(next) {}

void MinMaxRangeFiteringPointsProcessor::Process(
    std::unique_ptr<PointsBatch> batch) {
  std::vector<int> to_remove;
  for (size_t i = 0; i < batch->points.size(); ++i) {
    const float range = (batch->points[i] - batch->origin).norm();
    if (!(min_range_ <= range && range <= max_range_)) {
      to_remove.push_back(i);
    }
  }
  RemovePoints(to_remove, batch.get());
  next_->Process(std::move(batch));
}

PointsProcessor::FlushResult MinMaxRangeFiteringPointsProcessor::Flush() {
  return next_->Flush();
}

}  // namespace io
}  // namespace cartographer
