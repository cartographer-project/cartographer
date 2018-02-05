#include "cartographer/metrics/histogram.h"

namespace cartographer {
namespace metrics {

namespace {

// Implementation of histogram that does nothing.
class NullHistogram : public Histogram {
 public:
  void Observe(double value) override {}
};

}  // namespace

Histogram* Histogram::Null() {
  static NullHistogram nullHistogram;
  return &nullHistogram;
}

Histogram::BucketBoundaries Histogram::FixedWidth(double width,
                                                  int num_finite_buckets) {
  BucketBoundaries result;
  double boundary = 0;
  for (double i = 0; i < num_finite_buckets; ++i) {
    boundary += width;
    result.push_back(boundary);
  }
  return result;
}

Histogram::BucketBoundaries Histogram::ScaledPowersOf(double base,
                                                      double scale_factor,
                                                      double max_value) {
  BucketBoundaries result;
  double boundary = scale_factor;
  while (boundary < max_value) {
    result.push_back(boundary);
    boundary *= base;
  }
  return result;
}

}  // namespace metrics
}  // namespace cartographer
