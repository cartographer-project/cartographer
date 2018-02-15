/*
 * Copyright 2018 The Cartographer Authors
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

#include "cartographer/metrics/histogram.h"

#include "glog/logging.h"

namespace cartographer {
namespace metrics {

namespace {

// Implementation of histogram that does nothing.
class NullHistogram : public Histogram {
 public:
  void Observe(double) override {}
};

}  // namespace

Histogram* Histogram::Null() {
  static NullHistogram null_histogram;
  return &null_histogram;
}

Histogram::BucketBoundaries Histogram::FixedWidth(double width,
                                                  int num_finite_buckets) {
  BucketBoundaries result;
  double boundary = 0;
  for (int i = 0; i < num_finite_buckets; ++i) {
    boundary += width;
    result.push_back(boundary);
  }
  return result;
}

Histogram::BucketBoundaries Histogram::ScaledPowersOf(double base,
                                                      double scale_factor,
                                                      double max_value) {
  CHECK_GT(base, 1);
  CHECK_GT(scale_factor, 0);
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
