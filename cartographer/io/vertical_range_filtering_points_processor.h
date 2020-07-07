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

#ifndef CARTOGRAPHER_IO_VERTICAL_RANGE_FILTERING_POINTS_PROCESSOR_H_
#define CARTOGRAPHER_IO_VERTICAL_RANGE_FILTERING_POINTS_PROCESSOR_H_

#include <memory>

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/io/points_processor.h"

namespace cartographer {
namespace io {

// Filters all points which distance in the Z direction from their 'origin'
// exceeds 'max_z' or 'min_z'.
class VerticalRangeFilteringPointsProcessor : public PointsProcessor {
 public:
  constexpr static const char* kConfigurationFileActionName =
      "vertical_range_filter";
  VerticalRangeFilteringPointsProcessor(double min_z, double max_z,
                                        PointsProcessor* next);
  static std::unique_ptr<VerticalRangeFilteringPointsProcessor> FromDictionary(
      common::LuaParameterDictionary* dictionary, PointsProcessor* next);

  ~VerticalRangeFilteringPointsProcessor() override {}

  VerticalRangeFilteringPointsProcessor(
      const VerticalRangeFilteringPointsProcessor&) = delete;
  VerticalRangeFilteringPointsProcessor& operator=(
      const VerticalRangeFilteringPointsProcessor&) = delete;

  void Process(std::unique_ptr<PointsBatch> batch) override;
  FlushResult Flush() override;

 private:
  const double min_z_;
  const double max_z_;
  PointsProcessor* const next_;
};

}  // namespace io
}  // namespace cartographer

#endif  // CARTOGRAPHER_IO_VERTICAL_RANGE_FILTERING_POINTS_PROCESSOR_H_
