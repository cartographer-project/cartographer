/*
 * Copyright 2017 The Cartographer Authors
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

#ifndef CARTOGRAPHER_IO_FRAME_ID_FILTERING_POINTS_PROCESSOR_H_
#define CARTOGRAPHER_IO_FRAME_ID_FILTERING_POINTS_PROCESSOR_H_

#include <unordered_set>

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/io/points_processor.h"

namespace cartographer {
namespace io {

// Filters all points with blacklisted frame id or a non-whitelisted frame id.
// Note that you can either specify the whitelist or the blacklist, but not both
// at the same time.
class FrameIdFilteringPointsProcessor : public PointsProcessor {
 public:
  constexpr static const char* kConfigurationFileActionName = "frame_id_filter";
  FrameIdFilteringPointsProcessor(
      const std::unordered_set<string>& keep_frame_ids,
      const std::unordered_set<string>& drop_frame_ids, PointsProcessor* next);
  static std::unique_ptr<FrameIdFilteringPointsProcessor> FromDictionary(
      common::LuaParameterDictionary* dictionary, PointsProcessor* next);
  ~FrameIdFilteringPointsProcessor() override {}

  FrameIdFilteringPointsProcessor(const FrameIdFilteringPointsProcessor&) =
      delete;
  FrameIdFilteringPointsProcessor& operator=(
      const FrameIdFilteringPointsProcessor&) = delete;

  void Process(std::unique_ptr<PointsBatch> batch) override;
  FlushResult Flush() override;

 private:
  const std::unordered_set<string> keep_frame_ids_;
  const std::unordered_set<string> drop_frame_ids_;
  PointsProcessor* const next_;
};

}  // namespace io
}  // namespace cartographer

#endif  // CARTOGRAPHER_IO_FRAME_ID_FILTERING_POINTS_PROCESSOR_H_
