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

#include "cartographer/io/frame_id_filtering_points_processor.h"

#include "absl/memory/memory.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/io/points_batch.h"
#include "glog/logging.h"

namespace cartographer {
namespace io {

std::unique_ptr<FrameIdFilteringPointsProcessor>
FrameIdFilteringPointsProcessor::FromDictionary(
    common::LuaParameterDictionary* dictionary, PointsProcessor* next) {
  std::vector<std::string> keep_frames, drop_frames;
  if (dictionary->HasKey("keep_frames")) {
    keep_frames =
        dictionary->GetDictionary("keep_frames")->GetArrayValuesAsStrings();
  }
  if (dictionary->HasKey("drop_frames")) {
    drop_frames =
        dictionary->GetDictionary("drop_frames")->GetArrayValuesAsStrings();
  }
  return absl::make_unique<FrameIdFilteringPointsProcessor>(
      absl::flat_hash_set<std::string>(keep_frames.begin(), keep_frames.end()),
      absl::flat_hash_set<std::string>(drop_frames.begin(), drop_frames.end()),
      next);
}

FrameIdFilteringPointsProcessor::FrameIdFilteringPointsProcessor(
    const absl::flat_hash_set<std::string>& keep_frame_ids,
    const absl::flat_hash_set<std::string>& drop_frame_ids,
    PointsProcessor* next)
    : keep_frame_ids_(keep_frame_ids),
      drop_frame_ids_(drop_frame_ids),
      next_(next) {
  CHECK_NE(keep_frame_ids.empty(), drop_frame_ids.empty())
      << "You have to specify exactly one of the `keep_frames` property or the "
      << "`drop_frames` property, but not both at the same time.";
}

void FrameIdFilteringPointsProcessor::Process(
    std::unique_ptr<PointsBatch> batch) {
  if ((!keep_frame_ids_.empty() && keep_frame_ids_.count(batch->frame_id)) ||
      (!drop_frame_ids_.empty() && !drop_frame_ids_.count(batch->frame_id))) {
    next_->Process(std::move(batch));
  }
}

PointsProcessor::FlushResult FrameIdFilteringPointsProcessor::Flush() {
  return next_->Flush();
}

}  // namespace io
}  // namespace cartographer
