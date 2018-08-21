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

#include "cartographer/io/coloring_points_processor.h"

#include "Eigen/Core"
#include "absl/memory/memory.h"
#include "glog/logging.h"

namespace cartographer {
namespace io {

std::unique_ptr<ColoringPointsProcessor>
ColoringPointsProcessor::FromDictionary(
    common::LuaParameterDictionary* const dictionary,
    PointsProcessor* const next) {
  const std::string frame_id = dictionary->GetString("frame_id");
  const std::vector<double> color_values =
      dictionary->GetDictionary("color")->GetArrayValuesAsDoubles();
  const Uint8Color color = {{static_cast<uint8>(color_values[0]),
                             static_cast<uint8>(color_values[1]),
                             static_cast<uint8>(color_values[2])}};
  return absl::make_unique<ColoringPointsProcessor>(ToFloatColor(color),
                                                    frame_id, next);
}

ColoringPointsProcessor::ColoringPointsProcessor(const FloatColor& color,
                                                 const std::string& frame_id,
                                                 PointsProcessor* const next)
    : color_(color), frame_id_(frame_id), next_(next) {}

void ColoringPointsProcessor::Process(std::unique_ptr<PointsBatch> batch) {
  if (batch->frame_id == frame_id_) {
    batch->colors.clear();
    for (size_t i = 0; i < batch->points.size(); ++i) {
      batch->colors.push_back(color_);
    }
  }
  next_->Process(std::move(batch));
}

PointsProcessor::FlushResult ColoringPointsProcessor::Flush() {
  return next_->Flush();
}

}  // namespace io
}  // namespace cartographer
