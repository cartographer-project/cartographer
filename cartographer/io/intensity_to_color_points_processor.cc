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

#include "cartographer/io/intensity_to_color_points_processor.h"

#include "Eigen/Core"
#include "absl/memory/memory.h"
#include "cartographer/common/math.h"
#include "glog/logging.h"

namespace cartographer {
namespace io {

std::unique_ptr<IntensityToColorPointsProcessor>
IntensityToColorPointsProcessor::FromDictionary(
    common::LuaParameterDictionary* const dictionary,
    PointsProcessor* const next) {
  const std::string frame_id =
      dictionary->HasKey("frame_id") ? dictionary->GetString("frame_id") : "";
  const float min_intensity = dictionary->GetDouble("min_intensity");
  const float max_intensity = dictionary->GetDouble("max_intensity");
  return absl::make_unique<IntensityToColorPointsProcessor>(
      min_intensity, max_intensity, frame_id, next);
}

IntensityToColorPointsProcessor::IntensityToColorPointsProcessor(
    const float min_intensity, const float max_intensity,
    const std::string& frame_id, PointsProcessor* const next)
    : min_intensity_(min_intensity),
      max_intensity_(max_intensity),
      frame_id_(frame_id),
      next_(next) {}

void IntensityToColorPointsProcessor::Process(
    std::unique_ptr<PointsBatch> batch) {
  if (!batch->intensities.empty() &&
      (frame_id_.empty() || batch->frame_id == frame_id_)) {
    batch->colors.clear();
    for (const float intensity : batch->intensities) {
      const float gray = cartographer::common::Clamp(
          (intensity - min_intensity_) / (max_intensity_ - min_intensity_), 0.f,
          1.f);
      batch->colors.push_back({{gray, gray, gray}});
    }
  }
  next_->Process(std::move(batch));
}

PointsProcessor::FlushResult IntensityToColorPointsProcessor::Flush() {
  return next_->Flush();
}

}  // namespace io
}  // namespace cartographer
