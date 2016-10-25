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

#include "cartographer/io/counting_points_processor.h"

#include "cartographer/common/make_unique.h"
#include "glog/logging.h"

namespace cartographer {
namespace io {

CountingPointsProcessor::CountingPointsProcessor(PointsProcessor* next)
    : num_points_(0), next_(next) {}

std::unique_ptr<CountingPointsProcessor>
CountingPointsProcessor::FromDictionary(
    common::LuaParameterDictionary* const dictionary,
    PointsProcessor* const next) {
  return common::make_unique<CountingPointsProcessor>(next);
}

void CountingPointsProcessor::Process(std::unique_ptr<PointsBatch> batch) {
  num_points_ += batch->points.size();
  next_->Process(std::move(batch));
}

PointsProcessor::FlushResult CountingPointsProcessor::Flush() {
  switch (next_->Flush()) {
    case FlushResult::kFinished:
      LOG(INFO) << "Processed " << num_points_ << " and finishing.";
      return FlushResult::kFinished;

    case FlushResult::kRestartStream:
      LOG(INFO) << "Processed " << num_points_ << " and restarting stream.";
      num_points_ = 0;
      return FlushResult::kRestartStream;
  }
  LOG(FATAL);
}

}  // namespace io
}  // namespace cartographer
