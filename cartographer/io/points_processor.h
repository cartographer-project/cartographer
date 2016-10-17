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

#ifndef CARTOGRAPHER_IO_POINTS_PROCESSOR_H_
#define CARTOGRAPHER_IO_POINTS_PROCESSOR_H_

#include <memory>

#include "cartographer/io/points_batch.h"

namespace cartographer {
namespace io {

// A processor in a pipeline. It processes a 'points_batch' and hands it to the
// next processor in the pipeline.
class PointsProcessor {
 public:
  enum class FlushResult {
    kRestartStream,
    kFinished,
  };

  PointsProcessor() {}
  virtual ~PointsProcessor() {}

  PointsProcessor(const PointsProcessor&) = delete;
  PointsProcessor& operator=(const PointsProcessor&) = delete;

  // Receive a 'points_batch', process it and pass it on.
  virtual void Process(std::unique_ptr<PointsBatch> points_batch) = 0;

  // Some implementations will perform expensive computations and others that do
  // multiple passes over the data might ask for restarting the stream.
  virtual FlushResult Flush() = 0;
};

}  // namespace io
}  // namespace cartographer

#endif  // CARTOGRAPHER_IO_POINTS_PROCESSOR_H_
