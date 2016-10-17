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

#include "cartographer/io/points_batch.h"

namespace cartographer {
namespace io {

// A processor in a pipeline. It processes a 'points_batch' and hands it to the
// next processor in the pipeline. Once 'flush' is called no more data will be
// send through the pipeline.
class PointsProcessor {
 public:
  PointsProcessor() {}
  virtual ~PointsProcessor() {}

  PointsProcessor(const PointsProcessor&) = delete;
  PointsProcessor& operator=(const PointsProcessor&) = delete;

  virtual void Process(const PointsBatch& points_batch) = 0;
  virtual void Flush() = 0;
};

}  // namespace io
}  // namespace cartographer

#endif  // CARTOGRAPHER_IO_POINTS_PROCESSOR_H_
