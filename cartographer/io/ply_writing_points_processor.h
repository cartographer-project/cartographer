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

#include <fstream>

#include "cartographer/io/points_processor.h"

namespace cartographer {
namespace io {

// Streams a PLY file to disk. The header is written in 'Flush'.
class PlyWritingPointsProcessor : public PointsProcessor {
 public:
  PlyWritingPointsProcessor(const string& filename, PointsProcessor* next);
  ~PlyWritingPointsProcessor() override {}

  PlyWritingPointsProcessor(const PlyWritingPointsProcessor&) = delete;
  PlyWritingPointsProcessor& operator=(const PlyWritingPointsProcessor&) =
      delete;

  void Process(std::unique_ptr<PointsBatch> batch) override;
  FlushResult Flush() override;

 private:
  PointsProcessor* const next_;

  int64 num_points_;
  bool has_colors_;
  std::ofstream file_;
};

}  // namespace io
}  // namespace cartographer
