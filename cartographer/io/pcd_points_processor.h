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

#ifndef CARTOGRAPHER_IO_PCD_POINTS_PROCESSOR_H_
#define CARTOGRAPHER_IO_PCD_POINTS_PROCESSOR_H_

#include "cartographer/io/points_processor.h"
#include "cartographer/mapping_3d/hybrid_grid.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace io {

// Creates PCD file from voxel filtered points
class PCDPointsProcessor : public PointsProcessor {
 public:
  PCDPointsProcessor(double voxel_size,
                    const string& output_filename, PointsProcessor* next);

  ~PCDPointsProcessor() override {}

  void Process(const PointsBatch& batch) override;
  void Flush() override;

 private:
  using Voxels = mapping_3d::HybridGridBase<bool>;

  PointsProcessor* const next_;
  const string output_filename_;
  Voxels voxels_;

};

}  // namespace io
}  // namespace cartographer

#endif  // CARTOGRAPHER_IO_XRAY_POINTS_PROCESSOR_H_
