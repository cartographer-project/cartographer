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

#ifndef CARTOGRAPHER_IO_OUTLIER_REMOVING_POINTS_PROCESSOR_H_
#define CARTOGRAPHER_IO_OUTLIER_REMOVING_POINTS_PROCESSOR_H_

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/io/points_processor.h"
#include "cartographer/mapping/3d/hybrid_grid.h"

namespace cartographer {
namespace io {

// Voxel filters the data and only passes on points that we believe are on
// non-moving objects.
class OutlierRemovingPointsProcessor : public PointsProcessor {
 public:
  constexpr static const char* kConfigurationFileActionName =
      "voxel_filter_and_remove_moving_objects";

  OutlierRemovingPointsProcessor(double voxel_size, double miss_per_hit_limit,
                                 PointsProcessor* next);

  static std::unique_ptr<OutlierRemovingPointsProcessor> FromDictionary(
      common::LuaParameterDictionary* dictionary, PointsProcessor* next);

  ~OutlierRemovingPointsProcessor() override {}

  OutlierRemovingPointsProcessor(const OutlierRemovingPointsProcessor&) =
      delete;
  OutlierRemovingPointsProcessor& operator=(
      const OutlierRemovingPointsProcessor&) = delete;

  void Process(std::unique_ptr<PointsBatch> batch) override;
  FlushResult Flush() override;

 private:
  // To reduce memory consumption by not having to keep all rays in memory, we
  // filter outliers in three phases each going over all data: First we compute
  // all voxels containing any hits, then we compute the rays passing through
  // each of these voxels, and finally we output all hits in voxels that are
  // considered obstructed.
  struct VoxelData {
    int hits = 0;
    int rays = 0;
  };
  enum class State {
    kPhase1,
    kPhase2,
    kPhase3,
  };

  // First phase counts the number of hits per voxel.
  void ProcessInPhaseOne(const PointsBatch& batch);

  // Second phase counts how many rays pass through each voxel. This is only
  // done for voxels that contain hits. This is to reduce memory consumption by
  // not adding data to free voxels.
  void ProcessInPhaseTwo(const PointsBatch& batch);

  // Third phase produces the output containing all inliers. We consider each
  // hit an inlier if it is inside a voxel that has a sufficiently high
  // hit-to-ray ratio.
  void ProcessInPhaseThree(std::unique_ptr<PointsBatch> batch);

  const double voxel_size_;
  const double miss_per_hit_limit_;
  PointsProcessor* const next_;
  State state_;
  mapping::HybridGridBase<VoxelData> voxels_;
};

}  // namespace io
}  // namespace cartographer

#endif  // CARTOGRAPHER_IO_OUTLIER_REMOVING_POINTS_PROCESSOR_H_
