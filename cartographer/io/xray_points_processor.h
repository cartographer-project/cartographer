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

#ifndef CARTOGRAPHER_IO_XRAY_POINTS_PROCESSOR_H_
#define CARTOGRAPHER_IO_XRAY_POINTS_PROCESSOR_H_

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/io/points_processor.h"
#include "cartographer/mapping_3d/hybrid_grid.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace io {

// Creates X-ray cuts through the points with pixels being 'voxel_size' big. All
// images created from a single XRayPointsProcessor have the same dimensions and
// are centered on the center of the bounding box, so that they can easily be
// combined into a movie.
class XRayPointsProcessor : public PointsProcessor {
 public:
  constexpr static const char* kConfigurationFileActionName =
      "write_xray_image";
  XRayPointsProcessor(double voxel_size, const transform::Rigid3f& transform,
                      const string& output_filename, PointsProcessor* next);

  static std::unique_ptr<XRayPointsProcessor> FromDictionary(
      common::LuaParameterDictionary* dictionary, PointsProcessor* next);

  ~XRayPointsProcessor() override {}

  void Process(std::unique_ptr<PointsBatch> batch) override;
  FlushResult Flush() override;

 private:
  using Voxels = mapping_3d::HybridGridBase<bool>;

  void WriteImage();

  PointsProcessor* const next_;
  const string output_filename_;
  const transform::Rigid3f transform_;
  Voxels voxels_;
};

}  // namespace io
}  // namespace cartographer

#endif  // CARTOGRAPHER_IO_XRAY_POINTS_PROCESSOR_H_
