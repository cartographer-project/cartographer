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

#include <map>

#include "Eigen/Core"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/io/file_writer.h"
#include "cartographer/io/points_processor.h"
#include "cartographer/mapping/3d/hybrid_grid.h"
#include "cartographer/mapping/detect_floors.h"
#include "cartographer/mapping/proto/trajectory.pb.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace io {

// Creates X-ray cuts through the points with pixels being 'voxel_size' big.
class XRayPointsProcessor : public PointsProcessor {
 public:
  constexpr static const char* kConfigurationFileActionName =
      "write_xray_image";
  enum class DrawTrajectories { kNo, kYes };
  XRayPointsProcessor(
      double voxel_size, const transform::Rigid3f& transform,
      const std::vector<mapping::Floor>& floors,
      const DrawTrajectories& draw_trajectories,
      const std::string& output_filename,
      const std::vector<mapping::proto::Trajectory>& trajectories,
      FileWriterFactory file_writer_factory, PointsProcessor* next);

  static std::unique_ptr<XRayPointsProcessor> FromDictionary(
      const std::vector<mapping::proto::Trajectory>& trajectories,
      FileWriterFactory file_writer_factory,
      common::LuaParameterDictionary* dictionary, PointsProcessor* next);

  ~XRayPointsProcessor() override {}

  void Process(std::unique_ptr<PointsBatch> batch) override;
  FlushResult Flush() override;

  Eigen::AlignedBox3i bounding_box() const { return bounding_box_; }

 private:
  struct ColumnData {
    float sum_r = 0.;
    float sum_g = 0.;
    float sum_b = 0.;
    uint32_t count = 0;
  };

  struct Aggregation {
    mapping::HybridGridBase<bool> voxels;
    std::map<std::pair<int, int>, ColumnData> column_data;
  };

  void WriteVoxels(const Aggregation& aggregation,
                   FileWriter* const file_writer);
  void Insert(const PointsBatch& batch, Aggregation* aggregation);

  const DrawTrajectories draw_trajectories_;
  const std::vector<mapping::proto::Trajectory> trajectories_;
  FileWriterFactory file_writer_factory_;
  PointsProcessor* const next_;

  // If empty, we do not separate into floors.
  std::vector<mapping::Floor> floors_;

  const std::string output_filename_;
  const transform::Rigid3f transform_;

  // Only has one entry if we do not separate into floors.
  std::vector<Aggregation> aggregations_;

  // Bounding box containing all cells with data in all 'aggregations_'.
  Eigen::AlignedBox3i bounding_box_;
};

}  // namespace io
}  // namespace cartographer

#endif  // CARTOGRAPHER_IO_XRAY_POINTS_PROCESSOR_H_
