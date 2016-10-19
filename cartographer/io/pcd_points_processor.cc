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

#include "cartographer/io/pcd_points_processor.h"

#include <cmath>
#include <string>
#include <iostream>
#include <fstream>

#include "Eigen/Core"
#include "cartographer/common/math.h"
#include "cartographer/io/cairo_types.h"
#include "cartographer/mapping_3d/hybrid_grid.h"

namespace cartographer {
namespace io {

PCDPointsProcessor::PCDPointsProcessor(double voxel_size,
                  const string& output_filename, PointsProcessor* next)
  : next_(next),
    output_filename_(output_filename),
    voxels_(voxel_size, Eigen::Vector3f::Zero()) {
}

void PCDPointsProcessor::Process(const PointsBatch& batch) {
  for (const auto& point : batch.points) {
    const Eigen::Vector3f camera_point = point;
    *voxels_.mutable_value(voxels_.GetCellIndex(camera_point)) = true;
  }
  next_->Process(batch);
}

void PCDPointsProcessor::Flush() {
  std::fstream fs;
  fs.open(output_filename_, std::ios::out);

  float voxel_size = voxels_.resolution();
  int num = 0;
  for (Voxels::Iterator it(voxels_); !it.Done(); it.Next()) {
    num ++;
  }

  fs << "VERSION .7" << std::endl;
  fs << "FIELDS x y z" << std::endl;
  fs << "SIZE 4 4 4" << std::endl;
  fs << "TYPE F F F" << std::endl;
  fs << "COUNT 1 1 1" << std::endl;
  fs << "WIDTH " << num << std::endl;
  fs << "HEIGHT 1" << std::endl;
  fs << "VIEWPOINT 0 0 0 1 0 0 0" << std::endl;
  fs << "POINTS " << num << std::endl;
  fs << "DATA ascii" << std::endl;

  for (Voxels::Iterator it(voxels_); !it.Done(); it.Next()) {
    const Eigen::Array3i pixel = it.GetCellIndex();
    fs << pixel.x() * voxel_size << " ";
    fs << pixel.y() * voxel_size << " ";
    fs << pixel.z() * voxel_size << std::endl;
  }
  return next_->Flush();
}

}  // namespace io
}  // namespace cartographer
