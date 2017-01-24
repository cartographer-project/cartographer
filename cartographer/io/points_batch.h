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

#ifndef CARTOGRAPHER_IO_POINTS_BATCH_H_
#define CARTOGRAPHER_IO_POINTS_BATCH_H_

#include <cstdint>
#include <vector>

#include "Eigen/Core"
#include "cartographer/common/time.h"

namespace cartographer {
namespace io {

// A point's color.
using Color = std::array<uint8_t, 3>;

// A number of points, captured around the same 'time' and by a
// sensor at the same 'origin'.
struct PointsBatch {
  PointsBatch() {
    origin = Eigen::Vector3f::Zero();
    trajectory_index = 0;
  }

  // Time at which this batch has been acquired.
  common::Time time;

  // Origin of the data, i.e. the location of the sensor in the world at
  // 'time'.
  Eigen::Vector3f origin;

  // Sensor that generated this data's 'frame_id' or empty if this information
  // is unknown.
  string frame_id;

  // Trajectory index that produced this point.
  int trajectory_index;

  // Geometry of the points in a metric frame.
  std::vector<Eigen::Vector3f> points;

  // Intensities are optional and may be unspecified. The meaning of these
  // intensity values varies by device. For example, the VLP16 provides values
  // in the range [0, 100] for non-specular return values and values up to 255
  // for specular returns. On the other hand, Hokuyo lasers provide a 16-bit
  // value that rarely peaks above 4096.
  std::vector<float> intensities;

  // Colors are optional. If set, they are RGB values.
  std::vector<Color> colors;
};

// Removes the indices in 'to_remove' from 'batch'.
void RemovePoints(std::vector<int> to_remove, PointsBatch* batch);

}  // namespace io
}  // namespace cartographer

#endif  // CARTOGRAPHER_IO_POINTS_BATCH_H_
