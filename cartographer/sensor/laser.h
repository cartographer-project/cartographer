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

#ifndef CARTOGRAPHER_SENSOR_LASER_H_
#define CARTOGRAPHER_SENSOR_LASER_H_

#include "cartographer/common/port.h"
#include "cartographer/sensor/compressed_point_cloud.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/sensor/proto/sensor.pb.h"

namespace cartographer {
namespace sensor {

// A 3D variant of LaserFan. Rays begin at 'origin'. 'returns' are the points
// where laser returns were detected. 'misses' are points in the direction of
// rays for which no return was detected, and were inserted at a configured
// distance. It is assumed that between the 'origin' and 'misses' is free space.
struct LaserFan {
  Eigen::Vector3f origin;
  PointCloud returns;
  PointCloud misses;

  // Reflectivity value of returns.
  std::vector<uint8> reflectivities;
};

// Builds a LaserFan from 'proto' and separates any beams with ranges outside
// the range ['min_range', 'max_range']. Beams beyond 'max_range' are inserted
// into the 'misses' point cloud with length 'missing_echo_ray_length'. The
// points in both clouds are stored in scan order.
LaserFan ToLaserFan(const proto::LaserScan& proto, float min_range,
                    float max_range, float missing_echo_ray_length);

// Converts 3D 'laser_fan' to a proto::LaserFan.
proto::LaserFan ToProto(const LaserFan& laser_fan);

// Converts 'proto' to a LaserFan.
LaserFan FromProto(const proto::LaserFan& proto);

LaserFan TransformLaserFan(const LaserFan& laser_fan,
                           const transform::Rigid3f& transform);

// Crops 'laser_fan' according to the region defined by 'min_z' and 'max_z'.
LaserFan CropLaserFan(const LaserFan& laser_fan, float min_z, float max_z);

// Like LaserFan but with compressed point clouds. The point order changes
// when converting from LaserFan.
struct CompressedLaserFan {
  Eigen::Vector3f origin;
  CompressedPointCloud returns;
  CompressedPointCloud misses;

  // Reflectivity value of returns.
  std::vector<uint8> reflectivities;
};

CompressedLaserFan Compress(const LaserFan& laser_fan);

LaserFan Decompress(const CompressedLaserFan& compressed_laser_fan);

}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER_SENSOR_LASER_H_
