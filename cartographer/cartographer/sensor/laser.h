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

// Builds a LaserFan from 'proto' and separates any beams with ranges outside
// the range ['min_range', 'max_range']. Beams beyond 'max_range' inserted into
// the 'missing_echo_point_cloud' with length 'missing_echo_ray_length'. The
// points in both clouds are stored in scan order.
struct LaserFan {
  Eigen::Vector2f origin;
  PointCloud2D point_cloud;
  PointCloud2D missing_echo_point_cloud;
};
LaserFan ToLaserFan(const proto::LaserScan& proto, float min_range,
                    float max_range, float missing_echo_ray_length);

// Transforms 'laser_fan' according to 'transform'.
LaserFan TransformLaserFan(const LaserFan& laser_fan,
                           const transform::Rigid2f& transform);

// A 3D variant of LaserFan. Rays begin at 'origin'. 'returns' are the points
// where laser returns were detected. 'misses' are points in the direction of
// rays for which no return was detected, and were inserted at a configured
// distance. It is assumed that between the 'origin' and 'misses' is free space.
struct LaserFan3D {
  Eigen::Vector3f origin;
  PointCloud returns;
  PointCloud misses;

  // Reflectivity value of returns.
  std::vector<uint8> reflectivities;
};

// Like LaserFan3D but with compressed point clouds. The point order changes
// when converting from LaserFan3D.
struct CompressedLaserFan3D {
  Eigen::Vector3f origin;
  CompressedPointCloud returns;
  CompressedPointCloud misses;

  // Reflectivity value of returns.
  std::vector<uint8> reflectivities;
};

LaserFan3D Decompress(const CompressedLaserFan3D& compressed_laser_fan);

CompressedLaserFan3D Compress(const LaserFan3D& laser_fan);

// Converts 3D 'laser_fan' to a proto::LaserFan3D.
proto::LaserFan3D ToProto(const LaserFan3D& laser_fan);

// Converts 'proto' to a LaserFan3D.
LaserFan3D FromProto(const proto::LaserFan3D& proto);

LaserFan3D ToLaserFan3D(const LaserFan& laser_fan);

LaserFan3D TransformLaserFan3D(const LaserFan3D& laser_fan,
                               const transform::Rigid3f& transform);

// Projects 'laser_fan' into 2D and crops it according to the cuboid defined by
// 'min' and 'max'.
LaserFan ProjectCroppedLaserFan(const LaserFan3D& laser_fan,
                                const Eigen::Vector3f& min,
                                const Eigen::Vector3f& max);

// Filter a 'laser_fan', retaining only the returns that have no more than
// 'max_range' distance from the laser origin. Removes misses and reflectivity
// information.
LaserFan3D FilterLaserFanByMaxRange(const LaserFan3D& laser_fan,
                                    float max_range);

}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER_SENSOR_LASER_H_
