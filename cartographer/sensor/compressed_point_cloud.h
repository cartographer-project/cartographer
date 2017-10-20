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

#ifndef CARTOGRAPHER_SENSOR_COMPRESSED_POINT_CLOUD_H_
#define CARTOGRAPHER_SENSOR_COMPRESSED_POINT_CLOUD_H_

#include <iterator>
#include <vector>

#include "Eigen/Core"
#include "cartographer/common/port.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/sensor/proto/sensor.pb.h"

namespace cartographer {
namespace sensor {

// A compressed representation of a point cloud consisting of a collection of
// points (Vector3f) without time information.
// Internally, points are grouped by blocks. Each block encodes a bit of meta
// data (number of points in block, coordinates of the block) and encodes each
// point with a fixed bit rate in relation to the block.
class CompressedPointCloud {
 public:
  class ConstIterator;

  CompressedPointCloud() : num_points_(0) {}
  explicit CompressedPointCloud(const PointCloud& point_cloud);
  explicit CompressedPointCloud(const proto::CompressedPointCloud& proto);

  // Returns decompressed point cloud.
  PointCloud Decompress() const;

  bool empty() const;
  size_t size() const;
  ConstIterator begin() const;
  ConstIterator end() const;

  bool operator==(const CompressedPointCloud& right_hand_container) const;
  proto::CompressedPointCloud ToProto() const;

 private:
  std::vector<int32> point_data_;
  size_t num_points_;
};

// Forward iterator for compressed point clouds.
class CompressedPointCloud::ConstIterator {
 public:
  using iterator_category = std::forward_iterator_tag;
  using value_type = Eigen::Vector3f;
  using difference_type = int64;
  using pointer = const Eigen::Vector3f*;
  using reference = const Eigen::Vector3f&;

  // Creates begin iterator.
  explicit ConstIterator(const CompressedPointCloud* compressed_point_cloud);

  // Creates end iterator.
  static ConstIterator EndIterator(
      const CompressedPointCloud* compressed_point_cloud);

  Eigen::Vector3f operator*() const;
  ConstIterator& operator++();
  bool operator!=(const ConstIterator& it) const;

 private:
  // Reads next point from buffer. Also handles reading the meta data of the
  // next block, if the current block is depleted.
  void ReadNextPoint();

  const CompressedPointCloud* compressed_point_cloud_;
  size_t remaining_points_;
  int32 remaining_points_in_current_block_;
  Eigen::Vector3f current_point_;
  Eigen::Vector3i current_block_coordinates_;
  std::vector<int32>::const_iterator input_;
};

}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER_SENSOR_COMPRESSED_POINT_CLOUD_H_
