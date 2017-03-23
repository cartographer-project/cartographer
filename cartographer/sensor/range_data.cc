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

#include "cartographer/sensor/range_data.h"

#include "cartographer/sensor/proto/sensor.pb.h"
#include "cartographer/transform/transform.h"

namespace cartographer {
namespace sensor {

namespace {

// Reorders reflectivities according to index mapping.
std::vector<uint8> ReorderReflectivities(
    const std::vector<uint8>& reflectivities,
    const std::vector<int>& new_to_old) {
  std::vector<uint8> reordered(reflectivities.size());
  for (size_t i = 0; i < reordered.size(); ++i) {
    reordered[i] = reflectivities[new_to_old[i]];
  }
  return reordered;
}

}  // namespace

PointCloud ToPointCloud(const proto::LaserScan& proto) {
  return ToPointCloudWithIntensities(proto).points;
}

PointCloudWithIntensities ToPointCloudWithIntensities(
    const proto::LaserScan& proto) {
  CHECK_GE(proto.range_min(), 0.f);
  CHECK_GE(proto.range_max(), proto.range_min());
  if (proto.angle_increment() > 0.f) {
    CHECK_GT(proto.angle_max(), proto.angle_min());
  } else {
    CHECK_GT(proto.angle_min(), proto.angle_max());
  }
  PointCloudWithIntensities point_cloud;
  float angle = proto.angle_min();
  for (int i = 0; i < proto.range_size(); ++i) {
    const auto& range = proto.range(i);
    if (range.value_size() > 0) {
      const float first_echo = range.value(0);
      if (proto.range_min() <= first_echo && first_echo <= proto.range_max()) {
        const Eigen::AngleAxisf rotation(angle, Eigen::Vector3f::UnitZ());
        point_cloud.points.push_back(rotation *
                                     (first_echo * Eigen::Vector3f::UnitX()));
        if (proto.intensity_size() > 0) {
          point_cloud.intensities.push_back(proto.intensity(i).value(0));
        } else {
          point_cloud.intensities.push_back(0.f);
        }
      }
    }
    angle += proto.angle_increment();
  }
  return point_cloud;
}

proto::RangeData ToProto(const RangeData& range_data) {
  proto::RangeData proto;
  *proto.mutable_origin() = transform::ToProto(range_data.origin);
  *proto.mutable_point_cloud() = ToProto(range_data.returns);
  *proto.mutable_missing_echo_point_cloud() = ToProto(range_data.misses);
  std::copy(range_data.reflectivities.begin(), range_data.reflectivities.end(),
            RepeatedFieldBackInserter(proto.mutable_reflectivity()));
  return proto;
}

RangeData FromProto(const proto::RangeData& proto) {
  auto range_data = RangeData{
      transform::ToEigen(proto.origin()), ToPointCloud(proto.point_cloud()),
      ToPointCloud(proto.missing_echo_point_cloud()),
  };
  std::copy(proto.reflectivity().begin(), proto.reflectivity().end(),
            std::back_inserter(range_data.reflectivities));
  return range_data;
}

RangeData TransformRangeData(const RangeData& range_data,
                             const transform::Rigid3f& transform) {
  return RangeData{
      transform * range_data.origin,
      TransformPointCloud(range_data.returns, transform),
      TransformPointCloud(range_data.misses, transform),
      range_data.reflectivities,
  };
}

RangeData CropRangeData(const RangeData& range_data, const float min_z,
                        const float max_z) {
  return RangeData{range_data.origin, Crop(range_data.returns, min_z, max_z),
                   Crop(range_data.misses, min_z, max_z)};
}

CompressedRangeData Compress(const RangeData& range_data) {
  std::vector<int> new_to_old;
  CompressedPointCloud compressed_returns =
      CompressedPointCloud::CompressAndReturnOrder(range_data.returns,
                                                   &new_to_old);
  return CompressedRangeData{
      range_data.origin, std::move(compressed_returns),
      CompressedPointCloud(range_data.misses),
      ReorderReflectivities(range_data.reflectivities, new_to_old)};
}

RangeData Decompress(const CompressedRangeData& compressed_range_data) {
  return RangeData{compressed_range_data.origin,
                   compressed_range_data.returns.Decompress(),
                   compressed_range_data.misses.Decompress(),
                   compressed_range_data.reflectivities};
}

}  // namespace sensor
}  // namespace cartographer
