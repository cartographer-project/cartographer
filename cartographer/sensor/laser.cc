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

#include "cartographer/sensor/laser.h"

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

LaserFan ToLaserFan(const proto::LaserScan& proto, const float min_range,
                    const float max_range,
                    const float missing_echo_ray_length) {
  CHECK_GE(min_range, 0.f);
  CHECK_GT(proto.angle_increment(), 0.f);
  CHECK_GT(proto.angle_max(), proto.angle_min());
  LaserFan laser_fan = {Eigen::Vector3f::Zero(), {}, {}};
  float angle = proto.angle_min();
  for (const auto& range : proto.range()) {
    if (range.value_size() > 0) {
      const float first_echo = range.value(0);
      if (!std::isnan(first_echo) && first_echo >= min_range) {
        const Eigen::AngleAxisf rotation(angle, Eigen::Vector3f::UnitZ());
        if (first_echo <= max_range) {
          laser_fan.returns.push_back(rotation *
                                      (first_echo * Eigen::Vector3f::UnitX()));
        } else {
          laser_fan.misses.push_back(
              rotation * (missing_echo_ray_length * Eigen::Vector3f::UnitX()));
        }
      }
    }
    angle += proto.angle_increment();
  }
  return laser_fan;
}

proto::LaserFan ToProto(const LaserFan& laser_fan) {
  proto::LaserFan proto;
  *proto.mutable_origin() = transform::ToProto(laser_fan.origin);
  *proto.mutable_point_cloud() = ToProto(laser_fan.returns);
  *proto.mutable_missing_echo_point_cloud() = ToProto(laser_fan.misses);
  std::copy(laser_fan.reflectivities.begin(), laser_fan.reflectivities.end(),
            RepeatedFieldBackInserter(proto.mutable_reflectivity()));
  return proto;
}

LaserFan FromProto(const proto::LaserFan& proto) {
  auto laser_fan = LaserFan{
      transform::ToEigen(proto.origin()), ToPointCloud(proto.point_cloud()),
      ToPointCloud(proto.missing_echo_point_cloud()),
  };
  std::copy(proto.reflectivity().begin(), proto.reflectivity().end(),
            std::back_inserter(laser_fan.reflectivities));
  return laser_fan;
}

LaserFan TransformLaserFan(const LaserFan& laser_fan,
                           const transform::Rigid3f& transform) {
  return LaserFan{
      transform * laser_fan.origin,
      TransformPointCloud(laser_fan.returns, transform),
      TransformPointCloud(laser_fan.misses, transform),
      laser_fan.reflectivities,
  };
}

LaserFan CropLaserFan(const LaserFan& laser_fan, const float min_z,
                      const float max_z) {
  return LaserFan{laser_fan.origin, Crop(laser_fan.returns, min_z, max_z),
                  Crop(laser_fan.misses, min_z, max_z)};
}

CompressedLaserFan Compress(const LaserFan& laser_fan) {
  std::vector<int> new_to_old;
  CompressedPointCloud compressed_returns =
      CompressedPointCloud::CompressAndReturnOrder(laser_fan.returns,
                                                   &new_to_old);
  return CompressedLaserFan{
      laser_fan.origin, std::move(compressed_returns),
      CompressedPointCloud(laser_fan.misses),
      ReorderReflectivities(laser_fan.reflectivities, new_to_old)};
}

LaserFan Decompress(const CompressedLaserFan& compressed_laser_fan) {
  return LaserFan{compressed_laser_fan.origin,
                  compressed_laser_fan.returns.Decompress(),
                  compressed_laser_fan.misses.Decompress(),
                  compressed_laser_fan.reflectivities};
}

}  // namespace sensor
}  // namespace cartographer
