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

#include "cartographer/sensor/point_cloud.h"

#include "cartographer/sensor/proto/sensor.pb.h"
#include "cartographer/transform/transform.h"

namespace cartographer {
namespace sensor {

namespace {

template <typename PointCloudType, typename TransformType>
PointCloudType Transform(const PointCloudType& point_cloud,
                         const TransformType& transform) {
  PointCloudType result;
  result.reserve(point_cloud.size());
  for (const auto& point : point_cloud) {
    result.emplace_back(transform * point);
  }
  return result;
}

}  // namespace

PointCloud TransformPointCloud(const PointCloud& point_cloud,
                               const transform::Rigid3f& transform) {
  return Transform(point_cloud, transform);
}

PointCloud2D TransformPointCloud2D(const PointCloud2D& point_cloud_2d,
                                   const transform::Rigid2f& transform) {
  return Transform(point_cloud_2d, transform);
}

PointCloud ToPointCloud(const PointCloud2D& point_cloud_2d) {
  sensor::PointCloud point_cloud;
  point_cloud.reserve(point_cloud_2d.size());
  for (const auto& point : point_cloud_2d) {
    point_cloud.emplace_back(point.x(), point.y(), 0.f);
  }
  return point_cloud;
}

PointCloud2D ProjectToPointCloud2D(const PointCloud& point_cloud) {
  sensor::PointCloud2D point_cloud_2d;
  point_cloud_2d.reserve(point_cloud.size());
  for (const auto& point : point_cloud) {
    point_cloud_2d.emplace_back(point.x(), point.y());
  }
  return point_cloud_2d;
}

PointCloud Crop(const PointCloud& point_cloud, const Eigen::Vector3f& min,
                const Eigen::Vector3f& max) {
  PointCloud cropped_point_cloud;
  for (const auto& point : point_cloud) {
    if (min.x() <= point.x() && point.x() <= max.x() && min.y() <= point.y() &&
        point.y() <= max.y() && min.z() <= point.z() && point.z() <= max.z()) {
      cropped_point_cloud.push_back(point);
    }
  }
  return cropped_point_cloud;
}

proto::PointCloud ToProto(const PointCloud& point_cloud) {
  proto::PointCloud proto;
  for (const auto& point : point_cloud) {
    proto.add_x(point.x());
    proto.add_y(point.y());
    proto.add_z(point.z());
  }
  return proto;
}

PointCloud ToPointCloud(const proto::PointCloud& proto) {
  PointCloud point_cloud;
  const int size = std::min({proto.x_size(), proto.y_size(), proto.z_size()});
  point_cloud.reserve(size);
  for (int i = 0; i != size; ++i) {
    point_cloud.emplace_back(proto.x(i), proto.y(i), proto.z(i));
  }
  return point_cloud;
}

}  // namespace sensor
}  // namespace cartographer
