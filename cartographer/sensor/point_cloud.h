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

#ifndef CARTOGRAPHER_SENSOR_POINT_CLOUD_H_
#define CARTOGRAPHER_SENSOR_POINT_CLOUD_H_

#include <vector>

#include "Eigen/Core"
#include "Eigen/StdVector"
#include "Eigen/Geometry"
#include "cartographer/sensor/proto/sensor.pb.h"
#include "cartographer/transform/rigid_transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace sensor {


typedef std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> PointCloud;
typedef std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>> PointCloud2D;

// Transforms 'point_cloud' according to 'transform'.
PointCloud TransformPointCloud(const PointCloud& point_cloud,
                               const transform::Rigid3f& transform);

// Transforms 'point_cloud_2d' according to 'transform'.
PointCloud2D TransformPointCloud2D(const PointCloud2D& point_cloud_2d,
                                   const transform::Rigid2f& transform);

// Converts 'point_cloud_2d' to a 3D point cloud.
PointCloud ToPointCloud(const PointCloud2D& point_cloud_2d);

// Converts 'point_cloud' to a 2D point cloud by removing the z component.
PointCloud2D ProjectToPointCloud2D(const PointCloud& point_cloud);

// Returns a new point cloud without points that fall outside the axis-aligned
// cuboid defined by 'min' and 'max'.
PointCloud Crop(const PointCloud& point_cloud, const Eigen::Vector3f& min,
                const Eigen::Vector3f& max);

// Converts 'point_cloud' to a proto::PointCloud.
proto::PointCloud ToProto(const PointCloud& point_cloud);

// Converts 'proto' to a PointCloud.
PointCloud ToPointCloud(const proto::PointCloud& proto);

}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER_SENSOR_POINT_CLOUD_H_
