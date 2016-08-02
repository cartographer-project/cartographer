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

#include "msg_conversion.h"

#include "cartographer/common/port.h"
#include "cartographer/common/time.h"
#include "cartographer/sensor/proto/sensor.pb.h"
#include "cartographer/transform/proto/transform.pb.h"
#include "cartographer/transform/transform.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Vector3.h"
#include "glog/logging.h"
#include "ros/ros.h"
#include "ros/serialization.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/MultiEchoLaserScan.h"
#include "sensor_msgs/PointCloud2.h"

#include "time_conversion.h"

namespace cartographer_ros {
namespace {

// The ros::sensor_msgs::PointCloud2 binary data contains 4 floats for each
// point. The last one must be this value or RViz is not showing the point cloud
// properly.
constexpr float kPointCloudComponentFourMagic = 1.;

void ToMessage(const ::cartographer::transform::proto::Vector3d& proto,
               geometry_msgs::Vector3* vector) {
  vector->x = proto.x();
  vector->y = proto.y();
  vector->z = proto.z();
}

void ToMessage(const ::cartographer::transform::proto::Quaterniond& proto,
               geometry_msgs::Quaternion* quaternion) {
  quaternion->w = proto.w();
  quaternion->x = proto.x();
  quaternion->y = proto.y();
  quaternion->z = proto.z();
}

}  // namespace

sensor_msgs::MultiEchoLaserScan ToMultiEchoLaserScanMessage(
    const int64 timestamp, const string& frame_id,
    const ::cartographer::sensor::proto::LaserScan& laser_scan) {
  sensor_msgs::MultiEchoLaserScan msg;
  msg.header.stamp = ToRos(::cartographer::common::FromUniversal(timestamp));
  msg.header.frame_id = frame_id;

  msg.angle_min = laser_scan.angle_min();
  msg.angle_max = laser_scan.angle_max();
  msg.angle_increment = laser_scan.angle_increment();
  msg.time_increment = laser_scan.time_increment();
  msg.scan_time = laser_scan.scan_time();
  msg.range_min = laser_scan.range_min();
  msg.range_max = laser_scan.range_max();

  for (const auto& echoes : laser_scan.range()) {
    msg.ranges.emplace_back();
    std::copy(echoes.value().begin(), echoes.value().end(),
              std::back_inserter(msg.ranges.back().echoes));
  }

  for (const auto& echoes : laser_scan.intensity()) {
    msg.intensities.emplace_back();
    std::copy(echoes.value().begin(), echoes.value().end(),
              std::back_inserter(msg.intensities.back().echoes));
  }
  return msg;
}

sensor_msgs::Imu ToImuMessage(const int64 timestamp, const string& frame_id,
                              const ::cartographer::sensor::proto::Imu& imu) {
  sensor_msgs::Imu message;
  message.header.stamp =
      ToRos(::cartographer::common::FromUniversal(timestamp));
  message.header.frame_id = frame_id;

  ToMessage(imu.orientation(), &message.orientation);
  message.orientation_covariance = {{0., 0., 0., 0., 0., 0., 0., 0., 0.}};
  ToMessage(imu.angular_velocity(), &message.angular_velocity);
  message.angular_velocity_covariance = {{0., 0., 0., 0., 0., 0., 0., 0., 0.}};
  ToMessage(imu.linear_acceleration(), &message.linear_acceleration);
  message.linear_acceleration_covariance = {
      {0., 0., 0., 0., 0., 0., 0., 0., 0.}};
  return message;
}

sensor_msgs::LaserScan ToLaserScan(
    const int64 timestamp, const string& frame_id,
    const ::cartographer::sensor::proto::LaserScan& laser_scan) {
  sensor_msgs::LaserScan msg;
  msg.header.stamp = ToRos(::cartographer::common::FromUniversal(timestamp));
  msg.header.frame_id = frame_id;

  msg.angle_min = laser_scan.angle_min();
  msg.angle_max = laser_scan.angle_max();
  msg.angle_increment = laser_scan.angle_increment();
  msg.time_increment = laser_scan.time_increment();
  msg.scan_time = laser_scan.scan_time();
  msg.range_min = laser_scan.range_min();
  msg.range_max = laser_scan.range_max();

  for (const auto& echoes : laser_scan.range()) {
    if (echoes.value_size() > 0) {
      msg.ranges.push_back(echoes.value(0));
    } else {
      msg.ranges.push_back(0.);
    }
  }

  bool has_intensities = false;
  for (const auto& echoes : laser_scan.intensity()) {
    if (echoes.value_size() > 0) {
      msg.intensities.push_back(echoes.value(0));
      has_intensities = true;
    } else {
      msg.intensities.push_back(0);
    }
  }
  if (!has_intensities) {
    msg.intensities.clear();
  }

  return msg;
}

sensor_msgs::PointCloud2 ToPointCloud2Message(
    const int64 timestamp, const string& frame_id,
    const ::cartographer::sensor::proto::LaserFan3D& laser_scan_3d) {
  CHECK(::cartographer::transform::ToEigen(laser_scan_3d.origin()).norm() == 0)
      << "Trying to convert a laser_scan_3d that is not at the origin.";

  sensor_msgs::PointCloud2 msg;
  msg.header.stamp = ToRos(::cartographer::common::FromUniversal(timestamp));
  msg.header.frame_id = frame_id;

  const auto& point_cloud = laser_scan_3d.point_cloud();
  CHECK_EQ(point_cloud.x_size(), point_cloud.y_size());
  CHECK_EQ(point_cloud.x_size(), point_cloud.z_size());
  const auto num_points = point_cloud.x_size();

  msg.height = 1;
  msg.width = num_points;
  msg.fields.resize(3);
  msg.fields[0].name = "x";
  msg.fields[0].offset = 0;
  msg.fields[0].datatype = 7;
  msg.fields[0].count = 1;
  msg.fields[1].name = "y";
  msg.fields[1].offset = 4;
  msg.fields[1].datatype = 7;
  msg.fields[1].count = 1;
  msg.fields[2].name = "z";
  msg.fields[2].offset = 8;
  msg.fields[2].datatype = 7;
  msg.fields[2].count = 1;
  msg.is_bigendian = false;
  msg.point_step = 16;
  msg.row_step = 16 * msg.width;
  msg.is_dense = true;
  msg.data.resize(16 * num_points);

  ::ros::serialization::OStream stream(msg.data.data(), msg.data.size());
  for (int i = 0; i < num_points; ++i) {
    stream.next(point_cloud.x(i));
    stream.next(point_cloud.y(i));
    stream.next(point_cloud.z(i));
    stream.next(kPointCloudComponentFourMagic);
  }
  return msg;
}

::cartographer::sensor::proto::Imu ToCartographer(const sensor_msgs::Imu& msg) {
  ::cartographer::sensor::proto::Imu proto;

  if (msg.orientation_covariance[0] != -1) {
    auto* orientation = proto.mutable_orientation();
    orientation->set_x(msg.orientation.x);
    orientation->set_y(msg.orientation.y);
    orientation->set_z(msg.orientation.z);
    orientation->set_w(msg.orientation.w);
  }

  if (msg.angular_velocity_covariance[0] != -1) {
    auto* angular_velocity = proto.mutable_angular_velocity();
    angular_velocity->set_x(msg.angular_velocity.x);
    angular_velocity->set_y(msg.angular_velocity.y);
    angular_velocity->set_z(msg.angular_velocity.z);
  }

  if (msg.linear_acceleration_covariance[0] != -1) {
    auto* linear_acceleration = proto.mutable_linear_acceleration();
    linear_acceleration->set_x(msg.linear_acceleration.x);
    linear_acceleration->set_y(msg.linear_acceleration.y);
    linear_acceleration->set_z(msg.linear_acceleration.z);
  }
  return proto;
}

::cartographer::sensor::proto::LaserScan ToCartographer(
    const sensor_msgs::LaserScan& msg) {
  ::cartographer::sensor::proto::LaserScan proto;
  proto.set_angle_min(msg.angle_min);
  proto.set_angle_max(msg.angle_max);
  proto.set_angle_increment(msg.angle_increment);
  proto.set_time_increment(msg.time_increment);
  proto.set_scan_time(msg.scan_time);
  proto.set_range_min(msg.range_min);
  proto.set_range_max(msg.range_max);

  for (const auto& range : msg.ranges) {
    proto.add_range()->mutable_value()->Add(range);
  }

  for (const auto& intensity : msg.intensities) {
    proto.add_intensity()->mutable_value()->Add(intensity);
  }
  return proto;
}

::cartographer::sensor::proto::LaserFan3D ToCartographer(
    const pcl::PointCloud<pcl::PointXYZ>& pcl_points) {
  ::cartographer::sensor::proto::LaserFan3D proto;

  auto* origin = proto.mutable_origin();
  origin->set_x(0.);
  origin->set_y(0.);
  origin->set_z(0.);

  auto* point_cloud = proto.mutable_point_cloud();
  for (const auto& point : pcl_points) {
    point_cloud->add_x(point.x);
    point_cloud->add_y(point.y);
    point_cloud->add_z(point.z);
  }
  return proto;
}

}  // namespace cartographer_ros
