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

#ifndef CARTOGRAPHER_ROS_GOOGLE_CARTOGRAPHER_SRC_MSG_CONVERSION_H_
#define CARTOGRAPHER_ROS_GOOGLE_CARTOGRAPHER_SRC_MSG_CONVERSION_H_

#include "cartographer/common/port.h"
#include "cartographer/sensor/proto/sensor.pb.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/MultiEchoLaserScan.h"
#include "sensor_msgs/PointCloud2.h"

namespace cartographer_ros {

// Only uses first echo of each beam.
sensor_msgs::LaserScan ToLaserScan(
    int64 timestamp, const string& frame_id,
    const ::cartographer::sensor::proto::LaserScan& laser_scan);

sensor_msgs::MultiEchoLaserScan ToMultiEchoLaserScanMessage(
    int64 timestamp, const string& frame_id,
    const ::cartographer::sensor::proto::LaserScan& laser_scan);

sensor_msgs::Imu ToImuMessage(int64 timestamp, const string& frame_id,
                              const ::cartographer::sensor::proto::Imu& imu);

sensor_msgs::PointCloud2 ToPointCloud2Message(
    int64 timestamp, const string& frame_id,
    const ::cartographer::sensor::proto::LaserFan3D& laser_scan_3d);

::cartographer::sensor::proto::Imu ToCartographer(const sensor_msgs::Imu& msg);

::cartographer::sensor::proto::LaserScan ToCartographer(
    const sensor_msgs::LaserScan& msg);

::cartographer::sensor::proto::LaserFan3D ToCartographer(
    const pcl::PointCloud<pcl::PointXYZ>& pcl_points);

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_GOOGLE_CARTOGRAPHER_SRC_MSG_CONVERSION_H_
