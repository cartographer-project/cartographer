/*
 * Copyright 2017 The Cartographer Authors
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

#include "cartographer/sensor/timed_point_cloud_data.h"

#include "cartographer/transform/proto/transform.pb.h"
#include "cartographer/transform/transform.h"

namespace cartographer {
namespace sensor {

proto::TimedPointCloudData ToProto(
    const TimedPointCloudData& timed_point_cloud_data) {
  proto::TimedPointCloudData proto;
  proto.set_timestamp(common::ToUniversal(timed_point_cloud_data.time));
  *proto.mutable_origin() = transform::ToProto(timed_point_cloud_data.origin);
  proto.mutable_point_data()->Reserve(timed_point_cloud_data.ranges.size());
  for (const Eigen::Vector4f& range : timed_point_cloud_data.ranges) {
    *proto.add_point_data() = transform::ToProto(range);
  }
  return proto;
}

TimedPointCloudData FromProto(const proto::TimedPointCloudData& proto) {
  TimedPointCloud timed_point_cloud;
  timed_point_cloud.reserve(proto.point_data().size());
  std::transform(
      proto.point_data().begin(), proto.point_data().end(),
      std::back_inserter(timed_point_cloud),
      static_cast<Eigen::Vector4f (*)(const transform::proto::Vector4f&)>(
          transform::ToEigen));
  return TimedPointCloudData{common::FromUniversal(proto.timestamp()),
                             transform::ToEigen(proto.origin()),
                             timed_point_cloud};
}

}  // namespace sensor
}  // namespace cartographer
