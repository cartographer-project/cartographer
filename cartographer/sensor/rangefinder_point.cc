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

RangefinderPoint RangefinderPoint::RestrictDistanceFromOrigin(
    const Eigen::Vector3f& origin, float distance) const {
  RangefinderPoint result = *this;
  result.position_ = origin + distance * (position_ - origin).normalized();
  return result;
}

RangefinderPoint::RangefinderPoint(
    const proto::RangefinderPoint& rangefinder_point_proto) {
  position_ = transform::ToEigen(rangefinder_point_proto.position());
}

TimedRangefinderPoint::TimedRangefinderPoint(
    const proto::TimedRangefinderPoint& timed_rangefinder_point_proto)
    : RangefinderPoint(timed_rangefinder_point_proto.rangefinder_point()),
      time_(timed_rangefinder_point_proto.time()) {}

proto::RangefinderPoint RangefinderPoint::ToProto() const {
  proto::RangefinderPoint proto;
  *proto.mutable_position() = transform::ToProto(position_);
  return proto;
}

proto::TimedRangefinderPoint TimedRangefinderPoint::ToProto() const {
  proto::TimedRangefinderPoint proto;
  *proto.mutable_rangefinder_point() = RangefinderPoint::ToProto();
  proto.set_time(time_);
  return proto;
}

}  // namespace sensor
}  // namespace cartographer
