/*
 * Copyright 2018 The Cartographer Authors
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

RangefinderPoint FromProto(
    const proto::RangefinderPoint& rangefinder_point_proto) {
  return {transform::ToEigen(rangefinder_point_proto.position())};
}

proto::RangefinderPoint ToProto(const RangefinderPoint& rangefinder_point) {
  proto::RangefinderPoint proto;
  *proto.mutable_position() = transform::ToProto(rangefinder_point.position);
  return proto;
}

TimedRangefinderPoint FromProto(
    const proto::TimedRangefinderPoint& timed_rangefinder_point_proto) {
  return {transform::ToEigen(timed_rangefinder_point_proto.position()),
          timed_rangefinder_point_proto.time()};
}

proto::TimedRangefinderPoint ToProto(
    const TimedRangefinderPoint& timed_rangefinder_point) {
  proto::TimedRangefinderPoint proto;
  *proto.mutable_position() =
      transform::ToProto(timed_rangefinder_point.position);
  proto.set_time(timed_rangefinder_point.time);
  return proto;
}

RangefinderPoint ToRangefinderPoint(
    const TimedRangefinderPoint& timed_rangefinder_point) {
  return {timed_rangefinder_point.position};
}

TimedRangefinderPoint ToTimedRangefinderPoint(
    const RangefinderPoint& rangefinder_point, const float time) {
  return {rangefinder_point.position, time};
}

}  // namespace sensor
}  // namespace cartographer
