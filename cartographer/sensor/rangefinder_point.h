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

#ifndef CARTOGRAPHER_SENSOR_RANGEFINDER_POINT_H_
#define CARTOGRAPHER_SENSOR_RANGEFINDER_POINT_H_

#include <vector>

#include "Eigen/Core"
#include "cartographer/sensor/proto/sensor.pb.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace sensor {

// Stores 3D position of a point observed by a rangefinder sensor.
struct RangefinderPoint {
  Eigen::Vector3f position;
};

// Stores 3D position of a point with its relative measurement time.
// See point_cloud.h for more details.
struct TimedRangefinderPoint {
  Eigen::Vector3f position;
  float time;
};

template <class T>
inline RangefinderPoint operator*(const transform::Rigid3<T>& lhs,
                                  const RangefinderPoint& rhs) {
  RangefinderPoint result = rhs;
  result.position = lhs * rhs.position;
  return result;
}

template <class T>
inline TimedRangefinderPoint operator*(const transform::Rigid3<T>& lhs,
                                       const TimedRangefinderPoint& rhs) {
  TimedRangefinderPoint result = rhs;
  result.position = lhs * rhs.position;
  return result;
}

inline bool operator==(const RangefinderPoint& lhs,
                       const RangefinderPoint& rhs) {
  return lhs.position == rhs.position;
}

inline bool operator==(const TimedRangefinderPoint& lhs,
                       const TimedRangefinderPoint& rhs) {
  return lhs.position == rhs.position && lhs.time == rhs.time;
}

inline RangefinderPoint FromProto(
    const proto::RangefinderPoint& rangefinder_point_proto) {
  return {transform::ToEigen(rangefinder_point_proto.position())};
}

inline proto::RangefinderPoint ToProto(
    const RangefinderPoint& rangefinder_point) {
  proto::RangefinderPoint proto;
  *proto.mutable_position() = transform::ToProto(rangefinder_point.position);
  return proto;
}

inline TimedRangefinderPoint FromProto(
    const proto::TimedRangefinderPoint& timed_rangefinder_point_proto) {
  return {transform::ToEigen(timed_rangefinder_point_proto.position()),
          timed_rangefinder_point_proto.time()};
}

inline proto::TimedRangefinderPoint ToProto(
    const TimedRangefinderPoint& timed_rangefinder_point) {
  proto::TimedRangefinderPoint proto;
  *proto.mutable_position() =
      transform::ToProto(timed_rangefinder_point.position);
  proto.set_time(timed_rangefinder_point.time);
  return proto;
}

inline RangefinderPoint ToRangefinderPoint(
    const TimedRangefinderPoint& timed_rangefinder_point) {
  return {timed_rangefinder_point.position};
}

inline TimedRangefinderPoint ToTimedRangefinderPoint(
    const RangefinderPoint& rangefinder_point, const float time) {
  return {rangefinder_point.position, time};
}

}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER_SENSOR_RANGEFINDER_POINT_H_
