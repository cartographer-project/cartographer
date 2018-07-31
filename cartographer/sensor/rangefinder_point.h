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

#ifndef CARTOGRAPHER_SENSOR_RANGEFINDER_POINT_H_
#define CARTOGRAPHER_SENSOR_RANGEFINDER_POINT_H_

#include <vector>

#include "Eigen/Core"
#include "cartographer/sensor/proto/sensor.pb.h"
#include "cartographer/transform/rigid_transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace sensor {

// Stores 3D position of a point observed by a rangefinder sensor.
class RangefinderPoint {
 public:
  RangefinderPoint(const Eigen::Vector3f& position) : position_(position){};
  RangefinderPoint(const proto::RangefinderPoint& rangefinder_point_proto);
  Eigen::Vector3f position() const { return position_; };

  template <class T>
  friend RangefinderPoint operator*(const transform::Rigid3<T>& lhs,
                                    const RangefinderPoint& rhs);
  RangefinderPoint RestrictDistanceFromOrigin(const Eigen::Vector3f& origin,
                                              float distance) const;

  proto::RangefinderPoint ToProto() const;

 protected:
  Eigen::Vector3f position_;
};

template <class T>
RangefinderPoint operator*(const transform::Rigid3<T>& lhs,
                           const RangefinderPoint& rhs) {
  RangefinderPoint result = rhs;
  result.position_ = lhs * rhs.position_;
  return result;
}

// Stores 3D position of a point with its relative measurement time.
// See point_cloud.h for more details.
class TimedRangefinderPoint : public RangefinderPoint {
 public:
  TimedRangefinderPoint(const Eigen::Vector3f& position, float time)
      : RangefinderPoint(position), time_(time){};
  TimedRangefinderPoint(const RangefinderPoint& rangefinder_point, float time)
      : RangefinderPoint(rangefinder_point), time_(time){};

  TimedRangefinderPoint(
      const proto::TimedRangefinderPoint& timed_rangefinder_point_proto);
  float time() const { return time_; };
  void set_time(const float& time) { time_ = time; }

  template <class T>
  friend TimedRangefinderPoint operator*(const transform::Rigid3<T>& lhs,
                                         const TimedRangefinderPoint& rhs);

  proto::TimedRangefinderPoint ToProto() const;

 protected:
  float time_;
};

template <class T>
TimedRangefinderPoint operator*(const transform::Rigid3<T>& lhs,
                                const TimedRangefinderPoint& rhs) {
  TimedRangefinderPoint result = rhs;
  result.position_ = lhs * rhs.position_;
  return result;
}

}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER_SENSOR_RANGEFINDER_POINT_H_
