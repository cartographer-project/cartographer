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

#ifndef CARTOGRAPHER_SENSOR_LANDMARK_H_
#define CARTOGRAPHER_SENSOR_LANDMARK_H_

#include "Eigen/Core"
#include "cartographer/common/port.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace sensor {

struct Landmark {
  string id;
  Eigen::Vector3d translation;
  double variance;
  double huber_scale;
};

struct OrientedLandmark {
  string id;
  transform::Rigid3d transform;
  double translation_variance;
  double rotation_variance;
  double huber_scale;
};

}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER_SENSOR_LANDMARK_H_
