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

#ifndef CARTOGRAPHER_SENSOR_LANDMARK_DATA_H_
#define CARTOGRAPHER_SENSOR_LANDMARK_DATA_H_

#include <string>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/common/port.h"
#include "cartographer/common/time.h"
#include "cartographer/sensor/proto/sensor.pb.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace sensor {

struct Landmark {
  std::string id;
  transform::Rigid3d transform;
  double translation_weight;
  double rotation_weight;
};

struct LandmarkData {
  common::Time time;
  std::vector<Landmark> landmarks;
};

// Converts 'landmark_data' to a proto::LandmarkData.
proto::LandmarkData ToProto(const LandmarkData& landmark_data);

// Converts 'proto' to an LandmarkData.
LandmarkData FromProto(const proto::LandmarkData& proto);

}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER_SENSOR_LANDMARK_DATA_H_
