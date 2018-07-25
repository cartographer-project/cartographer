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

#include "cartographer/pose_graph/node/imu_calibration.h"

namespace cartographer {
namespace pose_graph {

ImuCalibration::ImuCalibration(const NodeId& node_id, bool constant,
                               const proto::ImuCalibration& imu_calibration)
    : Node(node_id, constant),
      gravity_constant_(imu_calibration.gravity_constant()),
      orientation_{{imu_calibration.orientation().x(),
                    imu_calibration.orientation().y(),
                    imu_calibration.orientation().z(),
                    imu_calibration.orientation().w()}} {}

proto::Parameters ImuCalibration::ToParametersProto() const {
  proto::Parameters parameters;
  auto* imu_calibration = parameters.mutable_imu_calibration();

  imu_calibration->set_gravity_constant(gravity_constant_);

  // TODO(pifon): Use a common method to convert from Eigen::Quaterniond to
  // proto. Probably, the one defined in transform.h.
  auto* orientation = imu_calibration->mutable_orientation();
  orientation->set_x(orientation_[0]);
  orientation->set_y(orientation_[1]);
  orientation->set_z(orientation_[2]);
  orientation->set_w(orientation_[3]);

  return parameters;
}

}  // namespace pose_graph
}  // namespace cartographer
