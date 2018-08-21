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

#ifndef CARTOGRAPHER_POSE_GRAPH_NODE_IMU_CALIBRATION_H_
#define CARTOGRAPHER_POSE_GRAPH_NODE_IMU_CALIBRATION_H_

#include "cartographer/pose_graph/node/node.h"

#include <array>

#include "cartographer/pose_graph/node/parameterization/parameterization.h"
#include "cartographer/transform/transform.h"

namespace cartographer {
namespace pose_graph {

class ImuCalibration : public Node {
 public:
  ImuCalibration(const NodeId& node_id, bool constant,
                 const proto::ImuCalibration& imu_calibration);

  double* mutable_gravity_constant() { return &gravity_constant_; }
  double gravity_constant() const { return gravity_constant_; }

  std::array<double, 4>* mutable_orientation() { return &orientation_; }
  const std::array<double, 4>& orientation() const { return orientation_; }

  ceres::LocalParameterization* orientation_parameterization() const {
    return orientation_parameterization_.ceres_parameterization();
  }

 protected:
  proto::Parameters ToParametersProto() const final;

 private:
  double gravity_constant_;
  std::array<double, 4> orientation_;
  Parameterization orientation_parameterization_;
};

}  // namespace pose_graph
}  // namespace cartographer

#endif  // CARTOGRAPHER_POSE_GRAPH_NODE_IMU_CALIBRATION_H_
