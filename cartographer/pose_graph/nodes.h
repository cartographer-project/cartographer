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

#ifndef CARTOGRAPHER_POSE_GRAPH_NODES_H_
#define CARTOGRAPHER_POSE_GRAPH_NODES_H_

#include "cartographer/pose_graph/imu_calibration.h"
#include "cartographer/pose_graph/pose_2d.h"
#include "cartographer/pose_graph/pose_3d.h"

#include <map>

namespace cartographer {
namespace pose_graph {

struct Nodes {
  // TODO(pifon): Should it really be an std::map or smth else?
  std::map<NodeId, Pose2D> pose_2d_nodes;
  std::map<NodeId, Pose3D> pose_3d_nodes;
  std::map<NodeId, Pose3D> imu_calibration_nodes;
};

}  // namespace pose_graph
}  // namespace cartographer

#endif  // CARTOGRAPHER_POSE_GRAPH_NODES_H_
