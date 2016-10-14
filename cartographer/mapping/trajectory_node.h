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

#ifndef CARTOGRAPHER_MAPPING_TRAJECTORY_NODE_H_
#define CARTOGRAPHER_MAPPING_TRAJECTORY_NODE_H_

#include <deque>
#include <vector>

#include "Eigen/Core"
#include "cartographer/common/time.h"
#include "cartographer/sensor/laser.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace mapping {

struct Submaps;

struct TrajectoryNode {
  struct ConstantData {
    common::Time time;

    // LaserFan in 'pose' frame. Only used in the 2D case.
    sensor::LaserFan3D laser_fan_2d;

    // LaserFan in 'pose' frame. Only used in the 3D case.
    sensor::CompressedLaserFan3D laser_fan_3d;

    // Trajectory this node belongs to.
    // TODO(jmason): The naming here is confusing because 'trajectory' doesn't
    // seem like a good name for a Submaps*. Sort this out.
    const Submaps* trajectory;

    // Transform from the 3D 'tracking' frame to the 'pose' frame of the
    // laser, which contains roll, pitch and height for 2D. In 3D this is
    // always identity.
    transform::Rigid3d tracking_to_pose;
  };

  common::Time time() const { return constant_data->time; }

  const ConstantData* constant_data;

  transform::Rigid3d pose;
};

// Users will only be interested in 'trajectory_nodes'. But 'constant_data'
// is referenced by 'trajectory_nodes'. This struct guarantees that their
// lifetimes are bound.
struct TrajectoryNodes {
  std::deque<mapping::TrajectoryNode::ConstantData> constant_data;
  std::vector<mapping::TrajectoryNode> trajectory_nodes;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_TRAJECTORY_NODE_H_
