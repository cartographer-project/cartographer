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

#ifndef VIAM_CARTOGRAPHER_IO_DRAW_TRAJECTORIES_H_
#define VIAM_CARTOGRAPHER_IO_DRAW_TRAJECTORIES_H_

#include "cairo/cairo.h"
#include "cartographer/io/image.h"
#include "cartographer/io/color.h"
#include "cartographer/mapping/proto/trajectory.pb.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/mapping/trajectory_node.h"
#include "cartographer/mapping/id.h"

#include "viam/src/io/color.h"

namespace viam {
namespace io {

using PoseToPixelFunction = std::function<Eigen::Array2i(const cartographer::transform::Rigid3d& pose)>;

// Draws the 'trajectory' with the given 'color' onto 'surface'. Function must translate a trajectory node's position into the
// pixel on 'surface'.
cartographer::io::UniqueCairoSurfacePtr DrawTrajectoryNodes(
    const cartographer::mapping::MapById<cartographer::mapping::NodeId,
                                         cartographer::mapping::TrajectoryNode>& trajectory_nodes_poses,
    float resolution,
    cartographer::transform::Rigid3d slice_pose,
    cairo_surface_t* surface);

}  // namespace io
}  // namespace viam

#endif  // VIAM_CARTOGRAPHER_IO_DRAW_TRAJECTORIES_H_
