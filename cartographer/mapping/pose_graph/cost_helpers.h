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

#ifndef CARTOGRAPHER_MAPPING_POSE_GRAPH_COST_HELPERS_H_
#define CARTOGRAPHER_MAPPING_POSE_GRAPH_COST_HELPERS_H_

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"

namespace cartographer {
namespace mapping {
namespace pose_graph {

// Computes the error between the node-to-submap alignment 'zbar_ij' and the
// difference of submap pose 'c_i' and node pose 'c_j' which are both in an
// arbitrary common frame.
template <typename T>
static std::array<T, 3> ComputeUnscaledError2D(
    const transform::Rigid2d& zbar_ij, const T* const c_i, const T* const c_j);

// Computes the error between the node-to-submap alignment 'zbar_ij' and the
// difference of submap pose 'c_i' and node pose 'c_j' which are both in an
// arbitrary common frame.
template <typename T>
static std::array<T, 6> ComputeUnscaledError3D(
    const transform::Rigid3d& zbar_ij, const T* const c_i_rotation,
    const T* const c_i_translation, const T* const c_j_rotation,
    const T* const c_j_translation);

}  // namespace pose_graph
}  // namespace mapping
}  // namespace cartographer

#include "cartographer/mapping/pose_graph/cost_helpers_impl.h"

#endif  // CARTOGRAPHER_MAPPING_POSE_GRAPH_COST_HELPERS_H_
