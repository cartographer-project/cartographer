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

// Computes the error between the given relative pose and the difference of
// poses 'start' and 'end' which are both in an arbitrary common frame.
//
// 'start' and 'end' poses have the format [x, y, rotation].
template <typename T>
static std::array<T, 3> ComputeUnscaledError(
    const transform::Rigid2d& relative_pose, const T* const start,
    const T* const end);
template <typename T>
std::array<T, 3> ScaleError(std::array<T, 3> error, T translation_weight,
                            T rotation_weight);

// Computes the error between the given relative pose and the difference of
// poses 'start' and 'end' which are both in an arbitrary common frame.
//
// 'start' and 'end' translation has the format [x, y, z].
// 'start' and 'end' rotation are quaternions in the format [w, n_1, n_2, n_3].
template <typename T>
static std::array<T, 6> ComputeUnscaledError(
    const transform::Rigid3d& relative_pose, const T* const start_rotation,
    const T* const start_translation, const T* const end_rotation,
    const T* const end_translation);

template <typename T>
std::array<T, 6> ScaleError(std::array<T, 6> error, T translation_weight,
                            T rotation_weight);

// Computes spherical linear interpolation of unit quaternions.
//
// 'start' and 'end' are quaternions in the format [w, n_1, n_2, n_3].
template <typename T>
std::array<T, 4> SlerpQuaternions(const T* const start, const T* const end,
                                  T factor);
}  // namespace pose_graph
}  // namespace mapping
}  // namespace cartographer

#include "cartographer/mapping/pose_graph/cost_helpers_impl.h"

#endif  // CARTOGRAPHER_MAPPING_POSE_GRAPH_COST_HELPERS_H_
