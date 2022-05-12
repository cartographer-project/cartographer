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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_3D_ROTATION_PARAMETERIZATION_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_3D_ROTATION_PARAMETERIZATION_H_

#include "cartographer/common/math.h"
#include "ceres/jet.h"
#include "ceres/rotation.h"

namespace cartographer {
namespace mapping {

// Provides operations used to create a Ceres Manifold with a 4-D ambient
// space and a 1-D tangent space that represents a yaw rotation only.
struct YawOnlyQuaternionOperations {
  template <typename T>
  bool Plus(const T* x, const T* delta, T* x_plus_delta) const {
    const T clamped_delta = common::Clamp(delta[0], T(-0.5), T(0.5));
    T q_delta[4];
    q_delta[0] = ceres::sqrt(1. - clamped_delta * clamped_delta);
    q_delta[1] = T(0.);
    q_delta[2] = T(0.);
    q_delta[3] = clamped_delta;
    ceres::QuaternionProduct(q_delta, x, x_plus_delta);
    return true;
  }
  template <typename T>
  bool Minus(const T* y, const T* x, T* y_minus_x) const {
    T minus_x[4] = {x[0], -x[1], -x[2], -x[3]};
    T q_delta[4];
    ceres::QuaternionProduct(y, minus_x, q_delta);
    y_minus_x[0] = q_delta[3];
    return true;
  }
};

// Provides operations used to create a Ceres Manifold with a 4-D ambient
// space and a 2-D tangent space that represents a rotation only in pitch and
// roll, but no yaw.
struct ConstantYawQuaternionOperations {
  template <typename T>
  bool Plus(const T* x, const T* delta, T* x_plus_delta) const {
    const T delta_norm =
        ceres::sqrt(common::Pow2(delta[0]) + common::Pow2(delta[1]));
    const T sin_delta_over_delta =
        delta_norm < 1e-6 ? T(1.) : ceres::sin(delta_norm) / delta_norm;
    T q_delta[4];
    q_delta[0] = delta_norm < 1e-6 ? T(1.) : ceres::cos(delta_norm);
    q_delta[1] = sin_delta_over_delta * delta[0];
    q_delta[2] = sin_delta_over_delta * delta[1];
    q_delta[3] = T(0.);
    // We apply the 'delta' which is interpreted as an angle-axis rotation
    // vector in the xy-plane of the submap frame. This way we can align to
    // gravity because rotations around the z-axis in the submap frame do not
    // change gravity alignment, while disallowing random rotations of the map
    // that have nothing to do with gravity alignment (i.e. we disallow steps
    // just changing "yaw" of the complete map).
    ceres::QuaternionProduct(x, q_delta, x_plus_delta);
    return true;
  }
  template <typename T>
  bool Minus(const T* y, const T* x, T* y_minus_x) const {
    T minus_x[4] = {x[0], -x[1], -x[2], -x[3]};
    T q_delta[4];
    ceres::QuaternionProduct(minus_x, y, q_delta);
    const T& cos_delta_norm = q_delta[0];
    const T sin_delta_norm =
        ceres::sqrt(common::Pow2(q_delta[1]) + common::Pow2(q_delta[2]));
    const T delta_norm = atan2(sin_delta_norm, cos_delta_norm);
    const T delta_over_sin_delta =
        delta_norm < 1e-6 ? T(1.) : delta_norm / sin_delta_norm;
    y_minus_x[0] = q_delta[1] * delta_over_sin_delta;
    y_minus_x[1] = q_delta[2] * delta_over_sin_delta;
    return true;
  }
};

// Type aliases used to create manifolds.
using YawOnlyQuaternionManifold =
    ceres::AutoDiffManifold<YawOnlyQuaternionOperations, 4, 1>;
using ConstantYawQuaternionManifold =
    ceres::AutoDiffManifold<ConstantYawQuaternionOperations, 4, 2>;

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_3D_ROTATION_PARAMETERIZATION_H_
