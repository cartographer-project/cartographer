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

#ifndef CARTOGRAPHER_COMMON_MATH_H_
#define CARTOGRAPHER_COMMON_MATH_H_

#include <cmath>
#include <vector>

#include "Eigen/Core"
#include "cartographer/common/port.h"
#include "ceres/ceres.h"

namespace cartographer {
namespace common {

// Clamps 'value' to be in the range ['min', 'max'].
template <typename T>
T Clamp(const T value, const T min, const T max) {
  if (value > max) {
    return max;
  }
  if (value < min) {
    return min;
  }
  return value;
}

// Calculates 'base'^'exponent'.
template <typename T>
constexpr T Power(T base, int exponent) {
  return (exponent != 0) ? base * Power(base, exponent - 1) : T(1);
}

// Calculates a^2.
template <typename T>
constexpr T Pow2(T a) {
  return Power(a, 2);
}

// Calculates the real part of the square root of 'a'. This is helpful when
// rounding errors generate a small negative argument. Otherwise std::sqrt
// returns NaN if its argument is negative.
template <typename T>
constexpr T RealSqrt(T a) {
  return sqrt(std::max(T(0.), a));
}

// Converts from degrees to radians.
constexpr double DegToRad(double deg) { return M_PI * deg / 180.; }

// Converts form radians to degrees.
constexpr double RadToDeg(double rad) { return 180. * rad / M_PI; }

// Bring the 'difference' between two angles into [-pi; pi].
template <typename T>
T NormalizeAngleDifference(T difference) {
  while (difference > M_PI) {
    difference -= T(2. * M_PI);
  }
  while (difference < -M_PI) {
    difference += T(2. * M_PI);
  }
  return difference;
}

template <typename T>
T atan2(const Eigen::Matrix<T, 2, 1>& vector) {
  return ceres::atan2(vector.y(), vector.x());
}

// Computes 'A'^{-1/2} for A being symmetric, positive-semidefinite.
// Eigenvalues of 'A' are clamped to be at least 'lower_eigenvalue_bound'.
template <int N>
Eigen::Matrix<double, N, N> ComputeSpdMatrixSqrtInverse(
    const Eigen::Matrix<double, N, N>& A, const double lower_eigenvalue_bound) {
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, N, N>>
      covariance_eigen_solver(A);
  if (covariance_eigen_solver.info() != Eigen::Success) {
    LOG(WARNING) << "SelfAdjointEigenSolver failed; A =\n" << A;
    return Eigen::Matrix<double, N, N>::Identity();
  }
  // Since we compute the inverse, we do not allow smaller values to avoid
  // infinity and NaN.
  const double relative_lower_bound = lower_eigenvalue_bound;
  return covariance_eigen_solver.eigenvectors() *
         covariance_eigen_solver.eigenvalues()
             .cwiseMax(relative_lower_bound)
             .cwiseInverse()
             .cwiseSqrt()
             .asDiagonal() *
         covariance_eigen_solver.eigenvectors().inverse();
}

}  // namespace common
}  // namespace cartographer

#endif  // CARTOGRAPHER_COMMON_MATH_H_
