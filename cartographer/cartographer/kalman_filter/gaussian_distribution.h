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

#ifndef CARTOGRAPHER_KALMAN_FILTER_GAUSSIAN_DISTRIBUTION_H_
#define CARTOGRAPHER_KALMAN_FILTER_GAUSSIAN_DISTRIBUTION_H_

#include "Eigen/Cholesky"
#include "Eigen/Core"
#include "Eigen/Geometry"

namespace cartographer {
namespace kalman_filter {

template <typename T, int N>
class GaussianDistribution {
 public:
  GaussianDistribution(const Eigen::Matrix<T, N, 1>& mean,
                       const Eigen::Matrix<T, N, N>& covariance)
      : mean_(mean), covariance_(covariance) {}

  const Eigen::Matrix<T, N, 1>& GetMean() const { return mean_; }

  const Eigen::Matrix<T, N, N>& GetCovariance() const { return covariance_; }

 private:
  Eigen::Matrix<T, N, 1> mean_;
  Eigen::Matrix<T, N, N> covariance_;
};

template <typename T, int N>
GaussianDistribution<T, N> operator+(const GaussianDistribution<T, N>& lhs,
                                     const GaussianDistribution<T, N>& rhs) {
  return GaussianDistribution<T, N>(lhs.GetMean() + rhs.GetMean(),
                                    lhs.GetCovariance() + rhs.GetCovariance());
}

template <typename T, int N, int M>
GaussianDistribution<T, N> operator*(const Eigen::Matrix<T, N, M>& lhs,
                                     const GaussianDistribution<T, M>& rhs) {
  return GaussianDistribution<T, N>(
      lhs * rhs.GetMean(), lhs * rhs.GetCovariance() * lhs.transpose());
}

}  // namespace kalman_filter
}  // namespace cartographer

#endif  // CARTOGRAPHER_KALMAN_FILTER_GAUSSIAN_DISTRIBUTION_H_
