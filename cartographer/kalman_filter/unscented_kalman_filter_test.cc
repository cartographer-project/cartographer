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

#include "cartographer/kalman_filter/unscented_kalman_filter.h"

#include "cartographer/kalman_filter/gaussian_distribution.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace kalman_filter {
namespace {

Eigen::Matrix<double, 1, 1> Scalar(double value) {
  return value * Eigen::Matrix<double, 1, 1>::Identity();
}

// A simple model that copies the first to the second state variable.
Eigen::Matrix<double, 2, 1> g(const Eigen::Matrix<double, 2, 1>& state) {
  Eigen::Matrix<double, 2, 1> new_state;
  new_state << state[0], state[0];
  return new_state;
}

// A simple observation of the first state variable.
Eigen::Matrix<double, 1, 1> h(const Eigen::Matrix<double, 2, 1>& state) {
  return Scalar(state[0]) - Scalar(5.);
}

TEST(KalmanFilterTest, testConstructor) {
  UnscentedKalmanFilter<double, 2> filter(GaussianDistribution<double, 2>(
      Eigen::Vector2d(0., 42.), 10. * Eigen::Matrix2d::Identity()));
  EXPECT_NEAR(42., filter.GetBelief().GetMean()[1], 1e-9);
}

TEST(KalmanFilterTest, testPredict) {
  UnscentedKalmanFilter<double, 2> filter(GaussianDistribution<double, 2>(
      Eigen::Vector2d(42., 0.), 10. * Eigen::Matrix2d::Identity()));
  filter.Predict(
      g, GaussianDistribution<double, 2>(Eigen::Vector2d(0., 0.),
                                         Eigen::Matrix2d::Identity() * 1e-9));
  EXPECT_NEAR(filter.GetBelief().GetMean()[0], 42., 1e-2);
  EXPECT_NEAR(filter.GetBelief().GetMean()[1], 42., 1e-2);
}

TEST(KalmanFilterTest, testObserve) {
  UnscentedKalmanFilter<double, 2> filter(GaussianDistribution<double, 2>(
      Eigen::Vector2d(0., 42.), 10. * Eigen::Matrix2d::Identity()));
  for (int i = 0; i < 500; ++i) {
    filter.Predict(
        g, GaussianDistribution<double, 2>(Eigen::Vector2d(0., 0.),
                                           Eigen::Matrix2d::Identity() * 1e-9));
    filter.Observe<1>(
        h, GaussianDistribution<double, 1>(Scalar(0.), Scalar(1e-2)));
  }
  EXPECT_NEAR(filter.GetBelief().GetMean()[0], 5, 1e-2);
  EXPECT_NEAR(filter.GetBelief().GetMean()[1], 5, 1e-2);
}

}  // namespace
}  // namespace kalman_filter
}  // namespace cartographer
