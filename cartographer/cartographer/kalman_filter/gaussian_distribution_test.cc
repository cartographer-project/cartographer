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

#include "cartographer/kalman_filter/gaussian_distribution.h"

#include "gtest/gtest.h"

namespace cartographer {
namespace kalman_filter {
namespace {

TEST(GaussianDistributionTest, testConstructor) {
  Eigen::Matrix2d covariance;
  covariance << 1., 2., 3., 4.;
  GaussianDistribution<double, 2> distribution(Eigen::Vector2d(0., 1.),
                                               covariance);
  EXPECT_NEAR(0., distribution.GetMean()[0], 1e-9);
  EXPECT_NEAR(1., distribution.GetMean()[1], 1e-9);
  EXPECT_NEAR(2., distribution.GetCovariance()(0, 1), 1e-9);
}

}  // namespace
}  // namespace kalman_filter
}  // namespace cartographer
