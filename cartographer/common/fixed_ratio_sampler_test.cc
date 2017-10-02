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

#include "cartographer/common/fixed_ratio_sampler.h"

#include "gtest/gtest.h"

namespace cartographer {
namespace common {
namespace {

TEST(FixedRatioSamplerTest, AlwaysTrue) {
  FixedRatioSampler fixed_ratio_sampler(1.);
  for (int i = 0; i < 100; ++i) {
    EXPECT_TRUE(fixed_ratio_sampler.Pulse());
  }
}

TEST(FixedRatioSamplerTest, NonSensicalRatio) {
  EXPECT_DEATH(FixedRatioSampler(0.), "ratio");
  EXPECT_DEATH(FixedRatioSampler(2.), "ratio");
  EXPECT_DEATH(FixedRatioSampler(-0.1), "ratio");
}

TEST(FixedRatioSamplerTest, SometimesTrue) {
  FixedRatioSampler fixed_ratio_sampler(0.5);
  for (int i = 0; i < 100; ++i) {
    EXPECT_EQ(i % 2 == 0, fixed_ratio_sampler.Pulse());
  }
}

TEST(FixedRatioSamplerTest, FirstPulseIsTrue) {
  // Choose a very very small positive number for the ratio.
  FixedRatioSampler fixed_ratio_sampler(1e-20);
  EXPECT_TRUE(fixed_ratio_sampler.Pulse());
  for (int i = 0; i < 100; ++i) {
    EXPECT_FALSE(fixed_ratio_sampler.Pulse());
  }
}

}  // namespace
}  // namespace common
}  // namespace cartographer
