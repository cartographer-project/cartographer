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

#include "cartographer/mapping/submaps.h"

#include <cmath>

#include "gtest/gtest.h"

namespace cartographer {
namespace mapping {
namespace {

// Converts the given log odds to a probability. This function is known to be
// very slow, because expf is incredibly slow.
inline float Expit(float log_odds) {
  const float exp_log_odds = std::exp(log_odds);
  return exp_log_odds / (1.f + exp_log_odds);
}

TEST(SubmapsTest, LogOddsConversions) {
  EXPECT_NEAR(Expit(Logit(kMinProbability)), kMinProbability, 1e-6);
  EXPECT_NEAR(Expit(Logit(kMaxProbability)), kMaxProbability, 1e-6);
  EXPECT_NEAR(Expit(Logit(0.5)), 0.5, 1e-6);
}

}  // namespace
}  // namespace mapping
}  // namespace cartographer
