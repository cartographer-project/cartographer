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

#include "cartographer/mapping/probability_values.h"

#include "gtest/gtest.h"

namespace cartographer {
namespace mapping {
namespace {

TEST(ProbabilityValuesTest, OddsConversions) {
  EXPECT_NEAR(ProbabilityFromOdds(Odds(kMinProbability)), kMinProbability,
              1e-6);
  EXPECT_NEAR(ProbabilityFromOdds(Odds(kMaxProbability)), kMaxProbability,
              1e-6);
  EXPECT_NEAR(ProbabilityFromOdds(Odds(0.5)), 0.5, 1e-6);
}

}  // namespace
}  // namespace mapping
}  // namespace cartographer
