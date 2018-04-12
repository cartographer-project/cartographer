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

#include "cartographer/common/rate_timer.h"

#include "gtest/gtest.h"

namespace cartographer {
namespace common {
namespace {

TEST(RateTimerTest, ComputeRate) {
  RateTimer<> rate_timer(common::FromSeconds(1.));
  common::Time time = common::FromUniversal(42);
  for (int i = 0; i < 100; ++i) {
    rate_timer.Pulse(time);
    time += common::FromSeconds(0.1);
  }
  EXPECT_NEAR(10., rate_timer.ComputeRate(), 1e-3);
}

struct SimulatedClock {
  using rep = std::chrono::steady_clock::rep;
  using period = std::chrono::steady_clock::period;
  using duration = std::chrono::steady_clock::duration;
  using time_point = std::chrono::steady_clock::time_point;

  static time_point time;
  static time_point now() noexcept { return time; }
};

SimulatedClock::time_point SimulatedClock::time;

TEST(RateTimerTest, ComputeWallTimeRateRatio) {
  common::Time time = common::FromUniversal(42);
  RateTimer<SimulatedClock> rate_timer(common::FromSeconds(1.));
  for (int i = 0; i < 100; ++i) {
    rate_timer.Pulse(time);
    time += common::FromSeconds(0.1);
    SimulatedClock::time +=
        std::chrono::duration_cast<SimulatedClock::duration>(
            std::chrono::duration<double>(0.05));
  }
  EXPECT_NEAR(2., rate_timer.ComputeWallTimeRateRatio(), 1e-3);
}

}  // namespace
}  // namespace common
}  // namespace cartographer
