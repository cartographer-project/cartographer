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

#include <chrono>
#include <thread>

#include "cartographer_grpc/framework/retry.h"
#include "glog/logging.h"

namespace cartographer_grpc {
namespace framework {

RetryStrategy CreateRetryStrategy(RetryIndicator retry_indicator,
                                  RetryDelayCalculator retry_delay_calculator) {
  return [retry_indicator, retry_delay_calculator](int failed_attempts) {
    if (!retry_indicator(failed_attempts)) {
      return optional<Duration>();
    }
    return optional<Duration>(retry_delay_calculator(failed_attempts));
  };
}

RetryIndicator CreateLimitedRetryIndicator(int max_attempts) {
  return [max_attempts](int failed_attempts) {
    return failed_attempts < max_attempts;
  };
}

RetryDelayCalculator CreateBackoffDelayCalculator(Duration min_delay,
                                                  float backoff_factor) {
  return [min_delay, backoff_factor](int failed_attempts) -> Duration {
    CHECK_GE(failed_attempts, 0);
    using cartographer::common::FromSeconds;
    using cartographer::common::ToSeconds;
    return FromSeconds(std::pow(backoff_factor, failed_attempts - 1) *
                       ToSeconds(min_delay));
  };
}

RetryStrategy CreateLimitedBackoffStrategy(Duration min_delay,
                                           float backoff_factor,
                                           int max_attempts) {
  return CreateRetryStrategy(
      CreateLimitedRetryIndicator(max_attempts),
      CreateBackoffDelayCalculator(min_delay, backoff_factor));
}

bool RetryWithStrategy(RetryStrategy retry_strategy, std::function<bool()> op,
                       std::function<void()> reset) {
  optional<Duration> delay;
  int failed_attemps = 0;
  for (;;) {
    if (op()) {
      return true;
    }
    if (!retry_strategy) {
      return false;
    }
    delay = retry_strategy(++failed_attemps);
    if (!delay.has_value()) {
      break;
    }
    LOG(INFO) << "Retrying after "
              << std::chrono::duration_cast<std::chrono::milliseconds>(
                     delay.value())
                     .count()
              << " milliseconds.";
    std::this_thread::sleep_for(delay.value());
    if (reset) {
      reset();
    }
  }
  return false;
}

}  // namespace framework
}  // namespace cartographer_grpc
