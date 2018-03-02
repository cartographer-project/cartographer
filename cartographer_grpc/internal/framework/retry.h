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

#ifndef CARTOGRAPHER_GRPC_INTERNAL_FRAMEWORK_RETRY_H
#define CARTOGRAPHER_GRPC_INTERNAL_FRAMEWORK_RETRY_H

#include "cartographer/common/optional.h"
#include "cartographer/common/time.h"
#include "grpc++/grpc++.h"

namespace cartographer_grpc {
namespace framework {

using cartographer::common::Duration;
using cartographer::common::optional;

using RetryStrategy =
    std::function<optional<Duration>(int /* failed_attempts */)>;
using RetryIndicator = std::function<bool(int /* failed_attempts */)>;
using RetryDelayCalculator = std::function<Duration(int /* failed_attempts */)>;

RetryStrategy CreateRetryStrategy(RetryIndicator retry_indicator,
                                  RetryDelayCalculator retry_delay_calculator);

RetryIndicator CreateLimitedRetryIndicator(int max_attempts);
RetryDelayCalculator CreateBackoffDelayCalculator(Duration min_delay,
                                                  float backoff_factor);
RetryStrategy CreateLimitedBackoffStrategy(Duration min_delay,
                                           float backoff_factor,
                                           int max_attempts);

bool RetryWithStrategy(RetryStrategy retry_strategy, std::function<bool()> op,
                       std::function<void()> reset = nullptr);

}  // namespace framework
}  // namespace cartographer_grpc

#endif  // CARTOGRAPHER_GRPC_INTERNAL_FRAMEWORK_RETRY_H
