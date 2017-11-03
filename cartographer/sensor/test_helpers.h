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

#ifndef CARTOGRAPHER_SENSOR_TEST_HELPERS_H_
#define CARTOGRAPHER_SENSOR_TEST_HELPERS_H_

#include <string>
#include <tuple>

#include "gmock/gmock.h"

namespace cartographer {
namespace sensor {

MATCHER_P(Near, point, std::string(negation ? "Doesn't" : "Does") + " match.") {
  return arg.isApprox(point, 0.001f);
}

}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER_SENSOR_TEST_HELPERS_H_
