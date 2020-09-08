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

#include <memory>

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/io/points_processor.h"

namespace cartographer {
namespace io {

// Tries to filter objects that follow the robot without
// removing too much of the scene by being more specific
// than the remove moving objects filter
class FollowerFiteringPointsProcessor : public PointsProcessor {
 public:
  constexpr static const char* kConfigurationFileActionName =
      "follower_filter";
  FollowerFiteringPointsProcessor(int yaw_range, double follow_distance_,
                                     PointsProcessor* next);
  static std::unique_ptr<FollowerFiteringPointsProcessor> FromDictionary(
      common::LuaParameterDictionary* dictionary, PointsProcessor* next);

  ~FollowerFiteringPointsProcessor() override {}

  FollowerFiteringPointsProcessor(
      const FollowerFiteringPointsProcessor&) = delete;
  FollowerFiteringPointsProcessor& operator=(
      const FollowerFiteringPointsProcessor&) = delete;

  void Process(std::unique_ptr<PointsBatch> batch) override;
  FlushResult Flush() override;

 private:
  const int yaw_range_;
  const double follow_distance_;
  PointsProcessor* const next_;
};

}  // namespace io
}  // namespace cartographer
