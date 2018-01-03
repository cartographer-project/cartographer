/*
 * Copyright 2017 The Cartographer Authors
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

#include "cartographer/internal/mapping/test_helpers.h"

#include "cartographer/common/config.h"
#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/common/lua_parameter_dictionary_test_helpers.h"
#include "cartographer/common/make_unique.h"
#include "cartographer/sensor/timed_point_cloud_data.h"

namespace cartographer {
namespace mapping {
namespace test {

std::unique_ptr<::cartographer::common::LuaParameterDictionary>
ResolveLuaParameters(const std::string& lua_code) {
  auto file_resolver = ::cartographer::common::make_unique<
      ::cartographer::common::ConfigurationFileResolver>(
      std::vector<std::string>{
          std::string(::cartographer::common::kSourceDirectory) +
          "/configuration_files"});
  return common::make_unique<::cartographer::common::LuaParameterDictionary>(
      lua_code, std::move(file_resolver));
}

std::vector<cartographer::sensor::TimedPointCloudData>
GenerateFakeRangeMeasurements(double travel_distance, double duration,
                              double time_step) {
  std::vector<cartographer::sensor::TimedPointCloudData> measurements;
  cartographer::sensor::TimedPointCloud point_cloud;
  for (double angle = 0.; angle < M_PI; angle += 0.01) {
    constexpr double kRadius = 5;
    point_cloud.emplace_back(kRadius * std::cos(angle),
                             kRadius * std::sin(angle), 0., 0.);
  }
  const Eigen::Vector3f kDirection = Eigen::Vector3f(2., 1., 0.).normalized();
  const Eigen::Vector3f kVelocity = travel_distance / duration * kDirection;
  for (double elapsed_time = 0.; elapsed_time < duration;
       elapsed_time += time_step) {
    cartographer::common::Time time =
        cartographer::common::FromUniversal(123) +
        cartographer::common::FromSeconds(elapsed_time);
    cartographer::transform::Rigid3f pose =
        cartographer::transform::Rigid3f::Translation(elapsed_time * kVelocity);
    cartographer::sensor::TimedPointCloud ranges =
        cartographer::sensor::TransformTimedPointCloud(point_cloud,
                                                       pose.inverse());
    measurements.emplace_back(cartographer::sensor::TimedPointCloudData{
        time, Eigen::Vector3f::Zero(), ranges});
  }
  return measurements;
}

}  // namespace test
}  // namespace mapping
}  // namespace cartographer
