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

#ifndef CARTOGRAPHER_MAPPING_COLLATED_TRAJECTORY_BUILDER_H_
#define CARTOGRAPHER_MAPPING_COLLATED_TRAJECTORY_BUILDER_H_

#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <unordered_set>

#include "cartographer/common/port.h"
#include "cartographer/common/rate_timer.h"
#include "cartographer/mapping/global_trajectory_builder_interface.h"
#include "cartographer/mapping/submaps.h"
#include "cartographer/mapping/trajectory_builder.h"
#include "cartographer/sensor/collator.h"
#include "cartographer/sensor/data.h"

namespace cartographer {
namespace mapping {

// Handles collating sensor data using a sensor::Collator, then passing it on to
// a mapping::GlobalTrajectoryBuilderInterface which is common for 2D and 3D.
class CollatedTrajectoryBuilder : public TrajectoryBuilder {
 public:
  CollatedTrajectoryBuilder(
      sensor::Collator* sensor_collator, int trajectory_id,
      const std::unordered_set<std::string>& expected_sensor_ids,
      std::unique_ptr<GlobalTrajectoryBuilderInterface>
          wrapped_trajectory_builder);
  ~CollatedTrajectoryBuilder() override;

  CollatedTrajectoryBuilder(const CollatedTrajectoryBuilder&) = delete;
  CollatedTrajectoryBuilder& operator=(const CollatedTrajectoryBuilder&) =
      delete;

  void AddSensorData(const std::string& sensor_id,
                     std::unique_ptr<sensor::Data> data) override;

 private:
  void HandleCollatedSensorData(const std::string& sensor_id,
                                std::unique_ptr<sensor::Data> data);

  sensor::Collator* const sensor_collator_;
  const int trajectory_id_;
  std::unique_ptr<GlobalTrajectoryBuilderInterface> wrapped_trajectory_builder_;

  // Time at which we last logged the rates of incoming sensor data.
  std::chrono::steady_clock::time_point last_logging_time_;
  std::map<std::string, common::RateTimer<>> rate_timers_;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_COLLATED_TRAJECTORY_BUILDER_H_
