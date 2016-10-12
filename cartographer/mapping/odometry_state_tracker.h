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

#ifndef CARTOGRAPHER_MAPPING_ODOMETRY_STATE_TRACKER_H_
#define CARTOGRAPHER_MAPPING_ODOMETRY_STATE_TRACKER_H_

#include <deque>

#include "cartographer/common/time.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace mapping {

struct OdometryState {
  OdometryState(common::Time time, const transform::Rigid3d& odometer_pose,
                const transform::Rigid3d& state_pose);
  OdometryState() {}

  common::Time time = common::Time::min();
  transform::Rigid3d odometer_pose = transform::Rigid3d::Identity();
  transform::Rigid3d state_pose = transform::Rigid3d::Identity();
};

// Keeps track of the odometry states by keeping sliding window over some
// number of them.
class OdometryStateTracker {
 public:
  using OdometryStates = std::deque<OdometryState>;

  explicit OdometryStateTracker(int window_size);

  const OdometryStates& odometry_states() const;

  // Adds a new 'odometry_state' and makes sure the maximum number of previous
  // odometry states is not exceeded.
  void AddOdometryState(const OdometryState& odometry_state);

  // Returns true if no elements are present in the odometry queue.
  bool empty() const;

  // Retrieves the most recent OdometryState. Must not be called when empty.
  const OdometryState& newest() const;

 private:
  OdometryStates odometry_states_;
  size_t window_size_;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_ODOMETRY_STATE_TRACKER_H_
