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

#include "cartographer/mapping/pose_extrapolator.h"

#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

PoseExtrapolator::PoseExtrapolator(const common::Duration pose_queue_duration)
    : pose_queue_duration_(pose_queue_duration) {}

common::Time PoseExtrapolator::GetLastPoseTime() const {
  if (timed_pose_queue_.empty()) {
    return common::Time::min();
  }
  return timed_pose_queue_.back().time;
}

void PoseExtrapolator::AddPose(const common::Time time,
                               const transform::Rigid3d& pose) {
  timed_pose_queue_.push_back(TimedPose{time, pose});
  while (timed_pose_queue_.size() > 2 &&
         timed_pose_queue_[1].time <= time - pose_queue_duration_) {
    timed_pose_queue_.pop_front();
  }
}

transform::Rigid3d PoseExtrapolator::ExtrapolatePose(common::Time time) {
  CHECK(!timed_pose_queue_.empty());
  const TimedPose& newest_timed_pose = timed_pose_queue_.back();
  const auto newest_time = newest_timed_pose.time;
  const transform::Rigid3d& newest_pose = newest_timed_pose.pose;
  CHECK(time >= newest_time);
  if (timed_pose_queue_.size() == 1) {
    return newest_pose;
  }
  const TimedPose& oldest_timed_pose = timed_pose_queue_.front();
  const auto oldest_time = oldest_timed_pose.time;
  const transform::Rigid3d& oldest_pose = oldest_timed_pose.pose;
  const double queue_delta = common::ToSeconds(newest_time - oldest_time);
  if (queue_delta < 0.001) {  // 1 ms
    LOG(WARNING) << "Queue too short for extrapolation, returning most recent "
                    "pose. Queue duration: "
                 << queue_delta << " ms";
    return newest_pose;
  }
  const Eigen::Vector3d linear_velocity =
      (newest_pose.translation() - oldest_pose.translation()) / queue_delta;
  const Eigen::Vector3d angular_velocity =
      transform::RotationQuaternionToAngleAxisVector(
          oldest_pose.rotation().inverse() * newest_pose.rotation()) /
      queue_delta;
  const double extrapolation_delta = common::ToSeconds(time - newest_time);
  return transform::Rigid3d::Translation(extrapolation_delta *
                                         linear_velocity) *
         newest_pose *
         transform::Rigid3d::Rotation(
             transform::AngleAxisVectorToRotationQuaternion(
                 Eigen::Vector3d(extrapolation_delta * angular_velocity)));
}

}  // namespace mapping
}  // namespace cartographer
