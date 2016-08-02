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

#include "time_conversion.h"

#include <chrono>

#include "cartographer/common/time.h"
#include "gtest/gtest.h"
#include "ros/ros.h"

namespace cartographer_ros {

namespace {

TEST(TimeConversion, testToRos) {
  std::vector<int64> values = {0, 1469091375, 1466481821, 1462101382,
                               1468238899};
  for (int64 seconds_since_epoch : values) {
    ::ros::Time ros_now;
    ros_now.fromSec(seconds_since_epoch);
    ::cartographer::common::Time cartographer_now(
        ::cartographer::common::FromSeconds(
            seconds_since_epoch + kUtsEpochOffsetFromUnixEpochInSeconds));
    EXPECT_EQ(cartographer_now, ::cartographer_ros::FromRos(ros_now));
    EXPECT_EQ(ros_now, ::cartographer_ros::ToRos(cartographer_now));
  }
}

}  // namespace
}  // namespace cartographer_ros
