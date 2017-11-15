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

#include "cartographer/ground_truth/relations_text_file.h"

#include <fstream>

#include "cartographer/common/time.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace ground_truth {

namespace {

common::Time UnixToCommonTime(double unix_time) {
  constexpr int64 kUtsTicksPerSecond = 10000000;
  return common::FromUniversal(common::kUtsEpochOffsetFromUnixEpochInSeconds *
                               kUtsTicksPerSecond) +
         common::FromSeconds(unix_time);
}

}  // namespace

proto::GroundTruth ReadRelationsTextFile(
    const std::string& relations_filename) {
  proto::GroundTruth ground_truth;
  std::ifstream relations_stream(relations_filename.c_str());
  double unix_time_1, unix_time_2, x, y, z, roll, pitch, yaw;
  while (relations_stream >> unix_time_1 >> unix_time_2 >> x >> y >> z >>
         roll >> pitch >> yaw) {
    const common::Time common_time_1 = UnixToCommonTime(unix_time_1);
    const common::Time common_time_2 = UnixToCommonTime(unix_time_2);
    const transform::Rigid3d expected =
        transform::Rigid3d(transform::Rigid3d::Vector(x, y, z),
                           transform::RollPitchYaw(roll, pitch, yaw));
    auto* const new_relation = ground_truth.add_relation();
    new_relation->set_timestamp1(common::ToUniversal(common_time_1));
    new_relation->set_timestamp2(common::ToUniversal(common_time_2));
    *new_relation->mutable_expected() = transform::ToProto(expected);
  }
  CHECK(relations_stream.eof());
  return ground_truth;
}

}  // namespace ground_truth
}  // namespace cartographer
