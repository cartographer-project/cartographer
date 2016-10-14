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

#include "cartographer/mapping_2d/submaps.h"

#include <map>
#include <string>

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/lua_parameter_dictionary_test_helpers.h"
#include "cartographer/common/port.h"
#include "cartographer/transform/transform.h"
#include "gmock/gmock.h"

namespace cartographer {
namespace mapping_2d {
namespace {

TEST(SubmapsTest, TheRightNumberOfScansAreInserted) {
  constexpr int kNumLaserFans = 10;
  auto parameter_dictionary = common::MakeDictionary(
      "return {"
      "resolution = 0.05, "
      "half_length = 10., "
      "num_laser_fans = " +
      std::to_string(kNumLaserFans) +
      ", "
      "output_debug_images = false, "
      "laser_fan_inserter = {"
      "insert_free_space = true, "
      "hit_probability = 0.53, "
      "miss_probability = 0.495, "
      "},"
      "}");
  Submaps submaps{CreateSubmapsOptions(parameter_dictionary.get())};
  auto num_inserted = [&submaps](const int i) {
    return submaps.Get(i)->end_laser_fan_index -
           submaps.Get(i)->begin_laser_fan_index;
  };
  for (int i = 0; i != 1000; ++i) {
    submaps.InsertLaserFan({Eigen::Vector3f::Zero(), {}, {}});
    const int matching = submaps.matching_index();
    // Except for the first, maps should only be returned after enough scans.
    if (matching != 0) {
      EXPECT_LE(kNumLaserFans, num_inserted(matching));
    }
  }
  for (int i = 0; i != submaps.size() - 2; ++i) {
    // Submaps should not be left without the right number of scans in them.
    EXPECT_EQ(kNumLaserFans * 2, num_inserted(i));
    EXPECT_EQ(i * kNumLaserFans, submaps.Get(i)->begin_laser_fan_index);
    EXPECT_EQ((i + 2) * kNumLaserFans, submaps.Get(i)->end_laser_fan_index);
  }
}

}  // namespace
}  // namespace mapping_2d
}  // namespace cartographer
