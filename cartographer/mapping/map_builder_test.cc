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

#include "cartographer/mapping/map_builder.h"

#include <string>
#include <vector>

#include "cartographer/common/config.h"
#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/common/lua_parameter_dictionary_test_helpers.h"
#include "cartographer/common/make_unique.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace mapping {
namespace {

class MapBuilderTest : public ::testing::Test {
 protected:
  void SetUp() override {
    const std::string kCode = R"text(
      include "map_builder.lua"
      MAP_BUILDER.use_trajectory_builder_2d = true
      return MAP_BUILDER)text";
    auto file_resolver = ::cartographer::common::make_unique<
        ::cartographer::common::ConfigurationFileResolver>(
        std::vector<std::string>{
            std::string(::cartographer::common::kSourceDirectory) +
            "/configuration_files"});
    ::cartographer::common::LuaParameterDictionary parameter_dictionary(
        kCode, std::move(file_resolver));
    proto::MapBuilderOptions options =
        CreateMapBuilderOptions(&parameter_dictionary);
    map_builder_ = common::make_unique<MapBuilder>(options);
  }

  std::unique_ptr<MapBuilderInterface> map_builder_;
  common::Time time_ = common::FromUniversal(12345678);
};

TEST_F(MapBuilderTest, SetUp) { EXPECT_TRUE(map_builder_.get() != nullptr); }

}  // namespace
}  // namespace mapping
}  // namespace cartographer
