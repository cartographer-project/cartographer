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

}  // namespace test
}  // namespace mapping
}  // namespace cartographer
