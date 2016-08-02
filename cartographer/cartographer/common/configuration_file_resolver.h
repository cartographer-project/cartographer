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

#ifndef CARTOGRAPHER_COMMON_CONFIGURATION_FILE_RESOLVER_H_
#define CARTOGRAPHER_COMMON_CONFIGURATION_FILE_RESOLVER_H_

#include <vector>

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/port.h"

namespace cartographer {
namespace common {

// A 'FileResolver' for the 'LuaParameterDictionary' that reads files from disk.
// It searches the 'configuration_files_directories' in order to find the
// requested filename. The last place searched is always the
// 'configuration_files/' directory installed with Cartographer. It contains
// reasonable configuration for the various Cartographer components which
// provide a good starting ground for new platforms.
class ConfigurationFileResolver : public FileResolver {
 public:
  explicit ConfigurationFileResolver(
      const std::vector<string>& configuration_files_directories);

  string GetFullPathOrDie(const string& basename) override;
  string GetFileContentOrDie(const string& basename) override;

 private:
  std::vector<string> configuration_files_directories_;
};

}  // namespace common
}  // namespace cartographer

#endif  // CARTOGRAPHER_COMMON_CONFIGURATION_FILE_RESOLVER_H_
