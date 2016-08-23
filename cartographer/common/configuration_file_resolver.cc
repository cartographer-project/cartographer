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

#include "cartographer/common/configuration_file_resolver.h"

#include <fstream>
#include <iostream>
#include <streambuf>

#include "cartographer/common/config.h"
#include "glog/logging.h"

namespace cartographer {
namespace common {

ConfigurationFileResolver::ConfigurationFileResolver(
    const std::vector<string>& configuration_files_directories)
    : configuration_files_directories_(configuration_files_directories) {
  configuration_files_directories_.push_back(kConfigurationFilesDirectory);
}

string ConfigurationFileResolver::GetFullPathOrDie(const string& basename) {
  for (const auto& path : configuration_files_directories_) {
    const string filename = path + "/" + basename;
    std::ifstream stream(filename.c_str());
    if (stream.good()) {
      LOG(INFO) << "Found '" << filename << "' for '" << basename << "'.";
      return filename;
    }
  }
  LOG(FATAL) << "File '" << basename << "' was not found.";
}

string ConfigurationFileResolver::GetFileContentOrDie(const string& basename) {
  const string filename = GetFullPathOrDie(basename);
  std::ifstream stream(filename.c_str());
  return string((std::istreambuf_iterator<char>(stream)),
                std::istreambuf_iterator<char>());
}

}  // namespace common
}  // namespace cartographer
