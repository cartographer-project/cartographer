/*
 * Copyright 2019 The Cartographer Authors
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

#include <iostream>
#include <vector>

#include "absl/memory/memory.h"
#include "absl/strings/str_split.h"
#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "gflags/gflags.h"
#include "glog/logging.h"

DEFINE_string(configuration_directories, "",
              "Comma separated list of directories in which configuration files"
              " are searched, the last is always the Cartographer installation"
              " to allow including files from there.");
DEFINE_string(configuration_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");
DEFINE_string(subdictionary, "",
              "Only dump a subdictionary referenced by its Lua ID, e.g.: "
              "'--subdictionary_id trajectory_builder.trajectory_builder_3d'");

namespace cartographer {
namespace common {

std::unique_ptr<LuaParameterDictionary> LoadLuaDictionary(
    const std::vector<std::string>& configuration_directories,
    const std::string& configuration_basename) {
  auto file_resolver =
      absl::make_unique<ConfigurationFileResolver>(configuration_directories);
  const std::string code =
      file_resolver->GetFileContentOrDie(configuration_basename);
  // We use no reference count because we just want to dump the configuration.
  std::unique_ptr<LuaParameterDictionary> lua_parameter_dictionary =
      LuaParameterDictionary::NonReferenceCounted(code,
                                                  std::move(file_resolver));
  return lua_parameter_dictionary;
}

void PrintSubdictionaryById(LuaParameterDictionary* lua_parameter_dictionary,
                            const std::string& subdictionary_id) {
  const std::vector<std::string> subdictionary_keys =
      absl::StrSplit(subdictionary_id, '.', absl::SkipEmpty());
  CHECK(!subdictionary_keys.empty()) << "Failed to parse 'subdictionary_id'.";
  // Keep a stack to avoid memory glitches due to unique_ptr deletion.
  std::vector<std::unique_ptr<LuaParameterDictionary>> stack;
  for (const auto& key : subdictionary_keys) {
    if (stack.empty()) {
      stack.push_back(lua_parameter_dictionary->GetDictionary(key));
      continue;
    }
    stack.push_back(stack.back()->GetDictionary(key));
  }
  CHECK(!stack.empty());
  std::cout << stack.back()->ToString() << std::endl;
}

}  // namespace common
}  // namespace cartographer

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::SetUsageMessage(
      "Dumps a resolved Lua configuration to stdout.\n"
      "The output can be restricted to a subdictionary using the optional "
      "'--subdictionary_id' parameter, which can be given in Lua syntax.\n"
      "The logs of the configuration file resolver are written to stderr if "
      "'--logtostderr' is given.");
  google::ParseCommandLineFlags(&argc, &argv, true);

  CHECK(!FLAGS_configuration_directories.empty())
      << "-configuration_directories is missing.";
  CHECK(!FLAGS_configuration_basename.empty())
      << "-configuration_basename is missing.";

  const std::vector<std::string> configuration_directories =
      absl::StrSplit(FLAGS_configuration_directories, ',', absl::SkipEmpty());

  auto lua_parameter_dictionary = ::cartographer::common::LoadLuaDictionary(
      configuration_directories, FLAGS_configuration_basename);

  if (!FLAGS_subdictionary.empty()) {
    ::cartographer::common::PrintSubdictionaryById(
        lua_parameter_dictionary.get(), FLAGS_subdictionary);
  } else {
    std::cout << lua_parameter_dictionary->ToString() << std::endl;
  }
}
