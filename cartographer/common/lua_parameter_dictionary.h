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

#ifndef CARTOGRAPHER_COMMON_LUA_PARAMETER_DICTIONARY_H_
#define CARTOGRAPHER_COMMON_LUA_PARAMETER_DICTIONARY_H_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "cartographer/common/lua.h"
#include "cartographer/common/port.h"
#include "glog/logging.h"

namespace cartographer {
namespace common {

// Resolves file paths and file content for the Lua 'read' and 'include'
// functions. Use this to configure where those functions load other files from.
class FileResolver {
 public:
  virtual ~FileResolver() {}
  virtual std::string GetFullPathOrDie(const std::string& basename) = 0;
  virtual std::string GetFileContentOrDie(const std::string& basename) = 0;
};

// A parameter dictionary that gets loaded from Lua code.
class LuaParameterDictionary {
 public:
  // Constructs the dictionary from a Lua Table specification.
  LuaParameterDictionary(const std::string& code,
                         std::unique_ptr<FileResolver> file_resolver);

  LuaParameterDictionary(const LuaParameterDictionary&) = delete;
  LuaParameterDictionary& operator=(const LuaParameterDictionary&) = delete;

  // Constructs a LuaParameterDictionary without reference counting.
  static std::unique_ptr<LuaParameterDictionary> NonReferenceCounted(
      const std::string& code, std::unique_ptr<FileResolver> file_resolver);

  ~LuaParameterDictionary();

  // Returns all available keys.
  std::vector<std::string> GetKeys() const;

  // Returns true if the key is in this dictionary.
  bool HasKey(const std::string& key) const;

  // These methods CHECK() that the 'key' exists.
  std::string GetString(const std::string& key);
  double GetDouble(const std::string& key);
  int GetInt(const std::string& key);
  bool GetBool(const std::string& key);
  std::unique_ptr<LuaParameterDictionary> GetDictionary(const std::string& key);

  // Gets an int from the dictionary and CHECK()s that it is non-negative.
  int GetNonNegativeInt(const std::string& key);

  // Returns a string representation for this LuaParameterDictionary.
  std::string ToString() const;

  // Returns the values of the keys '1', '2', '3' as the given types.
  std::vector<double> GetArrayValuesAsDoubles();
  std::vector<std::string> GetArrayValuesAsStrings();
  std::vector<std::unique_ptr<LuaParameterDictionary>>
  GetArrayValuesAsDictionaries();

 private:
  enum class ReferenceCount { YES, NO };
  LuaParameterDictionary(const std::string& code,
                         ReferenceCount reference_count,
                         std::unique_ptr<FileResolver> file_resolver);

  // For GetDictionary().
  LuaParameterDictionary(lua_State* L, ReferenceCount reference_count,
                         std::shared_ptr<FileResolver> file_resolver);

  // Function that recurses to keep track of indent for ToString().
  std::string DoToString(const std::string& indent) const;

  // Pop the top of the stack and CHECKs that the type is correct.
  double PopDouble() const;
  int PopInt() const;
  bool PopBool() const;

  // Pop the top of the stack and CHECKs that it is a string. The returned value
  // is either quoted to be suitable to be read back by a Lua interpretor or
  // not.
  enum class Quoted { YES, NO };
  std::string PopString(Quoted quoted) const;

  // Creates a LuaParameterDictionary from the Lua table at the top of the
  // stack, either with or without reference counting.
  std::unique_ptr<LuaParameterDictionary> PopDictionary(
      ReferenceCount reference_count) const;

  // CHECK() that 'key' is in the dictionary.
  void CheckHasKey(const std::string& key) const;

  // CHECK() that 'key' is in this dictionary and reference it as being used.
  void CheckHasKeyAndReference(const std::string& key);

  // If desired, this can be called in the destructor of a derived class. It
  // will CHECK() that all keys defined in the configuration have been used
  // exactly once and resets the reference counter.
  void CheckAllKeysWereUsedExactlyOnceAndReset();

  // Reads a file into a Lua string.
  static int LuaRead(lua_State* L);

  // Handles inclusion of other Lua files and prevents double inclusion.
  static int LuaInclude(lua_State* L);

  lua_State* L_;  // The name is by convention in the Lua World.
  int index_into_reference_table_;

  // This is shared with all the sub dictionaries.
  const std::shared_ptr<FileResolver> file_resolver_;

  // If true will check that all keys were used on destruction.
  const ReferenceCount reference_count_;

  // This is modified with every call to Get* in order to verify that all
  // parameters are read exactly once.
  std::map<std::string, int> reference_counts_;

  // List of all included files in order of inclusion. Used to prevent double
  // inclusion.
  std::vector<std::string> included_files_;
};

}  // namespace common
}  // namespace cartographer

#endif  // CARTOGRAPHER_COMMON_LUA_PARAMETER_DICTIONARY_H_
