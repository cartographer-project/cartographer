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

#include "cartographer/common/lua_parameter_dictionary.h"

#include <algorithm>
#include <cmath>
#include <memory>

#include "absl/memory/memory.h"
#include "cartographer/common/lua_parameter_dictionary_test_helpers.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace common {
namespace {

std::unique_ptr<LuaParameterDictionary> MakeNonReferenceCounted(
    const std::string& code) {
  return LuaParameterDictionary::NonReferenceCounted(
      code, absl::make_unique<DummyFileResolver>());
}

class LuaParameterDictionaryTest : public ::testing::Test {
 protected:
  void ReferenceAllKeysAsIntegers(LuaParameterDictionary* dict) {
    for (const std::string& key : dict->GetKeys()) {
      dict->GetInt(key);
    }
  }
};

TEST_F(LuaParameterDictionaryTest, GetInt) {
  auto dict = MakeDictionary("return { blah = 100 }");
  ASSERT_EQ(dict->GetInt("blah"), 100);
}

TEST_F(LuaParameterDictionaryTest, GetString) {
  auto dict = MakeDictionary("return { blah = 'is_a_string' }\n");
  ASSERT_EQ(dict->GetString("blah"), "is_a_string");
}

TEST_F(LuaParameterDictionaryTest, GetDouble) {
  auto dict = MakeDictionary("return { blah = 3.1415 }");
  ASSERT_DOUBLE_EQ(dict->GetDouble("blah"), 3.1415);
}

TEST_F(LuaParameterDictionaryTest, GetBoolTrue) {
  auto dict = MakeDictionary("return { blah = true }");
  ASSERT_TRUE(dict->GetBool("blah"));
}

TEST_F(LuaParameterDictionaryTest, GetBoolFalse) {
  auto dict = MakeDictionary("return { blah = false }");
  ASSERT_FALSE(dict->GetBool("blah"));
}

TEST_F(LuaParameterDictionaryTest, GetDictionary) {
  auto dict =
      MakeDictionary("return { blah = { blue = 100, red = 200 }, fasel = 10 }");

  std::unique_ptr<LuaParameterDictionary> sub_dict(dict->GetDictionary("blah"));
  std::vector<std::string> keys = sub_dict->GetKeys();
  ASSERT_EQ(keys.size(), 2);
  std::sort(keys.begin(), keys.end());
  ASSERT_EQ(keys[0], "blue");
  ASSERT_EQ(keys[1], "red");
  ASSERT_TRUE(sub_dict->HasKey("blue"));
  ASSERT_TRUE(sub_dict->HasKey("red"));
  ASSERT_EQ(sub_dict->GetInt("blue"), 100);
  ASSERT_EQ(sub_dict->GetInt("red"), 200);

  ASSERT_EQ(dict->GetString("fasel"), "10");
}

TEST_F(LuaParameterDictionaryTest, GetKeys) {
  auto dict = MakeDictionary("return { blah = 100, blah1 = 200}");

  std::vector<std::string> keys = dict->GetKeys();
  ASSERT_EQ(keys.size(), 2);
  std::sort(keys.begin(), keys.end());
  ASSERT_EQ(keys[0], "blah");
  ASSERT_EQ(keys[1], "blah1");

  ReferenceAllKeysAsIntegers(dict.get());
}

TEST_F(LuaParameterDictionaryTest, NonExistingKey) {
  auto dict = MakeDictionary("return { blah = 100 }");
  ReferenceAllKeysAsIntegers(dict.get());
  ASSERT_DEATH(dict->GetInt("blah_fasel"), "Key.* not in dictionary.");
}

TEST_F(LuaParameterDictionaryTest, UintNegative) {
  auto dict = MakeDictionary("return { blah = -100}");
  ASSERT_DEATH(dict->GetNonNegativeInt("blah"), ".*-100 is negative.");
  ReferenceAllKeysAsIntegers(dict.get());
}

TEST_F(LuaParameterDictionaryTest, ToString) {
  auto dict = MakeDictionary(R"(return {
  ceta = { yolo = "hurray" },
  fasel = 1234.456786,
  fasel1 = -math.huge,
  fasel2 = math.huge,
  blubber = 123,
  blub = 'hello',
  alpha = true,
  alpha1 = false,
  })");

  const std::string golden = R"({
  alpha = true,
  alpha1 = false,
  blub = "hello",
  blubber = 123.000000,
  ceta = {
    yolo = "hurray",
  },
  fasel = 1234.456786,
  fasel1 = -math.huge,
  fasel2 = math.huge,
})";
  EXPECT_EQ(golden, dict->ToString());

  auto dict1 = MakeDictionary("return " + dict->ToString());

  EXPECT_EQ(dict1->GetBool("alpha"), true);
  EXPECT_EQ(dict1->GetBool("alpha1"), false);
  EXPECT_EQ(dict1->GetInt("blubber"), 123);
  EXPECT_EQ(dict1->GetString("blub"), "hello");
  EXPECT_EQ(dict1->GetDictionary("ceta")->GetString("yolo"), "hurray");
  EXPECT_NEAR(dict1->GetDouble("fasel"), 1234.456786, 1e-3);
  EXPECT_TRUE(std::isinf(-dict1->GetDouble("fasel1")));
  EXPECT_TRUE(std::isinf(dict1->GetDouble("fasel2")));

  EXPECT_EQ(dict->GetBool("alpha"), true);
  EXPECT_EQ(dict->GetBool("alpha1"), false);
  EXPECT_EQ(dict->GetInt("blubber"), 123);
  EXPECT_EQ(dict->GetString("blub"), "hello");
  EXPECT_EQ(dict->GetDictionary("ceta")->GetString("yolo"), "hurray");
  EXPECT_NEAR(dict->GetDouble("fasel"), 1234.456786, 1e-3);
  EXPECT_TRUE(std::isinf(-dict->GetDouble("fasel1")));
  EXPECT_TRUE(std::isinf(dict->GetDouble("fasel2")));
}

TEST_F(LuaParameterDictionaryTest, ToStringForArrays) {
  auto dict = MakeNonReferenceCounted(
      R"(return {
      "blub", 3, 3.1,
      foo = "ups",
  })");

  const std::string golden = R"({
  "blub",
  3.000000,
  3.100000,
  foo = "ups",
})";
  EXPECT_EQ(golden, dict->ToString());
}

TEST_F(LuaParameterDictionaryTest, GetArrayValuesAsStrings) {
  auto dict = MakeDictionary("return { 'a', 'b', 'c' }");
  EXPECT_EQ(0, dict->GetKeys().size());
  const std::vector<std::string> values = dict->GetArrayValuesAsStrings();
  EXPECT_EQ(3, values.size());
  EXPECT_EQ("a", values[0]);
  EXPECT_EQ("b", values[1]);
  EXPECT_EQ("c", values[2]);
}

TEST_F(LuaParameterDictionaryTest, GetArrayValuesAsDoubles) {
  auto dict = MakeDictionary("return { 1., 2., 3. }");
  EXPECT_EQ(0, dict->GetKeys().size());
  const std::vector<double> values = dict->GetArrayValuesAsDoubles();
  EXPECT_EQ(3, values.size());
  EXPECT_NEAR(1., values[0], 1e-7);
  EXPECT_NEAR(2., values[1], 1e-7);
  EXPECT_NEAR(3., values[2], 1e-7);
}

TEST_F(LuaParameterDictionaryTest, GetArrayValuesAsDictionaries) {
  auto dict = MakeDictionary("return { { a = 1 }, { b = 3 } }");
  EXPECT_EQ(0, dict->GetKeys().size());
  const std::vector<std::unique_ptr<LuaParameterDictionary>> values =
      dict->GetArrayValuesAsDictionaries();
  EXPECT_EQ(2, values.size());
  EXPECT_EQ(1., values[0]->GetInt("a"));
  EXPECT_EQ(3., values[1]->GetInt("b"));
}

TEST_F(LuaParameterDictionaryTest, TestChooseTrue) {
  auto dict = MakeDictionary("return { a = choose(true, 1, 0) }");
  EXPECT_EQ(1, dict->GetInt("a"));
}

TEST_F(LuaParameterDictionaryTest, TestChoseFalse) {
  auto dict = MakeDictionary("return { a = choose(false, 1, 0) }");
  EXPECT_EQ(0, dict->GetInt("a"));
}

TEST_F(LuaParameterDictionaryTest, TestChooseInvalidArgument) {
  EXPECT_DEATH(MakeDictionary("return { a = choose('truish', 1, 0) }"),
               "condition is not a boolean value.");
}

}  // namespace
}  // namespace common
}  // namespace cartographer
