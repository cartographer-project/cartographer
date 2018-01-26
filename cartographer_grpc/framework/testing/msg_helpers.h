/*
 * Copyright 2018 The Cartographer Authors
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

#ifndef CARTOGRAPHER_GRPC_FRAMEWORK_MSG_HELPERS_H
#define CARTOGRAPHER_GRPC_FRAMEWORK_MSG_HELPERS_H

namespace cartographer_grpc {
namespace framework {
namespace testing {

template <typename T>
void SetField(google::protobuf::Message *msg,
              const google::protobuf::FieldDescriptor *field, T &value) {
  LOG(FATAL) << "Not implemented";
}

template <>
void SetField(google::protobuf::Message *msg,
              const google::protobuf::FieldDescriptor *field, int32_t &value) {
  const google::protobuf::Reflection *reflection = msg->GetReflection();
  reflection->SetInt32(msg, field, value);
}

template <>
void SetField(google::protobuf::Message *msg,
              const google::protobuf::FieldDescriptor *field, int64_t &value) {
  const google::protobuf::Reflection *reflection = msg->GetReflection();
  reflection->SetInt64(msg, field, value);
}

template <>
void SetField(google::protobuf::Message *msg,
              const google::protobuf::FieldDescriptor *field, double &value) {
  const google::protobuf::Reflection *reflection = msg->GetReflection();
  reflection->SetDouble(msg, field, value);
}

template <std::size_t N>
void SetField(google::protobuf::Message *msg,
              const google::protobuf::FieldDescriptor *field,
              const char (&value)[N]) {
  const google::protobuf::Reflection *reflection = msg->GetReflection();
  reflection->SetString(msg, field, value);
}

template <typename T>
void SetField(google::protobuf::Message *msg,
              const google::protobuf::FieldDescriptor *field,
              std::unique_ptr<T> &value) {
  const google::protobuf::Reflection *reflection = msg->GetReflection();
  reflection->SetAllocatedMessage(msg, value.release(), field);
}

void SetFields(google::protobuf::Message *msg, int starting_field_index) {
  CHECK_EQ(starting_field_index, msg->GetDescriptor()->field_count() + 1);
}

template <typename T, typename... Values>
void SetFields(google::protobuf::Message *msg, int starting_field_index,
               T &value, Values &&... values) {
  const int remaining_args = sizeof...(values);
  const int field_count = msg->GetDescriptor()->field_count();
  CHECK_EQ(starting_field_index + remaining_args, field_count);
  const google::protobuf::FieldDescriptor *field =
      msg->GetDescriptor()->field(starting_field_index - 1);
  CHECK(field);
  SetField(msg, field, value);
  SetFields(msg, starting_field_index + 1, values...);
}

template <typename MsgType, typename... Values>
std::unique_ptr<MsgType> MakeMsg(Values &&... values) {
  auto msg = cartographer::common::make_unique<MsgType>();

  if (msg->GetDescriptor()->field_count() > 0) {
    SetFields(msg.get(), 1, values...);
  }
  return std::move(msg);
}

} // namespace testing
} // namespace framework
} // namespace cartographer_grpc

#endif // CARTOGRAPHER_GRPC_FRAMEWORK_MSG_HELPERS_H
