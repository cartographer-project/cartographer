# Copyright 2018 The Cartographer Authors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

workspace(name = "com_github_googlecartographer_cartographer")

load("//:bazel/repositories.bzl", "cartographer_repositories")

cartographer_repositories()

# This can't be inside cartographer_repositories() because of:
# https://github.com/bazelbuild/bazel/issues/1550
load("@com_github_nelhage_rules_boost//:boost/boost.bzl", "boost_deps")

boost_deps()

load("@com_github_grpc_grpc//bazel:grpc_deps.bzl", "grpc_deps")

grpc_deps()

load("@com_github_jupp0r_prometheus_cpp//:repositories.bzl", "prometheus_cpp_repositories")

prometheus_cpp_repositories()
