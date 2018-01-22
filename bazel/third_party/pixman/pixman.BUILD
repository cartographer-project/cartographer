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

# Description:
#   pixman is a library that provides low-level pixel manipulation features.

licenses(["notice"])  # X11/MIT

genrule(
    name = "config_h",
    srcs = ["@com_github_googlecartographer_cartographer//bazel/third_party/pixman:config.h"],
    outs = ["pixman_internal/config.h"],
    cmd = "cp $< $@",
)

COMMON_SRCS = [
    "pixman/pixman.c",
    "pixman/pixman-access.c",
    "pixman/pixman-access-accessors.c",
    "pixman/pixman-arm.c",
    "pixman/pixman-bits-image.c",
    "pixman/pixman-combine-float.c",
    "pixman/pixman-combine32.c",
    "pixman/pixman-conical-gradient.c",
    "pixman/pixman-edge-accessors.c",
    "pixman/pixman-edge.c",
    "pixman/pixman-fast-path.c",
    "pixman/pixman-filter.c",
    "pixman/pixman-general.c",
    "pixman/pixman-glyph.c",
    "pixman/pixman-gradient-walker.c",
    "pixman/pixman-image.c",
    "pixman/pixman-implementation.c",
    "pixman/pixman-linear-gradient.c",
    "pixman/pixman-matrix.c",
    "pixman/pixman-mips.c",
    "pixman/pixman-noop.c",
    "pixman/pixman-ppc.c",
    "pixman/pixman-radial-gradient.c",
    "pixman/pixman-region16.c",
    "pixman/pixman-region32.c",
    "pixman/pixman-solid-fill.c",
    "pixman/pixman-timer.c",
    "pixman/pixman-trap.c",
    "pixman/pixman-utils.c",
    "pixman/pixman-x86.c",
]

# TODO(rodrigoq): use MMX extensions where possible
cc_library(
    name = "pixman",
    srcs = COMMON_SRCS + [
        "pixman/pixman-accessor.h",
        "pixman/pixman-combine32.h",
        "pixman/pixman-compiler.h",
        "pixman/pixman-edge-imp.h",
        "pixman/pixman-inlines.h",
        "pixman/pixman-private.h",
        "pixman/pixman-version.h",
        "pixman/pixman.h",
        "pixman_internal/config.h",
    ],
    hdrs = [
        "pixman/pixman-access.c",
        "pixman/pixman-edge.c",
        "pixman/pixman-region.c",
    ],
    copts = [
        "-Wno-unused-const-variable",
        "-Wno-unused-local-typedefs",
        "-DHAVE_CONFIG_H",
        "-I$(GENDIR)/external/org_cairographics_pixman/pixman_internal",
    ],
    includes = ["pixman"],
    visibility = ["//visibility:public"],
)
