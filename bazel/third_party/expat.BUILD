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
#   Expat is an XML parser library written in C.

licenses(["notice"])  # MIT-style

load("@com_github_antonovvk_bazel_rules//:config.bzl", "cc_fix_config")

cc_fix_config(
    name = "expat_config_h",
    cmake = True,
    files = {"expat_config.h.cmake": "expat_config.h"},
    values = {
        "BYTEORDER": "1234",
        "HAVE_BCOPY": "1",
        "HAVE_DLFCN": "1",
        "HAVE_FCNTL_H": "1",
        "HAVE_GETPAGESIZE": "1",
        "HAVE_INTTYPES_H": "1",
        "HAVE_MEMMOVE": "1",
        "HAVE_MEMORY_H": "1",
        "HAVE_STDINT_H": "1",
        "HAVE_STDLIB_H": "1",
        "HAVE_STRINGS_H": "1",
        "HAVE_STRING_H": "1",
        "HAVE_SYS_PARAM_H": "1",
        "HAVE_SYS_STAT_H": "1",
        "HAVE_SYS_TYPES_H": "1",
        "HAVE_UNISTD_H": "1",
        "LT_OBJDIR": "\".libs/\"",
        "PACKAGE_BUGREPORT": "expat-bugs@libexpat.org",
        "PACKAGE_NAME": "expat",
        "PACKAGE_STRING": "expat 2.2.4",
        "PACKAGE_TARNAME": "expat",
        "PACKAGE_URL": "",
        "PACKAGE_VERSION": "2.2.4",
        "STDC_HEADERS": "1",
        "XML_CONTEXT_BYTES": "1024",
        "XML_DTD": "1",
        "XML_NS": "1",
    },
)

# TODO(rodrigoq): review if we're exposing more headers than users need.
cc_library(
    name = "expat",
    srcs = [
        "lib/xmlparse.c",
        "lib/xmlrole.c",
        "lib/xmltok.c",
    ],
    hdrs = [
        "expat_config.h",
        "lib/ascii.h",
        "lib/asciitab.h",
        "lib/expat.h",
        "lib/expat_external.h",
        "lib/iasciitab.h",
        "lib/internal.h",
        "lib/latin1tab.h",
        "lib/nametab.h",
        "lib/siphash.h",
        "lib/utf8tab.h",
        "lib/xmlrole.h",
        "lib/xmltok.h",
        "lib/xmltok_impl.c",
        "lib/xmltok_impl.h",
        "lib/xmltok_ns.c",
    ],
    copts = [
        "-DHAVE_EXPAT_CONFIG_H",
        "-DXML_DEV_URANDOM",
    ],
    defines = ["XML_STATIC"],
    includes = [
        ".",
        "lib",
    ],
    visibility = ["//visibility:public"],
)
