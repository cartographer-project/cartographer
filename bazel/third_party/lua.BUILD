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
#   Lua language interpreter.

package(
    default_visibility = ["//visibility:public"],
)

licenses(["notice"])  # MIT

cc_library(
    name = "lua_includes",
    hdrs = [
        "src/lauxlib.h",
        "src/lua.h",
        "src/lua.hpp",
        "src/luaconf.h",
        "src/lualib.h",
    ],
    includes = ["src"],
)

cc_library(
    name = "lua",
    srcs = [
        # Core language
        "src/lapi.c",
        "src/lapi.h",
        "src/lcode.c",
        "src/lcode.h",
        "src/lctype.c",
        "src/lctype.h",
        "src/ldebug.c",
        "src/ldebug.h",
        "src/ldo.c",
        "src/ldo.h",
        "src/ldump.c",
        "src/lfunc.c",
        "src/lfunc.h",
        "src/lgc.c",
        "src/lgc.h",
        "src/llex.c",
        "src/llex.h",
        "src/llimits.h",
        "src/lmem.c",
        "src/lmem.h",
        "src/lobject.c",
        "src/lobject.h",
        "src/lopcodes.c",
        "src/lopcodes.h",
        "src/lparser.c",
        "src/lparser.h",
        "src/lstate.c",
        "src/lstate.h",
        "src/lstring.c",
        "src/lstring.h",
        "src/ltable.c",
        "src/ltable.h",
        "src/ltm.c",
        "src/ltm.h",
        "src/lundump.c",
        "src/lundump.h",
        "src/lvm.c",
        "src/lvm.h",
        "src/lzio.c",

        # Standard libraries
        "src/lauxlib.c",
        "src/lbaselib.c",
        "src/lbitlib.c",
        "src/lcorolib.c",
        "src/ldblib.c",
        "src/linit.c",
        "src/liolib.c",
        "src/lmathlib.c",
        "src/loadlib.c",
        "src/loslib.c",
        "src/lstrlib.c",
        "src/ltablib.c",
        "src/lzio.h",
    ],
    hdrs = [
        "src/lauxlib.h",
        "src/lua.h",
        "src/lua.hpp",
        "src/luaconf.h",
        "src/lualib.h",
    ],
    copts = ["-w"],
    defines = ["LUA_USE_LINUX"],
    includes = ["src"],
    linkopts = [
        "-lm",
        "-ldl",
    ],
)
