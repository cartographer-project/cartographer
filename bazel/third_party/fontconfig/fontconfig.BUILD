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
#   Fontconfig is a library for font customization and configuration.

licenses(["notice"])  # BSD-like

genrule(
    name = "config_h",
    srcs = ["@com_github_googlecartographer_cartographer//bazel/third_party/fontconfig:config.h"],
    outs = ["fontconfig_internal/config.h"],
    cmd = "cp $< $@",
)

genrule(
    name = "fcalias",
    srcs = [
        "fontconfig/fontconfig.h",
        "src/fcdeprecate.h",
        "fontconfig/fcprivate.h",
    ] + glob(["src/*.c"]),
    outs = [
        "src/fcalias.h",
        "src/fcaliastail.h",
    ],
    cmd = """./$(location src/makealias) \
        $$(dirname $(location src/makealias)) \
        $(OUTS) \
        $(location fontconfig/fontconfig.h) \
        $(location src/fcdeprecate.h) \
        $(location fontconfig/fcprivate.h)""",
    tools = ["src/makealias"],
)

genrule(
    name = "fcftalias",
    srcs = ["fontconfig/fcfreetype.h"] + glob(["src/*.c"]),
    outs = [
        "src/fcftalias.h",
        "src/fcftaliastail.h",
    ],
    cmd = """./$(location src/makealias) \
        $$(dirname $(location src/makealias)) \
        $(OUTS) \
        $(location fontconfig/fcfreetype.h)""",
    tools = ["src/makealias"],
)

cc_library(
    name = "fontconfig",
    srcs = [
        "fc-blanks/fcblanks.h",
        "fc-case/fccase.h",
        "fc-glyphname/fcglyphname.h",
        "fc-lang/fclang.h",
        "fontconfig/fcfreetype.h",
        "fontconfig/fcprivate.h",
        "src/fcarch.h",
        "src/fcatomic.c",
        "src/fcatomic.h",
        "src/fcblanks.c",
        "src/fccache.c",
        "src/fccfg.c",
        "src/fccharset.c",
        "src/fccompat.c",
        "src/fcdbg.c",
        "src/fcdefault.c",
        "src/fcdeprecate.h",
        "src/fcdir.c",
        "src/fcformat.c",
        "src/fcfreetype.c",
        "src/fcfs.c",
        "src/fcftint.h",
        "src/fcinit.c",
        "src/fcint.h",
        "src/fclang.c",
        "src/fclist.c",
        "src/fcmatch.c",
        "src/fcmatrix.c",
        "src/fcmutex.h",
        "src/fcname.c",
        "src/fcobjs.c",
        "src/fcobjs.h",
        "src/fcobjshash.h",
        "src/fcpat.c",
        "src/fcrange.c",
        "src/fcserialize.c",
        "src/fcstat.c",
        "src/fcstdint.h",
        "src/fcstr.c",
        "src/fcweight.c",
        "src/fcxml.c",
        "src/ftglue.c",
        "src/ftglue.h",
        ":config_h",
        ":fcalias",
        ":fcftalias",
    ],
    hdrs = [
        "fontconfig/fontconfig.h",
    ],
    copts = [
        "-Iexternal/org_freedesktop_fontconfig/src",
        "-I$(GENDIR)/external/org_freedesktop_fontconfig/src",
        "-I$(GENDIR)/external/org_freedesktop_fontconfig/fontconfig_internal",
        "-DFC_CACHEDIR='\"/var/cache/fontconfig\"'",
        "-DFONTCONFIG_PATH='\"/etc/fonts\"'",
        "-DHAVE_CONFIG_H",
        "-Wno-strict-aliasing",
    ],
    includes = ["."],
    visibility = ["//visibility:public"],
    deps = [
        "@com_github_libexpat_libexpat//:expat",
        "@net_zlib_zlib//:zlib",
        "@org_freetype_freetype2//:freetype2",
    ],
)
