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
#   GD is a library for the dynamic creation of images.

load("@com_github_antonovvk_bazel_rules//:config.bzl", "cc_fix_config")

licenses(["notice"])  # simple notice-style license

cc_fix_config(
    name = "config_h",
    cmake = True,
    files = {"src/config.h.cmake": "gd_internal/config.h"},
    values = {
        "HAVE_DIRENT_H": "1",
        "HAVE_DLFCN_H": "1",
        "HAVE_ERRNO_H": "1",
        "HAVE_FT2BUILD_H": "1",
        "HAVE_ICONV": "1",
        "HAVE_ICONV_H": "1",
        "HAVE_ICONV_T_DEF": "1",
        "HAVE_INTTYPES_H": "1",
        "HAVE_LIBFREETYPE": "1",
        "HAVE_LIBJPEG": "1",
        "HAVE_LIBM": "1",
        "HAVE_LIBPNG": "1",
        "HAVE_LIBZ": "1",
        "HAVE_LIMITS_H": "1",
        "HAVE_MEMORY_H": "1",
        "HAVE_PTHREAD": "1",
        "HAVE_PTHREAD_PRIO_INHERIT": "1",
        "HAVE_STDDEF_H": "1",
        "HAVE_STDINT_H": "1",
        "HAVE_STDLIB_H": "1",
        "HAVE_STRINGS_H": "1",
        "HAVE_STRING_H": "1",
        "HAVE_SYS_STAT_H": "1",
        "HAVE_SYS_TYPES_H": "1",
        "HAVE_UNISTD_H": "1",
        "HAVE_VISIBILITY": "1",
        "ICONV_CONST": "",
        "LT_OBJDIR": "\".libs/\"",
        "PACKAGE": "libgd",
        "PACKAGE_BUGREPORT": "https://bitbucket.org/libgd/gd-libgd/issues",
        "PACKAGE_NAME": "GD",
        "PACKAGE_STRING": "GD 2.2.4",
        "PACKAGE_TARNAME": "libgd",
        "PACKAGE_URL": "http://lib.gd",
        "PACKAGE_VERSION": "2.2.4",
        "STDC_HEADERS": "1",
        "VERSION": "2.2.4",
    },
)

cc_library(
    name = "gd",
    srcs = [
        "src/gd.c",
        "src/gd_color.c",
        "src/gd_gd.c",
        "src/gd_gd2.c",
        "src/gd_gif_in.c",
        "src/gd_gif_out.c",
        "src/gd_io.c",
        "src/gd_io_dp.c",
        "src/gd_io_file.c",
        "src/gd_io_ss.c",
        "src/gd_jpeg.c",
        "src/gd_nnquant.c",
        "src/gd_png.c",
        "src/gd_security.c",
        "src/gd_ss.c",
        "src/gd_topal.c",
        "src/gd_wbmp.c",
        "src/gd_xbm.c",
        "src/gdcache.c",
        "src/gdfontg.c",
        "src/gdfontl.c",
        "src/gdfontmb.c",
        "src/gdfonts.c",
        "src/gdfontt.c",
        "src/gdft.c",
        "src/gdfx.c",
        "src/gdhelpers.c",
        "src/gdkanji.c",
        "src/gdtables.c",
        "src/gdxpm.c",
        "src/wbmp.c",
        "gd_internal/config.h",
    ] + glob([
        "src/*.h",
    ]),
    hdrs = [
        "src/gd.h",
        "src/gdhelpers.h",
    ],
    copts = [
        "-I$(GENDIR)/external/org_libgd_libgd/gd_internal",
        "-DFC_CACHEDIR='\"/var/cache/fontconfig\"'",
        "-DFONTCONFIG_PATH='\"/etc/fonts\"'",
        "-DHAVE_CONFIG_H",
    ],
    includes = ["src"],
    linkopts = ["-lm"],
    visibility = ["//visibility:public"],
    deps = [
        "@libjpeg//:jpeg",
        "@net_zlib_zlib//:zlib",
        "@org_freetype_freetype2//:freetype2",
        "@org_libpng_libpng//:libpng",
    ],
)
