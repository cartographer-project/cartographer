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
#   The Independent JPEG Group's JPEG runtime library.

licenses(["notice"])  # custom notice-style license, see LICENSE

cc_library(
    name = "jpeg",
    srcs = [
        "cderror.h",
        "cdjpeg.h",
        "jaricom.c",
        "jcapimin.c",
        "jcapistd.c",
        "jcarith.c",
        "jccoefct.c",
        "jccolor.c",
        "jcdctmgr.c",
        "jchuff.c",
        "jcinit.c",
        "jcmainct.c",
        "jcmarker.c",
        "jcmaster.c",
        "jcomapi.c",
        "jconfig.h",
        "jcparam.c",
        "jcprepct.c",
        "jcsample.c",
        "jctrans.c",
        "jdapimin.c",
        "jdapistd.c",
        "jdarith.c",
        "jdatadst.c",
        "jdatasrc.c",
        "jdcoefct.c",
        "jdcolor.c",
        "jdct.h",
        "jddctmgr.c",
        "jdhuff.c",
        "jdinput.c",
        "jdmainct.c",
        "jdmarker.c",
        "jdmaster.c",
        "jdmerge.c",
        "jdpostct.c",
        "jdsample.c",
        "jdtrans.c",
        "jerror.c",
        "jfdctflt.c",
        "jfdctfst.c",
        "jfdctint.c",
        "jidctflt.c",
        "jidctfst.c",
        "jidctint.c",
        "jinclude.h",
        "jmemmgr.c",
        "jmemnobs.c",
        "jmemsys.h",
        "jmorecfg.h",
        "jquant1.c",
        "jquant2.c",
        "jutils.c",
        "jversion.h",
        "transupp.h",
    ],
    hdrs = [
        "jerror.h",
        "jpegint.h",
        "jpeglib.h",
    ],
    includes = ["."],
    visibility = ["//visibility:public"],
)

genrule(
    name = "configure",
    outs = ["jconfig.h"],
    cmd = "cat <<EOF >$@\n" +
          "#define HAVE_PROTOTYPES 1\n" +
          "#define HAVE_UNSIGNED_CHAR 1\n" +
          "#define HAVE_UNSIGNED_SHORT 1\n" +
          "#define HAVE_STDDEF_H 1\n" +
          "#define HAVE_STDLIB_H 1\n" +
          "#ifdef WIN32\n" +
          "#define INLINE __inline\n" +
          "#else\n" +
          "#define INLINE __inline__\n" +
          "#endif\n" +
          "EOF\n",
)
