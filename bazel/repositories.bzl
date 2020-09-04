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

"""External dependencies for Cartographer."""

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def cartographer_repositories():
    _maybe(
        http_archive,
        name = "com_github_nelhage_rules_boost",
        sha256 = "371f49e7b29e44a718baf8b9a2dd3eca865005a851c9ecf8fb6a10a715aa58dd",
        strip_prefix = "rules_boost-a5a95642f6097f8949020646ffe89d7243008981",
        urls = [
            "https://mirror.bazel.build/github.com/nelhage/rules_boost/archive/a5a95642f6097f8949020646ffe89d7243008981.tar.gz",
            "https://github.com/nelhage/rules_boost/archive/a5a95642f6097f8949020646ffe89d7243008981.tar.gz",
        ],
    )

    _maybe(
        http_archive,
        name = "com_github_antonovvk_bazel_rules",
        sha256 = "2f5327a2dc9a0cc8ead93953a5d2ae2e0308aece685e46cc89c27538a7e9a73a",
        strip_prefix = "bazel_rules-c76e47ebe6f0a03b9dd99e245d5a0611813c36f9",
        urls = [
            "https://github.com/drigz/bazel_rules/archive/c76e47ebe6f0a03b9dd99e245d5a0611813c36f9.tar.gz",
        ],
    )

    _maybe(
        http_archive,
        name = "com_github_gflags_gflags",
        sha256 = "6e16c8bc91b1310a44f3965e616383dbda48f83e8c1eaa2370a215057b00cabe",
        strip_prefix = "gflags-77592648e3f3be87d6c7123eb81cbad75f9aef5a",
        urls = [
            "https://mirror.bazel.build/github.com/gflags/gflags/archive/77592648e3f3be87d6c7123eb81cbad75f9aef5a.tar.gz",
            "https://github.com/gflags/gflags/archive/77592648e3f3be87d6c7123eb81cbad75f9aef5a.tar.gz",
        ],
    )

    _maybe(
        http_archive,
        name = "com_google_glog",
        sha256 = "dfc074b41a5b86fc5dda4f0e2e2d6cc5b21f798c9fcc8ed5fea9c8f7c4613be6",
        strip_prefix = "glog-dd2b93d761a19860190cb3fa92066c8031e912e3",
        urls = [
            "https://mirror.bazel.build/github.com/google/glog/archive/dd2b93d761a19860190cb3fa92066c8031e912e3.tar.gz",
            "https://github.com/google/glog/archive/dd2b93d761a19860190cb3fa92066c8031e912e3.tar.gz",
        ],
    )

    _maybe(
        http_archive,
        name = "net_zlib_zlib",
        sha256 = "6d4d6640ca3121620995ee255945161821218752b551a1a180f4215f7d124d45",
        build_file = "@com_github_googlecartographer_cartographer//bazel/third_party:zlib.BUILD",
        strip_prefix = "zlib-cacf7f1d4e3d44d871b605da3b647f07d718623f",
        urls = [
            "https://mirror.bazel.build/github.com/madler/zlib/archive/cacf7f1d4e3d44d871b605da3b647f07d718623f.tar.gz",
            "https://github.com/madler/zlib/archive/cacf7f1d4e3d44d871b605da3b647f07d718623f.tar.gz",
        ],
    )

    _maybe(
        http_archive,
        name = "org_cairographics_pixman",
        build_file = "@com_github_googlecartographer_cartographer//bazel/third_party/pixman:pixman.BUILD",
        sha256 = "21b6b249b51c6800dc9553b65106e1e37d0e25df942c90531d4c3997aa20a88e",
        strip_prefix = "pixman-0.34.0",
        urls = [
            "https://mirror.bazel.build/www.cairographics.org/releases/pixman-0.34.0.tar.gz",
            "https://www.cairographics.org/releases/pixman-0.34.0.tar.gz",
        ],
    )

    _maybe(
        http_archive,
        name = "org_cairographics_cairo",
        build_file = "@com_github_googlecartographer_cartographer//bazel/third_party/cairo:cairo.BUILD",
        sha256 = "7e87878658f2c9951a14fc64114d4958c0e65ac47530b8ac3078b2ce41b66a09",
        strip_prefix = "cairo-1.14.10",
        urls = [
            "https://mirror.bazel.build/www.cairographics.org/releases/cairo-1.14.10.tar.xz",
            "https://www.cairographics.org/releases/cairo-1.14.10.tar.xz",
        ],
    )

    _maybe(
        http_archive,
        name = "org_freetype_freetype2",
        build_file = "@com_github_googlecartographer_cartographer//bazel/third_party:freetype2.BUILD",
        sha256 = "33a28fabac471891d0523033e99c0005b95e5618dc8ffa7fa47f9dadcacb1c9b",
        strip_prefix = "freetype-2.8",
        urls = [
            "https://mirror.bazel.build/download.savannah.gnu.org/releases/freetype/freetype-2.8.tar.gz",
            "http://download.savannah.gnu.org/releases/freetype/freetype-2.8.tar.gz",
        ],
    )

    _maybe(
        http_archive,
        name = "org_libgd_libgd",
        build_file = "@com_github_googlecartographer_cartographer//bazel/third_party:gd.BUILD",
        sha256 = "a66111c9b4a04e818e9e2a37d7ae8d4aae0939a100a36b0ffb52c706a09074b5",
        strip_prefix = "libgd-2.2.5",
        urls = [
            "https://mirror.bazel.build/github.com/libgd/libgd/releases/download/gd-2.2.5/libgd-2.2.5.tar.gz",
            "https://github.com/libgd/libgd/releases/download/gd-2.2.5/libgd-2.2.5.tar.gz",
        ],
    )

    _maybe(
        http_archive,
        name = "org_freedesktop_fontconfig",
        build_file = "@com_github_googlecartographer_cartographer//bazel/third_party/fontconfig:fontconfig.BUILD",
        sha256 = "fd5a6a663f4c4a00e196523902626654dd0c4a78686cbc6e472f338e50fdf806",
        strip_prefix = "fontconfig-2.12.4",
        urls = [
            "https://mirror.bazel.build/www.freedesktop.org/software/fontconfig/release/fontconfig-2.12.4.tar.gz",
            "https://www.freedesktop.org/software/fontconfig/release/fontconfig-2.12.4.tar.gz",
        ],
    )

    _maybe(
        http_archive,
        name = "org_ceres_solver_ceres_solver",
        build_file = "@com_github_googlecartographer_cartographer//bazel/third_party:ceres.BUILD",
        sha256 = "ede5b4205ee8d7c7e029e9da74c7ee759fee6961f7e6bfa694274e4a55b8c7ca",
        strip_prefix = "ceres-solver-58c5edae2f7c4d2533fe8a975c1f5f0b892dfd3e",
        urls = [
            "https://mirror.bazel.build/github.com/ceres-solver/ceres-solver/archive/58c5edae2f7c4d2533fe8a975c1f5f0b892dfd3e.tar.gz",
            "https://github.com/ceres-solver/ceres-solver/archive/58c5edae2f7c4d2533fe8a975c1f5f0b892dfd3e.tar.gz",
        ],
    )

    _maybe(
        http_archive,
        name = "org_tuxfamily_eigen",
        build_file = "@com_github_googlecartographer_cartographer//bazel/third_party:eigen.BUILD",
        sha256 = "ca7beac153d4059c02c8fc59816c82d54ea47fe58365e8aded4082ded0b820c4",
        strip_prefix = "eigen-eigen-f3a22f35b044",
        urls = [
            "http://mirror.bazel.build/bitbucket.org/eigen/eigen/get/f3a22f35b044.tar.gz",
            "https://bitbucket.org/eigen/eigen/get/f3a22f35b044.tar.gz",
        ],
    )

    _maybe(
        http_archive,
        name = "com_github_libexpat_libexpat",
        build_file = "@com_github_googlecartographer_cartographer//bazel/third_party:expat.BUILD",
        sha256 = "b5dcb503e40f615a0296a18acc6d975f2f1a3d01c7b3a4b3bb69de27bc9475c3",
        strip_prefix = "libexpat-R_2_2_4/expat",
        urls = [
            "https://mirror.bazel.build/github.com/libexpat/libexpat/archive/R_2_2_4.tar.gz",
            "https://github.com/libexpat/libexpat/archive/R_2_2_4.tar.gz",
        ],
    )

    _maybe(
        http_archive,
        name = "libjpeg",
        build_file = "@com_github_googlecartographer_cartographer//bazel/third_party:libjpeg.BUILD",
        sha256 = "240fd398da741669bf3c90366f58452ea59041cacc741a489b99f2f6a0bad052",
        strip_prefix = "jpeg-9b",
        urls = [
            "https://mirror.bazel.build/www.ijg.org/files/jpegsrc.v9b.tar.gz",
            "http://www.ijg.org/files/jpegsrc.v9b.tar.gz",
        ],
    )

    _maybe(
        http_archive,
        name = "org_libpng_libpng",
        build_file = "@com_github_googlecartographer_cartographer//bazel/third_party:libpng.BUILD",
        sha256 = "7f415186d38ca71c23058386d7cf5135c8beda821ee1beecdc2a7a26c0356615",
        strip_prefix = "libpng-1.2.57",
        urls = [
            "https://mirror.bazel.build/github.com/glennrp/libpng/archive/v1.2.57.tar.gz",
            "https://github.com/glennrp/libpng/archive/v1.2.57.tar.gz",
        ],
    )

    _maybe(
        http_archive,
        name = "com_google_googletest",
        sha256 = "c18f281fd6621bb264570b99860a0241939b4a251c9b1af709b811d33bc63af8",
        strip_prefix = "googletest-e3bd4cbeaeef3cee65a68a8bd3c535cb779e9b6d",
        urls = [
            "https://mirror.bazel.build/github.com/google/googletest/archive/e3bd4cbeaeef3cee65a68a8bd3c535cb779e9b6d.tar.gz",
            "https://github.com/google/googletest/archive/e3bd4cbeaeef3cee65a68a8bd3c535cb779e9b6d.tar.gz",
        ],
    )

    _maybe(
        http_archive,
        name = "bazel_skylib",
        sha256 = "e5d90f0ec952883d56747b7604e2a15ee36e288bb556c3d0ed33e818a4d971f2",
        strip_prefix = "bazel-skylib-1.0.2",
        urls = ["https://github.com/bazelbuild/bazel-skylib/archive/1.0.2.tar.gz"],
    )

    _maybe(
        http_archive,
        name = "com_google_protobuf",
        sha256 = "1c744a6a1f2c901e68c5521bc275e22bdc66256eeb605c2781923365b7087e5f",
        strip_prefix = "protobuf-3.13.0",
        urls = [
            "https://mirror.bazel.build/github.com/google/protobuf/archive/v3.13.0.zip",
            "https://github.com/google/protobuf/archive/v3.13.0.zip",
        ],
        repo_mapping = {"@zlib": "@net_zlib_zlib"},
    )

    _maybe(
        http_archive,
        name = "org_lua_lua",
        build_file = "@com_github_googlecartographer_cartographer//bazel/third_party:lua.BUILD",
        sha256 = "b9e2e4aad6789b3b63a056d442f7b39f0ecfca3ae0f1fc0ae4e9614401b69f4b",
        strip_prefix = "lua-5.2.4",
        urls = [
            "https://mirror.bazel.build/www.lua.org/ftp/lua-5.2.4.tar.gz",
            "https://www.lua.org/ftp/lua-5.2.4.tar.gz",
        ],
    )

    _maybe(
        http_archive,
        name = "com_github_grpc_grpc",
        sha256 = "f869c648090e8bddaa1260a271b1089caccbe735bf47ac9cd7d44d35a02fb129",
        strip_prefix = "grpc-1.19.1",
        urls = [
            "https://mirror.bazel.build/github.com/grpc/grpc/archive/v1.19.1.tar.gz",
            "https://github.com/grpc/grpc/archive/v1.19.1.tar.gz",
        ],
    )

    _maybe(
        http_archive,
        name = "com_github_jupp0r_prometheus_cpp",
        sha256 = "07a704819cb90ed619cbf1a2713ba39faab27b8898b4561cc11a3c8b3ace83ea",
        strip_prefix = "prometheus-cpp-4b11ee7a0aa7157494df06c4a324bf6d11bd0eec",
        urls = [
            "https://github.com/jupp0r/prometheus-cpp/archive/4b11ee7a0aa7157494df06c4a324bf6d11bd0eec.tar.gz",
        ],
    )

    _maybe(
        http_archive,
        name = "com_github_googlecartographer_async_grpc",
        sha256 = "83c2a27c92979787f38810adc4b6bb67aa09607c53dbadca3430a5f29e0a1cd3",
        strip_prefix = "async_grpc-771af45374af7f7bfc3b622ed7efbe29a4aba403",
        urls = [
            "https://github.com/cartographer-project/async_grpc/archive/771af45374af7f7bfc3b622ed7efbe29a4aba403.tar.gz",
        ],
    )

    _maybe(
        http_archive,
        name = "com_google_absl",
        sha256 = "c8ba586a9ab12bc4a67bb419fc0d2146200942b072bac95f50490f977b7fb04f",
        strip_prefix = "abseil-cpp-5441bbe1db5d0f2ca24b5b60166367b0966790af",
        urls = ["https://github.com/abseil/abseil-cpp/archive/5441bbe1db5d0f2ca24b5b60166367b0966790af.tar.gz"],
    )

    _maybe(
        http_archive,
        name = "rules_python",
        sha256 = "e5470e92a18aa51830db99a4d9c492cc613761d5bdb7131c04bd92b9834380f6",
        strip_prefix = "rules_python-4b84ad270387a7c439ebdccfd530e2339601ef27",
        urls = [
            "https://mirror.bazel.build/github.com/bazelbuild/rules_python/archive/4b84ad270387a7c439ebdccfd530e2339601ef27.tar.gz",
            "https://github.com/bazelbuild/rules_python/archive/4b84ad270387a7c439ebdccfd530e2339601ef27.tar.gz",
        ],
    )

    # TODO(rodrigoq): remove these binds once grpc#14140 has been merged, as well
    # as removing `use_external` in cartographer_grpc/BUILD.bazel.
    # https://github.com/grpc/grpc/pull/14140
    native.bind(
        name = "grpc_cpp_plugin",
        actual = "@com_github_grpc_grpc//:grpc_cpp_plugin",
    )
    native.bind(
        name = "grpc++_codegen_proto",
        actual = "@com_github_grpc_grpc//:grpc++_codegen_proto",
    )

def _maybe(repo_rule, name, **kwargs):
    if name not in native.existing_rules():
        repo_rule(name = name, **kwargs)
