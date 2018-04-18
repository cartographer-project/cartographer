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

def cartographer_repositories():
  _maybe(native.http_archive,
      name = "com_github_nelhage_boost",
      sha256 = "5c88fc077f6b8111e997fec5146e5f9940ae9a2016eb9949447fcb4b482bcdb3",
      strip_prefix = "rules_boost-7289bb1d8f938fdf98078297768c122ee9e11c9e",
      urls = [
          "https://mirror.bazel.build/github.com/nelhage/rules_boost/archive/7289bb1d8f938fdf98078297768c122ee9e11c9e.tar.gz",
          "https://github.com/nelhage/rules_boost/archive/7289bb1d8f938fdf98078297768c122ee9e11c9e.tar.gz",
      ],
  )

  _maybe(native.http_archive,
      name = "com_github_antonovvk_bazel_rules",
      sha256 = "ba75b07d3fd297375a6688e9a16583eb616e7a74b3d5e8791e7a222cf36ab26e",
      strip_prefix = "bazel_rules-98ddd7e4f7c63ea0868f08bcc228463dac2f9f12",
      urls = [
          "https://mirror.bazel.build/github.com/antonovvk/bazel_rules/archive/98ddd7e4f7c63ea0868f08bcc228463dac2f9f12.tar.gz",
          "https://github.com/antonovvk/bazel_rules/archive/98ddd7e4f7c63ea0868f08bcc228463dac2f9f12.tar.gz",
      ],
  )

  _maybe(native.http_archive,
      name = "com_github_gflags_gflags",
      sha256 = "6e16c8bc91b1310a44f3965e616383dbda48f83e8c1eaa2370a215057b00cabe",
      strip_prefix = "gflags-77592648e3f3be87d6c7123eb81cbad75f9aef5a",
      urls = [
          "https://mirror.bazel.build/github.com/gflags/gflags/archive/77592648e3f3be87d6c7123eb81cbad75f9aef5a.tar.gz",
          "https://github.com/gflags/gflags/archive/77592648e3f3be87d6c7123eb81cbad75f9aef5a.tar.gz",
      ],
  )

  _maybe(native.http_archive,
      name = "com_google_glog",
      sha256 = "dfc074b41a5b86fc5dda4f0e2e2d6cc5b21f798c9fcc8ed5fea9c8f7c4613be6",
      strip_prefix = "glog-dd2b93d761a19860190cb3fa92066c8031e912e3",
      urls = [
          "https://mirror.bazel.build/github.com/google/glog/archive/dd2b93d761a19860190cb3fa92066c8031e912e3.tar.gz",
          "https://github.com/google/glog/archive/dd2b93d761a19860190cb3fa92066c8031e912e3.tar.gz",
      ],
  )

  _maybe(native.new_http_archive,
      name = "net_zlib_zlib",
      build_file = "@com_github_googlecartographer_cartographer//bazel/third_party:zlib.BUILD",
      sha256 = "6d4d6640ca3121620995ee255945161821218752b551a1a180f4215f7d124d45",
      strip_prefix = "zlib-cacf7f1d4e3d44d871b605da3b647f07d718623f",
      urls = [
          "https://mirror.bazel.build/github.com/madler/zlib/archive/cacf7f1d4e3d44d871b605da3b647f07d718623f.tar.gz",
          "https://github.com/madler/zlib/archive/cacf7f1d4e3d44d871b605da3b647f07d718623f.tar.gz",
      ],
  )

  _maybe(native.new_http_archive,
      name = "org_cairographics_pixman",
      build_file = "@com_github_googlecartographer_cartographer//bazel/third_party/pixman:pixman.BUILD",
      sha256 = "21b6b249b51c6800dc9553b65106e1e37d0e25df942c90531d4c3997aa20a88e",
      strip_prefix = "pixman-0.34.0",
      urls = [
          "https://mirror.bazel.build/www.cairographics.org/releases/pixman-0.34.0.tar.gz",
          "https://www.cairographics.org/releases/pixman-0.34.0.tar.gz",
      ],
  )

  _maybe(native.new_http_archive,
      name = "org_cairographics_cairo",
      build_file = "@com_github_googlecartographer_cartographer//bazel/third_party/cairo:cairo.BUILD",
      sha256 = "7e87878658f2c9951a14fc64114d4958c0e65ac47530b8ac3078b2ce41b66a09",
      strip_prefix = "cairo-1.14.10",
      urls = [
          "https://mirror.bazel.build/www.cairographics.org/releases/cairo-1.14.10.tar.xz",
          "https://www.cairographics.org/releases/cairo-1.14.10.tar.xz",
      ],
  )

  _maybe(native.new_http_archive,
      name = "org_freetype_freetype2",
      build_file = "@com_github_googlecartographer_cartographer//bazel/third_party:freetype2.BUILD",
      sha256 = "33a28fabac471891d0523033e99c0005b95e5618dc8ffa7fa47f9dadcacb1c9b",
      strip_prefix = "freetype-2.8",
      urls = [
          "https://mirror.bazel.build/download.savannah.gnu.org/releases/freetype/freetype-2.8.tar.gz",
          "http://download.savannah.gnu.org/releases/freetype/freetype-2.8.tar.gz",
      ],
  )

  _maybe(native.new_http_archive,
      name = "org_libgd_libgd",
      build_file = "@com_github_googlecartographer_cartographer//bazel/third_party:gd.BUILD",
      sha256 = "a66111c9b4a04e818e9e2a37d7ae8d4aae0939a100a36b0ffb52c706a09074b5",
      strip_prefix = "libgd-2.2.5",
      urls = [
          "https://mirror.bazel.build/github.com/libgd/libgd/releases/download/gd-2.2.5/libgd-2.2.5.tar.gz",
          "https://github.com/libgd/libgd/releases/download/gd-2.2.5/libgd-2.2.5.tar.gz",
      ],
  )

  _maybe(native.new_http_archive,
      name = "org_freedesktop_fontconfig",
      build_file = "@com_github_googlecartographer_cartographer//bazel/third_party/fontconfig:fontconfig.BUILD",
      sha256 = "fd5a6a663f4c4a00e196523902626654dd0c4a78686cbc6e472f338e50fdf806",
      strip_prefix = "fontconfig-2.12.4",
      urls = [
          "https://mirror.bazel.build/www.freedesktop.org/software/fontconfig/release/fontconfig-2.12.4.tar.gz",
          "https://www.freedesktop.org/software/fontconfig/release/fontconfig-2.12.4.tar.gz",
      ],
  )

  _maybe(native.new_http_archive,
      name = "org_ceres_solver_ceres_solver",
      build_file = "@com_github_googlecartographer_cartographer//bazel/third_party:ceres.BUILD",
      sha256 = "ede5b4205ee8d7c7e029e9da74c7ee759fee6961f7e6bfa694274e4a55b8c7ca",
      strip_prefix = "ceres-solver-58c5edae2f7c4d2533fe8a975c1f5f0b892dfd3e",
      urls = [
          "https://mirror.bazel.build/github.com/ceres-solver/ceres-solver/archive/58c5edae2f7c4d2533fe8a975c1f5f0b892dfd3e.tar.gz",
          "https://github.com/ceres-solver/ceres-solver/archive/58c5edae2f7c4d2533fe8a975c1f5f0b892dfd3e.tar.gz",
      ],
  )

  _maybe(native.new_http_archive,
      name = "org_tuxfamily_eigen",
      build_file = "@com_github_googlecartographer_cartographer//bazel/third_party:eigen.BUILD",
      sha256 = "ca7beac153d4059c02c8fc59816c82d54ea47fe58365e8aded4082ded0b820c4",
      strip_prefix = "eigen-eigen-f3a22f35b044",
      urls = [
          "http://mirror.bazel.build/bitbucket.org/eigen/eigen/get/f3a22f35b044.tar.gz",
          "https://bitbucket.org/eigen/eigen/get/f3a22f35b044.tar.gz",
      ],
  )

  _maybe(native.new_http_archive,
      name = "com_github_libexpat_libexpat",
      build_file = "@com_github_googlecartographer_cartographer//bazel/third_party:expat.BUILD",
      sha256 = "b5dcb503e40f615a0296a18acc6d975f2f1a3d01c7b3a4b3bb69de27bc9475c3",
      strip_prefix = "libexpat-R_2_2_4/expat",
      urls = [
          "https://mirror.bazel.build/github.com/libexpat/libexpat/archive/R_2_2_4.tar.gz",
          "https://github.com/libexpat/libexpat/archive/R_2_2_4.tar.gz",
      ],
  )

  _maybe(native.new_http_archive,
      name = "libjpeg",
      build_file = "@com_github_googlecartographer_cartographer//bazel/third_party:libjpeg.BUILD",
      sha256 = "240fd398da741669bf3c90366f58452ea59041cacc741a489b99f2f6a0bad052",
      strip_prefix = "jpeg-9b",
      urls = [
          "https://mirror.bazel.build/www.ijg.org/files/jpegsrc.v9b.tar.gz",
          "http://www.ijg.org/files/jpegsrc.v9b.tar.gz",
      ],
  )

  _maybe(native.new_http_archive,
      name = "org_libpng_libpng",
      build_file = "@com_github_googlecartographer_cartographer//bazel/third_party:libpng.BUILD",
      sha256 = "7f415186d38ca71c23058386d7cf5135c8beda821ee1beecdc2a7a26c0356615",
      strip_prefix = "libpng-1.2.57",
      urls = [
          "https://mirror.bazel.build/github.com/glennrp/libpng/archive/v1.2.57.tar.gz",
          "https://github.com/glennrp/libpng/archive/v1.2.57.tar.gz",
      ],
  )

  _maybe(native.http_archive,
      name = "com_google_googletest",
      sha256 = "c18f281fd6621bb264570b99860a0241939b4a251c9b1af709b811d33bc63af8",
      strip_prefix = "googletest-e3bd4cbeaeef3cee65a68a8bd3c535cb779e9b6d",
      urls = [
          "https://mirror.bazel.build/github.com/google/googletest/archive/e3bd4cbeaeef3cee65a68a8bd3c535cb779e9b6d.tar.gz",
          "https://github.com/google/googletest/archive/e3bd4cbeaeef3cee65a68a8bd3c535cb779e9b6d.tar.gz",
      ],
  )

  _maybe(native.http_archive,
      name = "com_google_protobuf",
      sha256 = "0cc6607e2daa675101e9b7398a436f09167dffb8ca0489b0307ff7260498c13c",
      strip_prefix = "protobuf-3.5.0",
      urls = [
          "https://mirror.bazel.build/github.com/google/protobuf/archive/v3.5.0.tar.gz",
          "https://github.com/google/protobuf/archive/v3.5.0.tar.gz",
      ],
  )

  _maybe(native.new_http_archive,
      name = "org_lua_lua",
      build_file = "@com_github_googlecartographer_cartographer//bazel/third_party:lua.BUILD",
      sha256 = "b9e2e4aad6789b3b63a056d442f7b39f0ecfca3ae0f1fc0ae4e9614401b69f4b",
      strip_prefix = "lua-5.2.4",
      urls = [
          "https://mirror.bazel.build/www.lua.org/ftp/lua-5.2.4.tar.gz",
          "https://www.lua.org/ftp/lua-5.2.4.tar.gz",
      ],
  )

  _maybe(native.http_archive,
      name = "com_github_grpc_grpc",
      sha256 = "2fdde7d64e6fb1a397bf2aa23aeddcdcf276652d9e48270e94eb0dc94d7c1345",
      strip_prefix = "grpc-20e7074e4101b4fdbae1764caa952301b38957c4",
      urls = [
          "https://mirror.bazel.build/github.com/grpc/grpc/archive/20e7074e4101b4fdbae1764caa952301b38957c4.tar.gz",
          "https://github.com/grpc/grpc/archive/20e7074e4101b4fdbae1764caa952301b38957c4.tar.gz",
      ],
  )

  _maybe(native.http_archive,
      name = "com_github_jupp0r_prometheus_cpp",
      sha256 = "0d3e999dfbfc49bb117698154a01dac26fb59e77d0354ccb81107a6da7b014d0",
      strip_prefix = "prometheus-cpp-8330b3f753fb774c9e0567baaac20ffb7042723b",
      urls = [
          "https://github.com/jupp0r/prometheus-cpp/archive/8330b3f753fb774c9e0567baaac20ffb7042723b.tar.gz",
      ],
  )

  _maybe(native.http_archive,
      name = "com_github_googlecartographer_async_grpc",
      strip_prefix = "async_grpc-c2c68f56904a595ab5ba24c1fb19b4b8e954fa15",
      urls = [
          "https://github.com/googlecartographer/async_grpc/archive/c2c68f56904a595ab5ba24c1fb19b4b8e954fa15.tar.gz",
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
    repo_rule(name=name, **kwargs)
