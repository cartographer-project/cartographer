package(default_visibility = ["//visibility:public"])

cc_library(
    name = "src",
    hdrs = glob(["Eigen/src/**/*"]),
)

cc_library(
    name = "core",
    hdrs = glob(['Eigen/*']),
    deps = ["src"],
)
