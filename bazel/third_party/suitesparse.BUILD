licenses(["notice"])

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "suitesparse",
    hdrs = glob([
        "suitesparse/*.h",
        "suitesparse/*.hpp",
    ]),
    includes = ["suitesparse"],
    linkopts = [
        "-lsuitesparseconfig",
        "-lcholmod",
        "-lumfpack",
    ],
)
