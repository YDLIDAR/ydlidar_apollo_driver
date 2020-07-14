load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "libydlidar_driver_component.so",
    linkopts = ["-shared"],
    linkstatic = False,
    deps = [":ydlidar_driver_component_lib"],
)

cc_library(
    name = "ydlidar_driver_component_lib",
    srcs = ["ydlidar_driver_component.cc"],
    hdrs = ["ydlidar_driver_component.h"],
    copts = ['-DMODULE_NAME=\\"ydlidar\\"'],
    linkopts = [
        "-lydlidar_sdk",
    ],
    deps = [
        "//cyber",
        "//modules/common/util",
        "//modules/drivers/ydlidar/proto:ydlidar_proto",
        "//modules/drivers/proto:sensor_proto",
    ],
)

cc_test(
    name = "driver_test",
    size = "small",
    srcs = ["driver_test.cc"],
    copts = ['-DMODULE_NAME=\\"ydlidar\\"'],
    linkopts = [
        "-lydlidar_sdk",
    ],
    deps = [
        "//modules/drivers/ydlidar/proto:ydlidar_proto",
        "@gtest//:main",
    ],
)

cpplint()
