load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "ydlidar_proto",
    deps = [
        ":sensor_ydlidar_proto",
    ],
)

cc_proto_library(
    name = "sensor_ydlidar_proto",
    deps = [
        ":ydlidar_proto_lib",
    ],
)

proto_library(
    name = "ydlidar_proto_lib",
    srcs = [
        "config.proto",
        "ydlidar.proto",
    ],
    deps = [
        "//modules/common/proto:header_proto_lib",
    ],
)

cpplint()
