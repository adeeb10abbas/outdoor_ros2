## Author : @adeeb10abbas
## Date : 2022-31-12
## Description : This file is used to build the workspace for the project
## LICENSE : MIT

workspace(name = "outdoorSLAM")

load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository", "new_git_repository")
load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

git_repository(
    name = "com_google_benchmark",
    branch = "main",
    remote = "https://github.com/google/benchmark",
)

git_repository(
    name = "com_google_googletest",
    branch = "main",
    remote = "https://github.com/google/googletest",
)

git_repository(
    name = "com_github_google_rules_install",
    branch = "main",
    remote = "https://github.com/google/bazel_rules_install",
)

load("@com_github_google_rules_install//:deps.bzl", "install_rules_dependencies")

install_rules_dependencies()

load("@com_github_google_rules_install//:setup.bzl", "install_rules_setup")

install_rules_setup()

############# DRAKE #############
DRAKE_TAG = "v1.10.0"
DRAKE_CHECKSUM = "78bd251bcfb349c988ee9225175a803a50cc53eaacdeb3bba200dfc82dcea305"  # noqa

http_archive(
    name = "drake",
    sha256 = DRAKE_CHECKSUM,
    strip_prefix = "drake-{}".format(DRAKE_TAG.lstrip("v")),
    urls = [
        "https://github.com/RobotLocomotion/drake/archive/refs/tags/{}.tar.gz".format(DRAKE_TAG),  # noqa
    ],
)

load("@drake//tools/workspace:default.bzl", "add_default_workspace")
load("@drake//tools/workspace:github.bzl", "github_archive")

add_default_workspace()
##########################

##### DRAKE ROS #####
## Adding Bazel_ROS2_Rules for drake-ros stuff to work ##
DRAKE_ROS_commit = "7867b38e9b321823c692e43894963d971510dedf"
DRAKE_ROS_sha256 = "1948da0d0912857a6b61c1f57c153a6d74d80b49b90e0edfd114e4efcc0bcd56"
## Ref: ECousineau's awesome script - 
## https://github.com/EricCousineau-TRI/repro/blob/50c3f52c6b745f686bef9567568437dc609a7f91/bazel/bazel_hash_and_cache.py


github_archive(
        name = "bazel_ros2_rules",
        repository = "RobotLocomotion/drake-ros",
        extra_strip_prefix = "bazel_ros2_rules",
        commit = DRAKE_ROS_commit,
        sha256 = DRAKE_ROS_sha256,
        mirrors = {
            "github":["https://github.com/RobotLocomotion/drake-ros/ref/tags/main.tar.gz",],
        }
    )

load("@bazel_ros2_rules//deps:defs.bzl", "add_bazel_ros2_rules_dependencies")
add_bazel_ros2_rules_dependencies()

github_archive(
    name = "drake_ros_core",
    repository = "RobotLocomotion/drake-ros",
    extra_strip_prefix = "drake_ros_core",
    commit = DRAKE_ROS_commit,
    sha256 = DRAKE_ROS_sha256,
    mirrors = {
        "github":["https://github.com/RobotLocomotion/drake-ros/refs/tags/main.tar.gz",],
    }
)

github_archive(
    name = "drake_ros_tf2",
    repository = "RobotLocomotion/drake-ros",
    extra_strip_prefix = "drake_ros_tf2",
    # TODO(drake-ros#158): Use provided BUILD file.
    commit = DRAKE_ROS_commit,
    sha256 = DRAKE_ROS_sha256,
    mirrors = {
        "github":["https://github.com/RobotLocomotion/drake-ros/refs/tags/main.tar.gz",],
    }
)

github_archive(
    name = "drake_ros_viz",
    repository = "RobotLocomotion/drake-ros",
    extra_strip_prefix = "drake_ros_viz",
    # TODO(drake-ros#158): Use provided BUILD file.
    commit = DRAKE_ROS_commit,
    sha256 = DRAKE_ROS_sha256,
    mirrors = {
        "github":["https://github.com/RobotLocomotion/drake-ros/refs/tags/main.tar.gz",],
    }
)

load("@bazel_ros2_rules//deps:defs.bzl", "add_bazel_ros2_rules_dependencies")
add_bazel_ros2_rules_dependencies()

load("@bazel_ros2_rules//ros2:defs.bzl", "ros2_archive")
load("@bazel_ros2_rules//ros2:defs.bzl", "ros2_local_repository")

ROS2_PACKAGES = [
    "action_msgs",
    "builtin_interfaces",
    "console_bridge_vendor",
    "rclcpp",
    "rclcpp_action",
    "rclpy",
    "ros2cli",
    "ros2cli_common_extensions",
    "rosidl_default_generators",
    "tf2_ros",
    "tf2_ros_py",
    "visualization_msgs",
    "rosidl_default_runtime",
    "image_transport",
    "vision_opencv",
    "cv_bridge",
] + [
    # These are possible RMW implementations. Uncomment one and only one to
    # change implementations
    "rmw_cyclonedds_cpp",
    # "rmw_fastrtps_cpp",
]
# Use ROS 2
ros2_local_repository(
    name = "ros2",
    workspaces = [ "/home/arrowhead/ros2_ws/install", "/opt/ros/humble",],
    include_packages = ROS2_PACKAGES,
)

## Additional Libraries ## See list here: https://github.com/mjbots/bazel_deps
load("//tools/workspace:default.bzl", "add_default_repositories")
add_default_repositories()
load("@com_github_mjbots_bazel_deps//tools/workspace:default.bzl",
     bazel_deps_add = "add_default_repositories")
bazel_deps_add()
