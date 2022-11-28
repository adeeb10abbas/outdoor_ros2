workspace(name = "playground")

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

all_content = """filegroup(name = "all", srcs = glob(["**"]), visibility = ["//visibility:public"])"""

new_git_repository(
    name = "opencv",
    branch = "4.x",
    build_file_content = all_content,
    remote = "https://github.com/opencv/opencv",
)

new_git_repository(
    name = "sfml",
    branch = "main",
    build_file_content = all_content,
    remote = "https://github.com/SFML/SFML",
)
git_repository(
    name = "rules_foreign_cc",
    branch = "main",
    remote = "https://github.com/bazelbuild/rules_foreign_cc",
)

load("@rules_foreign_cc//foreign_cc:repositories.bzl", "rules_foreign_cc_dependencies")

rules_foreign_cc_dependencies()
############# TEST #############

COMMIT = "main"
CHECKSUM = ""

new_git_repository(
    name = "drake",
    branch = "master",
    build_file_content = all_content,
    remote = "https://github.com/RobotLocomotion/drake",
)


load("@drake//tools/workspace:github.bzl", "github_archive")

## Adding Bazel_ROS2_Rules for drake-ros stuff to work ##
DRAKE_ROS_commit = "main"
DRAKE_ROS_sha256 = "b819c470da68e525201585524479bd42bd79daacfaa2729cb2f32757fc62052c"


github_archive(
        name = "bazel_ros2_rules",
        repository = "RobotLocomotion/drake-ros",
        extra_strip_prefix = "bazel_ros2_rules",
        commit = DRAKE_ROS_commit,
        sha256 = DRAKE_ROS_sha256,
        mirrors = {
            "github":["https://github.com/RobotLocomotion/drake-ros/archive/main.tar.gz",],
        }
    )

load("@bazel_ros2_rules//deps:defs.bzl", "add_bazel_ros2_rules_dependencies")
add_bazel_ros2_rules_dependencies()

####### DRAKE STUFF #######


github_archive(
    name = "drake_ros_core",
    repository = "RobotLocomotion/drake-ros",
    extra_strip_prefix = "drake_ros_core",
    commit = DRAKE_ROS_commit,
    sha256 = DRAKE_ROS_sha256,
    mirrors = {
        "github":["https://github.com/RobotLocomotion/drake-ros/archive/main.tar.gz",],
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
        "github":["https://github.com/RobotLocomotion/drake-ros/archive/main.tar.gz",],
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
        "github":["https://github.com/RobotLocomotion/drake-ros/archive/main.tar.gz",],
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
] + [
    # These are possible RMW implementations. Uncomment one and only one to
    # change implementations
    "rmw_cyclonedds_cpp",
    # "rmw_fastrtps_cpp",
]
# Use ROS 2
ros2_archive(
    name = "ros2",
    include_packages = ROS2_PACKAGES,
    sha256_url = "https://repo.ros2.org/ci_archives/drake-ros-underlay/ros2-rolling-linux-focal-amd64-ci-CHECKSUM",  # noqa
    strip_prefix = "ros2-linux",
    url = "https://repo.ros2.org/ci_archives/drake-ros-underlay/ros2-rolling-linux-focal-amd64-ci.tar.bz2",  # noqa
)

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")
http_archive(
    name = "bazel_skylib",
    urls = [
        "https://mirror.bazel.build/github.com/bazelbuild/bazel-skylib/releases/download/1.3.0/bazel-skylib-1.3.0.tar.gz",
        "https://github.com/bazelbuild/bazel-skylib/releases/download/1.3.0/bazel-skylib-1.3.0.tar.gz",
    ],
    sha256 = "74d544d96f4a5bb630d465ca8bbcfe231e3594e5aae57e1edbf17a6eb3ca2506",
)
load("@bazel_skylib//:workspace.bzl", "bazel_skylib_workspace")
bazel_skylib_workspace()

load("@drake//tools/workspace:default.bzl", "add_default_workspace")

add_default_workspace()
