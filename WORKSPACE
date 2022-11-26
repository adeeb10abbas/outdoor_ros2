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


############# TEST #############

COMMIT = "main"
CHECKSUM = ""
http_archive(
    name = "com_github_mvukov_rules_ros2",
    urls = [
        "https://github.com/mvukov/rules_ros2/archive/main.tar.gz",
    ],
    sha256 = CHECKSUM,
    strip_prefix = "rules_ros2-" + COMMIT,
)
all_content = """filegroup(name = "all", srcs = glob(["**"]), visibility = ["//visibility:public"])"""

new_git_repository(
    name = "opencv",
    branch = "master",
    build_file_content = all_content,
    remote = "https://github.com/opencv/opencv",
)

new_git_repository(
    name = "sfml",
    branch = "master",
    build_file_content = all_content,
    remote = "https://github.com/SFML/SFML",
)


## Adding Bazel_ROS2_Rules for drake-ros stuff to work ##

new_local_repository(
    name = "bazel_ros2_rules",
    path = "bazel_ros2_rules",
    build_file = "BUILD.bazel",
)
load("@bazel_ros2_rules//deps:defs.bzl", "add_bazel_ros2_rules_dependencies")
add_bazel_ros2_rules_dependencies()

load("@com_github_mvukov_rules_ros2//repositories:repositories.bzl", "ros2_repositories")
ros2_repositories()

load("@com_github_mvukov_rules_ros2//repositories:deps.bzl", "PIP_ANNOTATIONS", "ros2_deps")

ros2_deps()

load("@rules_python//python:repositories.bzl", "python_register_toolchains")

python_register_toolchains(
    name = "rules_ros2_python",
    python_version = "3.8.13",
)

load("@rules_python//python:pip.bzl", "pip_parse")
load("@rules_ros2_python//:defs.bzl", python_interpreter_target = "interpreter")

pip_parse(
    name = "rules_ros2_pip_deps",
    annotations = PIP_ANNOTATIONS,
    python_interpreter_target = python_interpreter_target,
    requirements_lock = "@com_github_mvukov_rules_ros2//:requirements_lock.txt",
)

load(
    "@rules_ros2_pip_deps//:requirements.bzl",
    install_rules_ros2_pip_deps = "install_deps",
)

install_rules_ros2_pip_deps()

# Below are listed only deps needed by examples: if you just need ROS2 you don't
# need to import/load anything below.

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

# rules_docker is not strictly necessary, but interesting if you want to create
# and/or push docker containers for the examples. The docker executable is
# needed only if you want to run an image using Bazel.
http_archive(
    name = "io_bazel_rules_docker",
    sha256 = "b1e80761a8a8243d03ebca8845e9cc1ba6c82ce7c5179ce2b295cd36f7e394bf",
    urls = ["https://github.com/bazelbuild/rules_docker/releases/download/v0.25.0/rules_docker-v0.25.0.tar.gz"],
)

load(
    "@io_bazel_rules_docker//repositories:repositories.bzl",
    container_repositories = "repositories",
)

container_repositories()

load("@io_bazel_rules_docker//repositories:deps.bzl", container_deps = "deps")

container_deps()

load(
    "@io_bazel_rules_docker//container:container.bzl",
    "container_pull",
)

container_pull(
    name = "ros_deploy_base",
    digest = "sha256:54967c8f59e8607cd4a40c0d614b3391bf71112482f2e344d93ff455f60b3723",
    registry = "docker.io",
    repository = "mvukov/ros-deploy-base",
)
