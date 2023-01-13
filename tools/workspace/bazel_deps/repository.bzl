load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")


def bazel_deps_repository():
    commit = "4af075fe9549f26063bbd6b3f2d039d9854fbdca"
    http_archive(
        name = "com_github_mjbots_bazel_deps",
        url = "https://github.com/mjbots/bazel_deps/archive/{}.zip".format(commit),
        # Try the following empty sha256 hash first, then replace with whatever
        # bazel says it is looking for once it complains.
        sha256 = "e96b28ac7d2b6b5583b78e3717f22b180ee6a514632e68f29451f3b7a6c17164",
        strip_prefix = "bazel_deps-{}".format(commit),
    )