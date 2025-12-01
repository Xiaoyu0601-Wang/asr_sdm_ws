#!/usr/bin/env python3

import os
import subprocess
import sys
from pathlib import Path

PINOCCHIO_REPO_URL = "https://github.com/stack-of-tasks/pinocchio.git"
PINOCCHIO_BRANCH = os.environ.get("PINOCCHIO_BRANCH", "devel")
PINOCCHIO_SRC_DIR = Path(os.environ.get("PINOCCHIO_SRC_DIR", str(Path.home() / "src" / "pinocchio"))).expanduser()
PINOCCHIO_BUILD_DIR = PINOCCHIO_SRC_DIR / "build"

EIGENPY_REPO_URL = "https://github.com/stack-of-tasks/eigenpy.git"
EIGENPY_BRANCH = os.environ.get("EIGENPY_BRANCH", "devel")
EIGENPY_SRC_DIR = Path(os.environ.get("EIGENPY_SRC_DIR", str(Path.home() / "src" / "eigenpy"))).expanduser()
EIGENPY_BUILD_DIR = EIGENPY_SRC_DIR / "build"

INSTALL_PREFIX = Path(os.environ.get("PINOCCHIO_INSTALL_PREFIX", "/opt/openrobots")).expanduser()
BUILD_COLLISION = os.environ.get("PINOCCHIO_BUILD_COLLISION", "OFF").upper() == "ON"
PYTHON_VERSION = f"{sys.version_info.major}.{sys.version_info.minor}"
BASHRC_PATH = Path.home() / ".bashrc"

APT_DEPENDENCIES = [
    "build-essential",
    "cmake",
    "git",
    "pkg-config",
    "python3-dev",
    "python3-numpy",
    "python3-pip",
    "python3-venv",
    "python3-pybind11",
    "libeigen3-dev",
    "libboost-all-dev",
    "liburdfdom-dev",
    "libassimp-dev",
    "libtinyxml2-dev",
]


def run_cmd(cmd, check=True, **kwargs):
    """执行命令并打印日志。"""
    print(f"执行命令: {' '.join(str(c) for c in cmd)}")
    subprocess.run(cmd, check=check, **kwargs)


def ensure_dependencies():
    """安装 Pinocchio 源码构建所需的依赖。"""
    run_cmd(["sudo", "apt", "update"])
    run_cmd(["sudo", "apt", "install", "-qqy", *APT_DEPENDENCIES])


def update_submodules(src_dir):
    """更新仓库子模块。"""
    run_cmd(
        [
            "git",
            "-C",
            str(src_dir),
            "submodule",
            "update",
            "--init",
            "--recursive",
            "--depth",
            "1",
        ]
    )


def clone_or_update_repo(repo_url, branch, src_dir, with_submodules=False):
    """克隆或更新指定仓库，必要时更新子模块。"""
    if src_dir.exists():
        print(f"{src_dir} 已存在，执行 git fetch 更新。")
        run_cmd(["git", "-C", str(src_dir), "fetch", "--all", "--tags"])
        run_cmd(["git", "-C", str(src_dir), "checkout", branch])
        run_cmd(["git", "-C", str(src_dir), "pull", "--ff-only"])
        if with_submodules:
            update_submodules(src_dir)
        return

    src_dir.parent.mkdir(parents=True, exist_ok=True)
    run_cmd(
        [
            "git",
            "clone",
            "--branch",
            branch,
            repo_url,
            str(src_dir),
        ]
    )
    if with_submodules:
        update_submodules(src_dir)


def eigenpy_installed():
    """检测 eigenpy 是否已经安装。"""
    cmake_dir = INSTALL_PREFIX / "lib" / "cmake" / "eigenpy"
    return cmake_dir.exists()


def build_eigenpy():
    """构建 eigenpy（Pinocchio Python 绑定所需）。"""
    if eigenpy_installed():
        print("检测到 eigenpy 已安装，跳过构建。")
        return

    clone_or_update_repo(EIGENPY_REPO_URL, EIGENPY_BRANCH, EIGENPY_SRC_DIR)
    EIGENPY_BUILD_DIR.mkdir(parents=True, exist_ok=True)
    cmake_cmd = [
        "cmake",
        "-S",
        str(EIGENPY_SRC_DIR),
        "-B",
        str(EIGENPY_BUILD_DIR),
        "-DCMAKE_BUILD_TYPE=Release",
        f"-DCMAKE_INSTALL_PREFIX={INSTALL_PREFIX}",
        "-DBUILD_TESTING=OFF",
        "-DBUILD_PYTHON_INTERFACE=ON",
        f"-DPYTHON_EXECUTABLE={sys.executable}",
    ]
    run_cmd(cmake_cmd)
    cpu_count = os.cpu_count() or 2
    jobs = str(max(1, cpu_count // 2))
    run_cmd(
        [
            "cmake",
            "--build",
            str(EIGENPY_BUILD_DIR),
            "--config",
            "Release",
            "-j",
            jobs,
        ]
    )
    run_cmd(["sudo", "cmake", "--install", str(EIGENPY_BUILD_DIR)])


def configure_build():
    """使用 CMake 配置 Pinocchio 构建。"""
    PINOCCHIO_BUILD_DIR.mkdir(parents=True, exist_ok=True)
    cmake_cmd = [
        "cmake",
        "-S",
        str(PINOCCHIO_SRC_DIR),
        "-B",
        str(PINOCCHIO_BUILD_DIR),
        "-DCMAKE_BUILD_TYPE=Release",
        f"-DCMAKE_INSTALL_PREFIX={INSTALL_PREFIX}",
        "-DBUILD_TESTING=OFF",
        "-DINSTALL_DOCUMENTATION=OFF",
        "-DBUILD_EXAMPLES=OFF",
        f"-DBUILD_WITH_COLLISION_SUPPORT={'ON' if BUILD_COLLISION else 'OFF'}",
        "-DBUILD_PYTHON_INTERFACE=ON",
        f"-DPYTHON_EXECUTABLE={sys.executable}",
    ]
    run_cmd(cmake_cmd)


def build_and_install():
    """编译并安装 Pinocchio。"""
    cpu_count = os.cpu_count() or 2
    jobs = str(max(1, cpu_count // 2))
    run_cmd(
        [
            "cmake",
            "--build",
            str(PINOCCHIO_BUILD_DIR),
            "--config",
            "Release",
            "-j",
            jobs,
        ]
    )
    run_cmd(["sudo", "cmake", "--install", str(PINOCCHIO_BUILD_DIR)])


def update_bashrc():
    """在 ~/.bashrc 中追加环境变量。"""
    env_block = f"""
# >>> pinocchio source install setup >>>
export PATH={INSTALL_PREFIX}/bin:$PATH
export PKG_CONFIG_PATH={INSTALL_PREFIX}/lib/pkgconfig:$PKG_CONFIG_PATH
export LD_LIBRARY_PATH={INSTALL_PREFIX}/lib:$LD_LIBRARY_PATH
export PYTHONPATH={INSTALL_PREFIX}/lib/python{PYTHON_VERSION}/site-packages:$PYTHONPATH
export CMAKE_PREFIX_PATH={INSTALL_PREFIX}:$CMAKE_PREFIX_PATH
# <<< pinocchio source install setup <<<
"""
    if BASHRC_PATH.exists():
        with open(BASHRC_PATH, "r", encoding="utf-8") as f:
            if "pinocchio source install setup" in f.read():
                print("~/.bashrc 已包含 Pinocchio 环境变量，跳过追加。")
                return

    with open(BASHRC_PATH, "a", encoding="utf-8") as f:
        f.write(env_block)
    print("已将 Pinocchio 环境变量写入 ~/.bashrc，请重新加载或重新登录终端。")


def main():
    ensure_dependencies()
    clone_or_update_repo(
        PINOCCHIO_REPO_URL, PINOCCHIO_BRANCH, PINOCCHIO_SRC_DIR, with_submodules=True
    )
    build_eigenpy()
    configure_build()
    build_and_install()
    update_bashrc()
    print("Pinocchio 源码安装和环境配置完成。")


if __name__ == "__main__":
    main()

