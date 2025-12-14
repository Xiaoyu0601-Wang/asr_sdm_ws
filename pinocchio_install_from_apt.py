#!/usr/bin/env python3

import subprocess
import sys
from pathlib import Path

def get_python_version():
    """Get the system Python version as 'major.minor' string (e.g., '3.12')."""
    return f"{sys.version_info.major}.{sys.version_info.minor}"

def get_python_version_short():
    """Get Python version without dot for package names (e.g., '312')."""
    return f"{sys.version_info.major}{sys.version_info.minor}"

ROBOTPKG_KEY_URL = "http://robotpkg.openrobots.org/packages/debian/robotpkg.asc"
ROBOTPKG_KEY_PATH = Path("/etc/apt/keyrings/robotpkg.asc")
ROBOTPKG_LIST_PATH = Path("/etc/apt/sources.list.d/robotpkg.list")
PYTHON_VERSION = get_python_version()
PYTHON_VERSION_SHORT = get_python_version_short()
PINOCCHIO_PACKAGES = [
    "robotpkg-pinocchio",
    f"robotpkg-py{PYTHON_VERSION_SHORT}-pinocchio",
    "robotpkg-coal",
    f"robotpkg-py{PYTHON_VERSION_SHORT}-coal",
    f"robotpkg-py{PYTHON_VERSION_SHORT}-eigenpy",
]
BASHRC_PATH = Path.home() / ".bashrc"

def run_cmd(cmd, check=True, **kwargs):
    """Run shell command with logging."""
    print(f"执行命令: {' '.join(cmd)}")
    subprocess.run(cmd, check=check, **kwargs)

def ensure_dependencies():
    """Install prerequisites for robotpkg repo registration."""
    run_cmd(["sudo", "apt", "update"])
    run_cmd(["sudo", "apt", "install", "-qqy", "lsb-release", "curl"])

def get_ubuntu_codename():
    """Return Ubuntu codename string (e.g., noble)."""
    return subprocess.check_output(["lsb_release", "-cs"], text=True).strip()

def add_robotpkg_repo():
    """Configure robotpkg apt repository and key."""
    run_cmd(["sudo", "mkdir", "-p", "/etc/apt/keyrings"])
    run_cmd([
        "sudo",
        "curl",
        "-fsSL",
        ROBOTPKG_KEY_URL,
        "-o",
        str(ROBOTPKG_KEY_PATH),
    ])

    codename = get_ubuntu_codename()
    repo_line = (
        f"deb [arch=amd64 signed-by={ROBOTPKG_KEY_PATH}] "
        f"http://robotpkg.openrobots.org/packages/debian/pub {codename} robotpkg"
    )

    if ROBOTPKG_LIST_PATH.exists():
        with open(ROBOTPKG_LIST_PATH, "r", encoding="utf-8") as f:
            content = f.read()
        if repo_line in content:
            print("robotpkg 源已存在，跳过写入。")
        else:
            run_cmd(
                ["sudo", "tee", "-a", str(ROBOTPKG_LIST_PATH)],
                text=True,
                input=repo_line + "\n",
            )
    else:
        run_cmd(
            ["sudo", "tee", str(ROBOTPKG_LIST_PATH)],
            text=True,
            input=repo_line + "\n",
        )

    run_cmd(["sudo", "apt", "update"])

def install_pinocchio():
    """Install Pinocchio and python bindings via robotpkg."""
    run_cmd(["sudo", "apt", "install", "--reinstall", "-qqy", *PINOCCHIO_PACKAGES])

def update_bashrc():
    """Append environment variables needed for Pinocchio."""
    env_block = f"""
# >>> pinocchio robotpkg setup >>>
export PATH=/opt/openrobots/bin:$PATH
export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH
export LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH
export PYTHONPATH=/opt/openrobots/lib/python{PYTHON_VERSION}/site-packages:$PYTHONPATH
export CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH
# <<< pinocchio robotpkg setup <<<
"""
    if BASHRC_PATH.exists():
        with open(BASHRC_PATH, "r", encoding="utf-8") as f:
            if "pinocchio robotpkg setup" in f.read():
                print("~/.bashrc 已包含 Pinocchio 环境变量，跳过追加。")
                return

    with open(BASHRC_PATH, "a", encoding="utf-8") as f:
        f.write(env_block)
    print("已将 Pinocchio 环境变量写入 ~/.bashrc，请重新加载或重新登录终端。")

def main():
    ensure_dependencies()
    add_robotpkg_repo()
    install_pinocchio()
    update_bashrc()
    print("Pinocchio 安装和环境配置完成。")

if __name__ == "__main__":
    main()
