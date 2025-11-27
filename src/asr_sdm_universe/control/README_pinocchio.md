# Pinocchio 安装脚本使用指南（Ubuntu 24.04 + ROS 2 Jazzy）

> 参考官方文档：https://stack-of-tasks.github.io/pinocchio/download.html  

## 0. 前置环境

- 操作系统：Ubuntu 24.04 LTS（x86_64）
- 中间件：ROS 2 Jazzy + 已创建的 `~/asr_sdm_ws`工作区
- Python：系统默认 3.12（脚本会安装 `robotpkg-py312-*` 系列包）
- 基础工具：`build-essential`、`cmake`、`git`、`python3-venv` 等（Ubuntu 默认已提供；若缺少可 `sudo apt install build-essential cmake git python3-venv`）
- 需具备 sudo 权限以执行 apt 安装

## 1. 脚本位置与功能

- 入口脚本：`dependency.repos.py`
- 功能：自动完成 robotpkg 仓库配置、Pinocchio 及其依赖（包含 `pinocchio` C++/Python 绑定、COAL、eigenpy 等）的安装，并将 `/opt/openrobots` 的相关路径永久写入 `~/.bashrc`。

运行脚本后，新开终端会自动加载这些环境变量，无需再手动 `source ~/.bashrc`。

## 2. 使用方法

```bash
cd ~/asr_sdm_ws
python3 dependency.repos.py
运行脚本完成后，运行: source ~/.bashrc，或者新开终端加载环境变量。
```

脚本会依次执行：
1. `sudo apt update && sudo apt install lsb-release curl`
2. 配置 robotpkg keyring 与 `robotpkg.list`
3. 安装 Pinocchio、COAL、eigenpy（含 Python 绑定），并使用 `--reinstall` 确保缺失文件可被覆盖
4. 在 `~/.bashrc` 追加以下区块

```bash
# >>> pinocchio robotpkg setup >>>
export PATH=/opt/openrobots/bin:$PATH
export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH
export LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH
export PYTHONPATH=/opt/openrobots/lib/python3.12/site-packages:$PYTHONPATH
export CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH
# <<< pinocchio robotpkg setup <<<
```

## 3. 使用提示

- **权限**：脚本需多次调用 `sudo`；若远程或非交互环境，请先 `sudo -v` 或以具备 sudo 权限的终端运行。
- **环境变量**：脚本重复运行会自动检测是否已存在上述区块，避免重复追加；如需清理，可删除该区块并手动卸载 `/opt/openrobots`。
- **ROS 集成**：安装完成后，`colcon build` 前仅需 `source ~/asr_sdm_ws/install/setup.bash`；Pinocchio 相关包（例如 `asr_sdm_kinematic_dynamic_model`）无需额外配置即可链接 `/opt/openrobots`。
- **验证**：
  ```bash
  python3 -c "import pinocchio; print(pinocchio.__version__)"
  pkg-config --modversion pinocchio
  ```

若安装失败或缺少动态库，直接重新运行脚本即可强制覆盖缺失文件。


