# Pinocchio 安装说明

本说明覆盖仓库中两套 Pinocchio 安装脚本：

- `pinocchio_set_up_repos.py`：通过 robotpkg/apt 提供的二进制包快速安装。
- `install_pinocchio_from_source.py`：从 GitHub 源码编译安装，可选择自定义分支与安装路径。

Pinocchio 项目地址：[stack-of-tasks/pinocchio](https://github.com/stack-of-tasks/pinocchio)。

---

## 1. 先决条件

1. Ubuntu 24.04 jazzy

执行脚本前建议更新仓库并确认 `python3` 可用：

```bash
cd /home/wwlwwl/asr_sdm_ws
python3 --version
```

---

## 2. apt/robotpkg 安装（`pinocchio_set_up_repos.py`）

### 2.1 功能

- 添加 robotpkg apt 源与公钥。
- 安装 `robotpkg-pinocchio`、`robotpkg-py312-pinocchio` 及相关依赖包。
- 在 `~/.bashrc` 追加 `/opt/openrobots` 环境变量块（标签为 `pinocchio robotpkg setup`）。

### 2.2 使用步骤

```bash
cd /home/wwlwwl/asr_sdm_ws
python3 pinocchio_set_up_repos.py
```

脚本会提示输入 `sudo` 密码以执行 `apt update/install`。运行完毕后重新登录终端或 `source ~/.bashrc`。

### 2.3 适用场景

- 需要快速、稳定、由 robotpkg 维护的 Pinocchio 版本。
- 希望由 apt 统一管理升级/卸载。

---

## 3. 源码编译安装（`install_pinocchio_from_source.py`）

### 3.1 功能

- 安装编译依赖（`build-essential`、`cmake`、`libeigen3-dev`、Boost、Assimp、TinyXML2、`python3-pybind11` 等）。
- 自动克隆/构建 `eigenpy`（Pinocchio Python 绑定所需），默认安装到与 Pinocchio 相同的前缀。
- 克隆或更新 Pinocchio 仓库并同步其 `cmake/` 子模块（默认 `devel` 分支，可配置）。
- 运行 CMake 配置，开启 Python 绑定与碰撞支持，关闭测试/示例/文档以加快构建。
- `cmake --build ... --install` 安装到指定前缀（默认 `/opt/openrobots`）。
- 写入标记为 `pinocchio source install setup` 的环境变量块。

### 3.2 关键环境变量

| 环境变量 | 默认值 | 说明 |
| --- | --- | --- |
| `PINOCCHIO_BRANCH` | `devel` | Pinocchio Git 分支/标签/提交 |
| `PINOCCHIO_SRC_DIR` | `~/src/pinocchio` | Pinocchio 源码目录 |
| `EIGENPY_BRANCH` | `devel` | eigenpy Git 分支 |
| `EIGENPY_SRC_DIR` | `~/src/eigenpy` | eigenpy 源码目录 |
| `PINOCCHIO_INSTALL_PREFIX` | `/opt/openrobots` | eigenpy 与 Pinocchio 共用的安装前缀 |
| `PINOCCHIO_BUILD_COLLISION` | `OFF` | 是否启用碰撞支持（需系统已有 hpp-fcl） |

示例：安装指定标签 `v3.8.0` 且自定义路径

```bash
export PINOCCHIO_BRANCH=v3.8.0
export PINOCCHIO_SRC_DIR=/home/wwlwwl/dev/pinocchio
export PINOCCHIO_INSTALL_PREFIX=/opt/pinocchio-3.8
python3 install_pinocchio_from_source.py
```

### 3.3 CMake 配置项

```text
-DCMAKE_BUILD_TYPE=Release
-DCMAKE_INSTALL_PREFIX=<INSTALL_PREFIX>
-DBUILD_TESTING=OFF
-DINSTALL_DOCUMENTATION=OFF
-DBUILD_EXAMPLES=OFF
-DBUILD_WITH_COLLISION_SUPPORT=ON/OFF（由 `PINOCCHIO_BUILD_COLLISION` 控制，默认 OFF）
-DBUILD_PYTHON_INTERFACE=ON
-DPYTHON_EXECUTABLE=<当前 Python>
```

### 3.3.1 选项说明

| 选项 | 值 | 作用 |
| --- | --- | --- |
| `CMAKE_BUILD_TYPE` | `Release` | 使用优化构建，提升运行效率。可改为 `Debug` 进行调试。 |
| `CMAKE_INSTALL_PREFIX` | 见环境变量 | 指定安装根目录，Pinocchio 与 eigenpy 都会安装到该前缀。 |
| `BUILD_TESTING` | `OFF` | 跳过 Pinocchio 自带测试，减少构建时间。需要测试时可设为 `ON` 后手动执行 `ctest`。 |
| `INSTALL_DOCUMENTATION` | `OFF` | 不生成 Doxygen 文档，避免额外依赖。 |
| `BUILD_EXAMPLES` | `OFF` | 不编译 C++ 示例，可在后续需要时改为 `ON`。 |
| `BUILD_WITH_COLLISION_SUPPORT` | `ON/OFF` | 控制 hpp-fcl 相关功能，脚本根据 `PINOCCHIO_BUILD_COLLISION` 设置；开启前请确保系统已安装 `hpp-fcl` 并加入 `CMAKE_PREFIX_PATH`。 |
| `BUILD_PYTHON_INTERFACE` | `ON` | 编译 Python 绑定，必要时可关掉仅保留 C++ 库。 |
| `PYTHON_EXECUTABLE` | `/usr/bin/python3` | 明确绑定使用的 Python 解释器。 |

脚本也会以相同风格配置 eigenpy：Release 构建、关闭测试、开启 Python 绑定，并使用与 Pinocchio 相同的安装前缀和 Python 解释器。若需要对 eigenpy 额外定制，可修改 `build_eigenpy()` 中的 CMake 选项。

若需要开启碰撞支持（`PINOCCHIO_BUILD_COLLISION=ON`），请确保系统已安装 `hpp-fcl`（可通过 robotpkg、源码或发行版包获得），并将其安装路径加入 `CMAKE_PREFIX_PATH`，否则 CMake 会提示未找到 `hpp-fcl`.
### 3.4 使用步骤

```bash
cd /home/wwlwwl/asr_sdm_ws
python3 install_pinocchio_from_source.py
```
```bash
cd /home/wwlwwl/asr_sdm_ws
python3 install_pinocchio_from_source.py
```

脚本会多次调用 `sudo`（安装依赖、安装文件到 `/opt`）。完成后刷新 `~/.bashrc`：

```bash
source ~/.bashrc
```

### 3.5 适用场景

- 需要最新的 `devel` 分支或指定版本。
- 需要修改 Pinocchio 源码或在本地打补丁。
- 需要与 apt 版本共存，使用单独前缀进行隔离。

---

## 4. 环境变量说明

两种脚本都会向 `~/.bashrc` 追加以下变量（路径由安装前缀决定）：

```bash
export PATH=/opt/openrobots/bin:$PATH
export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH
export LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH
export PYTHONPATH=/opt/openrobots/lib/python<版本>/site-packages:$PYTHONPATH
export CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH
```

当同时使用 apt 与源码安装时，环境块标签不同，可自行注释/删除对应段落以避免重复导出。

---

## 5. 常见问题

1. **网络/证书错误**：检查公司代理或在运行脚本前设置 `http_proxy/https_proxy`。
2. **缺少 sudo 权限**：脚本需要写 `/etc/apt` 或 `/opt`，若无权限请联系管理员或修改 `PINOCCHIO_INSTALL_PREFIX` 到用户目录，并手动移除 `sudo cmake --install` 中的 `sudo`。
3. **旧版本遗留**：若 `/opt/openrobots` 已存在旧文件，可备份或清理后再安装。
4. **卸载方式**：
   - apt 版本：`sudo apt remove 'robotpkg-*pinocchio*'`。
   - 源码版本：重新运行 `cmake --install` 时添加 `-DCMAKE_INSTALL_PREFIX` 指向目标，然后手动删除安装目录；或在 `build` 目录执行 `sudo cmake --build . --target uninstall`（若生成了 uninstall 目标）。

---

## 6. 验证安装

```bash
source ~/.bashrc
python3 -c "import pinocchio; print(pinocchio.__version__)"
```

若输出版本号且无错误，即安装完成。也可执行 `/opt/openrobots/bin/gepetto-gui` 或其它依赖 Pinocchio 的程序进一步验证。

