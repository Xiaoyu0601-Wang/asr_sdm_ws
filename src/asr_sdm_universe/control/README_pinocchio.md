# Pinocchio 安装指南（Ubuntu 24.04 + ROS 2 Jazzy）

> 参考官方文档：https://stack-of-tasks.github.io/pinocchio/download.html  

## 1. 基础依赖

```bash
sudo apt update
sudo apt install build-essential cmake git pkg-config \
                 libeigen3-dev libboost-all-dev
```

## 2. 通过 robotpkg 安装（推荐）

1. 添加 robotpkg APT 源（Ubuntu 24.04 代号为 `noble`）：
   ```bash
   sudo sh -c 'echo "deb [arch=amd64] http://robotpkg.openrobots.org/wip/packages/debian/pub noble robotpkg" > /etc/apt/sources.list.d/robotpkg.list'
   sudo apt-key adv --fetch-keys http://robotpkg.openrobots.org/wip/packages/debian/robotpkg.key
   sudo apt update
   ```
2. 安装 Pinocchio 及 Python 绑定（Python3）：
   ```bash
   sudo apt install robotpkg-py3*-pinocchio
   ```
   - `py3*` 请替换为系统 Python 版本，Ubuntu 24.04 默认为 Python 3.12，可使用 `robotpkg-py312-pinocchio`。
   - 安装后将其 lib & pkgconfig 路径加入环境变量：
     ```bash
     export PATH=/opt/openrobots/bin:$PATH
     export LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH
     export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH
     export PYTHONPATH=/opt/openrobots/lib/python3.12/site-packages:$PYTHONPATH
     ```
   - 若使用 Bash，可将上述导出语句写入 `~/.bashrc`，并在 ROS 2 overlay 工作区中确保 `colcon` 构建前执行 `source ~/.bashrc`。
   - 若需要将 Pinocchio 链接到 ROS 2 Jazzy 的包，可在 `package.xml` 中声明 `pinocchio` 或在 `CMakeLists.txt` 加入 `find_package(pinocchio REQUIRED)`，并确保 `ament_target_dependencies` 中包含 `pinocchio`.

## 3. 通过 pip 安装（纯 Python 使用场景）

```bash
python3 -m pip install --upgrade pip
python3 -m pip install pin
```

> 该方式仅提供 Python API，不包含 C++ 头文件/库，若需要与本仓库 C++ 代码联动，请使用 robotpkg 或源码构建方式。

## 4. 源码编译（可选，高度自定义）

```bash
git clone --recursive https://github.com/stack-of-tasks/pinocchio.git
cd pinocchio
git checkout v3.2.2        # 选择与 ROS 2 Jazzy 兼容的版本
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release \
         -DBUILD_PYTHON_INTERFACE=ON \
         -DPYTHON_EXECUTABLE=$(which python3) \
         -DPYTHON_DEB_LAYOUT=ON               \
         -DCMAKE_INSTALL_PREFIX=/opt/pinocchio
make -j$(nproc)
sudo make install
```

安装路径若设为 `/opt/pinocchio`，请追加：

```bash
export LD_LIBRARY_PATH=/opt/pinocchio/lib:$LD_LIBRARY_PATH
export PKG_CONFIG_PATH=/opt/pinocchio/lib/pkgconfig:$PKG_CONFIG_PATH
export PYTHONPATH=/opt/pinocchio/lib/python3.12/site-packages:$PYTHONPATH
```

随后在 ROS 2 overlay 工作区的 `CMakeLists.txt` 中，通过 `find_package(pinocchio REQUIRED)` 并把 `${pinocchio_INCLUDE_DIRS}`、`${pinocchio_LIBRARIES}` 链接到对应 target。

## 5. 安装验证

### C++ 头文件

```bash
pkg-config --modversion pinocchio
```

### Python 模块

```bash
python3 - <<'PY'
import pinocchio
print("Pinocchio version:", pinocchio.__version__)
PY
```


