# OpenEB 编译指南

> 本文档分为两部分：**第一部分**编译 OpenEB SDK（`openeb/` 子目录），**第二部分**编译 GUI 应用（`gui/` + `algo/`）。OpenEB SDK 是 GUI 的底层依赖，须先编译。

## 系统环境

| 项目       | 版本                    |
|------------|-------------------------|
| 操作系统   | Ubuntu 26.04            |
| 架构       | amd64 (x86_64)          |
| GCC        | 15.x（系统默认）        |
| CMake      | 4.2.3（最低要求 3.16）  |
| C++ 标准   | C++17（`CMAKE_CXX_STANDARD 17`）|
| Python     | 3.14（系统默认，不兼容 openeb） |
| 编译用 Python | 3.12（via deadsnakes PPA） |
| Qt         | 6.x（GUI 应用必需）     |
| OpenCV     | 4.x                     |

## 注意事项

1. **Python 版本**：OpenEB 官方仅支持 Python 3.9~3.12，系统自带 Python 3.14 不兼容（`numba` 等依赖限制）。需通过 deadsnakes PPA 安装 Python 3.12。
2. **GCC 15 兼容性**：GCC 15 不再隐式包含 `<cstdint>`，导致 `uint8_t`、`uint16_t` 等类型未声明。需在 CMakeLists.txt 中添加全局编译选项修复。
3. **包名变化**：Ubuntu 26 中 `libcanberra-gtk-module` 已替换为 `libcanberra-gtk3-module`。

## 编译步骤

### 1. 安装系统依赖

```bash
sudo apt update
sudo apt -y install apt-utils build-essential software-properties-common wget unzip curl git cmake
# libusb-1.0-0-dev 为可选项：未安装时 inivation（DAVIS/DVXplorer）设备层自动编译剔除，
# 其余功能不受影响（v3.0 起）。
sudo apt -y install libopencv-dev libboost-all-dev libusb-1.0-0-dev libprotobuf-dev protobuf-compiler
sudo apt -y install libhdf5-dev hdf5-tools libglew-dev libglfw3-dev libcanberra-gtk3-module ffmpeg
sudo apt -y install libgl-dev libglx-dev libopengl-dev
# 可选（测试用）：
sudo apt -y install libgtest-dev libgmock-dev
```

### 2. 安装 Python 3.12（via deadsnakes PPA）

```bash
sudo apt install software-properties-common
sudo add-apt-repository ppa:deadsnakes/ppa
sudo apt update
sudo apt install python3.12 python3.12-venv python3.12-dev
```

### 3. 安装 pybind11 v2.11.0

```bash
cd /tmp
wget https://github.com/pybind/pybind11/archive/v2.11.0.zip
unzip v2.11.0.zip
cd pybind11-2.11.0/
mkdir build && cd build
cmake .. -DPYBIND11_TEST=OFF -DPython3_EXECUTABLE=/usr/bin/python3.12
cmake --build .
sudo cmake --build . --target install
```

### 4. 创建 Python 虚拟环境并安装依赖

```bash
python3.12 -m venv /tmp/prophesee/py3venv --system-site-packages
/tmp/prophesee/py3venv/bin/python -m pip install pip --upgrade
/tmp/prophesee/py3venv/bin/python -m pip install -r OPENEB_SRC_DIR/utils/python/requirements_openeb.txt
```

> ML 依赖（`requirements_pytorch_cpu.txt`）可选安装，其中 `torch==2.9.1` 需确认是否支持 Python 3.12。

### 5. 修复 GCC 15 兼容性问题

> **注**：此修复已预先应用到本仓库的 `openeb/CMakeLists.txt`（第 24-27 行）和根 `CMakeLists.txt`（第 16-18 行），无需手动添加。以下说明仅供理解原理。

在 `OPENEB_SRC_DIR/CMakeLists.txt` 的 `project()` 行之后添加（如尚未存在）：

```cmake
# GCC 15+ no longer implicitly includes <cstdint>; add it globally to fix uint8_t/uint16_t etc.
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL "15")
    add_compile_options("-include;cstdint")
endif()
```

### 6. 编译

```bash
cd OPENEB_SRC_DIR
rm -rf build
mkdir build && cd build
cmake .. -DBUILD_TESTING=OFF -DPython3_EXECUTABLE=/tmp/prophesee/py3venv/bin/python3.12
cmake --build . --config Release -- -j$(nproc)
```

### 7. 配置环境变量（选择一种方式）

**方式一：从 build 目录直接使用**

```bash
source OPENEB_SRC_DIR/build/utils/scripts/setup_env.sh
# 可添加到 ~/.bashrc 使其永久生效
```

**方式二：部署到系统路径**

```bash
sudo cmake --build . --target install
# 然后设置环境变量（添加到 ~/.bashrc）：
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib
export HDF5_PLUGIN_PATH=$HDF5_PLUGIN_PATH:/usr/local/lib/hdf5/plugin
```

## 遇到的问题与解决方案

| 问题                                  | 原因                                        | 解决方案                                           |
|---------------------------------------|---------------------------------------------|----------------------------------------------------|
| `numba` 安装失败（Python 3.14）       | numba 仅支持 Python >=3.9,<3.13            | 使用 deadsnakes PPA 安装 Python 3.12              |
| `uint16_t`/`uint8_t` 未声明           | GCC 15 不再隐式包含 `<cstdint>`            | CMakeLists.txt 添加 `-include cstdint`            |
| `libcanberra-gtk-module` 无候选       | Ubuntu 26 中包名已变更                      | 使用 `libcanberra-gtk3-module`                    |
| `OpenGL` 库找不到                     | 未安装 OpenGL 开发库                        | 安装 `libgl-dev libglx-dev libopengl-dev`         |
| `GLEW` 库找不到                       | 未安装 GLEW 开发库                          | 安装 `libglew-dev`                                |
| `glfw3` 配置文件找不到                | 未安装 GLFW3 开发库                         | 安装 `libglfw3-dev`                               |

---

## 第二部分：GUI 应用编译

本部分编译 EBplus GUI 应用（`gui/` + `algo/` 目录）。前提条件：OpenEB SDK 已按第一部分编译并部署（`source OPENEB_SRC_DIR/build/utils/scripts/setup_env.sh` 或 `sudo make install`）。

### G1. 安装 GUI 额外依赖

```bash
# Qt 6（Widgets + OpenGL + OpenGLWidgets）
sudo apt -y install qt6-base-dev qt6-base-dev-tools libqt6opengl6-dev

# ONNX Runtime（E2VID 神经网络推理，可选但推荐）
# 见下方 G4 节单独安装步骤

# Google Test（GUI 单元测试，可选）
sudo apt -y install libgtest-dev libgmock-dev
```

### G2. 编译 GUI 应用

```bash
cd /path/to/GUI-for-openEB

# 配置（CMake 会自动检测 OpenEB SDK、Qt6、OpenCV、ONNX Runtime）
cmake -B build -DCMAKE_BUILD_TYPE=Release

# 编译
cmake --build build -- -j$(nproc)
```

**CMake 选项**：

| 选项 | 默认 | 说明 |
|------|------|------|
| `GUI_BUILD_TESTS` | `ON` | 编译 GUI 单元测试（`gui/tests/`） |
| `CMAKE_BUILD_TYPE` | — | 推荐 `Release` |

### G3. 运行 GUI

```bash
# 推荐方式：使用启动脚本（自动处理 Wayland 兼容、HAL 插件路径、OpenGL 后端）
./run.sh

# 或直接运行（需自行设置环境变量）
./build/gui/gui_for_openeb
```

`run.sh` 自动设置以下环境变量：
- `QT_QPA_PLATFORM=xcb`（Wayland 下强制 XCB 兼容）
- `QSG_RHI_BACKEND=opengl`（强制 OpenGL 渲染后端）
- HAL 插件路径（`MV_HAL_PLUGIN_PATH`）

### G4. ONNX Runtime 安装（E2VID 推理）

E2VID 模式使用 ONNX Runtime 进行神经网络推理。如未安装，E2VID 自动降级为启发式模式（voxel-grid sum + sigmoid）。

```bash
cd /path/to/GUI-for-openEB
mkdir -p third_party/onnxruntime && cd third_party/onnxruntime
wget https://github.com/microsoft/onnxruntime/releases/download/v1.19.2/onnxruntime-linux-x64-1.19.2.tgz
tar xzf onnxruntime-linux-x64-1.19.2.tgz --strip-components=1
cd ../..

# 安装后重新编译，CMake 会自动检测 third_party/onnxruntime/
cmake --build build -- -j$(nproc)
```

模型权重转换（PyTorch → ONNX）详见 [README.md](../README.md) §E2VID Neural Network Reconstruction。

### G4b. OpenVINO 安装（E2VID Intel 核显加速，可选）

安装后 E2VID 的推理设备 Auto/GPU 会通过 OpenVINO 在 Intel 核显上运行神经网络（实测比 CPU 快约 11 倍，同一份 .onnx 模型，无需重新转换）；未安装时自动回退 ONNX Runtime CPU，功能无损。依赖核显的用户态驱动（Ubuntu 26.04：`sudo apt install intel-opencl-icd`）。

```bash
cd /path/to/GUI-for-openEB
mkdir -p third_party/openvino && cd third_party/openvino
# 从 https://storage.openvinotoolkit.org/repositories/openvino/packages/ 选对应版本，
# 下载 openvino_toolkit_ubuntuXX_<版本>_x86_64.tgz 并解压到当前目录（含 runtime/）
tar xzf ../openvino_toolkit_ubuntu26_2026.4.0.*_x86_64.tgz --strip-components=1
cd ../..

# 安装后重新编译，CMake 会自动检测 third_party/openvino/runtime/
cmake --build build -- -j$(nproc)
```

验证：`ctest -R e2vid_inference` 加载真实模型并打印实际使用的运行时（`dev=gpu|cpu`）。

### G5. 运行测试

```bash
cd /path/to/GUI-for-openEB/build

# 运行全部测试（GUI + algo）
ctest --output-on-failure

# 仅运行 GUI 测试
ctest -R "test_algo_bridge|test_config_manager|test_display_strategy|test_layout_manager|test_theme_tokens" --output-on-failure

# 仅运行 algo 测试
ctest -R "test_phase|test_raw" --output-on-failure
```

**测试套件**（共注册 462 个用例;4 个环境门控用例在无真实录制/硬件时跳过）：
- `gui/tests/`：15 个可执行文件（算法桥接/配置/回放/AEDAT4 读写/设备协议/面板/标定等）
- `algo/tests/`：14 个可执行文件（各算法族套件 + raw 流集成测试）

### G6. GUI 构建注意事项

| 问题 | 原因 | 解决方案 |
|------|------|----------|
| `Qt6` 找不到 | 未安装 Qt6 开发包 | `sudo apt install qt6-base-dev libqt6opengl6-dev` |
| `MetavisionSDK::hal` 找不到 | OpenEB 未部署或环境变量未设 | `source OPENEB_SRC_DIR/build/utils/scripts/setup_env.sh` |
| E2VID 降级为启发式模式 | ONNX Runtime 未安装 | 按 G4 步骤安装到 `third_party/onnxruntime/` |
| Wayland 下窗口无法拖拽 | Wayland 原生模式不支持 frameless 拖拽 | 使用 `./run.sh`（强制 XCB），或 `export QT_QPA_PLATFORM=xcb` |
| `gtest` 找不到 | 未安装 GTest 开发包 | `sudo apt install libgtest-dev libgmock-dev` |
