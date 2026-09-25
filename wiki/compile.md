# OpenEB Build Guide

> This guide has two parts: **Part 1** builds the OpenEB SDK (the `openeb/` subtree), **Part 2** builds the GUI application (`gui/` + `algo/`). The OpenEB SDK is the GUI's underlying dependency and must be built first.

## Environment

| Item       | Version |
|------------|---------|
| OS         | Ubuntu 26.04 |
| Architecture | amd64 (x86_64) |
| GCC        | 15.x (system default) |
| CMake      | 4.2.3 (minimum 3.16) |
| C++ standard | C++17 (`CMAKE_CXX_STANDARD 17`) |
| Python     | 3.14 (system default; NOT compatible with openeb) |
| Build Python | 3.12 (via the deadsnakes PPA) |
| Qt         | 6.x (required by the GUI) |
| OpenCV     | 4.x |

## Notes

1. **Python version**: OpenEB officially supports Python 3.9–3.12 only; the system's Python 3.14 is incompatible (limited by dependencies such as `numba`). Install Python 3.12 from the deadsnakes PPA.
2. **GCC 15 compatibility**: GCC 15 no longer implicitly includes `<cstdint>`, leaving `uint8_t`, `uint16_t`, etc. undeclared. A global compile option fixes this.
3. **Package rename**: on Ubuntu 26, `libcanberra-gtk-module` has been replaced by `libcanberra-gtk3-module`.

## Part 1 — OpenEB SDK

### 1. Install system dependencies

```bash
sudo apt update
sudo apt -y install apt-utils build-essential software-properties-common wget unzip curl git cmake
# libusb-1.0-0-dev is OPTIONAL: without it the inivation (DAVIS/DVXplorer)
# device layer is compiled out and everything else is unaffected (v3.0+).
sudo apt -y install libopencv-dev libboost-all-dev libusb-1.0-0-dev libprotobuf-dev protobuf-compiler
sudo apt -y install libhdf5-dev hdf5-tools libglew-dev libglfw3-dev libcanberra-gtk3-module ffmpeg
sudo apt -y install libgl-dev libglx-dev libopengl-dev
# Optional (for the test suite):
sudo apt -y install libgtest-dev libgmock-dev
```

### 2. Install Python 3.12 (via the deadsnakes PPA)

```bash
sudo apt install software-properties-common
sudo add-apt-repository ppa:deadsnakes/ppa
sudo apt update
sudo apt install python3.12 python3.12-venv python3.12-dev
```

### 3. Install pybind11 v2.11.0

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

### 4. Create a Python venv and install the requirements

```bash
python3.12 -m venv /tmp/prophesee/py3venv --system-site-packages
/tmp/prophesee/py3venv/bin/python -m pip install pip --upgrade
/tmp/prophesee/py3venv/bin/python -m pip install -r OPENEB_SRC_DIR/utils/python/requirements_openeb.txt
```

> The ML requirements (`requirements_pytorch_cpu.txt`) are optional; note that `torch==2.9.1` needs checking against Python 3.12 support.

### 5. The GCC 15 compatibility fix

> **Note**: this fix is ALREADY applied in this repository (`openeb/CMakeLists.txt` lines 24–27 and the root `CMakeLists.txt` lines 16–18) — nothing to add manually. The snippet below just explains the mechanism.

After the `project()` line in `OPENEB_SRC_DIR/CMakeLists.txt` (if not already present):

```cmake
# GCC 15+ no longer implicitly includes <cstdint>; add it globally to fix uint8_t/uint16_t etc.
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL "15")
    add_compile_options("-include;cstdint")
endif()
```

### 6. Build

```bash
cd OPENEB_SRC_DIR
rm -rf build
mkdir build && cd build
cmake .. -DBUILD_TESTING=OFF -DPython3_EXECUTABLE=/tmp/prophesee/py3venv/bin/python3.12
cmake --build . --config Release -- -j$(nproc)
```

### 7. Environment variables (choose one)

**Option A: use directly from the build directory**

```bash
source OPENEB_SRC_DIR/build/utils/scripts/setup_env.sh
# Can be added to ~/.bashrc to persist.
```

**Option B: deploy to the system paths**

```bash
sudo cmake --build . --target install
# Then set the environment variables (e.g. in ~/.bashrc):
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib
export HDF5_PLUGIN_PATH=$HDF5_PLUGIN_PATH:/usr/local/lib/hdf5/plugin
```

## Troubleshooting (Part 1)

| Problem | Cause | Fix |
|---------|-------|-----|
| `numba` install fails (Python 3.14) | numba only supports Python >=3.9,<3.13 | Install Python 3.12 via the deadsnakes PPA |
| `uint16_t`/`uint8_t` not declared | GCC 15 no longer implicitly includes `<cstdint>` | Add `-include cstdint` in CMakeLists.txt |
| No candidate for `libcanberra-gtk-module` | Package renamed in Ubuntu 26 | Use `libcanberra-gtk3-module` |
| `OpenGL` libraries not found | OpenGL dev packages missing | Install `libgl-dev libglx-dev libopengl-dev` |
| `GLEW` not found | GLEW dev package missing | Install `libglew-dev` |
| `glfw3` config not found | GLFW3 dev package missing | Install `libglfw3-dev` |

---

## Part 2 — GUI application

This part builds the EB plus GUI (`gui/` + `algo/`). Prerequisite: the OpenEB SDK from Part 1 is built and deployed (`source OPENEB_SRC_DIR/build/utils/scripts/setup_env.sh` or `sudo make install`).

### G1. Install the GUI's extra dependencies

```bash
# Qt 6 (Widgets + OpenGL + OpenGLWidgets)
sudo apt -y install qt6-base-dev qt6-base-dev-tools libqt6opengl6-dev

# ONNX Runtime (E2VID neural inference; optional but recommended)
# See the standalone install steps in G4 below.

# Google Test (GUI unit tests; optional)
sudo apt -y install libgtest-dev libgmock-dev
```

### G2. Build the GUI

```bash
cd /path/to/GUI-for-openEB

# Configure (CMake auto-detects OpenEB SDK, Qt6, OpenCV, ONNX Runtime)
cmake -B build -DCMAKE_BUILD_TYPE=Release

# Build
cmake --build build -- -j$(nproc)
```

**CMake options**:

| Option | Default | Description |
|--------|---------|-------------|
| `GUI_BUILD_TESTS` | `ON` | Build the GUI unit tests (`gui/tests/`) |
| `CMAKE_BUILD_TYPE` | — | `Release` recommended |

### G3. Run the GUI

```bash
# Recommended: the launcher script (Wayland compatibility, HAL plugin
# paths and the OpenGL backend are handled for you)
./run.sh

# Or run directly (environment variables must be set yourself)
./build/gui/gui_for_openeb
```

`run.sh` sets, among others:
- `QT_QPA_PLATFORM=xcb` (forces XCB under Wayland)
- `QSG_RHI_BACKEND=opengl` (forces the OpenGL render backend)
- the HAL plugin path (`MV_HAL_PLUGIN_PATH`)

### G4. ONNX Runtime (E2VID inference)

The E2VID mode runs its neural inference through ONNX Runtime. Without it, E2VID falls back to a heuristic mode (voxel-grid sum + sigmoid).

```bash
cd /path/to/GUI-for-openEB
mkdir -p third_party/onnxruntime && cd third_party/onnxruntime
wget https://github.com/microsoft/onnxruntime/releases/download/v1.19.2/onnxruntime-linux-x64-1.19.2.tgz
tar xzf onnxruntime-linux-x64-1.19.2.tgz --strip-components=1
cd ../..

# Rebuild afterwards; CMake auto-detects third_party/onnxruntime/
cmake --build build -- -j$(nproc)
```

Model weight conversion (PyTorch → ONNX) is described in [README.md](../README.md), section "Neural Reconstruction".

### G4b. OpenVINO (optional Intel iGPU acceleration for E2VID)

With OpenVINO installed, E2VID's Auto/GPU inference device runs the network on the Intel iGPU (measured ~11× faster than CPU; the SAME .onnx models, no re-conversion). Without it the build silently falls back to ONNX Runtime CPU with no feature loss. Requires the iGPU userspace driver (Ubuntu 26.04: `sudo apt install intel-opencl-icd`).

```bash
cd /path/to/GUI-for-openEB
mkdir -p third_party/openvino && cd third_party/openvino
# Pick your version at https://storage.openvinotoolkit.org/repositories/openvino/packages/,
# download openvino_toolkit_ubuntuXX_<version>_x86_64.tgz and extract it here (contains runtime/)
tar xzf ../openvino_toolkit_ubuntu26_2026.4.0.*_x86_64.tgz --strip-components=1
cd ../..

# Rebuild afterwards; CMake auto-detects third_party/openvino/runtime/
cmake --build build -- -j$(nproc)
```

Verify: `ctest -R e2vid_inference` loads a real model and prints the active runtime (`dev=gpu|cpu`).

### G5. Run the tests

```bash
cd /path/to/GUI-for-openEB/build

# All tests (GUI + algo)
ctest --output-on-failure

# GUI tests only
ctest -R "test_algo_bridge|test_config_manager|test_display_strategy|test_layout_manager|test_theme_tokens" --output-on-failure

# algo tests only
ctest -R "test_phase|test_raw" --output-on-failure
```

**Test suites** (462 registered cases in total; four env-gated tests skip without real recordings / hardware):
- `gui/tests/`: 15 executables (algo bridge, config, playback, AEDAT4 writer/reader, device protocol, panels, calibration, …)
- `algo/tests/`: 14 executables (per-family algorithm suites + raw-stream integration)

### G6. GUI build troubleshooting

| Problem | Cause | Fix |
|---------|-------|-----|
| `Qt6` not found | Qt6 dev packages missing | `sudo apt install qt6-base-dev libqt6opengl6-dev` |
| `MetavisionSDK::hal` not found | OpenEB not deployed or env vars unset | `source OPENEB_SRC_DIR/build/utils/scripts/setup_env.sh` |
| E2VID falls back to heuristic mode | ONNX Runtime not installed | Install per G4 into `third_party/onnxruntime/` |
| Window cannot be dragged under Wayland | Native Wayland does not support frameless dragging | Use `./run.sh` (forces XCB), or `export QT_QPA_PLATFORM=xcb` |
| `gtest` not found | GTest dev packages missing | `sudo apt install libgtest-dev libgmock-dev` |
