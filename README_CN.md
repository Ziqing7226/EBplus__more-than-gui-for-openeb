<div align="center">

# EB plus

基于 [openEB](https://github.com/prophesee-ai/openeb) v5.2.0 的开源 Qt 6 事件相机桌面应用。

实时可视化 · 相机控制 · 录制回放 · 标定 · 25 个算法 · 可定制主题

![License](https://img.shields.io/badge/license-MIT%20%2F%20Apache--2.0-blue)
![Language](https://img.shields.io/badge/C%2B%2B17-Qt%206-orange)
![Platform](https://img.shields.io/badge/platform-Linux-lightgrey)
![Version](https://img.shields.io/badge/version-3.0.0-blue)

![主界面](pic/1.9.0.png)

</div>

---

## 3.0.0 新特性

- **Inivation 实时相机**：DAVIS240A/B/C、DAVIS346、DAVIS640、CDAVIS 与 DVXplorer 直连——事件流、完整偏置控制（含 Auto Bias）、DAVIS 硬件 ROI、IMU 三维姿态视图，以及 APS 帧预览（支持的传感器上为彩色）
- **IMU 姿态链重构**：打开即对齐、静止自动回正、闭环路径可回位；IMU 与 APS 视图改为可停靠侧栏
- **AEDAT4 录制**（inivation 相机，DV 原生且可互操作）可选携带 IMU 样本与 APS 帧，回放时如实机一般呈现；DV 的 LZ4 压缩录制可直接打开
- **标定**的 Auto Bias 区间与事件缓冲随相机分辨率自适应；square size 工作流不再丢弃抓拍
- **能力感知 UI**：相机缺少对应硬件时面板自动隐藏

---

## 这是什么？

**EB plus** 是一个美观、开源、功能丰富的事件相机 GUI 工具，支持 Prophesee / CenturyArks / inivation DAVIS / DVXplorer 事件相机。事件相机不采集帧——它以微秒级时间分辨率逐像素报告亮度变化。EB plus 提供完整的事件数据桌面工作流：

- **实时显示** 事件流（OpenGL，60+ FPS）
- **控制相机** —— biases、ROI、抗闪烁、触发
- **录制与回放** 事件文件（RAW，以及 AEDAT4、ALPDATA 录制中的事件流），支持速度控制与跳转
- **运行算法** —— 噪声过滤、光流、目标跟踪、事件转视频等
- **标定相机** —— 棋盘格向导
- **导出** 为 HDF5 / CSV / AVI

本项目完全开源，欢迎 fork 并按需修改。

> **什么是事件相机？** 与传统帧相机不同，事件相机输出异步的逐像素亮度变化——"事件"——具有微秒级时间分辨率、高动态范围和低功耗。

---

## 快速开始

```bash
# 编译
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -- -j$(nproc)

# 运行（启动脚本会自动设置所有必需的环境变量）
./run.sh
```

启动脚本会自动处理 Wayland 兼容、HAL 插件路径和 OpenGL 后端选择。

> **环境要求**：Ubuntu 22.04+ · GCC 13+ · Qt 6 · OpenCV 4。详见 [wiki/compile.md](wiki/compile.md)。

### 连接 inivation DAVIS / DVXplorer 相机（可选，初步支持）

**初步支持** inivation DAVIS240A/B/C、DAVIS346、DAVIS640、CDAVIS 与 DVXplorer——实时事件流、完整偏置控制（含 Auto Bias）、DAVIS 硬件 ROI、IMU 姿态视图与 APS 帧预览（支持的传感器上为彩色），以及 DV 原生 AEDAT4 录制（事件 + IMU + APS）。240/640/CDAVIS 型号遵循参考实现但尚未经过真机测试；本 GUI 仍以 **Prophesee** 相机为主要适配与测试对象。一次性安装 USB 访问规则：

```bash
sudo cp gui/davis/66-inivation.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

拔插一次相机，然后在设备面板连接。此功能可选：缺少 libusb-1.0 开发文件时构建会自动排除。详见 [wiki/Getting-Started.md](wiki/Getting-Started.md)。

---

## 功能特性

### 实时事件显示
- OpenGL 加速渲染（GLSL 3.30 core profile，letterbox 视口）
- 7 种帧模式：Integration、Diff、Histogram、Time Decay、Contrast Map、Periodic、On-Demand
- 4 种色彩主题：Dark、Light、CoolWarm、Gray
- 实时统计：事件率、ON/OFF 比、FPS、时间戳

### 相机控制面板
- **Biases 面板** —— 动态枚举所有 HAL bias，滑块 + 精确输入 + Reset，保存/加载 `.bias` 文件，Auto Bias 速率区间控制
- **ROI 面板** —— 多矩形 ROI / RONI（`I_ROI`），显示区拖拽选区
- **ESP 面板** —— Anti-Flicker（模式/频带/预设/占空比/阈值）、Trail Filter（类型/阈值）、ERC（目标事件率）
- **Trigger 面板** —— Trigger In（逐通道启用）+ Trigger Out（启用/周期/占空比）

所有面板在设备不支持对应 HAL facility 时优雅降级（如文件回放时四个硬件面板自动禁用）。

### 录制与回放
- RAW 录制 —— 实时相机流录制，带实时缓冲刷新（inivation 相机：DV 原生 AEDAT4，可选携带 IMU / APS 流）
- 文件回放 —— 速度控制、跳转、暂停/恢复、位置追踪
- 文件裁剪 —— 从事件文件中提取时间段

### 数据导出与转换
- RAW、HDF5、CSV 格式互转
- 事件数据导出为 AVI 视频（可配置 FPS、累积时间、画质、色彩模式）

### 事件预处理滤波链
4 级可叠加阶段，线程安全管线：Polarity Filter、Polarity Invert、Flip X、Flip Y。从侧栏切换。

### 算法（共 25 项）
EB plus 内置 **21 个自研算法** + **4 项 openEB 滤波阶段**，全部注册在统一的 `AlgoBridge` 注册表中。

| 类别 | 示例 |
|------|------|
| **滤波** | Hot Pixel Filter、Background Mask |
| **运动** | Sparse Optical Flow（4 模式）、Direction Selective、EIS / Optical Gyro |
| **检测** | Blob Detector、Corner Detector（Harris/FAST/AGAST）、Line Segment（ELiSeD）|
| **跟踪** | Object Tracker（RCT，对齐 jAER）、Hough Circle、Hough Line |
| **重建** | Event-to-Video —— **E2VID / E2VID+ / FireNet+ / HyperE2VID**（DL 模式）、BardowVariational、InteractingMaps |
| **DL 光流** | Dense Optical Flow (DL) —— EVFlowNet，HSV 编码稠密光流 |
| **分析** | Frequency Detector、Frequency Map |
| **可视化** | Time Surface、XYT 3D 点云、Orientation Cluster |
| **标定** | Intrinsic Calibration（闪烁棋盘格）|

算法**互斥**——启用一个会禁用上一个。计算密集算法会自动启用**居中的 256×144 统一 ROI**（禁用时恢复原状），所有算法共享 **"ROI → 噪声滤波 → 1/4 下采样"** 预处理阶段以控制计算量。所有算法参数仅在**侧栏**（`AlgorithmsPanel`）调节；算法显示窗口只展示标题与输出，避免两处独立参数面板不同步。

#### 噪声滤波（共享预处理）
8 种模式按所选滤波器在侧栏暴露：BAF、STCF、Refractory、DWF、AgePolarity、Harmonic、Repetitious、SpatialBP。

#### 神经网络重建（E2VID 系列）与 DL 光流

Event-to-Video 算法默认使用 **E2VID**，共提供 **4 种 DL 模式**（GUI 内切换，各模式拥有独立模型文件，可并存多套权重），另有非 DL 的 BardowVariational / InteractingMaps 模式。独立的 **Dense Optical Flow (DL)** 算法将 EVFlowNet 的逐像素光流渲染为按方向编码的 HSV 帧。

| 模式 | 模型 | 参考（论文 / 仓库） | 预训练权重 |
|------|------|---------------------|------------|
| 2 = E2VID（默认） | UNetRecurrent | [rpg_e2vid](https://github.com/uzh-rpg/rpg_e2vid) — Gallego 等, 2019 | [E2VID_lightweight.pth.tar](http://rpg.ifi.uzh.ch/data/E2VID/models/E2VID_lightweight.pth.tar) |
| 3 = E2VID+ | FlowNet（联合头） | [event_cnn_minimal](https://github.com/TimoStoff/event_cnn_minimal) — Stoffregen 等, ECCV 2020 | [模型包](https://drive.google.com/open?id=1J6PbqYPOGlyspYsdH4fgg5pZpc_l-BOD) → `reconstruction_model.pth` |
| 4 = FireNet+ | FireNet（约 4 万参数） | 同上 | 同上 → `firenet_all_cts.pth` |
| 5 = HyperE2VID | 超网络 UNet | [HyperE2VID](https://github.com/ercanburak/HyperE2VID) — Ercan 等, IEEE TIP 2024 | [model.pth](https://drive.google.com/drive/folders/1UuGnKwSz5C9di-cVH1QzSFjgTRNqpYep) |
| DL 光流 | EVFlowNet | event_cnn_minimal（架构：Zhu 等, 2018） | 同上 → `flow_model.pth` |

**部署**（一次性，约 10 分钟）：

```bash
cd /path/to/GUI-for-openEB

# 1. ONNX Runtime 1.19.2（CPU）装入 third_party/
mkdir -p third_party/onnxruntime && cd third_party/onnxruntime
wget https://github.com/microsoft/onnxruntime/releases/download/v1.19.2/onnxruntime-linux-x64-1.19.2.tgz
tar xzf onnxruntime-linux-x64-1.19.2.tgz --strip-components=1
cd ../..

# 2.（可选，核显加速）OpenVINO + Intel 计算驱动 —— 见 wiki/compile.md G4b

# 3. 模型转换用 Python venv
python3 -m venv .venv && . .venv/bin/activate
pip install torch --index-url https://download.pytorch.org/whl/cpu onnx onnxscript onnxruntime numpy scipy
deactivate

# 4. 参考仓库（转换脚本运行时导入；本仓库不分发）
git clone --depth 1 https://github.com/uzh-rpg/rpg_e2vid ref/rpg_e2vid
git clone --depth 1 https://github.com/TimoStoff/event_cnn_minimal ref/event_cnn_minimal
git clone --depth 1 https://github.com/ercanburak/HyperE2VID ref/HyperE2VID

# 5. 下载权重（见上表链接）放入 models/

# 6. 转换为 ONNX
. .venv/bin/activate
python models/convert_to_onnx.py --input models/E2VID_lightweight.pth.tar --output models/e2vid_lightweight.onnx
python models/convert_event_cnn_minimal_to_onnx.py --model e2vid_plus   --input reconstruction_model.pth --output models/e2vid_plus.onnx
python models/convert_event_cnn_minimal_to_onnx.py --model firenet_plus --input firenet_all_cts.pth      --output models/firenet_plus.onnx
python models/convert_event_cnn_minimal_to_onnx.py --model evflownet    --input flow_model.pth           --output models/evflownet.onnx
python models/convert_hypere2vid_to_onnx.py        --input model.pth                                    --output models/hypere2vid.onnx
deactivate

# 7. 重新编译（CMake 自动检测 ONNX Runtime / OpenVINO）
cmake --build build -- -j$(nproc)
```

GUI 中：**Algorithm → Event → Video**（默认 E2VID 模式：128×128 ROI、30fps、1/4 下采样），在侧栏选择模式；每个 DL 模式暴露各自的模型路径。**Dense Optical Flow (DL)** 在统一 ROI 上运行并内部 1/4 下采样。所有 DL 推理在可用时走核显（Inference device = Auto，OpenVINO），否则回退 ONNX Runtime CPU。

> **无 ONNX Runtime 时**：E2VID 自动回退到启发式模式（体素网格求和 + Sigmoid）。BardowVariational 和 InteractingMaps 无需任何额外依赖。

> **第三方模型声明**：EB plus **不分发**任何预训练权重或参考源码——转换脚本针对你自行克隆到 `ref/` 的参考仓库运行，权重从上方官方链接下载。被引用代码的许可证：rpg_e2vid = GPL-3.0（仅在转换时、在你机器上、由你的克隆被导入）；event_cnn_minimal = 无 LICENSE 文件（作者以科研用途分享权重）；HyperE2VID = MIT。预训练权重均为学术发布；商用前请自行核实相应许可与专利状况。

### 主题
- **5 种背景色**：Gray、Green、Yellow、Pink、Blue（默认）
- **3 种模式**：Follow System（默认）、Always Light、Always Dark
- 暗色模式使用所选颜色的**暗色变体**——而非纯黑
- 文字颜色自动适配（浅色背景用黑，暗色背景用白）
- 设置跨重启持久化；标题栏跟随主题

### 多窗口与布局
- XYT 3D 事件点云（GPU 加速）
- 额外算法显示窗口（可停靠）
- 布局保存/恢复到 JSON

---

## 目录结构

```
GUI-for-openEB/
├── gui/              # Qt 6 应用
│   ├── main_window.*     # 主窗口：标题栏菜单、dock、信号连接
│   ├── display/          # OpenGL 视口、叠加层、3D 点云
│   ├── panels/           # VSCode 风格侧栏面板（5 组 11 个面板）
│   ├── app/              # 控制器（相机、管线、主题…）
│   ├── algo_bridge/      # 算法注册表 + 滤波链
│   ├── recorder/         # RAW 录制 & 回放
│   ├── exporter/         # HDF5/CSV/AVI 导出
│   ├── calibration/      # 内参向导
│   └── widgets/          # 标题栏、ActivityBar、AlgoWindow、像素探针
├── algo/              # 自研算法库（29 模块）
├── openeb/            # openEB SDK（Apache 2.0，v5.2.0）
├── models/            # E2VID PyTorch → ONNX 转换
├── run.sh             # 启动脚本（环境变量设置）
├── wiki/              # 文档：编译指南、算法、架构
└── pic/               # 截图
```

---

## 运行

### 方式一：启动脚本（推荐）

```bash
./run.sh
```

脚本自动检测 Wayland，强制 XCB + OpenGL（避免黑屏），并设置 HAL/HDF5 插件路径。

### 方式二：手动启动

```bash
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}:/usr/local/lib"
export HDF5_PLUGIN_PATH="/usr/local/lib/hdf5/plugin"
export MV_HAL_PLUGIN_PATH=/usr/local/lib/metavision/hal/plugins  # Prophesee
# export MV_HAL_PLUGIN_PATH=/usr/lib/CenturyArks/hal/plugins     # CenturyArks
export QT_QPA_PLATFORM=xcb       # Wayland 对 QOpenGLWidget 渲染黑屏
export QSG_RHI_BACKEND=opengl    # Qt 6 可能默认使用 Vulkan

./build/gui/gui_for_openeb
```

### 相机厂商配置

| 厂商 | HAL 插件路径 |
|------|-------------|
| Prophesee | `/usr/local/lib/metavision/hal/plugins` |
| CenturyArks | `/usr/lib/CenturyArks/hal/plugins` |

---

## 常见问题

**启动后黑屏** —— 使用启动脚本。若手动启动，设置 `QT_QPA_PLATFORM=xcb` 和 `QSG_RHI_BACKEND=opengl`。

**相机未检测到** —— 确认 `MV_HAL_PLUGIN_PATH` 与你的厂商匹配，运行 `metavision_hal_ls` 检查。

**"NonMonotonicTimeHigh" 错误** —— 这是部分 Gen3.x 相机启动时约 50% 概率出现的 Evt3 协议瞬态警告。EB plus 将其视为非致命，保持流运行。无需处理。

**暗色模式不跟随系统** —— 需要 Qt 6.5+。旧版 Qt 请用 Theme → Mode → Dark。

---

## 快捷键

| 快捷键 | 功能 |
|--------|------|
| `Ctrl+O` | 打开文件 |
| `Ctrl+Shift+P` | 切换回放面板 |
| `F11` | 全屏 |
| `Ctrl+Q` | 退出 |

---

## 已知问题与反馈

EB plus 正在持续开发中，可能仍存在 BUG。如果你在使用过程中遇到任何问题——崩溃、渲染异常、控件失灵或非预期行为——欢迎[提交 issue](../../issues)。来自真实用户的反馈是最直接的帮助。

---

## 许可证

- **项目原创代码**：[MIT](LICENSE)
- **openEB SDK**：[Apache 2.0](openeb/licensing/LICENSE_OPEN) —— 版权归 Prophesee 所有
- **Inivation 实时相机支持**（`gui/davis/`）：协议、寄存器表与默认值移植自
  [dv-processing](https://gitlab.com/inivation/dv/dv-processing)
  （[Apache 2.0](https://gitlab.com/inivation/dv/dv-processing/-/blob/master/LICENSE)，© iniVation AG）——各文件内含出处声明
- **对齐 jAER 的算法**：参考行为来自
  [jAER](https://github.com/jaer-project/jaer3)（[LGPL-2.1](ref/jaer/COPYING)）——全部为重新实现,未内置任何源码
- **DV GUI**：未使用。iniVation 的 dv-gui 使用自定义（非标准）许可证;
  IMU 可视化为全新实现,仅共享通用的滚动曲线概念

---

<div align="center">

基于 Qt 6 · OpenCV · openEB SDK 构建

</div>
