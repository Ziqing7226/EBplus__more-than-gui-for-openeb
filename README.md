<div align="center">

# EB plus

A polished, open-source Qt 6 desktop app for event cameras — built on [openEB](https://github.com/prophesee-ai/openeb) v5.2.0.

Real-time visualization · camera control · recording & playback · calibration · 25 algorithms · customizable themes

![License](https://img.shields.io/badge/license-MIT%20%2F%20Apache--2.0-blue)
![Language](https://img.shields.io/badge/C%2B%2B17-Qt%206-orange)
![Platform](https://img.shields.io/badge/platform-Linux-lightgrey)
![Version](https://img.shields.io/badge/version-3.0.0-blue)

![Main Window](pic/1.9.0.png)

</div>

---

## What's new in 3.0.0

- **Inivation live cameras**: DAVIS240A/B/C, DAVIS346, DAVIS640, CDAVIS and DVXplorer connect directly — events, full bias control (Auto Bias included), hardware ROI on DAVIS, IMU stream with a 3D attitude view, and APS frame preview (color on supported sensors)
- **IMU attitude reworked**: instant alignment, rest re-centers the attitude, closed paths return; the IMU and APS views are now dockable side panels
- **AEDAT4 recording** for inivation cameras (DV-native, interoperable) now carries optional IMU samples and APS frames, and replays surface them like a live camera; LZ4-compressed DV recordings open directly
- **Calibration** scales its Auto Bias band and event buffering with the camera resolution; the square-size workflow no longer discards captures
- Capability-aware UI: panels auto-hide when the connected camera lacks the hardware behind them

## What is this?

**EB plus** is a beautiful, open-source, feature-rich GUI for event cameras (Prophesee / CenturyArks). Event cameras don't capture frames — they report per-pixel brightness changes at microsecond resolution. EB plus gives you a complete desktop workflow to work with this data:

- **See** the event stream in real time (OpenGL, 60+ FPS)
- **Control** the camera — biases, ROI, anti-flicker, triggers
- **Record & replay** event files (RAW, plus event streams from AEDAT4 and ALPDATA recordings) with speed control and seek
- **Run algorithms** — noise filtering, optical flow, object tracking, event-to-video, and more
- **Calibrate** the camera with a chessboard wizard
- **Export** to HDF5 / CSV / AVI

The whole project is open source — feel free to fork it and adapt it to whatever you need.

---

## Quick Start

```bash
# Build
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -- -j$(nproc)

# Run (the launcher sets all required env vars)
./run.sh
```

That's it. The launcher handles Wayland compatibility, HAL plugin paths, and OpenGL backend selection automatically.

> **Requirements**: Ubuntu 26.04 · GCC 15 · Qt 6 · OpenCV 4 — the tested environment (see [wiki/compile.md](wiki/compile.md)).

### Live inivation DAVIS / DVXplorer cameras (optional, preliminary)

**Preliminary support** for the inivation DAVIS240A/B/C, DAVIS346, DAVIS640, CDAVIS and DVXplorer — live events, full bias control with Auto Bias, hardware ROI (DAVIS), an IMU attitude view and APS frame preview (color on supported sensors), plus DV-native AEDAT4 recording (events + IMU + APS). The 240/640/CDAVIS models follow the reference implementation but are not yet hardware-tested; EB plus remains primarily designed and tested for **Prophesee** cameras. One-time setup — allow USB access for inivation devices:

```bash
sudo cp gui/davis/66-inivation.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Unplug and replug the camera, then connect from the Devices panel. This feature is optional: without libusb-1.0 development files the build simply excludes it. See [wiki/Getting-Started.md](wiki/Getting-Started.md) for details and current limitations.

---

## Features

### Real-time Display
- OpenGL-accelerated rendering with letterboxed viewport
- 7 frame modes: Integration, Diff, Histogram, Time Decay, Contrast Map, Periodic, On-Demand
- 4 color palettes: Dark, Light, CoolWarm, Gray
- Live statistics: event rate, ON/OFF ratio, FPS, timestamp

### Camera Control
- **Biases** — all HAL biases with slider + spinbox, save/load `.bias` files, Auto Bias rate-band control
- **ROI** — multi-rectangle ROI / RONI, drag-to-select on the display
- **ESP** — Anti-Flicker, Trail Filter, Event Rate Control
- **Trigger** — Trigger In (per-channel) + Trigger Out

All panels degrade gracefully when the device lacks the corresponding HAL facility (e.g. the four hardware panels auto-disable during file playback).

### Recording & Playback
- RAW recording from live cameras (inivation cameras: live preview + biases only)
- File playback with speed control, seek, pause/resume
- File cutter — extract a time range from an event file

### Export & Conversion
- Convert between RAW, HDF5, and CSV
- Export events to AVI video (configurable FPS, accumulation, quality, color mode)

### Preprocessing Filter Chain
4 stackable stages applied in a thread-safe pipeline: Polarity Filter, Polarity Invert, Flip X, Flip Y. Toggled from the sidebar.

### Algorithms (25 total)
EB plus ships **21 self-developed algorithms** plus **4 OpenEB filter stages**, all registered in a single `AlgoBridge` registry.

| Category | Examples |
|----------|----------|
| **Filtering** | Hot Pixel Filter, Background Mask |
| **Motion** | Sparse Optical Flow (4 modes), Direction Selective, EIS / Optical Gyro |
| **Detection** | Blob Detector, Corner Detector (Harris/FAST/AGAST), Line Segment (ELiSeD) |
| **Tracking** | Object Tracker (RCT, jAER-aligned), Hough Circle, Hough Line |
| **Reconstruction** | Event-to-Video — **E2VID / E2VID+ / FireNet+ / HyperE2VID** (DL modes), BardowVariational, InteractingMaps |
| **DL Optical Flow** | Dense Optical Flow (DL) — EVFlowNet, HSV-coded dense flow |
| **Analytics** | Frequency Detector, Frequency Map |
| **Visualization** | Time Surface, XYT 3D Point Cloud, Orientation Cluster |
| **Calibration** | Intrinsic Calibration (blinking chessboard) |

Algorithms are **mutually exclusive** — enabling one disables the previous. Compute-heavy algorithms auto-enable a **centered 256×144 unified ROI** (saved and restored on disable), and all algorithms share a **"ROI → noise filter → 1/4 downsample"** preprocessing stage to bound computational cost. All algorithm parameters are adjusted exclusively in the **sidebar** (`AlgorithmsPanel`); algorithm display windows show only the title and output, preventing parameter drift between two independent control panels.

#### Noise Filter (shared preprocessing)
8 modes exposed in the sidebar based on the selected filter: BAF, STCF, Refractory, DWF, AgePolarity, Harmonic, Repetitious, SpatialBP.

#### Neural Reconstruction (E2VID family) & DL Optical Flow

The Event-to-Video algorithm defaults to **E2VID** and offers **4 DL modes** (selected in the GUI; each mode has its own model file, so several weight sets can be installed side by side), plus the non-DL BardowVariational / InteractingMaps modes. A separate **Dense Optical Flow (DL)** algorithm renders EVFlowNet's per-pixel flow as a direction-coded HSV frame.

| Mode | Model | Reference (paper / repo) | Pretrained weights |
|------|-------|--------------------------|--------------------|
| 2 = E2VID (default) | UNetRecurrent | [rpg_e2vid](https://github.com/uzh-rpg/rpg_e2vid) — Gallego et al., 2019 | [E2VID_lightweight.pth.tar](http://rpg.ifi.uzh.ch/data/E2VID/models/E2VID_lightweight.pth.tar) |
| 3 = E2VID+ | FlowNet (joint head) | [event_cnn_minimal](https://github.com/TimoStoff/event_cnn_minimal) — Stoffregen et al., ECCV 2020 | [model pack](https://drive.google.com/open?id=1J6PbqYPOGlyspYsdH4fgg5pZpc_l-BOD) → `reconstruction_model.pth` |
| 4 = FireNet+ | FireNet (~40K params) | ibid. | ibid. → `firenet_all_cts.pth` |
| 5 = HyperE2VID | Hypernetwork UNet | [HyperE2VID](https://github.com/ercanburak/HyperE2VID) — Ercan et al., IEEE TIP 2024 | [model.pth](https://drive.google.com/drive/folders/1UuGnKwSz5C9di-cVH1QzSFjgTRNqpYep) |
| DL flow | EVFlowNet | event_cnn_minimal (arch.: Zhu et al., 2018) | ibid. → `flow_model.pth` |

**Setup** (one-time, ~10 minutes):

```bash
cd /path/to/GUI-for-openEB

# 1. ONNX Runtime 1.19.2 (CPU) into third_party/
mkdir -p third_party/onnxruntime && cd third_party/onnxruntime
wget https://github.com/microsoft/onnxruntime/releases/download/v1.19.2/onnxruntime-linux-x64-1.19.2.tgz
tar xzf onnxruntime-linux-x64-1.19.2.tgz --strip-components=1
cd ../..

# 2. (optional, iGPU acceleration) OpenVINO + Intel compute driver — see wiki/compile.md G4b

# 3. Python venv for model conversion
python3 -m venv .venv && . .venv/bin/activate
pip install torch --index-url https://download.pytorch.org/whl/cpu onnx onnxscript onnxruntime numpy scipy
deactivate

# 4. Reference repositories (conversion imports these; NOT redistributed)
git clone --depth 1 https://github.com/uzh-rpg/rpg_e2vid ref/rpg_e2vid
git clone --depth 1 https://github.com/TimoStoff/event_cnn_minimal ref/event_cnn_minimal
git clone --depth 1 https://github.com/ercanburak/HyperE2VID ref/HyperE2VID

# 5. Download the weights (links in the table above) into models/

# 6. Convert to ONNX
. .venv/bin/activate
python models/convert_to_onnx.py --input models/E2VID_lightweight.pth.tar --output models/e2vid_lightweight.onnx
python models/convert_event_cnn_minimal_to_onnx.py --model e2vid_plus   --input reconstruction_model.pth --output models/e2vid_plus.onnx
python models/convert_event_cnn_minimal_to_onnx.py --model firenet_plus --input firenet_all_cts.pth      --output models/firenet_plus.onnx
python models/convert_event_cnn_minimal_to_onnx.py --model evflownet    --input flow_model.pth           --output models/evflownet.onnx
python models/convert_hypere2vid_to_onnx.py        --input model.pth                                    --output models/hypere2vid.onnx
deactivate

# 7. Rebuild (CMake auto-detects ONNX Runtime / OpenVINO)
cmake --build build -- -j$(nproc)
```

In the GUI: **Algorithm → Event → Video** (defaults to E2VID mode: 128×128 ROI, 30 fps, 1/4 downsample) — pick the mode in the sidebar; each DL mode exposes its own model path. **Dense Optical Flow (DL)** runs at the unified ROI with internal 1/4 downsampling. All DL inference uses the iGPU via OpenVINO when available (Inference device = Auto), falling back to ONNX Runtime CPU.

> **Without ONNX Runtime**: E2VID falls back to a heuristic mode (voxel-grid sum + sigmoid). BardowVariational and InteractingMaps work without any setup.

> **Third-party models notice**: EB plus does NOT redistribute any pretrained weights or reference source code — the conversion scripts run against YOUR clones under `ref/` and the weights are downloaded from the official links above. Licenses of the referenced code: rpg_e2vid = GPL-3.0 (imported only at conversion time, on your machine, by your clone); event_cnn_minimal = no license file (weights shared by the authors for research use); HyperE2VID = MIT. The pretrained weights are academic releases; verify the applicable licenses (and patent landscape, where relevant) before commercial deployment.

### Theming
- **5 background colors**: Gray, Green, Yellow, Pink, Blue (default)
- **3 modes**: Follow System (default), Always Light, Always Dark
- Dark mode uses a **dark variant of the chosen color** — not just black
- Text color auto-adjusts (black on light, white on dark)
- Settings persist across restarts; the title bar follows the theme

### Multi-Window & Layout
- XYT 3D event point cloud (GPU-accelerated)
- Additional algorithm display windows (dockable)
- Save/restore dock layout to JSON

---

## Directory Structure

```
GUI-for-openEB/
├── gui/              # Qt 6 application
│   ├── main_window.*     # Main window: title-bar menus, docks, signal wiring
│   ├── display/          # OpenGL viewport, overlays, 3D cloud
│   ├── panels/           # VSCode-style sidebar panels (5 groups, 11 panels)
│   ├── app/              # Controllers (camera, pipeline, theme, …)
│   ├── algo_bridge/      # Algorithm registry + filter chain
│   ├── recorder/         # RAW recording & playback
│   ├── exporter/         # HDF5/CSV/AVI export
│   ├── calibration/      # Intrinsic wizard
│   └── widgets/          # Title bar, ActivityBar, AlgoWindow, pixel probe
├── algo/              # Self-developed algorithm library (29 modules)
├── openeb/            # openEB SDK (Apache 2.0, v5.2.0)
├── models/            # E2VID PyTorch → ONNX conversion
├── run.sh             # Launcher (sets env vars)
├── wiki/              # Docs: compile guide, algorithms, architecture
└── pic/               # Screenshots
```

---

## Running

### Option 1: Launcher (Recommended)

```bash
./run.sh
```

The launcher auto-detects Wayland, forces XCB + OpenGL (avoids black screen), and sets HAL/HDF5 plugin paths.

### Option 2: Manual

```bash
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}:/usr/local/lib"
export HDF5_PLUGIN_PATH="/usr/local/lib/hdf5/plugin"
export MV_HAL_PLUGIN_PATH=/usr/local/lib/metavision/hal/plugins  # Prophesee
# export MV_HAL_PLUGIN_PATH=/usr/lib/CenturyArks/hal/plugins     # CenturyArks
export QT_QPA_PLATFORM=xcb       # Wayland renders black for QOpenGLWidget
export QSG_RHI_BACKEND=opengl    # Qt 6 may default to Vulkan

./build/gui/gui_for_openeb
```

### Camera Vendor Paths

| Vendor | HAL plugin path |
|--------|----------------|
| Prophesee | `/usr/local/lib/metavision/hal/plugins` |
| CenturyArks | `/usr/lib/CenturyArks/hal/plugins` |

---

## Troubleshooting

**Black screen on startup** — Use the launcher script. If launching manually, set `QT_QPA_PLATFORM=xcb` and `QSG_RHI_BACKEND=opengl`.

**Camera not detected** — Verify `MV_HAL_PLUGIN_PATH` matches your vendor. Run `metavision_hal_ls` to check.

**"NonMonotonicTimeHigh" error** — This is a transient Evt3 protocol warning that occurs ~50% of the time on some Gen3.x cameras at startup. EB plus treats it as non-fatal and keeps the stream running. No action needed.

**Dark mode not following system** — Requires Qt 6.5+. On older Qt, use Theme → Mode → Dark.

---

## Keyboard Shortcuts

| Shortcut | Action |
|----------|--------|
| `Ctrl+O` | Open file |
| `Ctrl+Shift+P` | Toggle playback panel |
| `F11` | Fullscreen |
| `Ctrl+Q` | Quit |

---

## Known Issues & Feedback

EB plus is under active development and may still contain bugs. If you encounter any issue — crashes, rendering glitches, broken controls, or unexpected behavior — please [open an issue](../../issues). Bug reports from real users are the most direct help.

---

## License

- **Original code**: [MIT](LICENSE)
- **openEB SDK**: [Apache 2.0](openeb/licensing/LICENSE_OPEN) — copyright Prophesee
- **Inivation live-camera support** (`gui/davis/`): protocol, register tables and
  defaults ported from [dv-processing](https://gitlab.com/inivation/dv/dv-processing)
  ([Apache 2.0](https://gitlab.com/inivation/dv/dv-processing/-/blob/master/LICENSE), © iniVation AG) — attribution headers in each file
- **jAER-aligned algorithms**: reference behavior from
  [jAER](https://github.com/jaer-project/jaer3) ([LGPL-2.1](ref/jaer/COPYING)) — re-implementations, no code vendored
- **DV GUI**: not used. iniVation's dv-gui ships a custom (non-standard) license;
  the IMU visualization is a fresh implementation sharing only the generic
  rolling-curve concept

---

<div align="center">

Built with Qt 6 · OpenCV · openEB SDK

</div>
