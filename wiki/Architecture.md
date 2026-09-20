# Architecture

EB plus is split into two top-level layers: the **GUI application** (`gui/`) and the **self-developed algorithm library** (`algo/`). The openEB SDK (`openeb/`, Apache 2.0) is included as a subtree and provides the Prophesee/CenturyArks camera HAL and event decoding. Inivation DAVIS/DVXplorer cameras connect through a self-contained libusb device layer (`gui/davis/`, ported from the dv-processing reference).

```
┌─────────────────────────────────────────────────────────┐
│                       gui/  (Qt 6)                       │
│  main_window ── widgets ── panels ── display ── recorder │
│        │                                  │              │
│        └──────── algo_bridge ─────────────┘              │
│                  │                                       │
│                  │ AlgoBackend (abstract)                │
│                  ├─ backends/*.cpp (self + openeb)       │
│                  └─ filter_chain                         │
└──────────────────┼──────────────────────────────────────┘
                   │
        ┌──────────┴──────────┐
        │    algo/ (C++17)     │
        │  cv / analytics /    │
        │  calibration / common│
        └──────────┬──────────┘
                   │
        ┌──────────┴──────────┐
        │  openeb/ SDK v5.2.0  │
        │  HAL · Core · Base   │
        └─────────────────────┘
```

## Directory Layout

```
GUI-for-openEB/
├── gui/                  # Qt 6 application
│   ├── main.cpp              # entry point; env-var defaults, OpenGL format, font
│   ├── main_window.*         # main window: title-bar menus, docks, signal wiring
│   ├── widgets/              # CustomTitleBar, ActivityBar, AlgoWindow, pixel probe,
│   │                         #   imu_window (3D attitude), aps_window (frames + AEC),
│   │                         #   target labeler, mouse adaptor
│   ├── panels/               # 11 sidebar panels (AbstractPanel base)
│   │   ├── abstract_panel.*      # base class: camera lifecycle decoupling
│   │   ├── settings_panel.*      # ActivityBar + QStackedWidget container
│   │   ├── devices_panel.*       # camera discovery/connection
│   │   ├── information_panel.*   # sensor metadata
│   │   ├── display_panel.*       # frame mode/fps/palette
│   │   ├── statistics_panel.*    # event rate / drop rate / FPS
│   │   ├── biases_panel.*        # LL-bias sliders
│   │   ├── roi_panel.*           # multi-rect ROI / RONI
│   │   ├── esp_panel.*           # Anti-Flicker / Trail / ERC
│   │   ├── trigger_panel.*       # Trigger In / Out
│   │   ├── preprocessing_panel.* # 8-stage filter chain
│   │   ├── algorithms_panel.*    # algorithm selection + shared preproc + params
│   │   └── file_tools_panel.*    # recording / conversion / export
│   ├── display/              # OpenGL rendering
│   │   ├── event_display_widget.* # QOpenGLWidget, GLSL 3.30 core
│   │   ├── display_strategy.*     # IDisplayStrategy: Passive/Overlay/Replace/Standalone
│   │   ├── frame_annotator.*      # bbox/ID/trajectory/arrow overlays
│   │   └── space_time_display.*   # XYT 3D point cloud (VBO + GLSL)
│   ├── app/                  # controllers
│   │   ├── camera_controller.*    # camera lifecycle (live + file), conditioning
│   │   ├── stream_conditioner.*   # ONE conditioning pass per source (ROI →
│   │   │                          #   filters → noise → thin → undistort → flips)
│   │   ├── frame_pipeline.*       # CD events → QImage rendering
│   │   ├── file_frame_generator.* # file-source frame generation + loop/flip/seek
│   │   ├── external_file_source.* # AEDAT4/ALPDATA playback source interface
│   │   ├── aedat4_file_source.*   # DV-native AEDAT4 reader (events/IMU/APS)
│   │   ├── lz4_frame_decoder.*    # built-in LZ4 frame decoder
│   │   ├── statistics_controller.*# event-rate computation
│   │   ├── file_converter.*       # background RAW/HDF5/CSV conversion
│   │   ├── icon_provider.*        # SVG icon cache (theme-adaptive)
│   │   └── theme_controller.*     # 5 colors × 3 modes
│   ├── algo_bridge/          # algorithm registry + filter chain
│   │   ├── algo_bridge.*          # AlgoBridge: registry, list_algos(), enable()
│   │   ├── algo_backend.h         # AlgoBackend base + AlgoResult + AlgoInfo
│   │   ├── filter_chain.*         # thread-safe 8-stage preprocessing
│   │   └── backends/              # backend implementations
│   │       ├── backend_registry.h     # factory map
│   │       ├── backend_factory.cpp    # factory wiring
│   │       ├── backend_common.h       # shared param helpers (pint/pfloat/penum/...)
│   │       ├── cv_backends.cpp        # self CV algorithm backends
│   │       ├── cv_vector_backends.cpp # vector-output CV backends
│   │       ├── analytics_backends.cpp # analytics backends
│   │       ├── analytics_extra_backends.cpp
│   │       ├── display_backends.cpp   # display-mode wiring
│   │       ├── filter_backends.cpp    # self filter backends
│   │       ├── openeb_filter_backends.cpp    # openEB filter wrappers
│   │       ├── openeb_frame_backends.cpp     # openEB frame-mode wrappers
│   │       ├── openeb_preproc_backends.cpp  # openEB preprocessor wrappers
│   │       └── openeb_util_backends.cpp      # openEB utility wrappers
│   ├── recorder/             # RAW/AEDAT4 recording & playback
│   │   ├── recorder_controller.*
│   │   ├── aedat4_writer.*         # DV-native AEDAT4 writer (events+IMU+APS)
│   │   ├── playback_controller.*
│   │   └── playback_controls.*
│   ├── davis/                # inivation device layer (libusb, no SDK)
│   │   ├── davis_device.* / dvxplorer_device.*   # USB transport + chip init
│   │   ├── davis_parser.* / dvxplorer_parser.*   # wire decoders
│   │   ├── davis_biases.* (+ LL adapters)        # per-model bias maps
│   │   ├── batch_worker.h                        # USB→worker batch handoff
│   │   ├── imu_decoder.h / imu_pose.h            # IMU6 decode + attitude
│   │   ├── aps_decoder.h / auto_exposure.h       # APS frames + AEC
│   │   └── 66-inivation.rules                    # udev rules
│   ├── exporter/             # HDF5/CSV/AVI export
│   ├── calibration/          # intrinsic wizard
│   ├── config/               # JSON config + layout persistence
│   │   ├── config_manager.*
│   │   └── layout_manager.*
│   ├── resources/            # Qt resources (compiled in)
│   │   ├── theme/            #   tokens.h + base.qss.in
│   │   ├── icons/            #   Lucide-style SVG icons
│   │   ├── theme.qrc
│   │   └── icons.qrc
│   └── tests/                # GUI unit tests (GTest + CTest)
├── algo/                  # self-developed algorithm library (21 of these are registered)
│   ├── common/               # event packets, frame generator, filters, Kalman, LIF, ...
│   ├── cv/                   # 22 CV algorithm headers + noise_filter (8 modes)
│   ├── analytics/            # auto-bias controller, E2VID, frequency analytics,
│   │                         #   sensor self-test + e2vid/ ONNX inference
│   ├── calibration/          # intrinsic calibration (blink detect + two-pass solver)
│   └── tests/                # algorithm tests
├── openeb/                # openEB SDK (Apache 2.0, v5.2.0)
├── models/                # E2VID PyTorch → ONNX conversion (convert_to_onnx.py)
├── third_party/           # ONNX Runtime (user-installed, git-ignored)
├── wiki/                  # this wiki
├── pic/                   # screenshots
├── run.sh                 # launcher (env vars)
├── CMakeLists.txt         # v3.0.0
├── LICENSE                # MIT (original code)
├── README.md              # English
└── README_CN.md           # Chinese
```

## Key Abstractions

### AlgoBridge

The central algorithm registry (`gui/algo_bridge/algo_bridge.cpp`). Holds a `std::unordered_map<std::string, AlgoInfo>` of all 25 registered algorithms (21 self-developed + 4 OpenEB filter stages). Each entry has:

- `name` — registry key (e.g. `"object_tracker"`)
- `display_name` — UI label (e.g. `"Object Tracker"`)
- `category` — `"cv"` / `"analytics"` / `"calibration"` / `"openeb_*"`
- `source` — `"self"` or `"openeb"`
- `display_mode` — `Passive` / `Overlay` / `Replace` / `Standalone`
- `params` — parameter metadata (name, label, type, default, min, max, enum options, mode_filter)

`list_algos()` enumerates all; `enable(name)` activates one (disabling others). The GUI reads/writes parameters via `set_param` / `get_param`.

### AlgoBackend

Abstract base (`algo_backend.h`) implemented by each backend in `backends/`. Defines `set_param` / `get_param` / `process` / `reset` / `result`. The bridge owns one `AlgoInstance` per active algorithm, running on the GUI thread (online camera slow algorithms use an async worker thread that discards stale event batches).

### IDisplayStrategy

Four strategies (`display_strategy.h`) controlling how an algorithm's `AlgoResult` reaches the display: `Passive` (nothing), `Overlay` (annotate the live frame), `Replace` (swap the frame), `Standalone` (open an `AlgoWindow`).

### FilterChain

Thread-safe 8-stage preprocessing pipeline (`filter_chain.h`). Applied at render time to both the display and algorithm event windows. Mutex-protected; toggled from the Preprocessing panel.

### AbstractPanel

Base class for all sidebar panels (`panels/abstract_panel.*`). Decouples panels from camera lifecycle — panels react to camera start/stop signals rather than holding direct camera references, so they work correctly across camera/file-mode switches.

## Data Flow

### Prophesee / CenturyArks camera mode

```
Camera (HAL) → I_EventsStream callback → StreamConditioner
    → FramePipeline / display / AlgoBridge → IDisplayStrategy → display/AlgoWindow
```

### inivation DAVIS / DVXplorer mode

```
USB stream → wire decoder → event processing → display / algorithms
IMU samples and APS frames decode on the side and feed their windows
directly (always current, never delayed by the event flood).
```

- Events are conditioned once (ROI, filters, noise, undistort, flips) and
  every consumer shares that output.
- Recordings tap the decoded stream before display processing, so the file
  contains exactly the streams selected in the record dialog (events, plus
  the optional IMU / APS side streams).

### File playback mode

```
RAW (SDK)           → SDK offline stream → FramePipeline
AEDAT4 / ALPDATA    → external source reader thread → FramePipeline buffer
                    → FileFrameGenerator (loop / seek / rate) → display + AlgoBridge
```

- All events buffer on open (real_time_playback=false); playback rate,
  seek, pause/resume and loop run from the buffer. AEDAT4 replays surface
  their IMU/APS side streams through the same controller slots as live
  devices, and the IMU attitude animates in sync with the playback
  position (seek and loop replay the attitude too).
- Loop playback re-signals algorithm `reset()` to clear temporal state each iteration.

## Threading Model

- **GUI thread** — all panel interaction, display rendering, most algorithm processing.
- **SDK data thread** — openEB event-stream callback (FramePipeline). FilterChain is mutex-protected.
- **USB decode thread** (inivation devices) — receives camera data and decodes it; IMU/APS samples and the recording feed run here so they stay real-time.
- **Event processing thread** (inivation devices) — runs the display/algorithm pipeline so heavy processing can never delay the camera data.
- **External file reader thread** — decodes AEDAT4/ALPDATA files into the playback buffer.
- **Async worker thread** — used by `AlgoInstance` for slow online-camera algorithms; discards stale batches.
- **File converter thread** — background RAW/HDF5/CSV conversion (`file_converter.cpp`).

## Configuration & Persistence

- `ConfigManager` (`config/config_manager.*`) — JSON serialization for algo params and camera config; versioned schema (`"version": 1`).
- `LayoutManager` (`config/layout_manager.*`) — dock/window geometry to JSON.
- `QSettings` — theme color/mode, sidebar state, recent files.
- `.bias` files — camera bias presets.

## Build System

- `CMakeLists.txt` (root) — project version 3.0.0, C++17, GCC 15 `<cstdint>` fix.
- `find_package` for Qt6, MetavisionSDK 5.2.0, OpenCV.
- ONNX Runtime auto-detected from `third_party/onnxruntime/` (with RPATH configured).
- `enable_testing()` before `add_subdirectory` so GUI/algo tests register with CTest.
- `gui/tests/` and `algo/tests/` use `gtest_discover_tests()`.
