# EB plus Wiki

Welcome to the wiki for **EB plus** — a polished, open-source Qt 6 desktop GUI for event cameras, built on [openEB](https://github.com/prophesee-ai/openeb) v5.2.0.

EB plus gives you a complete desktop workflow for event-camera data: real-time visualization, camera control, recording & playback (RAW and DV-native AEDAT4 — events, IMU and APS frames), 25 registered algorithms (21 self-developed + 4 OpenEB filter stages), calibration, and data export.

---

## Pages

| Page | What's inside |
|------|---------------|
| [Getting Started](Getting-Started.md) | Build, run, environment variables, troubleshooting |
| [GUI Guide](GUI-Guide.md) | VSCode-style sidebar, panels, display modes, theming, shortcuts |
| [Algorithms](Algorithms.md) | Algorithm registry, categories, E2VID, preprocessing, noise filter |
| [Architecture](Architecture.md) | Directory layout, AlgoBridge, data flow, extension points |

---

## At a Glance

- **Platform**: Linux (Ubuntu 22.04+), Qt 6, OpenCV 4, C++17
- **Cameras**: Prophesee / CenturyArks event cameras via openEB HAL, plus **inivation DAVIS346/640, DAVIS240A/B/C, CDAVIS and DVXplorer** over USB (libusb, optional)
- **Display**: OpenGL 3.3 core, 7 frame modes, 4 palettes, 60+ FPS
- **Algorithms**: 25 registered (filtering, motion, detection, tracking, reconstruction, analytics, visualization) + the Intrinsic Wizard calibration tool
- **E2VID**: Deep-learning event-to-video reconstruction via ONNX Runtime, with optional Intel iGPU acceleration via OpenVINO (default mode)
- **inivation streams**: IMU 3D-attitude window, APS frame preview with automatic exposure (color on supported sensors), hardware ROI, per-model bias maps and Auto Bias
- **Recording**: Prophesee → RAW; inivation → DV-native AEDAT4 — the record dialog lets you include IMU samples and (on DAVIS) APS frames (both on by default), and replays show them like a live camera
- **Themes**: 5 colors × 3 modes (Follow System / Light / Dark)
- **License**: MIT (original code) + Apache 2.0 (openEB SDK; inivation device layer ported from dv-processing, Apache 2.0) — see the README for third-party notices

## Camera Vendors

| Vendor | Connection |
|--------|------------|
| Prophesee | openEB HAL plugin path |
| CenturyArks | openEB HAL plugin path |
| inivation DAVIS / DVXplorer | Direct USB (libusb) — see [Getting Started](Getting-Started.md) |

Set `MV_HAL_PLUGIN_PATH` to match your Prophesee/CenturyArks camera before launching; inivation devices are discovered over USB directly (udev rules included).
