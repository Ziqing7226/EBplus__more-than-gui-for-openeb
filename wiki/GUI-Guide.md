# GUI Guide

EB plus uses a VSCode-style layout: a custom title bar with dropdown menus, a left sidebar (ActivityBar + stacked panels), a central OpenGL display, and a right-side algorithm display area. There is no classic menu bar or toolbar — all configuration lives in the sidebar.

## Title Bar

The `CustomTitleBar` (36 px tall) shows the "EB plus" chip on the left, followed by dropdown menus:

| Menu | Contents |
|------|----------|
| **File** | Open File, Open Recent, Save/Load Camera Config, Save/Load Biases, Save/Load Algo Params, Exit |
| **View** | Toggle Playback Panel, Reset/Save/Load Layout, Fullscreen |
| **Theme** | Color submenu (5 colors) + Mode submenu (Follow System / Light / Dark) |
| **Tools** | Intrinsic Wizard (calibration), Sharpness |
| **Help** | About, About Qt |

Window control buttons (minimize / maximize / close) are on the right. The title bar follows the active theme.

## Sidebar (ActivityBar)

The left sidebar is a 48 px icon column (`ActivityBar`) that switches between 5 panel groups via `QStackedWidget`. Each group hosts one or more panels in a scrollable page.

| Group | Icon | Tooltip | Panels |
|-------|------|---------|--------|
| **Camera** | `camera` | Camera devices and connection info | Devices, Information |
| **Display & Stats** | `chart` | Display settings and statistics | Display, Statistics |
| **Hardware** | `cpu` | Biases, ROI, ESP and trigger configuration | Biases, ROI, ESP, Trigger, Preprocessing |
| **Algorithms** | `blocks` | Algorithm selection and preprocessing | File Tools, Algorithms |
| **Tools** | `tools` | File conversion and tools | (Calibration wizard placeholder) |

- The sidebar can be collapsed to 48 px (icon-only) or expanded to the default 380 px.
- Drag the blank area of the ActivityBar to resize (cursor feedback: open hand → closed hand).
- "Reset Layout" expands the sidebar if it was collapsed.

## Panels (11 total)

### Camera group
- **Devices** — camera discovery, connect/disconnect, refresh. Shows available cameras and connection status.
- **Information** — sensor metadata: model, resolution, serial number, firmware version, generation.

### Display & Stats group
- **Display** — accumulation time (1–1 000 000 us, exponential slider + spinbox), frame rate (1–60 fps), FPS limit (1–1000), color palette (Dark / Light / CoolWarm / Gray).
- **Statistics** — live event rate, peak rate, ON/OFF ratio, FPS, timestamp, and (in online camera mode) per-algorithm event drop rate.

### Hardware group
- **Biases** — dynamically enumerates all HAL LL-biases; slider + spinbox + reset per bias; save/load `.bias` files. On inivation devices an **Auto Bias** group appears: it keeps the event rate inside a user band, rebalances ON/OFF when one polarity dominates, and homes the biases to defaults when both constraints hold (DVXplorer drives the contrast thresholds with inverted signs).
- **ROI** — multi-rectangle ROI / RONI via `I_ROI`; drag-to-select on the display; apply/clear.
- **ESP** — Anti-Flicker (mode / band / presets / duty cycle / threshold), Trail Filter (type / threshold), ERC (target event rate).
- **Trigger** — Trigger In (per-channel enable) + Trigger Out (enable / period / duty cycle).
- **Preprocessing** — 8-stage filter chain (see [Preprocessing](#preprocessing-filter-chain)).

All hardware panels degrade gracefully when a device lacks a facility; on AEDAT4 replay the IMU/APS side streams are handled by the Devices-panel checkboxes instead (see Recording & Playback).

### Algorithms group
- **File Tools** — recording (RAW for Prophesee; AEDAT4 for inivation), file cutter, format conversion (RAW ↔ HDF5 ↔ CSV), AVI export.
- **Algorithms** — algorithm selection + shared preprocessing (ROI, noise filter, 1/4 downsample, undistort) + per-algorithm parameters. See [Algorithms](Algorithms.md).

## Display

The central `EventDisplayWidget` is an OpenGL 3.3 core-profile widget with a letterboxed viewport (preserves aspect ratio). It renders events using one of 7 frame modes:

| Frame Mode | Description |
|------------|-------------|
| Integration | Time-integrated event accumulation with decay |
| Diff | Frame-to-frame event difference |
| Histogram | ON/OFF event count histogram |
| Time Decay | Exponential decay visualization |
| Contrast Map | ON-OFF contrast difference |
| Periodic | Fixed-period frame generation |
| On-Demand | Manual/snapshot frame |

Color palettes: Dark, Light, CoolWarm, Gray.

The display also supports overlays drawn by algorithms (bounding boxes, trajectories, vectors, arrows) via `FrameAnnotator`, and a pixel probe (click to inspect an event sequence / ISI / polarity).

## Preprocessing Filter Chain

Thread-safe pipeline of 4 stackable stages, toggled from the Preprocessing panel. Applied in order:

1. Polarity Filter (OFF / ON)
2. Polarity Invert
3. Flip X
4. Flip Y

The filter chain is applied to both display rendering and algorithm event windows, so flipped/filtered events stay consistent everywhere.

## Tools Menu

The **Tools** dropdown menu (in the custom title bar) hosts two calibration-adjacent utilities:

### Intrinsic Wizard

A dialog that calibrates the camera intrinsics using only events (no APS frames). The pattern is embedded in the dialog: a **blinking chessboard** (9×6 inner corners = 10×7 squares — asymmetric so the checkerboard has a unique orientation) that alternates with a blank frame every 10 ms (a 20 ms full cycle), rendered as two cached pixmaps toggled by a timer for a stutter-free preview. The square size defaults to **0 = "not measured"** — captures are pixel observations and stay valid while the value is 0; measure one edge of a square with a ruler and type it in (screen DPI is deliberately not used, it is unreliable on X11). Entering the value runs the calibration automatically, the value can be corrected at any time without losing the captures, and Export reminds you while the size is still 0.

While the wizard is open, **Auto Bias** is enabled automatically with the rate band forced to 19–20 Mev/s, so the capture window sees a dense-but-bounded event stream no matter how strong the LCD backlight-PWM noise floor is. Closing the wizard disables Auto Bias again; the pre-wizard bias values and rate band are restored.

Workflow:

1. **Aim** — the live aim-view shows the camera feed with a **coverage overlay** (every accepted view's corners as dots, their convex hull, and a "Coverage: N%" label); point the camera at the on-screen board and keep it still. Guidance only — nothing gates the capture.
2. **Capture (Space)** — the wizard takes the last capture-window (default 100000 µs, tunable 200–200000 µs) of CD events, accumulates per-pixel ON/OFF event counts, thresholds them adaptively (0.30 × the 99th percentile of the per-pixel distribution, clamped 12–64) into a binary blink frame, and runs `cv::findChessboardCorners` (`CALIB_CB_FILTER_QUADS` only — no cornerSubPix on binary frames) with a radial straightness gate and a corner-bridge fallback. A successful detection is committed directly through the coverage (≥ 4% of the frame area) and duplicate-pose gates — there is no review dialog; a failure is reported on the status line.
3. **Run Calibration + Export** — when the target frame count (default 20) is reached, run the two-pass Zhang calibration (pass 1 fixes aspect ratio + K3 and drops views whose per-view RMS exceeds mean + 2·std, pass 2 refits the kept views with K3 free) and export to YAML. The default path is `~/Documents/EBplus/calibration/intrinsic.yml` (identical to the undistort preprocessor's default path) and writes `image_width`, `image_height`, `camera_matrix`, `distortion_coefficients`, `rms`, plus per-view RMS and kept/removed frame counts.

Controls: capture window, square size (mm, 0 = not measured), target frames, progress bar, live preview of the last accepted frame, and a **Delete this capture** button (removes the last accepted frame from the calibration).

### Sharpness

A dialog that plots the **variance of Laplacian** of the current event visualization frame as a rolling 2-second line chart, polled at 10 Hz. Higher values = sharper. Useful for bias tuning and lens focus: point the camera at a high-contrast static scene and watch the line climb as focus improves.

The chart's Y-axis uses a **fixed ceiling** computed from the current frame resolution rather than auto-scaling: the algorithm estimates the theoretical maximum variance for an ideal single-pixel checkerboard under `cv::Laplacian` (ksize=3, `BORDER_REPLICATE`) — `σ²_max = [n_int·1020² + n_edge·765² + n_corner·510²] / (W·H)` — so the scale stays stable across time and the line never jumps as the data range shifts. The ceiling converges to `1020² = 1 040 400` for large sensors and is recomputed each tick from the live frame dimensions.

### Undistort Preprocessing

Not a menu item, but related: in the **Algorithms** panel's Preprocessing group, the **Undistort (apply calibration)** checkbox loads the YAML written by the Intrinsic Wizard and applies a forward event-address LUT (via `cv::undistortPoints`, with K adjusted for the ROI origin and downsample factor) to every event after filter + downsample. Default path: `~/Documents/EBplus/calibration/intrinsic.yml` — identical to the wizard's default export path, so the two defaults always point at the same file. Click **Browse...** to point at any other YAML.

## Recording & Playback

- **Prophesee / CenturyArks** — record to `.raw` (SDK RAW format).
- **inivation DAVIS / DVXplorer** — record to **AEDAT4** (opens in DV and other inivation tools). The record dialog shows **Include IMU samples** / **Include APS frames** checkboxes (only for cameras that have the stream; both included by default) — the recorded file carries exactly what you selected.
- **Playback** — open `.raw`, `.aedat4` and `.alpdata` files; speed control, seek, pause/resume, position tracking. Playback window displays integer microseconds (no scientific notation); playback rate shows 6 decimal places.
- **AEDAT4 replay visualization** — a recording that contains IMU samples and/or APS frames shows the same **IMU stream** / **APS frames** checkboxes (Devices panel) as a live camera; the IMU and APS windows work against the replayed data exactly as against hardware.
- **Loop playback** — cyclic playback; algorithm temporal state resets on each loop to avoid frozen output.
- **File cutter** — extract a time range from an event file.

The playback dock can be toggled with `Ctrl+Shift+P`. Recording a replay is refused (recordings capture live cameras only).

## Export & Conversion

Available from the File Tools panel:

- **Format conversion**: RAW ↔ HDF5 ↔ CSV (background worker thread).
- **AVI export**: render events to a video file via `CDFrameGenerator` + `CvVideoRecorder`. Configurable FPS, accumulation time, quality, color mode.

## Theming

- **5 background colors**: Gray, Green, Yellow, Pink, Blue (default).
- **3 modes**: Follow System (default, requires Qt 6.5+), Always Light, Always Dark.
- Dark mode uses a **dark variant of the chosen color** — not pure black.
- Text color auto-adjusts (black on light, white on dark).
- The "EB plus" title chip uses inverse colors relative to the background for high contrast.
- Settings persist across restarts (`QSettings`).
- Theme changes apply immediately (style unpolish/polish).

## Multi-Window

- **XYT 3D point cloud** — GPU-accelerated 3D event visualization (`SpaceTimeDisplay`, VBO + GLSL).
- **IMU window** (inivation cameras and AEDAT4 replays containing IMU samples) — acceleration/gyroscope/temperature readouts plus a live 3D view of the camera orientation (a cuboid you can see tilt and turn with the camera). Just open the window while the camera rests: orientation is ready immediately, the sensor fine-tunes itself over the first seconds, movements are followed in real time and a closed path returns to where it started. Yaw (rotation around the vertical axis) drifts slowly over minutes — a 6-axis IMU has no compass, every device behaves this way.
- **APS window** (DAVIS cameras and AEDAT4 replays containing frames) — live grayscale preview with automatic exposure: point at something too dark or too bright and the exposure adjusts by itself.
- **Algorithm display windows** — `AlgoWindow` dockable windows showing algorithm title + output only (no parameters — those live in the sidebar).
- **Layout persistence** — save/restore dock geometry and window positions to JSON (View → Save/Load Layout).

## Keyboard Shortcuts

| Shortcut | Action |
|----------|--------|
| `Ctrl+O` | Open file |
| `Ctrl+Shift+P` | Toggle playback panel |
| `F11` | Fullscreen |
| `Ctrl+Q` | Quit |

## Configuration Files

- **`.bias`** — camera bias presets (save/load from File menu or Biases panel).
- **Algo params JSON** — per-algorithm parameter snapshots (File → Save/Load Algo Params).
- **Layout JSON** — dock/window geometry (View → Save/Load Layout).
- **QSettings** — theme color/mode, sidebar state, recent files.
