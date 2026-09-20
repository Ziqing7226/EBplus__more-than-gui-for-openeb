// gui/main_window.h — top-level QMainWindow.
//
// Layout (design §5.1 + §11):
//   - title bar:  CustomTitleBar (app name chip + 6 menu dropdowns + window
//                 controls), installed via setMenuWidget — no QMenuBar hack.
//   - central:    EventDisplayWidget (OpenGL)
//   - left dock:  SettingsPanel (VSCode-style sidebar: ActivityBar + stacked
//                 panels, no title bar — §11.2 point 5)
//   - right dock: AlgoWindow instances (event→video, XYT plot, etc. — §11.2
//                 point 3)
//   - bottom (status bar): connection | event rate | timestamp | recording state
//   - bottom (above status bar): PlaybackControls (file playback only)
//   - menus:      File | View | Theme | Camera | Tools | Help (Theme is a
//                 top-level dropdown to the right of View — §11.2 point 6)
//
// The sidebar has no title bar; its visibility is toggled via a button at the
// bottom of the ActivityBar (chevron direction depends on dock area + state).
//
// Phase 1-2: live camera + bias/roi/esp/trigger control.
// Phase 3:   recorder + playback controls.
// Phase 4:   exporter dialog + config manager.
// Phase 5:   preprocessing panel, algorithms panel, file tools panel.
// Phase 9:   calibration wizard.
// Phase 10:  multi-window layout, i18n, layout save/restore.

#ifndef GUI_MAIN_WINDOW_H
#define GUI_MAIN_WINDOW_H

#include <QHash>
#include <QCheckBox>
#include <QMainWindow>
#include <QPointer>
#include <atomic>
#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <metavision/sdk/base/utils/callback_id.h>
#include <metavision/sdk/base/events/event_cd.h>

#include "algo_bridge/algo_bridge.h"
#include "app/camera_controller.h"
#include "app/file_converter.h"
#include "app/theme_controller.h"
#include "config/config_manager.h"
#include "config/layout_manager.h"
#include "display/event_display_widget.h"
#include "algo/common/performance_meter.h"
#include "display/frame_annotator.h"
#include "exporter/exporter_controller.h"
#include "panels/settings_panel.h"
#include "recorder/playback_controller.h"
#include "recorder/recorder_controller.h"
#include "display/space_time_display.h"
#include "widgets/algo_window.h"
#include "widgets/custom_title_bar.h"

class QLabel;
class QAction;
class QMenu;
class QDockWidget;
class QTimer;

namespace gui {

class PlaybackControls;
class ExportDialog;
class CalibrationWizard;
class FocusDialog;
class ImuWindow;
class ApsWindow;

class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow();

protected:
    void closeEvent(QCloseEvent* event) override;
    void resizeEvent(QResizeEvent* event) override;

signals:

private slots:
    void on_open_file();
    void on_connect_first();
    void on_disconnect();
    void on_refresh_devices();
    void on_about();
    void on_save_config();
    void on_load_config();
    void on_save_biases();
    void on_load_biases();
    void on_toggle_roi_drag(bool on);
    /// Shared handler for both pages' "Enable ROI" checkboxes (Phase 2.6
    /// debug D-6): applies the stored rect (default center 256×144 when
    /// unset) and opens the settings dialog when turned on.
    void on_roi_enable_toggled(bool on);
    /// Opens the modal unified-ROI settings dialog (Phase 2.6 debug D-6).
    void open_roi_settings_dialog();

    // Phase 3 — recording / playback.
    void on_record_start();
    void do_record_start(const QString& path, bool save_biases,
                         bool include_imu, bool include_aps);
    void on_record_stop();
    void on_record_elapsed(std::chrono::seconds s);

    // Phase 4 — export.
    void on_export_dialog();

    // Phase 9 — calibration.
    void on_intrinsic_wizard();
    void on_focus();

    // Phase 10 — multi-window / layout / standalone algorithm views.
    void on_open_xyt_view();
    /// Phase 2: IMU stream toggle from the Devices panel (open/close the
    /// live readout window and start/stop the stream).
    void on_imu_toggled(bool on);
    /// Devices-panel IMU toggle (programmatic state sync).
    void set_imu_ui_state(bool on);
    /// Phase 3: APS stream toggle from the Devices panel.
    void on_aps_toggled(bool on);
    void set_aps_ui_state(bool on);
    void on_open_algo_window(const std::string& algo_name);
    void on_save_layout();
    void on_load_layout();
    void on_reset_layout();

    // Status bar recording-dot blink (design §3.8.2).
    void on_rec_blink();

private:
    void build_menus();
    void build_status_bar();
    void build_title_bar_controls();
    void wire_signals();
    void update_palettes(int index);
    void forward_panel_message(const QString& msg, bool isError);

    /// Updates the connection-status dot color (green connected / gray
    /// disconnected) and toggles the text label's class property so the QSS
    /// status-conn / status-disc rules recolor it. Call after setText().
    void set_conn_connected(bool connected);
    /// Starts the 500ms recording blink timer and shows the red dot.
    void start_rec_blink();
    /// Stops the blink timer and hides the red dot.
    void stop_rec_blink();

    /// Updates the ActivityBar toggle button's chevron icon based on the
    /// current dock area (left/right) and content visibility state (§11.2
    /// point 5):
    ///   - Left dock + visible  → chevron-left
    ///   - Left dock + hidden   → chevron-right
    ///   - Right dock + visible → chevron-right
    ///   - Right dock + hidden  → chevron-left
    void update_toggle_icon();

    /// Handles the SettingsPanel::content_toggled signal: saves the dock
    /// width before hiding content, restores it after showing, and updates
    /// the toggle button icon (§11.2 point 5).
    void on_sidebar_content_toggled(bool visible);

    void on_file_opened_for_playback(const QString& path);

    // Algorithm event/result pipeline — pushes CD events to all live
    // AlgoInstances and pulls results for overlay/replace/standalone display.
    void install_algo_callback();
    void remove_algo_callback();
    void process_algo_results(QImage& frame);

    /// File playback: feeds the events in the current accumulation window
    /// (emitted by FileFrameGenerator) to all live AlgoInstances and to the
    /// XYT 3D display, synchronously with the displayed frame. This replaces
    /// the SDK CD callback path for file sources, where all events arrive in
    /// ~10ms and would otherwise be processed before the first frame is shown.
    void on_events_window_ready(std::shared_ptr<std::vector<Metavision::EventCD>> events,
                                Metavision::timestamp ts);

    EventDisplayWidget* display_{nullptr};
    /// Main-display ROI view-mode toggle (Phase 2.6 step 4), in a bar below
    /// the main display: checked = (a) adaptive zoom (frame cropped to the
    /// unified ROI and scaled to the window); unchecked = (b) full canvas
    /// with only-ROI content (default). Enabled only while the unified ROI
    /// is active.
    QCheckBox* roi_zoom_cb_{nullptr};
    /// Dialog-initiated ROI draw in flight (Phase 2.6 debug D-6 follow-up):
    /// while roi_draw_pending_ is true, the next roi_dragged applies the
    /// drawn rect directly (the dialog stays closed, no confirmation).
    bool roi_draw_pending_{false};
    /// Saved unified-ROI state for the default-ROI automation (Phase 2.6
    /// debug D-7): heavy algorithms force the center 256×144 ROI on enable
    /// and restore this prior state on disable.
    struct RoiAutomationSave {
        bool enabled;
        int x0, y0, x1, y1;
        bool roni;
    };
    std::optional<RoiAutomationSave> roi_automation_save_;
    /// Saved shared-downsample state for the E2VID automation (Phase 3):
    /// E2VID forces the 1/4 downsample ON while enabled; this prior state
    /// is restored on disable.
    std::optional<bool> e2v_downsample_save_;
    SettingsPanel* settings_{nullptr};
    QDockWidget* settings_dock_{nullptr};  ///< Right-dock wrapper, for hide/show.
    PlaybackControls* playback_controls_{nullptr};

    QLabel* status_conn_{nullptr};
    QLabel* status_rate_{nullptr};
    QLabel* status_ts_{nullptr};
    QLabel* status_rec_{nullptr};
    QLabel* status_conn_dot_{nullptr};   ///< Colored dot (green/gray).
    QLabel* status_rec_dot_{nullptr};    ///< Red dot, blinks while recording.
    QLabel* status_rate_icon_{nullptr};  ///< Chart icon (theme-recolorable, BUG-4).
    QLabel* status_ts_icon_{nullptr};    ///< Clock icon (theme-recolorable, BUG-4).
    QTimer* rec_blink_timer_{nullptr};
    bool rec_blink_on_{false};

    // File menu actions.
    QAction* a_save_cfg_{nullptr};
    QAction* a_load_cfg_{nullptr};
    QAction* a_save_biases_{nullptr};
    QAction* a_load_biases_{nullptr};

    /// "Recent Files" submenu — persisted via QSettings so the list survives
    /// restarts. Most recent first, capped at 10 entries.
    QMenu* m_recent_files_{nullptr};
    void build_recent_files_menu();
    void add_recent_file(const QString& path);
    void on_open_recent_file(const QString& path);

    CameraController camera_;
    AlgoBridge algo_bridge_;

    // Phase 3.
    RecorderController recorder_;
    PlaybackController playback_;

    // Phase 4.
    ExporterController exporter_;
    ConfigManager config_;
    ExportDialog* export_dialog_{nullptr};
    class RecordDialog* record_dialog_{nullptr};

    // Phase 5.
    FileConverter file_converter_;

    // Phase 9 — owned lazily; built when the wizard is first opened.
    CalibrationWizard* calibration_wizard_{nullptr};
    FocusDialog*       focus_dialog_{nullptr};

    // Phase 10 — layout manager is owned from construction.
    std::unique_ptr<LayoutManager> layout_manager_;

    // Standalone algorithm windows (design §5.6.1 / §5.6.6).
    // xyt_visualizer keeps a dedicated SpaceTimeDisplay (QOpenGLWidget with
    // 3D rendering); all other algorithms use the generic AlgoWindow
    // (algo_windows_) for both parameter control and result display.
    QPointer<SpaceTimeDisplay> xyt_display_;
    std::shared_ptr<AlgoInstance> xyt_algo_;
    // Phase 2: live IMU readout window (inivation sources only).
    QPointer<ImuWindow> imu_window_;
    // Phase 3: live APS frame preview window (DAVIS only).
    QPointer<ApsWindow> aps_window_;

    /// Generic AlgoWindow instances keyed by algo name (design §5.6.6).
    /// Every self-developed algorithm gets an AlgoWindow when enabled, so the
    /// user can adjust any parameter (including the 5 ROI params) at runtime.
    QHash<std::string, QPointer<AlgoWindow>> algo_windows_;

    // Algorithm event/result pipeline. The live event feed is the
    // conditioned-stream listener on CameraController (install_algo_callback)
    // — no per-camera SDK callback ID is held here anymore.
    std::atomic<Metavision::timestamp> algo_last_xyt_post_us_{0};
    FrameAnnotator annotator_;
    Metavision::timestamp prev_frame_ts_{0};
    /// Wall-clock time of the previous displayed frame; drives the
    /// file-mode "Display FPS" metric (event timestamps are meaningless
    /// there — audit §六-P6).
    std::chrono::steady_clock::time_point prev_frame_wall_{};

    /// True while closeEvent() is tearing down; AlgoWindow `closing`
    /// handlers skip modal report dialogs in that case (audit §六-M3).
    bool closing_app_{false};

    /// Performance profiler: measures end-to-end latency (event arrival →
    /// frame display), total events/frames, and drop count. Fed from the
    /// SDK CD callback (tick_events) and the frame_ready signal (tick_frame).
    /// Throughput is derived from the SDK RateEstimator's rate × sizeof(EventCD).
    gui_algo::PerformanceMeter perf_meter_;
    QTimer* perf_timer_{nullptr};       ///< 10 Hz flush to InformationPanel
    double last_rate_eps_{0.0};         ///< Cached from StatisticsController::rate_updated

    /// Saved sidebar dock width before content was hidden via the toggle
    /// button. Used to restore the width when content is shown again
    /// (§11.2 point 5). 0 means no saved width (first toggle or never set).
    int saved_sidebar_width_{0};

    /// Application theme controller (background color + light/dark mode).
    /// Owned by MainWindow; the SettingsPanel sidebar exposes its UI.
    ThemeController theme_;

    /// Edge/corner resize handles for the frameless window.
    std::vector<ResizeGrip*> resize_grips_;

    /// Custom title bar (installed via setMenuWidget) — replaces the old
    /// QMenuBar hack. Holds the app icon/name, the 6 menu dropdown buttons,
    /// and the window control buttons (design §3.6.1 + §11.2 point 6).
    CustomTitleBar* title_bar_{nullptr};
};

} // namespace gui

#endif // GUI_MAIN_WINDOW_H
