// gui/main_window.cpp

#include "main_window.h"

#include <QAction>
#include <QActionGroup>
#include <QApplication>
#include <QCloseEvent>
#include <QDockWidget>
#include <QEvent>
#include <QFileDialog>
#include <QFileInfo>
#include <QHBoxLayout>
#include <QLabel>
#include <QMenuBar>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QMouseEvent>
#include <QPushButton>
#include <QSet>
#include <QSettings>
#include <QStatusBar>
#include <QStyle>
#include <QTimer>
#include <QVBoxLayout>
#include <QWindow>

#include <atomic>
#include <cmath>
#include <vector>

#include <metavision/sdk/core/utils/colors.h>

#include "panels/algorithms_panel.h"
#include "panels/biases_panel.h"
#include "panels/devices_panel.h"
#include "panels/display_panel.h"
#include "panels/esp_panel.h"
#include "panels/file_tools_panel.h"
#include "panels/information_panel.h"
#include "panels/preprocessing_panel.h"
#include "panels/roi_panel.h"
#include "panels/statistics_panel.h"
#include "panels/trigger_panel.h"
#include "recorder/playback_controls.h"
#include "exporter/export_dialog.h"
#include "recorder/record_dialog.h"
#include "calibration/calibration_wizard.h"
#include "calibration/focus_dialog.h"
#include "app/icon_provider.h"
#include "display/display_strategy.h"
#include "widgets/activity_bar.h"
#include "widgets/aps_window.h"
#include "widgets/imu_window.h"
#include "widgets/unified_roi_dialog.h"

namespace gui {

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent),
      camera_(nullptr),
      algo_bridge_(),
      recorder_(nullptr),
      playback_(nullptr),
      exporter_(nullptr),
      file_converter_(nullptr) {
    // Remove the native WM title bar and replace it with a custom title bar
    // widget (CustomTitleBar) whose background color follows the application
    // theme.  This is the same approach VSCode uses (frameless BrowserWindow
    // + custom titlebar part drawn in HTML/CSS).  The native title bar color
    // is controlled by the window manager and cannot be changed reliably
    // from Qt, so we draw our own.
    setWindowFlags(Qt::Window | Qt::FramelessWindowHint);
    setWindowTitle(QStringLiteral("EB plus"));
    resize(1280, 720);

    display_ = new EventDisplayWidget(this);
    // Central area = main display + a bottom bar holding the ROI view-mode
    // toggle (Phase 2.6 step 4). Mode (b) full-canvas is the default; mode
    // (a) adaptive zoom crops the displayed frame to the unified ROI rect
    // and lets the widget scale it up.
    auto* central = new QWidget(this);
    auto* central_vbox = new QVBoxLayout(central);
    central_vbox->setContentsMargins(0, 0, 0, 0);
    central_vbox->setSpacing(0);
    central_vbox->addWidget(display_, 1);
    auto* roi_bar = new QHBoxLayout();
    roi_bar->setContentsMargins(6, 2, 6, 2);
    roi_bar->addStretch(1);
    roi_zoom_cb_ = new QCheckBox(tr("Zoom to ROI"), central);
    roi_zoom_cb_->setToolTip(
        tr("Adaptive zoom: show only the unified-ROI region, scaled to the "
           "window. Unchecked: full sensor canvas (ROI content only)."));
    roi_zoom_cb_->setEnabled(false);  // enabled only while the unified ROI is active
    roi_bar->addWidget(roi_zoom_cb_);
    central_vbox->addLayout(roi_bar);
    setCentralWidget(central);
    connect(roi_zoom_cb_, &QCheckBox::toggled, this, [this](bool on) {
        // When zooming, the whole view IS the ROI — hide the yellow overlay
        // frame (its sensor coordinates no longer map onto the cropped
        // texture). Restore it when returning to the full-canvas mode.
        bool en = false;
        int x0 = 0, y0 = 0, x1 = 0, y1 = 0;
        camera_.unified_roi(en, x0, y0, x1, y1);
        display_->set_roi_overlay(x0, y0, x1 - x0, y1 - y0, en && !on);
    });

    // Theme controller must be attached before the menu is built (so the
    // Theme menu actions reflect the persisted choice) and before the
    // SettingsPanel is built. The controller also persists settings via
    // QSettings.
    theme_.set_target(this);

    settings_ = new SettingsPanel(&algo_bridge_, &file_converter_, this);
    // Let file operations (convert/cut) skip the blocking OSC duration query
    // when operating on the currently-open, fully buffered file.
    file_converter_.set_duration_provider(
        [this](const QString& src) -> Metavision::timestamp {
            return (src == playback_.current_file()) ? playback_.duration_us() : 0;
        });
    // Let the file-operation dialogs prefill the source path with the
    // currently open file (same pattern as ExportDialog::set_source).
    if (auto* ft = settings_->file_tools_panel()) {
        ft->set_source_provider([this]() { return playback_.current_file(); });
    }
    settings_dock_ = new QDockWidget(tr("Settings"), this);
    settings_dock_->setObjectName("SettingsDock");
    settings_dock_->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
    settings_dock_->setWidget(settings_);
    // No dock features (not movable, not closable) and no title bar — the
    // sidebar is controlled entirely via the ActivityBar toggle button
    // (§11.2 point 5). An empty QWidget as the title bar widget hides the
    // default title bar (title text + close button).
    settings_dock_->setFeatures(QDockWidget::NoDockWidgetFeatures);
    settings_dock_->setTitleBarWidget(new QWidget(this));
    addDockWidget(Qt::LeftDockWidgetArea, settings_dock_);
    settings_dock_->setMinimumWidth(200);

    // Sidebar content toggle (§11.2 point 5): when the user clicks the
    // toggle button at the bottom of the ActivityBar, SettingsPanel emits
    // content_toggled. MainWindow saves/restores the dock width and updates
    // the toggle icon's chevron direction based on dock area + visibility.
    connect(settings_, &SettingsPanel::content_toggled, this,
            &MainWindow::on_sidebar_content_toggled);
    // Also update the toggle icon when the dock is moved to a different area
    // (left → right or vice versa).
    connect(settings_dock_, &QDockWidget::dockLocationChanged, this,
            [this](Qt::DockWidgetArea) { update_toggle_icon(); });

    // Phase 3: bottom-bar playback transport (hidden until a file is opened).
    playback_controls_ = new PlaybackControls(this);
    playback_controls_->set_controller(&playback_);
    auto* pb_dock = new QDockWidget(tr("Playback"), this);
    pb_dock->setObjectName("PlaybackDock");
    pb_dock->setAllowedAreas(Qt::BottomDockWidgetArea | Qt::TopDockWidgetArea);
    pb_dock->setWidget(playback_controls_);
    pb_dock->setFeatures(QDockWidget::DockWidgetMovable | QDockWidget::DockWidgetClosable);
    addDockWidget(Qt::BottomDockWidgetArea, pb_dock);
    pb_dock->setVisible(false); // shown only when a file is opened

    // Phase 4: export dialog (lazy-shown via menu).
    export_dialog_ = new ExportDialog(&exporter_, this);
    // Let the exporter skip the blocking OSC duration query when the user
    // exports the file that is currently open (already fully buffered —
    // the playback controller knows its duration).
    export_dialog_->set_duration_provider(
        [this](const QString& src) -> Metavision::timestamp {
            return (src == playback_.current_file()) ? playback_.duration_us() : 0;
        });

    playback_.set_camera(&camera_);

    // Phase 10: layout manager (saves/restores dock geometry).
    layout_manager_ = std::make_unique<LayoutManager>(this, this);

    // Phase 3 (§3.6.1): real custom title bar — replaces the QMenuBar hack.
    // Installed via setMenuWidget so it sits at the very top of the window,
    // above the central widget and docks. build_menus() adds the 6 dropdown
    // menus to it; build_title_bar_controls() applies the title/icon/colors.
    title_bar_ = new CustomTitleBar(this);
    setMenuWidget(title_bar_);

    build_menus();
    build_status_bar();
    wire_signals();

    build_title_bar_controls();

    // Edge/corner resize handles — since the window is frameless, the WM
    // no longer provides resize borders.  These 8 invisible grips call
    // QWindow::startSystemResize() so the WM handles the actual resize.
    using P = ResizeGrip::Position;
    for (auto p : {P::Left, P::Right, P::Top, P::Bottom,
                   P::TopLeft, P::TopRight, P::BottomLeft, P::BottomRight}) {
        auto* grip = new ResizeGrip(p, this);
        resize_grips_.push_back(grip);
    }
    // Position the grips immediately: resizeEvent() is the only other place
    // that repositions them, and if the WM delivers no resize event after
    // show() the grips would keep their default geometry (0,0,100,30) —
    // raised above the title bar's File menu (audit §六-M1).
    {
        const QRect r = rect();
        for (auto* grip : resize_grips_) {
            if (grip) grip->reposition(r);
        }
    }

    // Capture the factory layout (all docks in their default positions) so
    // reset_layout() can restore it. Must be called before load_default()
    // which may overlay a user-customized layout.  Set the sidebar to the
    // default width BEFORE capturing so reset_layout() restores to this
    // width instead of Qt's initial wide default (§13.2).
    if (settings_dock_) {
        settings_dock_->setMinimumWidth(200);
        const QList<QDockWidget*> docks = {settings_dock_};
        const QList<int> sizes = {380};
        resizeDocks(docks, sizes, Qt::Horizontal);
    }
    layout_manager_->capture_default();

    // Try restoring the previous layout (silent on failure).
    layout_manager_->load_default();

    // Force the sidebar visible on startup regardless of the restored
    // layout state — the user explicitly requested it be shown by default.
    if (settings_dock_) {
        settings_dock_->setVisible(true);
    }
    // Explicitly set the sidebar to the default width.  setMinimumWidth
    // alone doesn't set the actual width — the dock's width is controlled
    // by the saved layout state (restoreState) which may have a previously
    // saved wider width.  resizeDocks() is the only way to programmatically
    // set the dock width.  Deferred via QTimer::singleShot so it runs after
    // the window is shown and the layout pass has settled (§13.2).
    QTimer::singleShot(0, this, [this]() {
        if (!settings_dock_ || !settings_dock_->isVisible()) return;
        settings_dock_->setMinimumWidth(200);
        const QList<QDockWidget*> docks = {settings_dock_};
        const QList<int> sizes = {380};
        resizeDocks(docks, sizes, Qt::Horizontal);
    });
    // Set the initial toggle icon based on the dock's current area (left by
    // default) and content visibility (visible by default).
    update_toggle_icon();

    // Populate the device list on startup.
    on_refresh_devices();
}

MainWindow::~MainWindow() = default;

void MainWindow::closeEvent(QCloseEvent* event) {
    if (layout_manager_) layout_manager_->save_default();

    // Mark shutdown in progress: the AlgoWindow `closing` handlers below
    // consult this to skip modal report dialogs that would block the close
    // sequence (audit §六-M3).
    closing_app_ = true;

    // ---- Phase 1: stop all data sources BEFORE deleting child widgets. ----
    // The CD callback and frame pipeline run on the camera's data thread.
    // If we delete child widgets (AlgoWindow displays, xyt_display_, etc.)
    // while the pipeline is still active, queued invokeMethod lambdas may
    // dereference already-deleted widgets.
    remove_algo_callback();   // explicitly remove the CD callback
    recorder_.stop();
    camera_.disconnect();     // stops the data thread + frame pipeline

    // ---- Phase 2: delete lazily-created child windows. ----
    // Their destroyed() handlers access MainWindow members (e.g. camera_)
    // that would already be gone by the time ~QWidget
    // runs its automatic child cleanup.
    if (calibration_wizard_) {
        delete calibration_wizard_;
        // calibration_wizard_ is nulled by the destroyed signal handler.
    }
    if (focus_dialog_) {
        delete focus_dialog_;
        // focus_dialog_ is nulled by the destroyed signal handler.
    }
    // Standalone algorithm windows — use close() (not delete) so that
    // closeEvent fires, the `closing` signal is emitted, and the cleanup
    // handler disables the AlgoInstance. WA_DeleteOnClose schedules
    // deleteLater() for actual destruction; any remaining docks are also
    // deleted by Qt's parent-child cleanup when MainWindow is destroyed.
    // Collect pointers first and clear the hash: the `closing` handler
    // calls algo_windows_.remove(name), which would invalidate iterators
    // if we were still iterating.
    std::vector<QPointer<AlgoWindow>> algo_ptrs;
    algo_ptrs.reserve(algo_windows_.size());
    for (auto it = algo_windows_.begin(); it != algo_windows_.end(); ++it) {
        if (it.value()) algo_ptrs.push_back(it.value());
    }
    algo_windows_.clear();
    for (auto& ptr : algo_ptrs) {
        if (ptr) ptr->close();
    }
    if (xyt_display_) { delete xyt_display_.data(); }
    // Pre-delete child widgets that hold raw pointers to MainWindow members.
    // Without this, ~QObject child cleanup runs after member destructors,
    // causing use-after-free in widget destructors.
    if (export_dialog_) { delete export_dialog_; export_dialog_ = nullptr; }
    event->accept();
}

void MainWindow::resizeEvent(QResizeEvent* event) {
    QMainWindow::resizeEvent(event);
    // Reposition the frameless-window resize grips to the new edges.
    const QRect r = rect();
    for (auto* grip : resize_grips_) {
        if (grip) grip->reposition(r);
    }
}

/// @brief Applies the title and theme colors to the custom title bar, and
/// keeps the colors in sync with ThemeController.
void MainWindow::build_title_bar_controls() {
    if (!title_bar_) return;
    title_bar_->setTitle(windowTitle());

    // Apply the theme background/text colors and re-apply whenever the theme
    // changes (user color switch or system light/dark flip). The native WM
    // title bar color cannot be changed from Qt, but our custom title bar's
    // color is set directly so it always tracks the application theme.
    //
    // §13 — title chip color rules:
    //   - title_box = the OPPOSITE theme mode's panel color (light panel
    //     when dark mode is active, dark panel when light mode is active)
    //     so the rounded chip stands out from the title bar background.
    //   - title_fg  = RGB-inverse of title_box, so the text always has
    //     maximum contrast against the chip background.
    //   - The camera icon has been removed; only the "EB plus" text chip
    //     remains as the leftmost element.
    //
    // BUG-4: the status bar chart/clock icons are rendered once via
    // IconProvider::get(name) which reads QPalette::WindowText at call time;
    // after a theme switch the palette changes but the cached pixmaps do
    // not, so we re-render them here. The ActivityBar icons are refreshed
    // via SettingsPanel::refresh_icons() for the same reason. The toolbar
    // action icons are re-rendered via the "icon_name" property (§13.1).
    auto apply = [this]() {
        if (!title_bar_) return;
        // Clear the icon cache so re-rendered icons pick up the new theme
        // color (BUG-R7: stale icons from the previous theme otherwise
        // persist in the cache).
        IconProvider::clear_cache();
        // §15.3: title bar uses the panel color (same as the status bar) so
        // both bars share the same shade, with a subtle difference from the
        // sidebar which uses the primary background.
        const QColor bg(theme_.effective_panel_hex());
        const QColor fg(theme_.effective_text_hex());
        // §13: title chip background = the opposite mode's panel color
        // (light panel in dark mode, dark panel in light mode) so the chip
        // stands out from the title bar. The title text color is the
        // RGB-inverse of the chip background, guaranteeing high contrast
        // between text and chip in both themes.
        const QColor title_box(theme_.panel_hex(!theme_.is_dark_mode()));
        const QColor title_fg(255 - title_box.red(),
                              255 - title_box.green(),
                              255 - title_box.blue());
        title_bar_->setColors(bg, fg, title_fg, title_box);
        // §15.2: set the ActivityBar's separator color to match the title
        // bar's bottom line so both lines are consistent.  Dark bg → lighter
        // line, light bg → darker line (same logic as CustomTitleBar).
        if (settings_) {
            if (auto* bar = settings_->findChild<ActivityBar*>()) {
                const QColor sep = bg.lightness() < 128
                    ? bg.lighter(140)
                    : bg.darker(120);
                bar->set_separator_color(sep);
            }
        }
        // Re-render theme-recolorable status bar icons (BUG-4).
        if (status_rate_icon_) {
            status_rate_icon_->setPixmap(
                IconProvider::get(QStringLiteral("chart")).pixmap(QSize(12, 12)));
        }
        if (status_ts_icon_) {
            status_ts_icon_->setPixmap(
                IconProvider::get(QStringLiteral("clock")).pixmap(QSize(12, 12)));
        }
        // Refresh ActivityBar icons so they track the new theme.
        if (settings_) settings_->refresh_icons();
    };
    apply();
    connect(&theme_, &ThemeController::theme_changed, this, apply);
}

// ---------------------------------------------------------------------------
// UI construction
// ---------------------------------------------------------------------------

void MainWindow::build_menus() {
    // Phase 3 (§3.6.1/§3.6.3): menus live as dropdown buttons inside the
    // custom title bar (no top-level QMenuBar). Each addMenu() call adds a
    // QPushButton to the title bar that pops up the returned QMenu. The 5
    // dropdowns are File | View | Theme | Tools | Help; the former Calibration
    // menu folds into Tools (design §3.6.3). All existing actions are preserved.
    auto* mb = title_bar_;

    // File
    auto* m_file = mb->addMenu(tr("&File"));
    m_file->addAction(tr("&Open File..."), QKeySequence::Open, this,
                      &MainWindow::on_open_file);
    m_recent_files_ = m_file->addMenu(tr("Open &Recent"));
    build_recent_files_menu();
    a_save_cfg_ = m_file->addAction(tr("Save Camera Config..."), this, &MainWindow::on_save_config);
    a_load_cfg_ = m_file->addAction(tr("Load Camera Config..."), this, &MainWindow::on_load_config);
    a_save_cfg_->setEnabled(false);
    a_load_cfg_->setEnabled(false);
    a_save_cfg_->setToolTip(tr("Requires a live camera connection"));
    a_load_cfg_->setToolTip(tr("Requires a live camera connection"));
    m_file->addSeparator();
    a_save_biases_ = m_file->addAction(tr("Save Biases..."), this, &MainWindow::on_save_biases);
    a_load_biases_ = m_file->addAction(tr("Load Biases..."), this, &MainWindow::on_load_biases);
    a_save_biases_->setEnabled(false);
    a_load_biases_->setEnabled(false);
    a_save_biases_->setToolTip(tr("Requires a live camera connection"));
    a_load_biases_->setToolTip(tr("Requires a live camera connection"));
    m_file->addSeparator();
    // Phase 10 — algorithm parameter save/load.
    // Recording and Export moved to the sidebar's File Tools panel (§14.5).
    m_file->addAction(tr("Save Algo Params..."), this, [this]() {
        const QString path = QFileDialog::getSaveFileName(
            this, tr("Save Algorithm Parameters"), {},
            tr("JSON (*.json);;All Files (*)"));
        if (path.isEmpty()) return;
        QString err;
        if (config_.save_algo_params_to_file(&algo_bridge_, path, err)) {
            statusBar()->showMessage(tr("Algorithm params saved to %1").arg(path), 3000);
        } else {
            QMessageBox::warning(this, tr("Save failed"), err);
        }
    });
    m_file->addAction(tr("Load Algo Params..."), this, [this]() {
        const QString path = QFileDialog::getOpenFileName(
            this, tr("Load Algorithm Parameters"), {},
            tr("JSON (*.json);;All Files (*)"));
        if (path.isEmpty()) return;
        QString err;
        std::map<std::string, std::string> legacy_roi;
        if (config_.load_algo_params_from_file(&algo_bridge_, path, err, &legacy_roi)) {
            auto* ap = settings_->algorithms_panel();
            // Phase 2.6: legacy per-algorithm roi_* entries were collected
            // instead of forwarded — map the first algorithm's ROI onto the
            // unified ROI. Goes through the single state source, so the
            // checkboxes/overlay/zoom/algorithm path all sync via
            // roi_state_changed (debug D-6).
            if (!legacy_roi.empty()) {
                auto parse = [](const std::map<std::string, std::string>& m,
                                const char* k, int def) {
                    auto it = m.find(k);
                    if (it == m.end()) return def;
                    try { return std::stoi(it->second); } catch (...) { return def; }
                };
                const bool on = parse(legacy_roi, "roi_enabled", 0) != 0;
                const int rx = parse(legacy_roi, "roi_x", -1);
                const int ry = parse(legacy_roi, "roi_y", -1);
                const int rw = parse(legacy_roi, "roi_w", 128);
                const int rh = parse(legacy_roi, "roi_h", 128);
                camera_.set_unified_roi(on, rx, ry, rw, rh);
            }
            // apply_algo_state wrote instances/caches directly — re-sync the
            // panel controls so the displayed values match the loaded ones
            // (audit §5.9-疑点4), then push the preproc rows so the shared
            // conditioners receive the restored values too (config load
            // drives the bridge route only; the conditioner route starts at
            // the panel — without this push it would keep the pre-load state
            // until the next user edit).
            if (ap) {
                ap->refresh_param_values();
                ap->push_all_preproc_params();
            }
            statusBar()->showMessage(tr("Algorithm params loaded from %1").arg(path), 3000);
        } else if (!err.isEmpty()) {
            QMessageBox::warning(this, tr("Load failed"), err);
        }
    });
    m_file->addSeparator();
    m_file->addAction(tr("E&xit"), QKeySequence::Quit, this, &QWidget::close);

    // View — sidebar toggle is no longer here; it lives in the ActivityBar
    // toggle button (§11.2 point 5). Playback panel toggle and layout actions
    // remain.
    auto* m_view = mb->addMenu(tr("&View"));
    auto* pb_toggle = m_view->addAction(tr("Toggle Playback Panel"));
    pb_toggle->setCheckable(true);
    pb_toggle->setChecked(false);
    pb_toggle->setShortcut(QKeySequence("Ctrl+Shift+P"));
    connect(pb_toggle, &QAction::toggled, this, [this](bool on) {
        auto* dock = findChild<QDockWidget*>("PlaybackDock");
        if (dock) dock->setVisible(on);
    });
    m_view->addAction(tr("Reset Layout"), this, &MainWindow::on_reset_layout);
    m_view->addAction(tr("Save Layout..."), this, &MainWindow::on_save_layout);
    m_view->addAction(tr("Load Layout..."), this, &MainWindow::on_load_layout);
    m_view->addSeparator();
    m_view->addAction(tr("&Fullscreen"), QKeySequence("F11"), this,
                      [this]() {
                          if (isFullScreen()) showNormal();
                          else showFullScreen();
                      });

    // Theme — top-level dropdown to the right of View (§11.2 point 6).
    // The controller builds its own submenu with radio actions for each
    // color and mode.
    auto* m_theme = mb->addMenu(tr("&Theme"));
    theme_.build_menu(m_theme);

    // The Camera menu has been removed — ROI Drag Mode lives in the
    // sidebar's ROI panel (Hardware section) per §14.5.
    // Connect/Disconnect/Refresh already live in the DevicesPanel.

    // Preprocess menu and Frame Mode menu have been removed — all
    // preprocessing stage toggles live in the PreprocessingPanel and all
    // frame mode selection lives in the DisplayPanel (both in the sidebar's
    // 算法模块 / 显示与统计 sections). This eliminates duplication with the sidebar.

    // The Algorithm menu has been removed — all algorithm configuration
    // (enable, ROI, parameters, configure) lives in the sidebar's "算法模块"
    // section (AlgorithmsPanel).

    // Tools (design §5.5) — the calibration wizard (folded in from the former
    // Calibration menu, §3.6.3). The former Add/Tile/Cascade/Close display-
    // window actions were removed (§14.1) — they were legacy multi-window
    // management functions irrelevant to the current dock-based GUI.
    // Algorithm-specific windows are opened from the sidebar's Algorithms
    // section, not duplicated here.
    auto* m_tools = mb->addMenu(tr("&Tools"));
    // Calibration (Phase 9) — launches the wizard lazily.
    m_tools->addAction(tr("&Intrinsic Wizard..."), this, &MainWindow::on_intrinsic_wizard);
    // Sharpness meter removed (Phase 5 step 2) — replaced by the Siemens-star
    // Focus Assistant.
    m_tools->addAction(tr("&Focus Assistant..."), this, &MainWindow::on_focus);

    // Help
    auto* m_help = mb->addMenu(tr("&Help"));
    m_help->addAction(tr("&About"), this, &MainWindow::on_about);
    m_help->addAction(tr("About &Qt"), this, &QApplication::aboutQt);
}

void MainWindow::build_status_bar() {
    auto* sb = statusBar();

    // Monospace font for numeric status labels (event rate / timestamp /
    // recording timer) — design §3.9.2.
    QFont mono(QStringLiteral("JetBrains Mono"), 9);
    mono.setFamilies({QStringLiteral("JetBrains Mono"),
                      QStringLiteral("Consolas"),
                      QStringLiteral("Menlo"),
                      QStringLiteral("Monospace")});

    // Builds a [icon][text] composite status item. @p icon_name selects an
    // SVG from the icon resource; @p text_out receives the text QLabel (so
    // existing setText() call sites keep working); @p icon_out, when non-null,
    // receives the icon QLabel (for dots whose color must change at runtime).
    const auto make_item = [this](const QString& icon_name,
                                  QLabel** text_out,
                                  QLabel** icon_out = nullptr) -> QWidget* {
        auto* w = new QWidget(this);
        auto* l = new QHBoxLayout(w);
        l->setContentsMargins(0, 0, 0, 0);
        l->setSpacing(4);
        auto* icon = new QLabel(w);
        icon->setPixmap(IconProvider::get(icon_name).pixmap(QSize(12, 12)));
        icon->setFixedSize(14, 14);
        icon->setAlignment(Qt::AlignCenter);
        auto* text = new QLabel(w);
        l->addWidget(icon);
        l->addWidget(text);
        *text_out = text;
        if (icon_out) *icon_out = icon;
        return w;
    };

    // Connection: colored dot (green connected / gray disconnected).
    sb->addWidget(make_item(QStringLiteral("circle"), &status_conn_, &status_conn_dot_));
    status_conn_->setText(tr("Disconnected"));
    set_conn_connected(false);

    // Event rate: chart icon + monospace numeric.
    sb->addWidget(make_item(QStringLiteral("chart"), &status_rate_, &status_rate_icon_));
    status_rate_->setText(tr("— ev/s"));
    status_rate_->setFont(mono);

    // Timestamp: clock icon + monospace numeric.
    sb->addWidget(make_item(QStringLiteral("clock"), &status_ts_, &status_ts_icon_));
    status_ts_->setText(tr("t: —"));
    status_ts_->setFont(mono);

    // Recording: red dot (blinks while recording) + monospace timer.
    sb->addPermanentWidget(make_item(QStringLiteral("circle"), &status_rec_, &status_rec_dot_));
    status_rec_->setText(tr("Idle"));
    status_rec_->setFont(mono);
    // Rec dot is red and hidden until recording starts.
    status_rec_dot_->setPixmap(
        IconProvider::get(QStringLiteral("circle"), QColor(229, 57, 53)).pixmap(QSize(12, 12)));
    status_rec_dot_->setVisible(false);

    // Recording blink timer (design §3.8.2: 500ms interval).
    rec_blink_timer_ = new QTimer(this);
    rec_blink_timer_->setInterval(500);
    connect(rec_blink_timer_, &QTimer::timeout, this, &MainWindow::on_rec_blink);

    // Performance metrics flush timer (10 Hz). Queries PerformanceMeter and
    // AlgoBridge, updates the InformationPanel with live latency, throughput,
    // and algorithm overload status. All metrics are derived from the
    // Metavision SDK's own RateEstimator and actual wall-clock measurements
    // — not a jAER port. Latency = SDK CD callback arrival → frame_ready
    // signal; throughput = SDK rate × sizeof(EventCD); algo status = actual
    // flood-guard state from AlgoInstance::is_overloaded().
    perf_timer_ = new QTimer(this);
    perf_timer_->setInterval(100);
    connect(perf_timer_, &QTimer::timeout, this, [this]() {
        if (!camera_.is_connected()) return;
        const double latency_ms = perf_meter_.latency_us() * 1.0e-3;
        // Throughput: SDK rate (events/s) × event struct size (bytes) → MB/s.
        constexpr double event_size = sizeof(Metavision::EventCD);
        const double throughput_mbs = last_rate_eps_ * event_size / 1.0e6;
        settings_->information_panel()->set_performance(latency_ms, throughput_mbs);
        // Algorithm overload status from AlgoBridge. Also compute the max
        // drop rate across all live instances (total_dropped / total_pushed)
        // so the user can see if any algorithm is falling behind the event
        // rate and silently discarding events via the flood guard.
        int active = 0, overloaded = 0;
        double max_drop_pct = 0.0;
        for (const auto& inst : algo_bridge_.list_live()) {
            if (inst->is_overloaded())
                ++overloaded;
            else if (inst->is_enabled())
                ++active;
            const std::size_t pushed = inst->total_pushed();
            if (pushed > 0) {
                const double pct = 100.0 * static_cast<double>(inst->total_dropped())
                                   / static_cast<double>(pushed);
                if (pct > max_drop_pct) max_drop_pct = pct;
            }
        }
        settings_->information_panel()->set_algo_status(active, overloaded);
        settings_->information_panel()->set_drop_rate(max_drop_pct);
    });
    perf_timer_->start();
}

void MainWindow::set_conn_connected(bool connected) {
    // Green dot when connected, gray when disconnected (design §3.8.2).
    const QColor dot_color = connected ? QColor(67, 160, 71)    // green
                                       : QColor(158, 158, 158); // gray
    if (status_conn_dot_) {
        status_conn_dot_->setPixmap(
            IconProvider::get(QStringLiteral("circle"), dot_color).pixmap(QSize(12, 12)));
    }
    if (status_conn_) {
        status_conn_->setProperty("class", connected ? "status-conn" : "status-disc");
        status_conn_->style()->unpolish(status_conn_);
        status_conn_->style()->polish(status_conn_);
    }
}

void MainWindow::start_rec_blink() {
    rec_blink_on_ = true;
    if (status_rec_dot_) {
        status_rec_dot_->setVisible(true);
    }
    if (rec_blink_timer_) rec_blink_timer_->start();
}

void MainWindow::stop_rec_blink() {
    if (rec_blink_timer_) rec_blink_timer_->stop();
    rec_blink_on_ = false;
    if (status_rec_dot_) status_rec_dot_->setVisible(false);
}

void MainWindow::on_rec_blink() {
    if (status_rec_dot_) {
        status_rec_dot_->setVisible(!status_rec_dot_->isVisible());
    }
}

void MainWindow::update_toggle_icon() {
    if (!settings_dock_ || !settings_) return;
    const auto area = dockWidgetArea(settings_dock_);
    const bool content_visible = settings_->is_content_visible();
    // Chevron direction (§11.2 point 5):
    //   Left dock + visible  → chevron-left
    //   Left dock + hidden   → chevron-right
    //   Right dock + visible → chevron-right
    //   Right dock + hidden  → chevron-left
    QString icon_name;
    if (area == Qt::LeftDockWidgetArea) {
        icon_name = content_visible ? QStringLiteral("chevron-left")
                                    : QStringLiteral("chevron-right");
    } else {
        icon_name = content_visible ? QStringLiteral("chevron-right")
                                    : QStringLiteral("chevron-left");
    }
    // Access the ActivityBar via SettingsPanel to set the toggle icon.
    // We use findChild to avoid exposing ActivityBar as a public accessor.
    if (auto* bar = settings_->findChild<ActivityBar*>()) {
        bar->set_toggle_icon(icon_name);
    }
}

void MainWindow::on_sidebar_content_toggled(bool visible) {
    if (!settings_dock_ || !settings_) return;
    if (visible) {
        // Restore the minimum width and the saved dock width (or a sensible
        // default if none saved).
        settings_dock_->setMinimumWidth(200);
        settings_dock_->setMaximumWidth(QWIDGETSIZE_MAX); // remove collapsed cap
        const int target = saved_sidebar_width_ > 0 ? saved_sidebar_width_ : 380;
        const QList<QDockWidget*> docks = {settings_dock_};
        const QList<int> sizes = {target};
        resizeDocks(docks, sizes, Qt::Horizontal);
    } else {
        // Save the current dock width before shrinking to ActivityBar width.
        saved_sidebar_width_ = settings_dock_->width();
        // Lower the minimum width and cap the maximum so the dock cannot be
        // dragged wider while collapsed (§15.5).
        settings_dock_->setMinimumWidth(48);
        settings_dock_->setMaximumWidth(48);
        const QList<QDockWidget*> docks = {settings_dock_};
        const QList<int> sizes = {48};
        resizeDocks(docks, sizes, Qt::Horizontal);
    }
    update_toggle_icon();
}

void MainWindow::wire_signals() {
    // Sidebar toggle is now handled by the ActivityBar toggle button →
    // SettingsPanel::content_toggled → on_sidebar_content_toggled (connected
    // in the constructor). No menu action or toolbar button involved.

    // Devices panel <-> controller
    auto* dp = settings_->devices_panel();
    connect(dp, &DevicesPanel::refresh_requested, this, &MainWindow::on_refresh_devices);
    connect(dp, &DevicesPanel::connect_first_requested, this, &MainWindow::on_connect_first);
    connect(dp, &DevicesPanel::connect_serial_requested, this,
            [this](const QString& serial) {
                // Stop the recorder before switching sources so it can close
                // the output file cleanly while the old camera is still alive.
                // connect_serial() calls teardown() which destroys the old
                // camera — after that, recorder_.stop() can no longer call
                // stop_log_raw_data() and the file is left without a footer.
                if (recorder_.is_recording()) recorder_.stop();
                camera_.connect_serial(serial.toStdString());
            });
    connect(dp, &DevicesPanel::disconnect_requested, this, &MainWindow::on_disconnect);
    // Phase 2: IMU stream toggle (Devices panel) — enables the inivation
    // IMU stream and opens the live readout window.
    connect(dp, &DevicesPanel::imu_stream_toggled, this, &MainWindow::on_imu_toggled);
    connect(dp, &DevicesPanel::aps_stream_toggled, this, &MainWindow::on_aps_toggled);
    // Sensor self-test — opens a Standalone AlgoWindow with the refractory-
    // period heatmap. On close, a report dialog is shown (design §4.4.8).
    connect(dp, &DevicesPanel::self_test_requested, this, [this]() {
        if (!camera_.is_connected()) {
            statusBar()->showMessage(tr("Connect a camera first."), 3000);
            return;
        }
        on_open_algo_window("sensor_self_test");
    });

    // Phase 2 (§3.3.2): bind every registered panel to the camera so each
    // panel auto-receives connected/disconnected events via its
    // on_camera_connected / on_camera_disconnected slots. This replaces the
    // per-panel on_camera_connected calls that were previously scattered in
    // the connected/disconnected lambdas below.
    for (const auto& panel : settings_->panels()) {
        panel->bind_camera(&camera_);
    }

    // Controller -> UI
    connect(&camera_, &CameraController::connected, this, [this](const SensorInfo& info) {
        settings_->information_panel()->set_info(info);
        settings_->devices_panel()->set_connected(true);
        // Propagate actual sensor dimensions to the algorithm bridge so
        // algorithm backends are created with the correct width/height.
        algo_bridge_.set_sensor_dimensions(info.width, info.height);
        status_conn_->setText(tr("Connected: %1").arg(info.generation_name.isEmpty()
                                                           ? info.integrator
                                                           : info.generation_name));
        set_conn_connected(true);
        // Enable config / bias file actions on live cameras only.
        const bool live = !info.is_file;
        a_save_cfg_->setEnabled(live);
        a_load_cfg_->setEnabled(live);
        a_save_biases_->setEnabled(live);
        a_load_biases_->setEnabled(live);
        // Recording + Export moved to the sidebar File Tools panel (§14.5).
        settings_->file_tools_panel()->set_record_enabled(live);
        settings_->file_tools_panel()->set_stop_enabled(false);
        settings_->file_tools_panel()->set_export_enabled(true);
        // Phase 3: show/hide playback bar.
        if (auto* pb = findChild<QDockWidget*>("PlaybackDock")) {
            pb->setVisible(info.is_file);
        }
        // Keep the XYT 3D display's sensor geometry in sync with the live
        // camera so point coordinates are normalized correctly. Without this
        // (e.g. when the XYT window was opened before a camera connected, or
        // after a reconnect) points render at raw pixel scale and the cloud
        // is clipped/garbled.
        if (xyt_display_) {
            xyt_display_->set_sensor_geometry(info.width, info.height);
        }
        // Reset ALL algorithm instances to clear temporal state from a
        // previous session (previous file or live camera). Without this,
        // stateful algorithms (EventToVideo's log_intensity_/current_t_/
        // last_frame_t_, InteractingMaps' I_map_, etc.) carry over stale
        // values, causing:
        //   - Wrong decay calculations (dt_us based on old current_t_)
        //   - Stale reconstructions / gray screens
        //   - NaN propagation from diverged state
        // This also covers the case where a new raw file starts at t=0 but
        // current_t_ is still at the previous file's end time → no new
        // events would update current_t_ (e.t > current_t_ is false) → the
        // algorithm freezes on stale output.
        //
        // Also update sensor dimensions on existing instances: when a new
        // file/camera connects with different dimensions, the ROI must be
        // recomputed at the new sensor size. Without this, the ROI stays at
        // the old sensor's center, filtering out most events and producing
        // dark/black output from frame-producing algorithms.
        for (auto& inst : algo_bridge_.list_live()) {
            inst->set_sensor_dimensions(info.width, info.height);
            inst->reset();
        }
        // Clear the XYT 3D display buffer so stale events from the previous
        // session don't linger in the point cloud. Combined with the
        // time_window-based Z normalization (xyt_visualizer.h render()),
        // the new session starts with an empty cloud that fills naturally.
        if (xyt_display_) {
            xyt_display_->clear();
        }
        // Re-install the conditioned-stream listener for the new source. The
        // old SDK callback lifecycle concerns are gone: the listener is a
        // plain std::function on the (long-lived) CameraController, not a
        // per-camera callback ID — re-registering unconditionally is safe.
        install_algo_callback();
        camera_.start();
        // Sync UI to FramePipeline's current (persisted) values so the
        // DisplayPanel and PlaybackControls reflect the actual fps /
        // accumulation / fps_limit after a reconnect or file reopen.
        if (auto* fp = camera_.frame_pipeline()) {
            settings_->display_panel()->set_accumulation_time_us(
                fp->accumulation_time_us());
            settings_->display_panel()->set_fps(fp->fps());
            settings_->display_panel()->set_fps_limit(fp->fps_limit());
            playback_controls_->on_time_window_changed(fp->accumulation_time_us());
            playback_controls_->on_frame_rate_changed(fp->fps());
            playback_controls_->on_fps_limit_changed(fp->fps_limit());
        }
        // Explicitly refresh facility-dependent panels AFTER camera_.start().
        // The AbstractPanel::bind_camera signal fires on_camera_connected
        // BEFORE start() completes, so HAL facilities (biases, ROI, trigger,
        // ESP) may not be fully available at that point. Calling again here
        // ensures panels see the live camera's facilities — matching the
        // pattern used by on_load_config. Safe to call
        // twice: each panel's populate() clears and rebuilds.
        settings_->biases_panel()->on_camera_connected(&camera_);
        settings_->roi_panel()->on_camera_connected(&camera_);
        settings_->esp_panel()->on_camera_connected(&camera_);
        settings_->trigger_panel()->on_camera_connected(&camera_);
        // Phase 1 capability framework: hide facility-backed panels the
        // connected source cannot back (inivation devices / file sources
        // have no trigger or ESP facilities).
        const auto caps = camera_.source_capabilities();
        settings_->apply_source_capabilities(caps.trigger, caps.esp);
        settings_->devices_panel()->set_imu_available(caps.imu);
        set_imu_ui_state(camera_.imu_enabled());
        settings_->devices_panel()->set_aps_available(caps.aps);
        set_aps_ui_state(camera_.aps_enabled());
    });
    // AEDAT4 replay: side streams appear by content — when the reader
    // decodes the first IMU/APS packet, check the box and open the window.
    connect(&camera_, &CameraController::file_side_stream_discovered, this,
            [this](bool imu) {
                if (imu) {
                    settings_->devices_panel()->set_imu_available(true);
                    on_imu_toggled(true);
                } else {
                    settings_->devices_panel()->set_aps_available(true);
                    on_aps_toggled(true);
                }
            });
    connect(&camera_, &CameraController::disconnected, this, [this]() {
        // Explicitly remove the CD callback before clearing the ID, so the
        // SDK data thread stops calling our lambda before any MainWindow
        // members are destroyed.
        remove_algo_callback();
        prev_frame_ts_ = 0;
        // XYT post throttle holds the last POSTED stream timestamp; the next
        // session's stream restarts near t=0 (TS reset), so a stale value
        // would gate every post (cur_ts - last < 0) for the whole previous
        // session's duration. Same session-scoped timestamp state class as
        // prev_frame_ts_ above.
        algo_last_xyt_post_us_.store(0);
        prev_frame_wall_ = {};
        perf_meter_.reset();
        last_rate_eps_ = 0.0;
        settings_->information_panel()->clear();
        settings_->statistics_panel()->clear();
        settings_->devices_panel()->set_connected(false);
        status_conn_->setText(tr("Disconnected"));
        set_conn_connected(false);
        status_rate_->setText(tr("— ev/s"));
        status_ts_->setText(tr("t: —"));
        a_save_cfg_->setEnabled(false);
        a_load_cfg_->setEnabled(false);
        a_save_biases_->setEnabled(false);
        a_load_biases_->setEnabled(false);
        // Recording + Export moved to the sidebar File Tools panel (§14.5).
        settings_->file_tools_panel()->set_record_enabled(false);
        settings_->file_tools_panel()->set_stop_enabled(false);
        settings_->file_tools_panel()->set_export_enabled(false);
        // Restore the pre-Phase-1 layout: with no source connected the
        // facility panels are visible again (they populate as "not
        // supported", exactly as before).
        settings_->apply_source_capabilities(true, true);
        // Phase 2: the IMU stream is session-scoped — a disconnect stops it
        // (teardown() also resets the controller flag) and closes the
        // readout window.
        if (imu_window_) imu_window_->close();
        settings_->devices_panel()->set_imu_available(false);
        if (aps_window_) aps_window_->close();
        settings_->devices_panel()->set_aps_available(false);
        roi_draw_pending_ = false;
        on_toggle_roi_drag(false);
        display_->clear();
        if (auto* pb = findChild<QDockWidget*>("PlaybackDock")) {
            pb->setVisible(false);
        }
        playback_controls_->activate(false);
    });
    connect(&camera_, &CameraController::started, this, [this]() {
        status_rec_->setText(tr("Streaming"));
        stop_rec_blink();
    });
    connect(&camera_, &CameraController::stopped, this, [this]() {
        // File source: the SDK camera stopping after buffering all events is
        // NOT playback stopping — the FileFrameGenerator keeps playing from
        // its buffer (same exemption as PlaybackController, audit §六-P3).
        if (camera_.is_file_source()) return;
        // Auto-stop the recorder when the camera stops (user stop, file EOF,
        // runtime error). Without this, the recorder keeps running with a
        // dead camera — the flush timer ticks but get_latest_raw_data()
        // returns nothing, and the output file is left without a clean
        // footer because stop_log_raw_data() is never called.
        if (recorder_.is_recording()) {
            recorder_.stop();
        } else {
            status_rec_->setText(tr("Idle"));
            stop_rec_blink();
        }
    });
    connect(&camera_, &CameraController::error, this, [this](const QString& msg) {
        // Auto-stop the recorder on camera error for the same reason as
        // stopped(). The error handler runs before stopped() (both are
        // queued from the error callback), so stop here to ensure the
        // file is closed while the camera handle is still valid.
        if (recorder_.is_recording()) recorder_.stop();
        QMessageBox::warning(this, tr("Camera error"), msg);
    });
    connect(&camera_, &CameraController::runtime_warning, this, [this](const QString& msg) {
        status_rec_->setText(tr("Stopped"));
        stop_rec_blink();
        statusBar()->showMessage(msg, 5000);
    });

    // Frame pipeline -> display
    connect(camera_.frame_pipeline(), &FramePipeline::frame_ready, this,
            [this](QImage frame, Metavision::timestamp ts) {
                process_algo_results(frame);
                // Phase 2.6 step 4 mode (a): adaptive zoom — crop the
                // (already overlay-annotated) frame to the unified ROI rect;
                // EventDisplayWidget scales it to the window. Mode (b) is
                // the pass-through default.
                if (roi_zoom_cb_->isChecked()) {
                    bool zen = false;
                    int zx0 = 0, zy0 = 0, zx1 = 0, zy1 = 0;
                    camera_.unified_roi(zen, zx0, zy0, zx1, zy1);
                    if (zen && zx1 > zx0 && zy1 > zy0) {
                        QRect zr(zx0, zy0, zx1 - zx0, zy1 - zy0);
                        zr &= frame.rect();
                        if (!zr.isEmpty() && zr != frame.rect()) {
                            frame = frame.copy(zr);
                        }
                    }
                }
                display_->set_frame(frame);
                settings_->statistics_panel()->set_timestamp(ts);
                status_ts_->setText(QStringLiteral("t: %1 s").arg(ts / 1.0e6, 0, 'f', 3));
                if (camera_.is_file_source()) {
                    // File mode: ts is the FILE's timestamp, so ts-delta FPS
                    // is distorted by the playback rate (slow motion shows
                    // absurd values like 10000 fps). Use the wall-clock
                    // interval between displayed frames instead (audit §六-P6).
                    const auto now = std::chrono::steady_clock::now();
                    if (prev_frame_wall_.time_since_epoch().count() > 0) {
                        const double dt =
                            std::chrono::duration<double>(now - prev_frame_wall_).count();
                        if (dt > 0.0) {
                            settings_->statistics_panel()->set_fps(1.0 / dt);
                        }
                    }
                    prev_frame_wall_ = now;
                } else if (prev_frame_ts_ > 0 && ts > prev_frame_ts_) {
                    const double fps = 1.0e6 / static_cast<double>(ts - prev_frame_ts_);
                    settings_->statistics_panel()->set_fps(fps);
                }
                prev_frame_ts_ = ts;
                perf_meter_.tick_frame();
            });

    // File playback: feed the events in each accumulation window to algorithm
    // instances + XYT display, synchronously with the displayed frame. This
    // signal is emitted just before frame_ready (see FileFrameGenerator::render_frame),
    // so by the time the frame_ready handler runs process_algo_results(), the
    // algorithms have already processed this window's events and their results
    // are ready to be pulled. For live cameras this signal is never emitted
    // (CDFrameGenerator path) and the SDK CD callback feeds algorithms instead.
    connect(camera_.frame_pipeline(), &FramePipeline::events_window_ready,
            this, &MainWindow::on_events_window_ready);

    // P6-A file-mode write-back: a stream-filter algorithm's per-window
    // output span repaints the playback frame (hot_pixel_filter removes
    // events, EIS compensates coordinates) — the file-mode counterpart of
    // the live conditioned-listener substitution.
    if (auto* fp = camera_.frame_pipeline()) {
        fp->set_file_algo_feed_callback([this]() {
            auto instances = algo_bridge_.list_live();
            for (auto& inst : instances) {
                if (!inst->is_enabled()) continue;
                auto [fb, fn] = inst->output_span();
                if (fb != nullptr && fn > 0) return std::make_pair(fb, fn);
                break;  // single algorithm slot
            }
            return std::pair<const Metavision::EventCD*, std::size_t>{nullptr, 0};
        });
    }

    // File loop: reset all algorithm instances when the cursor wraps to 0.
    // Stateful algorithms (time_surface current_t_, E2VID log_intensity_,
    // InteractingMaps I_map_, etc.) only increase their internal timestamps,
    // so after a loop the new events (t≈0) are less than current_t_ and get
    // ignored — the output freezes. Resetting clears this stale state so the
    // second pass renders correctly. The XYT point cloud is also cleared to
    // avoid mixing old and new timestamps on the Z axis.
    connect(camera_.frame_pipeline(), &FramePipeline::file_looped,
            this, [this]() {
                for (auto& inst : algo_bridge_.list_live()) {
                    inst->reset();
                }
                if (xyt_display_) {
                    xyt_display_->clear();
                }
            });

    // File seek: reset all algorithm instances when the cursor jumps to a
    // new position. Backward seeks trigger the same stale-state freeze as
    // looped() — algorithms with monotonically-increasing internal
    // timestamps (time_surface current_t_, E2VID log_intensity_, etc.)
    // would ignore new events whose timestamps are below current_t_. Even
    // forward seeks can leave algorithms with buffered state that doesn't
    // match the new cursor position, so we reset unconditionally. The XYT
    // point cloud is cleared to avoid mixing pre-seek and post-seek events
    // on the Z axis. The seeked signal is emitted BEFORE render_frame()
    // (see FileFrameGenerator::seek), so the reset takes effect before
    // the new window's events are pushed via events_window_ready.
    connect(camera_.frame_pipeline(), &FramePipeline::file_seeked,
            this, [this](Metavision::timestamp) {
                for (auto& inst : algo_bridge_.list_live()) {
                    inst->reset();
                }
                if (xyt_display_) {
                    xyt_display_->clear();
                }
            });

    // Statistics controller -> panel + status bar
    connect(camera_.statistics(), &StatisticsController::rate_updated, this,
            [this](double rate, double peak, Metavision::timestamp t) {
                settings_->statistics_panel()->set_rate(rate, peak, t);
                last_rate_eps_ = rate;
                if (rate >= 1.0e6) {
                    status_rate_->setText(QStringLiteral("%1 Mev/s")
                                              .arg(rate / 1.0e6, 0, 'f', 2));
                } else if (rate >= 1.0e3) {
                    status_rate_->setText(QStringLiteral("%1 kev/s")
                                              .arg(rate / 1.0e3, 0, 'f', 2));
                } else {
                    status_rate_->setText(QStringLiteral("%1 ev/s").arg(rate, 0, 'f', 0));
                }
            });
    connect(camera_.statistics(), &StatisticsController::on_off_updated,
            settings_->statistics_panel(), &StatisticsPanel::set_on_off);

    // Display panel -> FramePipeline (single source of truth for display params)
    connect(settings_->display_panel(), &DisplayPanel::color_palette_changed, this,
            &MainWindow::update_palettes);
    connect(settings_->display_panel(), &DisplayPanel::accumulation_time_changed_us,
            this, [this](int us) {
                camera_.frame_pipeline()->set_accumulation_time_us(us);
            });
    connect(settings_->display_panel(), &DisplayPanel::fps_changed, this,
            [this](int fps) {
                camera_.frame_pipeline()->set_fps(static_cast<std::uint16_t>(fps));
            });
    connect(settings_->display_panel(), &DisplayPanel::fps_limit_changed, this,
            [this](int limit) {
                camera_.frame_pipeline()->set_fps_limit(static_cast<std::uint16_t>(limit));
            });
    connect(settings_->display_panel(), &DisplayPanel::frame_mode_changed, this,
            [this](FrameMode mode) {
                camera_.frame_pipeline()->set_frame_mode(mode);
            });
    connect(settings_->display_panel(), &DisplayPanel::decay_time_changed_us, this,
            [this](int us) {
                camera_.frame_pipeline()->set_frame_decay_time_us(us);
            });

    // FramePipeline -> both UIs (DisplayPanel + PlaybackControls).
    // When the pipeline's params change (from either UI or programmatically),
    // sync both panels. The blocked setters prevent feedback loops.
    {
        auto* fp = camera_.frame_pipeline();
        connect(fp, &FramePipeline::accumulation_time_changed, this,
                [this](Metavision::timestamp us) {
                    if (settings_ && settings_->display_panel())
                        settings_->display_panel()->set_accumulation_time_us(static_cast<int>(us));
                    if (playback_controls_)
                        playback_controls_->on_time_window_changed(us);
                });
        connect(fp, &FramePipeline::fps_changed, this,
                [this](std::uint16_t fps) {
                    if (settings_ && settings_->display_panel())
                        settings_->display_panel()->set_fps(static_cast<int>(fps));
                    if (playback_controls_)
                        playback_controls_->on_frame_rate_changed(static_cast<unsigned>(fps));
                });
        connect(fp, &FramePipeline::fps_limit_changed, this,
                [this](std::uint16_t limit) {
                    if (settings_ && settings_->display_panel())
                        settings_->display_panel()->set_fps_limit(static_cast<int>(limit));
                    if (playback_controls_)
                        playback_controls_->on_fps_limit_changed(static_cast<unsigned>(limit));
                });
    }

    // ROI panel <-> display widget / unified state (Phase 2.6 debug D-6)
    auto* roi = settings_->roi_panel();
    // Drag-drawn rect always goes straight to the unified state source. A
    // dialog-initiated draw (Phase 2.6 debug D-6 follow-up) also applies
    // directly — the dialog stays closed after the drag, no confirmation.
    connect(display_, &EventDisplayWidget::roi_dragged, this,
            [this](int x, int y, int w, int h) {
                if (roi_draw_pending_) {
                    roi_draw_pending_ = false;
                    on_toggle_roi_drag(false);
                }
                camera_.set_unified_roi(true, x, y, w, h);
            });
    connect(roi, &RoiPanel::roi_enable_toggled, this,
            &MainWindow::on_roi_enable_toggled);
    connect(roi, &RoiPanel::roi_settings_requested, this,
            &MainWindow::open_roi_settings_dialog);

    // Recording + Export moved from File menu/toolbar to File Tools panel (§14.5).
    auto* ft = settings_->file_tools_panel();
    connect(ft, &FileToolsPanel::record_start_requested, this, &MainWindow::on_record_start);
    connect(ft, &FileToolsPanel::record_stop_requested, this, &MainWindow::on_record_stop);
    connect(ft, &FileToolsPanel::export_requested, this, &MainWindow::on_export_dialog);

    // Phase 3 — recorder.
    connect(&recorder_, &RecorderController::recording_started, this,
            [this](const QString& path) {
                status_rec_->setText(tr("REC: %1").arg(QFileInfo(path).fileName()));
                start_rec_blink();
                settings_->file_tools_panel()->set_record_enabled(false);
                settings_->file_tools_panel()->set_stop_enabled(true);
            });
    connect(&recorder_, &RecorderController::recording_stopped, this,
            [this](const QString& path) {
                status_rec_->setText(tr("Idle"));
                stop_rec_blink();
                settings_->file_tools_panel()->set_record_enabled(
                    camera_.is_connected() && !camera_.is_file_source());
                settings_->file_tools_panel()->set_stop_enabled(false);
                if (recorder_.is_processed_recording() || recorder_.events_written() > 0) {
                    statusBar()->showMessage(
                        tr("Recording saved: %1 (%2 events)")
                            .arg(path)
                            .arg(recorder_.events_written()), 5000);
                } else {
                    statusBar()->showMessage(tr("Recording saved: %1").arg(path), 5000);
                }
            });
    connect(&recorder_, &RecorderController::elapsed, this, &MainWindow::on_record_elapsed);
    connect(&recorder_, &RecorderController::error, this, [this](const QString& msg) {
        QMessageBox::warning(this, tr("Recording"), msg);
    });

    // Phase 3 — playback.
    // Parameter sync (time_window/fps/fps_limit) is handled by the
    // FramePipeline signal connections above — both the DisplayPanel and
    // PlaybackControls listen to the same FramePipeline signals.
    connect(&playback_, &PlaybackController::opened, this,
            [this](Metavision::timestamp) {
                playback_controls_->activate(true);
            });
    connect(&playback_, &PlaybackController::error, this, [this](const QString& msg) {
        statusBar()->showMessage(msg, 4000);
    });

    // Phase 5 — file converter signals are wired directly inside FileToolsPanel.

    // Panel info/error messages -> status bar / message box.
    auto forward = [this](const QString& msg, bool isError) {
        forward_panel_message(msg, isError);
    };
    connect(settings_->biases_panel(),  &BiasesPanel::info_message,  this,
            [forward](const QString& m) { forward(m, false); });
    connect(settings_->biases_panel(),  &BiasesPanel::error_message, this,
            [forward](const QString& m) { forward(m, true); });
    connect(roi, &RoiPanel::info_message,  this,
            [forward](const QString& m) { forward(m, false); });
    connect(roi, &RoiPanel::error_message, this,
            [forward](const QString& m) { forward(m, true); });
    connect(settings_->esp_panel(),     &EspPanel::info_message,  this,
            [forward](const QString& m) { forward(m, false); });
    connect(settings_->esp_panel(),     &EspPanel::error_message, this,
            [forward](const QString& m) { forward(m, true); });
    connect(settings_->trigger_panel(), &TriggerPanel::info_message,  this,
            [forward](const QString& m) { forward(m, false); });
    connect(settings_->trigger_panel(), &TriggerPanel::error_message, this,
            [forward](const QString& m) { forward(m, true); });
    if (auto* pp = settings_->preprocessing_panel()) {
        connect(pp, &PreprocessingPanel::info_message, this,
                [forward](const QString& m) { forward(m, false); });
        connect(pp, &PreprocessingPanel::error_message, this,
                [forward](const QString& m) { forward(m, true); });
    }
    if (auto* ap = settings_->algorithms_panel()) {
        connect(ap, &AlgorithmsPanel::info_message, this,
                [forward](const QString& m) { forward(m, false); });
        // Conditioned-stream params (2026-08-19 rework): every Preprocessing
        // panel change goes to CameraController::set_conditioner_param, which
        // feeds the live conditioner AND the file generator's — the shared
        // conditioning pass every consumer (display + algorithms) consumes.
        connect(ap, &AlgorithmsPanel::preproc_display_param_changed, this,
                [this](const QString& key, const QString& value) {
                        camera_.set_conditioner_param(key.toStdString(),
                                                      value.toStdString());
                });
        // Unified ROI (Phase 2.6 debug D-6): the Algorithms page and the
        // Hardware page each expose an "Enable ROI" checkbox + a "ROI
        // Settings..." button, all driving the single state source. All
        // downstream effects (overlay frame, zoom toggle, algorithm path,
        // checkbox sync) are driven by CameraController::roi_state_changed.
        connect(ap, &AlgorithmsPanel::roi_enable_toggled, this,
                &MainWindow::on_roi_enable_toggled);
        connect(ap, &AlgorithmsPanel::roi_settings_requested, this,
                &MainWindow::open_roi_settings_dialog);
        connect(&camera_, &CameraController::roi_state_changed, this,
                [this, ap](bool en, int x0, int y0, int x1, int y1) {
                    // Sync both pages' checkboxes (QSignalBlocker inside —
                    // no re-emission loops).
                    ap->set_roi_enabled(en);
                    settings_->roi_panel()->set_roi_enabled(en);
                    // The zoom toggle is only meaningful while the ROI is
                    // active; while zoomed the overlay frame is hidden (the
                    // whole view is the ROI).
                    roi_zoom_cb_->setEnabled(en);
                    const bool zoomed = en && roi_zoom_cb_->isChecked();
                    display_->set_roi_overlay(x0, y0, x1 - x0, y1 - y0,
                                              en && !zoomed);
                    // Same window drives the algorithm path: every live
                    // instance is resized to the ROI and fed ROI-relative
                    // events — except in RONI mode, where the source drops
                    // inside-rect events at absolute coordinates and the
                    // instances stay pass-through (debug D-5).
                    algo_bridge_.set_unified_roi_state(en, x0, y0, x1, y1,
                                                       camera_.unified_roi_roni());
                });
        connect(ap, &AlgorithmsPanel::algorithm_toggled, this,
                [this, ap](const QString& name, bool on) {
                    statusBar()->showMessage(
                        tr("%1: %2").arg(name).arg(on ? tr("enabled") : tr("disabled")), 3000);
                    const auto key = name.toStdString();
                    if (on) {
                        // Phase 2.6 debug D-7: heavy algorithms (e2v / ISI /
                        // TimeSurface / HoughLine / HoughCircle)
                        // UNCONDITIONALLY auto-enable the unified ROI at the
                        // default center 256×144 — they stall at full sensor.
                        // The prior ROI state is saved and restored on
                        // disable (algorithm mutex = single slot suffices).
                        if (AlgorithmsPanel::algo_defaults_to_roi(key)) {
                            bool en = false;
                            int x0 = 0, y0 = 0, x1 = 0, y1 = 0;
                            camera_.unified_roi(en, x0, y0, x1, y1);
                            roi_automation_save_ = {en, x0, y0, x1, y1,
                                                    camera_.unified_roi_roni()};
                            camera_.set_unified_roi(true, -1, -1, 256, 144);
                        }
                        // Phase 3 (user decision, reproduction-driven Phase
                        // scope): E2VID forces the shared 1/4 downsample ON
                        // while enabled — same save/restore pattern as the
                        // default ROI.
                        if (key == "event_to_video") {
                            e2v_downsample_save_ = ap->preproc_downsample_enabled();
                            ap->set_preproc_downsample(true);
                        }
                    } else {
                        // Close the AlgoWindow if open (its closing handler will
                        // disable the instance and uncheck the sidebar checkbox —
                        // both are idempotent given the blocker in set_algo_enabled).
                        auto it = algo_windows_.find(key);
                        if (it != algo_windows_.end() && it.value()) it.value()->close();
                        if (key == "xyt_visualizer" && xyt_display_) xyt_display_->close();
                        // Phase 2.6 debug D-7: restore the ROI state saved
                        // when the algorithm was enabled (only if the
                        // automation actually applied one). An empty saved
                        // rect (never configured) restores as the default
                        // 256×144 rect in the disabled state.
                        if (AlgorithmsPanel::algo_defaults_to_roi(key) &&
                            roi_automation_save_.has_value()) {
                            const auto s = *roi_automation_save_;
                            const int w = s.x1 - s.x0;
                            const int h = s.y1 - s.y0;
                            if (w > 0 && h > 0) {
                                camera_.set_unified_roi(s.enabled, s.x0, s.y0,
                                                        w, h, s.roni);
                            } else {
                                camera_.set_unified_roi(s.enabled, -1, -1,
                                                        256, 144, s.roni);
                            }
                            roi_automation_save_.reset();
                        }
                        // Phase 3: restore the downsample state saved when
                        // E2VID was enabled.
                        if (key == "event_to_video" &&
                            e2v_downsample_save_.has_value()) {
                            ap->set_preproc_downsample(*e2v_downsample_save_);
                            e2v_downsample_save_.reset();
                        }
                    }
                });
        // When an algorithm is enabled from the sidebar, open its AlgoWindow
        // so Standalone algorithms have a display and Overlay algorithms get
        // their ROI zoom view. Without this the sidebar-enabled algorithm
        // produces no visible output (the root cause of "侧栏算法不生效").
        connect(ap, &AlgorithmsPanel::open_algo_window_requested, this,
                [this](const std::string& name) {
                    on_open_algo_window(name);
                });
    }
}

void MainWindow::update_palettes(int index) {
    Metavision::ColorPalette p = Metavision::ColorPalette::Dark;
    switch (index) {
        case 1: p = Metavision::ColorPalette::Light; break;
        case 2: p = Metavision::ColorPalette::CoolWarm; break;
        case 3: p = Metavision::ColorPalette::Gray; break;
        default: break;
    }
    camera_.frame_pipeline()->set_color_palette(p);
}

// ---------------------------------------------------------------------------
// Slots
// ---------------------------------------------------------------------------

void MainWindow::on_open_file() {
    const QString path = QFileDialog::getOpenFileName(
        this, tr("Open event file"), QString(),
        tr("Event files (*.raw *.hdf5 *.h5 *.dat *.aedat4 *.alpdata);;All files (*)"));
    if (path.isEmpty()) {
        return;
    }
    on_file_opened_for_playback(path);
}

void MainWindow::on_file_opened_for_playback(const QString& path) {
    // Stop the recorder before switching sources. open_file() calls
    // connect_file() → teardown() which destroys the old camera — after that,
    // recorder_.stop() can no longer call stop_log_raw_data() and the file is
    // left without a clean footer (BUG-S2).
    if (recorder_.is_recording()) recorder_.stop();
    // Route through the playback controller so it can capture duration and
    // start the position probe timer.
    if (!playback_.open_file(path)) {
        // The failure was already reported via CameraController::error /
        // PlaybackController::error (message box / status bar) — showing a
        // second dialog here would duplicate it (audit §六-C3).
        return;
    }
    add_recent_file(path);
}

void MainWindow::build_recent_files_menu() {
    if (!m_recent_files_) return;
    m_recent_files_->clear();
    QSettings s;
    const QStringList recent = s.value("recentFiles").toStringList();
    if (recent.isEmpty()) {
        auto* a = m_recent_files_->addAction(tr("(none)"));
        a->setEnabled(false);
        return;
    }
    for (const QString& path : recent) {
        // Show just the file name in the menu, full path as tooltip.
        const QString name = QFileInfo(path).fileName();
        auto* a = m_recent_files_->addAction(name);
        a->setToolTip(path);
        connect(a, &QAction::triggered, this, [this, path]() {
            on_open_recent_file(path);
        });
    }
}

void MainWindow::add_recent_file(const QString& path) {
    if (path.isEmpty()) return;
    QSettings s;
    QStringList recent = s.value("recentFiles").toStringList();
    // Remove duplicates (case-insensitive on Windows, exact on Linux), then
    // prepend and cap at 10.
    recent.removeAll(path);
    recent.prepend(path);
    while (recent.size() > 10) {
        recent.removeLast();
    }
    s.setValue("recentFiles", recent);
    build_recent_files_menu();
}

void MainWindow::on_open_recent_file(const QString& path) {
    if (!QFileInfo::exists(path)) {
        QMessageBox::warning(this, tr("Open recent"),
                             tr("File no longer exists:\n%1").arg(path));
        // Remove stale entry.
        QSettings s;
        QStringList recent = s.value("recentFiles").toStringList();
        recent.removeAll(path);
        s.setValue("recentFiles", recent);
        build_recent_files_menu();
        return;
    }
    on_file_opened_for_playback(path);
}

void MainWindow::on_connect_first() {
    // Stop the recorder before switching sources. connect_first_available()
    // calls teardown() which destroys the old camera — after that,
    // recorder_.stop() can no longer call stop_log_raw_data() and the file is
    // left without a clean footer (BUG-S2).
    if (recorder_.is_recording()) recorder_.stop();
    // On failure, CameraController emits disconnected() (UI cleanup) then
    // error() (which already shows a QMessageBox::warning in the error
    // handler). No additional dialog needed here — that was a duplicate.
    camera_.connect_first_available();
}

void MainWindow::on_disconnect() {
    // Stop the recorder first so it closes the output file cleanly before the
    // camera's data thread disappears (BUG-G3: without this, the recorder's
    // CD callback may fire on a torn-down pipeline and crash).
    recorder_.stop();
    camera_.disconnect();
}

void MainWindow::on_save_config() {
    const QString path = QFileDialog::getSaveFileName(
        this, tr("Save camera config"), QString(),
        tr("Camera config (*.json);;All files (*)"));
    if (path.isEmpty()) return;
    QString err;
    if (!config_.save_to_file(&camera_, path, err)) {
        QMessageBox::warning(this, tr("Save config"),
                             err.isEmpty() ? tr("Failed to save camera config to:\n%1").arg(path)
                                           : err);
    } else {
        statusBar()->showMessage(tr("Config saved: %1").arg(path), 4000);
    }
}

void MainWindow::on_load_config() {
    const QString path = QFileDialog::getOpenFileName(
        this, tr("Load camera config"), QString(),
        tr("Camera config (*.json);;All files (*)"));
    if (path.isEmpty()) return;
    QString err;
    if (!config_.load_from_file(&camera_, path, err)) {
        QMessageBox::warning(this, tr("Load config"),
                             err.isEmpty() ? tr("Failed to load camera config from:\n%1").arg(path)
                                           : err);
        return;
    }
    // Refresh the panels so they reflect the newly-applied state.
    settings_->biases_panel()->on_camera_connected(&camera_);
    settings_->roi_panel()->on_camera_connected(&camera_);
    settings_->esp_panel()->on_camera_connected(&camera_);
    settings_->trigger_panel()->on_camera_connected(&camera_);
    statusBar()->showMessage(tr("Config loaded: %1").arg(path), 4000);
}

void MainWindow::on_save_biases() {
    const QString path = QFileDialog::getSaveFileName(
        this, tr("Save biases"), QString(),
        tr("Bias files (*.bias);;All files (*)"));
    if (path.isEmpty()) return;
    settings_->biases_panel()->save_to_file(path);
}

void MainWindow::on_load_biases() {
    const QString path = QFileDialog::getOpenFileName(
        this, tr("Load biases"), QString(),
        tr("Bias files (*.bias);;All files (*)"));
    if (path.isEmpty()) return;
    settings_->biases_panel()->load_from_file(path);
}

void MainWindow::on_toggle_roi_drag(bool on) {
    display_->set_roi_drag_mode(on);
    statusBar()->showMessage(on ? tr("ROI drag mode on — draw a rectangle on the display.")
                                : tr("ROI drag mode off."), 3000);
}

void MainWindow::on_roi_enable_toggled(bool on) {
    // Phase 2.6 debug D-6: shared handler for both pages' "Enable ROI"
    // checkboxes. Applies the CURRENT stored rect (kept by
    // CameraController/FileFrameGenerator while disabled); falls back to the
    // default center 256×144 when nothing was ever configured.
    bool en = false;
    int x0 = 0, y0 = 0, x1 = 0, y1 = 0;
    camera_.unified_roi(en, x0, y0, x1, y1);
    int x = x0, y = y0, w = x1 - x0, h = y1 - y0;
    if (w <= 0 || h <= 0) {
        x = -1;
        y = -1;
        w = 256;
        h = 144;
    }
    camera_.set_unified_roi(on, x, y, w, h);
    // Turning ROI on opens the settings dialog so the rect can be adjusted
    // right away (user decision).
    if (on) {
        open_roi_settings_dialog();
    }
}

void MainWindow::open_roi_settings_dialog() {
    bool en = false;
    int x0 = 0, y0 = 0, x1 = 0, y1 = 0;
    camera_.unified_roi(en, x0, y0, x1, y1);
    const auto& si = camera_.sensor_info();
    UnifiedRoiDialog dlg(this);
    // The enable state is owned by the sidebar checkboxes — the dialog only
    // edits rect/mode (user decision); OK keeps the current enable state.
    dlg.set_state(x0, y0, x1, y1, camera_.unified_roi_roni(),
                  si.width > 0 ? si.width : 1280,
                  si.height > 0 ? si.height : 720);
    const int rc = dlg.exec();
    if (rc == UnifiedRoiDialog::kDrawRequest) {
        // The modal exec has ENDED (a hidden-but-modal dialog still blocks
        // mouse input to the main display — the first version of this flow
        // made dragging impossible). The main display is interactive now:
        // enable drag mode and wait for roi_dragged, which applies the drawn
        // rect directly.
        roi_draw_pending_ = true;
        on_toggle_roi_drag(true);
        return;
    }
    if (rc == QDialog::Accepted) {
        camera_.set_unified_roi(en, dlg.x(), dlg.y(),
                                dlg.w(), dlg.h(), dlg.roni());
    }
}

void MainWindow::on_record_start() {
    if (recorder_.is_recording()) return;
    if (!record_dialog_) {
        record_dialog_ = new RecordDialog(this);
        record_dialog_->setAttribute(Qt::WA_DeleteOnClose);
        connect(record_dialog_, &QObject::destroyed, this, [this]() {
            record_dialog_ = nullptr;
        });
        connect(record_dialog_, &RecordDialog::start_recording, this,
                [this](const QString& path, bool save_biases, bool include_imu,
                       bool include_aps) {
                    do_record_start(path, save_biases, include_imu, include_aps);
                });
    }
    record_dialog_->set_aedat4_mode(camera_.is_inivation_source());
    const auto caps = camera_.source_capabilities();
    record_dialog_->set_side_stream_capabilities(caps.imu, caps.aps);
    record_dialog_->show();
    record_dialog_->raise();
    record_dialog_->activateWindow();
}

void MainWindow::do_record_start(const QString& path, bool save_biases,
                                 bool include_imu, bool include_aps) {
    // Save the current bias configuration alongside the RAW recording so
    // the file is reproducible — the event stream depends on the bias
    // settings at record time (matching Metavision Viewer behavior).
    // This is best-effort: cameras without a bias facility are silently
    // skipped.
    if (save_biases) {
        auto* biases = camera_.biases_facility();
        if (biases) {
            QFileInfo fi(path);
            QString bias_path = fi.absolutePath() + "/" + fi.completeBaseName() + ".bias";
            try {
                biases->save_to_file(std::filesystem::path(bias_path.toStdString()));
                statusBar()->showMessage(
                    tr("Biases saved to %1").arg(bias_path), 5000);
            } catch (const std::exception& e) {
                statusBar()->showMessage(
                    tr("Warning: could not save biases: %1").arg(QString::fromUtf8(e.what())), 5000);
            }
        }
    }
    // Processed-stream recording (Phase 2.5 step 5): when any conditioning
    // stage is active, record the PROCESSED event stream (what the display
    // sees); otherwise keep the SDK raw log.
    recorder_.set_include_side_streams(include_imu, include_aps);
    auto* fp = camera_.frame_pipeline();
    if (fp && camera_.conditioner_active()) {
        recorder_.start_processed(&camera_, path, fp);
    } else {
        recorder_.start(&camera_, path);
    }
}

void MainWindow::on_record_stop() {
    recorder_.stop();
}

void MainWindow::on_record_elapsed(std::chrono::seconds s) {
    const auto hrs = std::chrono::duration_cast<std::chrono::hours>(s).count();
    const auto mins = std::chrono::duration_cast<std::chrono::minutes>(s).count() % 60;
    const auto secs = s.count() % 60;
    const QString base = tr("REC");
    QString text = QStringLiteral("%1 %2:%3:%4")
                       .arg(base)
                       .arg(hrs, 2, 10, QLatin1Char('0'))
                       .arg(mins, 2, 10, QLatin1Char('0'))
                       .arg(secs, 2, 10, QLatin1Char('0'));
    // Processed-mode telemetry: the live written-event count makes an
    // empty/failed recording immediately visible (user report).
    if (recorder_.is_processed_recording()) {
        text += QStringLiteral(" · %1 ev").arg(recorder_.events_written());
    }
    status_rec_->setText(text);
}

void MainWindow::on_export_dialog() {
    if (export_dialog_) {
        // Pre-fill the source path with the file currently open for playback
        // (if any) so the user doesn't have to re-browse to it.
        export_dialog_->set_source(playback_.current_file());
        export_dialog_->show();
        export_dialog_->raise();
        export_dialog_->activateWindow();
    }
}

// ---------------------------------------------------------------------------
// Algorithm event/result pipeline (design §3.8, §5.6.1)
// ---------------------------------------------------------------------------

void MainWindow::install_algo_callback() {
    // Algorithms (and the XYT feed) consume the SHARED conditioned stream:
    // CameraController conditions each live batch ONCE (unified ROI →
    // polarity stages → noise filter → 1/4 thin → undistort → flips) and
    // hands the same span to the display pipeline and to this listener.
    // This used to be a second SDK callback that re-ran the FilterChain and
    // left the noise filter/downsample/undistort to each backend's own
    // Preprocessor — duplicate computation per consumer.
    // File sources never invoke the listener: file playback feeds algorithms
    // via the FramePipeline::events_window_ready signal (see
    // on_events_window_ready), synchronized with each displayed frame.
    camera_.set_conditioned_listener(
        [this](const Metavision::EventCD* rb, const Metavision::EventCD* re,
               const Metavision::EventCD* cb, const Metavision::EventCD* ce,
               const Metavision::EventCD*& ob, const Metavision::EventCD*& oe) {
            // Conditioned-empty (or empty raw) batch: still tick the profiler
            // with the RAW count so the event rate reflects camera output.
            if (rb == nullptr || re == nullptr || rb >= re) return;
            if (cb == nullptr || ce == nullptr || cb >= ce) {
                perf_meter_.tick_events(static_cast<std::size_t>(re - rb),
                                         (re - 1)->t);
                return;
            }
            {
                try {
                    // Push the conditioned events to every live algorithm
                    // instance. AlgoInstance is internally mutex-guarded, so
                    // this is safe from the SDK data thread. The
                    // reinterpret_cast inside AlgoBackend is valid because
                    // gui_algo::Event is layout-compatible with
                    // Metavision::EventCD (POD x,y,p,t).
                    auto instances = algo_bridge_.list_live();
                    for (auto& inst : instances) {
                        if (inst->is_enabled()) {
                            inst->push_events(cb, ce);
                        }
                    }
                    // P6-A stream write-back: the algorithm slot is mutually
                    // exclusive, so at most one instance can own the stream.
                    // If it exposes a per-batch output span (hot_pixel_filter
                    // removes events, EIS compensates coordinates), the
                    // display/recorded span is substituted with it. The span
                    // points into a backend buffer that stays valid until the
                    // next push_events (same thread), and add_events copies
                    // synchronously — no dangling.
                    for (auto& inst : instances) {
                        if (!inst->is_enabled()) continue;
                        auto [fb, fn] = inst->output_span();
                        if (fb != nullptr && fn > 0) {
                            ob = fb;
                            oe = fb + fn;
                        }
                        break;  // single algorithm slot
                    }
                } catch (const std::exception& e) {
                    // Audit §五-H3: never let an algorithm exception die
                    // silently — log throttled so a throwing algorithm is
                    // visible instead of looking like "no detections".
                    static std::atomic<int> live_push_errs{0};
                    const int c = ++live_push_errs;
                    if (c == 1 || c % 1000 == 0) {
                        qWarning("push_events to algorithms failed (x%d): %s", c, e.what());
                    }
                } catch (...) {
                    static std::atomic<int> live_push_errs_unknown{0};
                    const int c = ++live_push_errs_unknown;
                    if (c == 1 || c % 1000 == 0) {
                        qWarning("push_events to algorithms failed with unknown exception (x%d)", c);
                    }
                }
                // Feed the performance profiler for live-camera latency measurement.
                // Uses the RAW event count/timestamp so the rate reflects camera
                // output, not the post-filter count.
                perf_meter_.tick_events(static_cast<std::size_t>(re - rb), (re - 1)->t);

                // Throttled, downsampled feed to the XYT 3D window. The
                // SpaceTimeDisplay is a QOpenGLWidget and must only be touched
                // from the GUI thread, so we marshal via QMetaObject::invokeMethod.
                // For file sources the XYT feed is driven by events_window_ready
                // instead (synchronized with playback), so skip it here.
                // Uses the conditioned pointers (cb/ce) so the 3D point cloud
                // matches the display orientation.
                if (xyt_display_) {
                    const Metavision::timestamp cur_ts = (ce - 1)->t;
                    const Metavision::timestamp last =
                        algo_last_xyt_post_us_.load(std::memory_order_relaxed);
                    // 8ms post cadence (halved from 16ms): batches posted
                    // between posts are DROPPED, so this constant is also the
                    // t-axis gap between point-cloud sheets in live mode.
                    if (cur_ts - last >= 8000) {
                        algo_last_xyt_post_us_.store(cur_ts, std::memory_order_relaxed);
                        const std::size_t count = static_cast<std::size_t>(ce - cb);
                        auto copy = std::make_shared<std::vector<Metavision::EventCD>>();
                        if (count > 5000) {
                            const std::size_t stride = count / 5000;
                            copy->reserve(count / stride + 1);
                            for (std::size_t i = 0; i < count; i += stride) {
                                copy->push_back(cb[i]);
                            }
                        } else {
                            copy->assign(cb, ce);
                        }
                        QMetaObject::invokeMethod(this, [this, copy]() {
                            if (xyt_display_) {
                                // Sync time_window from algo parameter in case
                                // the user changed it in the sidebar.
                                if (xyt_algo_) {
                                    const auto tw = xyt_algo_->get_param("time_window_us");
                                    if (!tw.empty()) {
                                        // Malformed param (hand-edited JSON)
                                        // must not throw out of the slot —
                                        // keep the current window (audit §六-U5).
                                        try {
                                            xyt_display_->set_time_window_ms(
                                                static_cast<float>(std::stoi(tw)) / 1000.0f);
                                        } catch (const std::exception&) {}
                                    }
                                }
                                xyt_display_->push_events(copy->data(),
                                                          copy->data() + copy->size());
                            }
                        }, Qt::QueuedConnection);
                    }
                }
            }
        });
}

void MainWindow::on_events_window_ready(std::shared_ptr<std::vector<Metavision::EventCD>> events,
                                        Metavision::timestamp ts) {
    // An EMPTY window still feeds the algorithms an empty span. Without
    // this, an overlay-producing algorithm (line_segment) kept its LAST
    // result forever: the window had no events to push, so last_ was never
    // cleared and the drawn segments never vanished (user-reported: lines
    // never disappear). The empty push makes line_segment's process() see
    // an empty packet and emit nothing.
    if (!events) return;
    if (events->empty()) {
        // Same non-null pointer for begin/end: a valid empty span (two null
        // pointers would make end-begin undefined behaviour).
        static const Metavision::EventCD kEmptyEvent{};
        auto insts = algo_bridge_.list_live();
        for (auto& inst : insts) {
            if (inst->is_enabled()) {
                inst->push_events(&kEmptyEvent, &kEmptyEvent);
            }
        }
        return;
    }

    // ======================================================================
    // TIMESTAMP SCALING FOR ALGORITHM EVENT FEEDING (FILE PLAYBACK ONLY)
    // ======================================================================
    // Problem: During file playback, FileFrameGenerator replays events at a
    // user-controlled rate:
    //   playback_rate = fps * accumulation_window_us / 1e6
    //
    //   - Slow motion (rate < 1): e.g. fps=30, window=100us → rate=0.003.
    //     Each displayed frame contains events spanning only 100us of real
    //     event time, but 33ms of wall-clock time pass between frames.
    //   - Fast forward (rate > 1): e.g. fps=60, window=33000us → rate=1.98.
    //     Each frame spans 33000us of event time but only 16ms of wall-clock.
    //
    // Temporal algorithms (InteractingMaps decay-tau, E2VID recurrent state,
    // XYT time-window normalization, etc.) rely on the RELATIVE time between
    // events to evolve their internal state. When fed real event timestamps
    // in slow motion, the algorithm sees time barely advancing (100us per
    // frame) while its decay tau is 500ms — the state never evolves, causing
    // flickering (InteractingMaps), flat gray output (BardowVariational), or
    // a collapsed Z axis (XYT 3D display shows a single plane instead of a
    // cloud).
    //
    // Solution: Scale event timestamps by 1/playback_rate before feeding them
    // to algorithm instances. This makes the algorithm perceive time
    // advancing at wall-clock rate (1/fps per frame), exactly as it would
    // with a live camera at the same fps. The algorithm's temporal parameters
    // (decay tau, time windows) then behave as intended.
    //
    //     scaled_t = real_t / playback_rate
    //
    // IMPORTANT — DISPLAY USES REAL TIMESTAMPS:
    //   The XYT 3D display and any algorithm-output annotations that use
    //   timestamps for visualization MUST use real event timestamps, not
    //   scaled ones. The XYT display is fed the ORIGINAL (unscaled) events
    //   below. Algorithm output frames/overlays produced from scaled-time
    //   input do NOT carry timestamps in their results (AlgoResult contains
    //   spatial data: colored_events with x/y/p, trajectories with x/y, and
    //   cv::Mat frames — no timestamp fields), so no back-conversion is
    //   needed for display. The `ts` parameter in frame_ready is the real
    //   cursor timestamp from FileFrameGenerator and is used directly for
    //   status-bar/position display.
    //
    // This scaling is NOT a bug — it is an intentional adaptation for
    // rate-controlled file playback.
    // ======================================================================

    // Compute playback rate from FramePipeline's current parameters.
    double playback_rate = 1.0;
    if (auto* fp = camera_.frame_pipeline()) {
        const double fps = static_cast<double>(fp->fps());
        const double window_us = static_cast<double>(fp->accumulation_time_us());
        playback_rate = fps * window_us / 1.0e6;
    }
    // Guard against division by zero.
    if (playback_rate < 1.0e-9) playback_rate = 1.0;

    // Feed algorithm instances. If playback_rate ≈ 1.0 (real-time), feed the
    // original events directly. Otherwise, create a scaled-timestamp copy.
    // The 0.5% tolerance avoids unnecessary copies for near-real-time rates.
    const bool need_scaling = std::fabs(playback_rate - 1.0) > 0.005;
    try {
        auto instances = algo_bridge_.list_live();
        if (!instances.empty()) {
            if (need_scaling) {
                // Create a copy with scaled timestamps: scaled_t = real_t / rate.
                // This makes the algorithm see time advancing at wall-clock rate.
                auto scaled = std::make_shared<std::vector<Metavision::EventCD>>();
                scaled->reserve(events->size());
                const double inv_rate = 1.0 / playback_rate;
                for (const auto& ev : *events) {
                    Metavision::EventCD scaled_ev = ev;
                    scaled_ev.t = static_cast<Metavision::timestamp>(
                        static_cast<double>(ev.t) * inv_rate);
                    scaled->push_back(scaled_ev);
                }
                for (auto& inst : instances) {
                    if (inst->is_enabled()) {
                        inst->push_events(scaled->data(),
                                          scaled->data() + scaled->size());
                    }
                }
            } else {
                for (auto& inst : instances) {
                    if (inst->is_enabled()) {
                        inst->push_events(events->data(),
                                          events->data() + events->size());
                    }
                }
            }
        }
    } catch (const std::exception& e) {
        // Audit §五-H3: same throttled logging as the live-camera path —
        // a throwing algorithm must be visible, not silently stale.
        static std::atomic<int> file_push_errs{0};
        const int c = ++file_push_errs;
        if (c == 1 || c % 1000 == 0) {
            qWarning("push_events to algorithms failed (x%d): %s", c, e.what());
        }
    } catch (...) {
        static std::atomic<int> file_push_errs_unknown{0};
        const int c = ++file_push_errs_unknown;
        if (c == 1 || c % 1000 == 0) {
            qWarning("push_events to algorithms failed with unknown exception (x%d)", c);
        }
    }

    // Feed the performance profiler (file playback path). For file mode the
    // latency measured here → frame_ready is near-zero (both on GUI thread),
    // which correctly reflects that file playback has no real-time backlog.
    perf_meter_.tick_events(events->size(), ts);

    // Feed the XYT 3D display with REAL (unscaled) timestamps — the Z axis
    // must show the actual event time, not the scaled playback time.
    // Same downsampling as the live path so a large accumulation window
    // doesn't overwhelm the OpenGL point cloud.
    if (xyt_display_) {
        const std::size_t count = events->size();
        auto copy = std::make_shared<std::vector<Metavision::EventCD>>();
        if (count > 5000) {
            const std::size_t stride = count / 5000;
            copy->reserve(count / stride + 1);
            for (std::size_t i = 0; i < count; i += stride) {
                copy->push_back((*events)[i]);
            }
        } else {
            *copy = *events;
        }
        if (xyt_algo_) {
            const auto tw = xyt_algo_->get_param("time_window_us");
            if (!tw.empty()) {
                // Malformed param (hand-edited JSON) must not throw out of
                // the slot — keep the current window (audit §六-U5).
                try {
                    xyt_display_->set_time_window_ms(
                        static_cast<float>(std::stoi(tw)) / 1000.0f);
                } catch (const std::exception&) {}
            }
        }
        xyt_display_->push_events(copy->data(), copy->data() + copy->size());
    }
}

void MainWindow::remove_algo_callback() {
    camera_.set_conditioned_listener(nullptr);
}

void MainWindow::process_algo_results(QImage& frame) {
    // Iterate a snapshot of live instances and pull each one's latest result,
    // then dispatch to the instance's display strategy (design §3.5.4). The
    // per-mode drawing/routing that used to live in the switch below now lives
    // in the concrete IDisplayStrategy classes (gui/display/display_strategy.cpp).
    DisplayContext ctx{&annotator_, &algo_windows_, xyt_display_.data(), this,
                       &camera_, nullptr};
    // Snapshot once and reuse for draw_roi_overlays to avoid a redundant
    // list_live() call (N7).
    auto instances = algo_bridge_.list_live();
    for (auto& inst : instances) {
        // Surface the flood-guard auto-disable so the user knows why an algo
        // stopped producing output (design §5.6.7). Re-enabling it from the
        // sidebar's "算法模块" tab clears the overload and restarts processing.
        if (inst->is_overloaded()) {
            const QString nm = QString::fromStdString(inst->info().display_name);
            statusBar()->showMessage(
                tr("%1 auto-disabled: event rate too high (re-enable to retry)").arg(nm), 5000);
            // Reflect the disabled state in the AlgoWindow status label.
            const QString overload_text =
                tr("AUTO-DISABLED: event flooding detected. Re-enable from the sidebar.");
            auto wit = algo_windows_.find(inst->info().name);
            if (wit != algo_windows_.end() && wit.value()) {
                wit.value()->set_status_text(overload_text);
            }
            // Overlay algorithms have no AlgoWindow anymore (Phase 2.6 debug
            // D-1) — the status label lives in the sidebar.
            if (auto* ap = settings_->algorithms_panel()) {
                ap->set_algo_status(inst->info().name, overload_text);
            }
            continue;
        }
        if (!inst->is_enabled()) continue;
        AlgoResult r;
        try {
            r = inst->pull_result();
        } catch (...) {
            continue;
        }
        // apply_strategy runs mat_to_qimage / frame.copy etc. — a
        // cv::Exception escaping this Qt slot would terminate the app
        // (audit §五-H2). Skip this algorithm's output for one frame instead.
        try {
            inst->apply_strategy(frame, r, ctx);
        } catch (...) {
            continue;
        }
        // Overlay algorithms no longer have an AlgoWindow (Phase 2.6 debug
        // D-1) — surface their status line (detection counts / effective
        // params) in the sidebar instead.
        if (inst->info().display_mode == AlgoDisplayMode::Overlay &&
            !r.status.empty()) {
            if (auto* ap = settings_->algorithms_panel()) {
                ap->set_algo_status(inst->info().name,
                                    QString::fromStdString(r.status));
            }
        }
    }

    // Phase 2.6: draw_roi_overlays (per-algorithm yellow ROI frames) was
    // deleted with the legacy per-backend ROI. The unified ROI's frame is
    // drawn via EventDisplayWidget::set_roi_overlay, driven by
    // unified_roi_changed / RoiPanel::roi_applied.
}


void MainWindow::on_intrinsic_wizard() {
    if (!calibration_wizard_) {
        calibration_wizard_ = new CalibrationWizard(this);
        calibration_wizard_->setAttribute(Qt::WA_DeleteOnClose);
        connect(calibration_wizard_, &QObject::destroyed, this, [this]() {
            calibration_wizard_ = nullptr;
        });
        // The wizard runs the Auto Bias controller (19–20 Mev/s band) while
        // open and switches it off on close — keep the Biases panel in sync
        // with the controller state (bias rows AND the Auto Bias checkbox /
        // rate spinboxes) after each out-of-band change.
        connect(calibration_wizard_, &CalibrationWizard::biases_changed_externally,
                settings_->biases_panel(), &BiasesPanel::refresh_row_values);
        connect(calibration_wizard_, &CalibrationWizard::biases_changed_externally,
                settings_->biases_panel(), &BiasesPanel::sync_auto_bias_ui);
    }
    calibration_wizard_->set_camera(&camera_);
    calibration_wizard_->set_display(display_);
    calibration_wizard_->show_intrinsic();
}

void MainWindow::on_focus() {
    if (!focus_dialog_) {
        focus_dialog_ = new FocusDialog(this);
        focus_dialog_->setAttribute(Qt::WA_DeleteOnClose);
        connect(focus_dialog_, &QObject::destroyed, this, [this]() {
            focus_dialog_ = nullptr;
        });
    }
    focus_dialog_->set_display(display_);
    focus_dialog_->show();
    focus_dialog_->raise();
    focus_dialog_->activateWindow();
}

void MainWindow::on_save_layout() {
    if (!layout_manager_) return;
    const QString path = QFileDialog::getSaveFileName(
        this, tr("Save Layout"), {}, tr("Layout JSON (*.json);;All Files (*)"));
    if (!path.isEmpty()) {
        if (layout_manager_->save(path)) {
            statusBar()->showMessage(tr("Layout saved to %1").arg(path), 3000);
        } else {
            QMessageBox::warning(this, tr("Save Layout"), tr("Could not save layout."));
        }
    }
}

void MainWindow::on_load_layout() {
    if (!layout_manager_) return;
    const QString path = QFileDialog::getOpenFileName(
        this, tr("Load Layout"), {}, tr("Layout JSON (*.json);;All Files (*)"));
    if (!path.isEmpty()) {
        if (layout_manager_->load(path)) {
            update_toggle_icon();
            statusBar()->showMessage(tr("Layout loaded from %1").arg(path), 3000);
        } else {
            QMessageBox::warning(this, tr("Load Layout"), tr("Could not load layout."));
        }
    }
}

void MainWindow::on_reset_layout() {
    if (layout_manager_) layout_manager_->reset_layout();
    if (settings_dock_) {
        settings_dock_->setVisible(true);
        // §15.5: expand the sidebar if it was collapsed — reset should
        // restore the full default layout, not a collapsed state.
        if (settings_ && !settings_->is_content_visible()) {
            saved_sidebar_width_ = 380;  // use default width on reset
            settings_->toggle_content();  // emits content_toggled(true)
            // on_sidebar_content_toggled handles the resize; no need to
            // resize again here.
        } else {
            // Already expanded — just force the default width.
            settings_dock_->setMinimumWidth(200);
            settings_dock_->setMaximumWidth(QWIDGETSIZE_MAX);
            const QList<QDockWidget*> docks = {settings_dock_};
            const QList<int> sizes = {380};
            resizeDocks(docks, sizes, Qt::Horizontal);
        }
    }
    update_toggle_icon();
    statusBar()->showMessage(tr("Layout reset to default."), 3000);
}

void MainWindow::forward_panel_message(const QString& msg, bool isError) {
    if (isError) {
        QMessageBox::warning(this, tr("Camera control"), msg);
    } else {
        statusBar()->showMessage(msg, 4000);
    }
}

void MainWindow::on_refresh_devices() {
    const auto sources = camera_.list_online_sources();
    settings_->devices_panel()->refresh_sources(sources);
    if (sources.empty()) {
        statusBar()->showMessage(tr("No cameras detected."), 3000);
    } else {
        statusBar()->showMessage(tr("%1 camera(s) found.").arg(sources.size()), 3000);
    }
}

void MainWindow::on_about() {
    QMessageBox::about(this, tr("About GUI for openEB"),
        tr("<h3>GUI for openEB</h3>"
           "<p>Qt 6 + OpenEB 5.2.0.</p>"
           "<p>Camera discovery, real-time OpenGL event display, statistics, "
           "Bias / ROI / ESP / Trigger panels, recording & playback, HDF5 / AVI "
           "export, JSON config, OpenEB filter-chain preprocessing, "
           "file conversion tools, calibration wizard, "
           "30 self-developed CV/analytics algorithms with overlay/replace/"
           "standalone display modes, XYT 3D point cloud, and more.</p>"));
}

// ---------------------------------------------------------------------------
// Standalone algorithm windows (design §5.6.1)
// ---------------------------------------------------------------------------

void MainWindow::on_open_algo_window(const std::string& algo_name) {
    // Reuse an existing window if one is already open for this algorithm.
    auto it = algo_windows_.find(algo_name);
    if (it != algo_windows_.end() && it.value()) {
        it.value()->show();
        it.value()->raise();
        it.value()->activateWindow();
        return;
    }

    // Create the AlgoWindow (a QDockWidget). The constructor finds/creates
    // the live AlgoInstance and enables it.
    auto* w = new AlgoWindow(&algo_bridge_, algo_name, this);
    if (!w->instance()) {
        // Unknown algorithm — clean up and bail out.
        delete w;
        statusBar()->showMessage(tr("Unknown algorithm: %1")
                                     .arg(QString::fromStdString(algo_name)), 3000);
        return;
    }
    algo_windows_[algo_name] = w;

    // sensor_self_test: reset on (re)open so each session starts fresh
    // (set_enabled(true) in the AlgoWindow constructor preserves prior state
    // for pause-resume workflows, but the self-test should start clean).
    if (algo_name == "sensor_self_test" && w->instance()) {
        w->instance()->reset();
    }

    // For Standalone frame-producing algorithms, install an EventDisplayWidget
    // so process_algo_results() can push frames to it. The frame size may be
    // the ROI dimensions (e.g. event_to_video at 128×128) and the widget
    // scales it to fit.
    //
    // For self-developed Overlay algorithms (design §5.6.6) we also install
    // an EventDisplayWidget: it shows a zoomed view of just the ROI region
    // with the algorithm's overlay primitives drawn on it. Without this the
    // user cannot easily inspect the algorithm's result inside the small ROI
    // on the main (sensor-scale) display.
    const auto* info = algo_bridge_.find(algo_name);
    if (!info && w->instance()) {
        // Unregistered workflow (sensor_self_test from the Devices button):
        // use the window's built-in AlgoInfo so the display widget is still
        // installed (the registry lookup only works for real algorithms).
        info = &w->info();
    }
    if (info && info->display_mode == AlgoDisplayMode::Standalone) {
        // Note: background_mask is registered as Replace, not Standalone —
        // listing it here was a dead branch (audit §五-G5).
        if (algo_name == "time_surface" || algo_name == "event_to_video" ||
            algo_name == "sensor_self_test" ||
            algo_name == "frequency_map" || algo_name == "dl_optical_flow") {
            auto* disp = new EventDisplayWidget(nullptr);
            w->set_display_widget(disp);
        }
    } else if (info && info->display_mode == AlgoDisplayMode::Overlay &&
               info->source == "self") {
        // ROI zoom view for overlay algorithms (design §5.6.6).
        auto* disp = new EventDisplayWidget(nullptr);
        w->set_display_widget(disp);
    }

    // For xyt_visualizer, also open the dedicated SpaceTimeDisplay window
    // (the AlgoWindow shows only the status label — the 3D rendering is
    // handled by SpaceTimeDisplay's own QOpenGLWidget, and parameters are
    // adjusted in the sidebar).
    if (algo_name == "xyt_visualizer") {
        on_open_xyt_view();
    }

    // Sync the AlgorithmsPanel checkbox (blocked, no re-entry).
    if (auto* ap = settings_->algorithms_panel()) {
        ap->set_algo_enabled(algo_name, true);
    }

    // Dock the AlgoWindow into the main window. Default to the right edge
    // (§11.2 point 3) so it doesn't conflict with the settings sidebar
    // (left) or playback bar (bottom). If another algo dock is already
    // there, tab this one with it so multiple algo windows share a single
    // dock slot (the user can switch between them via the tab bar).
    addDockWidget(Qt::RightDockWidgetArea, w);
    // Find any other already-docked algo window to tabify with. Only tabify
    // if both docks are in the same area (BUG-G11: tabifyDockWidget moves the
    // second dock to the first's area, which would relocate the new window
    // if the other algo is in a different area).
    for (auto tit = algo_windows_.constBegin(); tit != algo_windows_.constEnd(); ++tit) {
        AlgoWindow* other = tit.value().data();
        if (other && other != w && !other->isFloating() &&
            dockWidgetArea(other) == Qt::RightDockWidgetArea) {
            tabifyDockWidget(other, w);
            break;
        }
    }

    // On close: disable the algo instance and remove the window from the map.
    // The AlgoWindow emits `closing` from its closeEvent, which fires before
    // Qt::WA_DeleteOnClose destroys it.
    connect(w, &AlgoWindow::closing, this, [this, algo_name](const std::string&) {
        algo_windows_.remove(algo_name);
        auto inst = algo_bridge_.find_live(algo_name);
        // sensor_self_test: request the full report before disabling so the
        // close dialog shows the latest analysis (design §4.4.8). The
        // "__final_report" flag tells the backend to compute and return the
        // full multi-line report (with stats + bad-pixel coords) in the next
        // pull_result().status, instead of the concise per-frame summary.
        if (algo_name == "sensor_self_test" && inst) {
            inst->set_param("__final_report", "1");
            auto r = inst->pull_result();
            inst->set_enabled(false);
            // During application shutdown the modal report would block the
            // close sequence — skip it (the window is going away anyway).
            if (!closing_app_) {
                // Scrollable, fixed-size report dialog: with many suspected
                // bad pixels the coordinate list is long, and a plain
                // QMessageBox would grow to an unusable height.
                QDialog dlg(this);
                dlg.setWindowTitle(tr("Sensor Self-Test Report"));
                dlg.resize(560, 460);
                auto* lay = new QVBoxLayout(&dlg);
                auto* text = new QPlainTextEdit(QString::fromStdString(r.status), &dlg);
                text->setReadOnly(true);
                QFont mono(QStringLiteral("Monospace"));
                mono.setStyleHint(QFont::TypeWriter);
                mono.setPointSize(9);
                text->setFont(mono);
                lay->addWidget(text);
                auto* bb = new QDialogButtonBox(QDialogButtonBox::Close, &dlg);
                connect(bb, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);
                lay->addWidget(bb);
                dlg.exec();
            }
        } else {
            if (inst) inst->set_enabled(false);
        }
        // Sync the AlgorithmsPanel checkbox (blocked, no re-entry).
        if (auto* ap = settings_->algorithms_panel()) {
            ap->set_algo_enabled(algo_name, false);
        }
        // xyt_visualizer: also close the SpaceTimeDisplay.
        if (algo_name == "xyt_visualizer" && xyt_display_) xyt_display_->close();
    });

    w->show();
    w->raise();
    if (info) {
        statusBar()->showMessage(
            tr("%1 window opened").arg(QString::fromStdString(info->display_name)), 2000);
    }
}

// XYT 3D view has a dedicated SpaceTimeDisplay (QOpenGLWidget) that is opened
// from on_open_algo_window("xyt_visualizer") or directly from the sidebar.
void MainWindow::on_open_xyt_view() {
    if (!xyt_display_) {
        xyt_display_ = new SpaceTimeDisplay(this);
        xyt_display_->setWindowFlag(Qt::Window, true);
        xyt_display_->setWindowTitle(tr("XYT 3D Event Cloud"));
        xyt_display_->setAttribute(Qt::WA_DeleteOnClose);
        if (camera_.is_connected()) {
            const auto& info = camera_.sensor_info();
            xyt_display_->set_sensor_geometry(info.width, info.height);
        }
        xyt_algo_ = algo_bridge_.find_live("xyt_visualizer");
        if (!xyt_algo_) xyt_algo_ = algo_bridge_.find_or_create("xyt_visualizer");
        if (xyt_algo_) xyt_algo_->set_enabled(true);
        // Sync the time_window from the algo parameter to the 3D display.
        // The GUI registers time_window_us (default 500000 = 500ms); the
        // SpaceTimeDisplay uses time_window_ms. Without this sync, the
        // display would always use its 50ms default.
        if (xyt_algo_) {
            const auto tw_us = xyt_algo_->get_param("time_window_us");
            if (!tw_us.empty()) {
                // Malformed param (hand-edited JSON) must not throw out of
                // the slot — keep the display's default window (audit §六-U5).
                try {
                    xyt_display_->set_time_window_ms(
                        static_cast<float>(std::stoi(tw_us)) / 1000.0f);
                } catch (const std::exception&) {}
            }
        }
        // Sync the sidebar checkbox (blocked, no re-entry).
        if (auto* ap = settings_->algorithms_panel()) {
            ap->set_algo_enabled("xyt_visualizer", true);
        }
        // The QOpenGLWidget is itself the window; on close we must null
        // xyt_display_ (QPointer does this), disable the algo instance so it
        // stops processing events, and sync the sidebar checkbox.
        connect(xyt_display_, &QObject::destroyed, this, [this]() {
            if (xyt_algo_) {
                xyt_algo_->set_enabled(false);
                xyt_algo_.reset();
            }
            if (auto* ap = settings_->algorithms_panel()) {
                ap->set_algo_enabled("xyt_visualizer", false);
            }
            // Also close the AlgoWindow if open.
            auto it = algo_windows_.find("xyt_visualizer");
            if (it != algo_windows_.end() && it.value()) it.value()->close();
        });
        xyt_display_->resize(800, 600);
        // Install the algo callback if a camera is already connected so the
        // XYT window starts receiving events immediately.
        install_algo_callback();
    }
    xyt_display_->show();
    xyt_display_->raise();
    statusBar()->showMessage(tr("XYT 3D window opened"), 2000);
}

void MainWindow::on_imu_toggled(bool on) {
    if (on) {
        if (!camera_.set_imu_enabled(true)) {
            set_imu_ui_state(false);
            statusBar()->showMessage(tr("IMU stream not available for this source."), 3000);
            return;
        }
        if (!imu_window_) {
            imu_window_ = new ImuWindow(&camera_, this);
            connect(imu_window_, &ImuWindow::window_closed, this, [this]() {
                camera_.set_imu_enabled(false);
                set_imu_ui_state(false);
            });
            addDockWidget(Qt::RightDockWidgetArea, imu_window_);
            // When the APS preview is already docked on the side, stack
            // this one BELOW it (top/bottom halves separated by the dock
            // splitter's narrow gap).
            if (aps_window_ && !aps_window_->isFloating() &&
                dockWidgetArea(aps_window_) == Qt::RightDockWidgetArea) {
                splitDockWidget(aps_window_, imu_window_, Qt::Vertical);
            }
        }
        imu_window_->show();
        imu_window_->raise();
        statusBar()->showMessage(tr("IMU stream enabled"), 2000);
    } else {
        camera_.set_imu_enabled(false);
        if (imu_window_) imu_window_->close();
    }
}

void MainWindow::set_imu_ui_state(bool on) {
    if (auto* dp = settings_->devices_panel()) dp->set_imu_checked(on);
}

void MainWindow::on_aps_toggled(bool on) {
    if (on) {
        if (!camera_.set_aps_enabled(true)) {
            set_aps_ui_state(false);
            statusBar()->showMessage(tr("APS frames not available for this source."), 3000);
            return;
        }
        if (!aps_window_) {
            aps_window_ = new ApsWindow(&camera_, this);
            connect(aps_window_, &ApsWindow::window_closed, this, [this]() {
                camera_.set_aps_enabled(false);
                set_aps_ui_state(false);
            });
            addDockWidget(Qt::RightDockWidgetArea, aps_window_);
            if (imu_window_ && !imu_window_->isFloating() &&
                dockWidgetArea(imu_window_) == Qt::RightDockWidgetArea) {
                splitDockWidget(imu_window_, aps_window_, Qt::Vertical);
            }
        }
        aps_window_->show();
        aps_window_->raise();
        statusBar()->showMessage(tr("APS stream enabled"), 2000);
    } else {
        camera_.set_aps_enabled(false);
        if (aps_window_) aps_window_->close();
    }
}

void MainWindow::set_aps_ui_state(bool on) {
    if (auto* dp = settings_->devices_panel()) dp->set_aps_checked(on);
}

} // namespace gui
