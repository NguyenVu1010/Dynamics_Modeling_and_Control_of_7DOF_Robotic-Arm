#pragma once

#include <QMainWindow>
#include <QString>
#include <QThread>
#include <array>
#include <filesystem>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "cobot/pipeline/overrides.hpp"
#include "cobot/studio/bridge_client.hpp"
#include "cobot/studio/profile_catalog.hpp"
#include "cobot/studio/run_store.hpp"
#include "cobot/studio/studio_config.hpp"

#include "metrics.hpp"  // bench: write_run_files, write_summary_* behind the export slots
#include "studio_worker.hpp"
#include "time_cursor.hpp"

class QAbstractButton;
class QCloseEvent;
class QSlider;
class QGraphicsLineItem;
class QTimer;
class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QLineEdit;
class QListWidget;
class QProgressBar;
class QPushButton;
class QSpinBox;
class QTreeWidget;
class QTreeWidgetItem;
namespace QtCharts {
class QChartView;
class QValueAxis;
}  // namespace QtCharts

// The cobot_studio window (feature studio-app, cobot_studio design 6.7, 6.9, ADR-021): a thin view
// over cobot_studio_core with the Configuration, Control, Tracking and Verification tabs. Widgets
// carry the object names of the UI contract.
namespace cobot::studio {

class ControlTab;
class ErrorModel;
class RobotView;

class MainWindow : public QMainWindow {
  Q_OBJECT
 public:
  // config_dir holds default.yaml and imperfections/ (discover_base_pairs); every run is
  // materialised under work_dir.
  MainWindow(std::filesystem::path config_dir,
             std::filesystem::path work_dir,
             QWidget* parent = nullptr);
  // bridge_name: "cobot_controller" -> region /cobot_cobot_controller (main.cpp --bridge).
  MainWindow(std::filesystem::path config_dir,
             std::filesystem::path work_dir,
             const std::string& bridge_name,
             QWidget* parent = nullptr);
  ~MainWindow() override;
  MainWindow(const MainWindow&) = delete;
  MainWindow& operator=(const MainWindow&) = delete;

  const StudioConfig& studio_config() const { return config_; }
  const RunStore& run_store() const { return store_; }  // GUI thread only

 public slots:
  // Adds or replaces one override exactly as an edit of its field would.
  void set_override(const pipeline::KeyOverride& override_value);
  // The slots behind the buttons, without the file dialog. true on success.
  bool save_studio(const QString& path);
  bool open_studio(const QString& path);
  bool export_run(const QString& csv_path);
  bool export_summary(const QString& csv_path);

 protected:
  void closeEvent(QCloseEvent* event) override;  // releases bridge authority

 private:
  // One editor row of paramTree: the catalog key and its value cells.
  struct ParamRow {
    std::string key;
    QTreeWidgetItem* item = nullptr;
    std::vector<QWidget*> cells;
  };

  QWidget* build_config_tab();
  QWidget* build_tracking_tab();
  void select_base(int index);
  void load_base_values();  // base_values_ of config_.base
  void remove_override(const std::string& key);
  void cell_edited(const ParamRow& row);
  void refresh_fields();    // every field from config_ (signals blocked)
  void refresh_validity();  // validate(): field marks and the run/sweep buttons
  void refresh_runs();      // errorTable, chart, status bar
  void refresh_chart();
  void draw_cursor();
  void refresh_view();  // the checked runs and the selection to the 3D view
  void start_job(bool sweep);
  void add_result(const EntryHandle& handle);
  void job_finished(const QString& error);
  void set_busy(bool busy);
  void error_box(const QString& text);
  void bridge_tick();  // bridgeTimer: tick, Control tab, live runs, Gazebo rows
  void apply_gazebo();

  std::filesystem::path work_dir_;
  std::vector<BasePair> pairs_;
  StudioConfig config_;
  std::map<std::string, pipeline::KeyOverride> base_values_;
  RunStore store_;
  std::map<std::uint64_t, int> sweep_batch_;  // RunEntry::id -> sweep number
  int sweeps_ = 0;
  bool busy_ = false;
  bool sweeping_ = false;  // the job in progress is a sweep (cancellable between runs)
  bool zoomed_ = false;
  bool drawing_ = false;
  bool valid_ = true;

  QThread thread_;
  std::unique_ptr<StudioWorker> worker_;
  TimeCursor* cursor_ = nullptr;
  ErrorModel* model_ = nullptr;

  QComboBox* base_combo_ = nullptr;
  QTreeWidget* tree_ = nullptr;
  std::vector<ParamRow> rows_;
  std::vector<std::pair<std::string, QWidget*>> fields_;  // key -> editor, for validate() marks
  QDoubleSpinBox* omega_ = nullptr;
  QCheckBox* advanced_ = nullptr;
  QDoubleSpinBox* hal_timeout_ = nullptr;
  std::array<QDoubleSpinBox*, 4> tol_{};  // goal pos, goal rot, path pos, path rot
  QLineEdit* waypoints_ = nullptr;
  QDoubleSpinBox* duration_ = nullptr;
  QSpinBox* seed_ = nullptr;
  QListWidget* run_list_ = nullptr;
  QtCharts::QChartView* chart_view_ = nullptr;
  RobotView* robot_view_ = nullptr;
  QtCharts::QValueAxis* x_axis_ = nullptr;
  QtCharts::QValueAxis* y_axis_ = nullptr;
  QGraphicsLineItem* cursor_line_ = nullptr;
  QTimer* redraw_ = nullptr;
  QComboBox* joint_combo_ = nullptr;
  std::vector<QAbstractButton*> toggles_;
  QAbstractButton* error_toggle_ = nullptr;  // showError
  QPushButton* run_button_ = nullptr;
  QPushButton* sweep_button_ = nullptr;
  QPushButton* cancel_button_ = nullptr;
  QProgressBar* progress_ = nullptr;
  QLabel* memory_label_ = nullptr;
  std::unique_ptr<BridgeClient> client_;
  ControlTab* control_ = nullptr;
  QTimer* bridge_timer_ = nullptr;
  QSlider* speed_ = nullptr;
  QPushButton* apply_gazebo_ = nullptr;
  QLabel* bridge_label_ = nullptr;
  bool greyed_ = false;  // unsupported paramTree rows disabled (connected)
  std::uint64_t apply_id_ = 0;
  StudioConfig applying_;                // sent with apply_id_
  std::optional<StudioConfig> applied_;  // the last accepted apply_config
  int applies_ = 0;
};

}  // namespace cobot::studio
