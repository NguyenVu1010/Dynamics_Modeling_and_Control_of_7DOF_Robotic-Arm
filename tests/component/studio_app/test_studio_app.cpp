#include <QAbstractButton>
#include <QAbstractItemModel>
#include <QComboBox>
#include <QElapsedTimer>
#include <QLabel>
#include <QListWidget>
#include <QListWidgetItem>
#include <QMetaObject>
#include <QProgressBar>
#include <QSlider>
#include <QStatusBar>
#include <QString>
#include <QTabWidget>
#include <QTableView>
#include <QTimer>
#include <QTreeWidget>
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <memory>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/pipeline/log_record.hpp"
#include "cobot/pipeline/overrides.hpp"

#include <QtCharts/QAbstractSeries>
#include <QtCharts/QChart>
#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>

#if __has_include("cobot/studio/run_store.hpp")
#include "cobot/studio/metrics.hpp"
#include "cobot/studio/param_catalog.hpp"
#include "cobot/studio/profile_catalog.hpp"
#include "cobot/studio/run_store.hpp"
#include "cobot/studio/studio_config.hpp"
#else
#include "metrics.hpp"
#include "param_catalog.hpp"
#include "profile_catalog.hpp"
#include "run_store.hpp"
#include "studio_config.hpp"
#endif
#if __has_include("run_data.hpp")
#include "run_data.hpp"
#endif

#include "main_window.hpp"
#include "studio_app_test_util.hpp"

// cobot_studio smoke test (feature studio-app, cobot_studio design 6.7, 6.9, 7; ADR-021), run
// with QT_QPA_PLATFORM=offscreen. Widgets are found by the object names of the brief's UI
// contract. Besides them, this test pins the window class of src/studio/app/main_window.hpp:
//
//   namespace cobot::studio {
//   class MainWindow : public QMainWindow {
//     Q_OBJECT
//    public:
//     // config_dir holds default.yaml and imperfections/ (discover_base_pairs, the --config-dir
//     // of cobot_studio); every run is materialised under work_dir (--work-dir).
//     MainWindow(std::filesystem::path config_dir, std::filesystem::path work_dir,
//                QWidget* parent = nullptr);
//     const StudioConfig& studio_config() const;  // what the Configuration tab shows
//     const RunStore& run_store() const;          // GUI thread only
//    public slots:
//     // Adds or replaces one override exactly as an edit of its field would (validate() and the
//     // run/sweep buttons follow); the spin boxes cannot express out-of-range values.
//     void set_override(const pipeline::KeyOverride& override_value);
//     // The slots behind the buttons, without the file dialog. true on success.
//     bool save_studio(const QString& path);     // saveButton: save_studio_file
//     bool open_studio(const QString& path);     // openButton: load_studio_file, fields updated
//     bool export_run(const QString& csv_path);  // exportRunButton: current runList item,
//                                                // bench::write_run_files (CSV + meta)
//     bool export_summary(const QString& csv_path);  // exportSummaryButton: checked runs,
//                                                    // write_summary_header + one row per run
//   };
//   }  // namespace cobot::studio
//
// Behaviour pinned beyond the brief's wording:
// - While a run or a sweep is in progress cancelButton is enabled; otherwise it is disabled.
// - New runList items are checkable; the time cursor is not a chart series, so with only `showQ`
//   on the chart holds one series per checked run.
// - errorTable column 2 is "final pos (mm)" (the brief's column order: run, changes, final pos,
//   ...); its Qt::DisplayRole text is QString::number(final_pos_err_m * 1e3, 'g', 4).
// - The base combo's item texts are the BasePair names.
namespace cobot::studio {
namespace {

namespace fs = std::filesystem;
using QtCharts::QAbstractSeries;
using QtCharts::QChart;
using QtCharts::QChartView;
using test::is_red;
using test::ModalHandler;
using test::pump;
using test::set_spin_value;
using test::spin_max;
using test::spin_min;
using test::spin_value;
using test::wait_until;

const fs::path kRepo = COBOT_REPO_DIR;
const fs::path kConfigDir = kRepo / "config";
const fs::path kOutput = fs::path(COBOT_TEST_OUTPUT_DIR) / "studio_app";

constexpr int kRunTimeoutMs = 60000;
constexpr int kColFinalPos = 2;
constexpr double kRunDurationS = 1.0;  // 1 s at control_rate_hz 1000 -> 1000 rows per run

const std::vector<std::string> kBaseNames = {
    "ideal", "realistic", "pessimistic", "premium_fr3", "mid_kr1205", "budget_xarm7"};

const char* const kToggles[] = {"showQRef",
                                "showQ",
                                "showQd",
                                "showQdd",
                                "showQTrue",
                                "showNoise",
                                "showError",
                                "showTau",
                                "showTauMotor",
                                "showTauFriction"};

std::string read_file(const fs::path& p) {
  std::ifstream in(p, std::ios::binary);
  return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

std::string first_line(const std::string& text) {
  return text.substr(0, text.find('\n'));
}

std::size_t line_count(const std::string& text) {
  std::size_t n = 0;
  for (const char c : text) {
    n += c == '\n' ? 1 : 0;
  }
  return n;
}

class StudioApp : public ::testing::Test {
 protected:
  void SetUp() override {
    const ::testing::TestInfo* info = ::testing::UnitTest::GetInstance()->current_test_info();
    work_dir_ = kOutput / "work" / info->name();
    fs::remove_all(work_dir_);
    fs::create_directories(work_dir_);
    window_ = std::make_unique<MainWindow>(kConfigDir, work_dir_);
    window_->show();
  }

  void TearDown() override { window_.reset(); }

  // The widget with this object name and type; throws (test failure) when it is missing.
  template <typename T>
  T& get(const char* name) const {
    T* w = window_->findChild<T*>(QString::fromLatin1(name));
    if (w == nullptr) {
      throw std::runtime_error(std::string("no widget '") + name + "' of the expected type");
    }
    return *w;
  }

  QWidget& widget(const char* name) const { return get<QWidget>(name); }
  QAbstractButton& button(const char* name) const { return get<QAbstractButton>(name); }
  QListWidget& run_list() const { return get<QListWidget>("runList"); }
  QAbstractItemModel& table() const {
    QAbstractItemModel* model = get<QTableView>("errorTable").model();
    if (model == nullptr) {
      throw std::runtime_error("errorTable has no model");
    }
    return *model;
  }

  bool idle() const { return !button("cancelButton").isEnabled(); }

  void select_base(const std::string& name) const {
    auto& combo = get<QComboBox>("baseCombo");
    const int index = combo.findText(QString::fromStdString(name));
    ASSERT_GE(index, 0) << name;
    combo.setCurrentIndex(index);
    pump(50);
    ASSERT_EQ(window_->studio_config().base.name, name) << modal_.log();
  }

  // Clicks runButton with durationSpin = duration_s and waits for one more runList item.
  void run_bench(double duration_s) const {
    const int before = run_list().count();
    set_spin_value(&widget("durationSpin"), duration_s);
    ASSERT_TRUE(button("runButton").isEnabled());
    button("runButton").click();
    ASSERT_TRUE(
        wait_until([&] { return run_list().count() == before + 1 && idle(); }, kRunTimeoutMs))
        << "runList " << run_list().count() << ", log: " << modal_.log();
  }

  // Opens the sweep dialog through sweepButton; the modal handler fills it.
  void start_sweep(const char* param, const char* levels, const char* seeds) {
    modal_.set_sweep(
        QString::fromLatin1(param), QString::fromLatin1(levels), QString::fromLatin1(seeds));
    const int dialogs = modal_.sweep_dialogs();
    ASSERT_TRUE(button("sweepButton").isEnabled());
    button("sweepButton").click();
    ASSERT_TRUE(wait_until([&] { return modal_.sweep_dialogs() > dialogs; }, 5000))
        << "no SweepDialog with levelsEdit opened";
  }

  void check_all_runs() const {
    for (int i = 0; i < run_list().count(); ++i) {
      run_list().item(i)->setCheckState(Qt::Checked);
    }
  }

  ModalHandler modal_;
  fs::path work_dir_;
  std::unique_ptr<MainWindow> window_;
};

TEST_F(StudioApp, WindowHasThreeTabsAndTheContractWidgets) {
  // Design 5.3 order (feature studio-control): Configuration, Control, Tracking.
  const auto& tabs = get<QTabWidget>("tabs");
  ASSERT_EQ(tabs.count(), 3);
  EXPECT_EQ(tabs.widget(0)->objectName(), QStringLiteral("configTab"));
  EXPECT_EQ(tabs.widget(1)->objectName(), QStringLiteral("controlTab"));
  EXPECT_EQ(tabs.widget(2)->objectName(), QStringLiteral("trackingTab"));
  EXPECT_EQ(tabs.tabText(0), QStringLiteral("Configuration"));
  EXPECT_EQ(tabs.tabText(1), QStringLiteral("Control"));
  EXPECT_EQ(tabs.tabText(2), QStringLiteral("Tracking"));

  for (const char* name :
       {"baseCombo",       "paramTree",          "omegaSpin",   "advancedGains", "halTimeoutSpin",
        "goalPosSpin",     "goalRotSpin",        "pathPosSpin", "pathRotSpin",   "speedSlider",
        "waypointsEdit",   "durationSpin",       "seedSpin",    "openButton",    "saveButton",
        "exportButton",    "resetButton",        "runList",     "chartView",     "jointCombo",
        "errorTable",      "runButton",          "sweepButton", "cancelButton",  "progressBar",
        "exportRunButton", "exportSummaryButton"}) {
    EXPECT_NE(window_->findChild<QWidget*>(QString::fromLatin1(name)), nullptr) << name;
  }
  for (const char* name : kToggles) {
    EXPECT_NE(window_->findChild<QAbstractButton*>(QString::fromLatin1(name)), nullptr) << name;
  }
  EXPECT_NE(window_->findChild<QChartView*>(QStringLiteral("chartView")), nullptr);
  EXPECT_NE(window_->findChild<QProgressBar*>(QStringLiteral("progressBar")), nullptr);

  // Input limits (design 5.3): omega [10, 200] rad/s, hal_timeout_ms [1, 50] ms, speed 1-100 %.
  EXPECT_EQ(spin_min(&widget("omegaSpin")), 10.0);
  EXPECT_EQ(spin_max(&widget("omegaSpin")), 200.0);
  EXPECT_EQ(spin_min(&widget("halTimeoutSpin")), 1.0);
  EXPECT_EQ(spin_max(&widget("halTimeoutSpin")), 50.0);
  const auto& speed = get<QSlider>("speedSlider");
  EXPECT_EQ(speed.minimum(), 1);
  EXPECT_EQ(speed.maximum(), 100);
  EXPECT_TRUE(speed.isEnabled());  // the phase B speed of manual control (studio-control)
  EXPECT_FALSE(speed.toolTip().isEmpty());

  // Base pairs: discover_base_pairs order, six profiles in config/.
  const auto& base = get<QComboBox>("baseCombo");
  const std::vector<BasePair> pairs = discover_base_pairs(kConfigDir);
  ASSERT_EQ(pairs.size(), kBaseNames.size());
  ASSERT_EQ(base.count(), static_cast<int>(kBaseNames.size()));
  for (std::size_t i = 0; i < kBaseNames.size(); ++i) {
    EXPECT_EQ(pairs[i].name, kBaseNames[i]);
    EXPECT_EQ(base.itemText(static_cast<int>(i)).toStdString(), kBaseNames[i]);
  }

  // One top-level paramTree item per catalog group.
  std::set<std::string> groups;
  for (const ParamInfo& p : param_catalog()) {
    groups.insert(p.group);
  }
  EXPECT_EQ(get<QTreeWidget>("paramTree").topLevelItemCount(), static_cast<int>(groups.size()));

  // Joint 1..7 and Task.
  const auto& joint = get<QComboBox>("jointCombo");
  ASSERT_EQ(joint.count(), 8);
  for (int j = 0; j < 7; ++j) {
    EXPECT_EQ(joint.itemText(j), QStringLiteral("Joint %1").arg(j + 1));
  }
  EXPECT_EQ(joint.itemText(7), QStringLiteral("Task"));

  // The time cursor shared with the 3D view: a QObject of class TimeCursor with timeChanged.
  bool cursor_found = false;
  for (const QObject* o : window_->findChildren<QObject*>()) {
    const QString cls = QString::fromLatin1(o->metaObject()->className());
    if (cls.endsWith(QStringLiteral("TimeCursor")) &&
        o->metaObject()->indexOfSignal("timeChanged(double)") >= 0) {
      cursor_found = true;
    }
  }
  EXPECT_TRUE(cursor_found);

  // Status bar names the work dir.
  bool work_dir_shown =
      window_->statusBar()->currentMessage().contains(QString::fromStdString(work_dir_.string()));
  for (const QLabel* label : window_->statusBar()->findChildren<QLabel*>()) {
    work_dir_shown =
        work_dir_shown || label->text().contains(QString::fromStdString(work_dir_.string()));
  }
  EXPECT_TRUE(work_dir_shown);

  // A fresh window has nothing to run against and no job in progress.
  EXPECT_EQ(run_list().count(), 0);
  EXPECT_EQ(table().rowCount(), 0);
  EXPECT_TRUE(idle());
  EXPECT_TRUE(button("runButton").isEnabled());
  EXPECT_TRUE(button("sweepButton").isEnabled());
}

// The initial omega is sqrt(kp_1) of the base pair's controller config: premium_fr3.yaml kp 2500
// -> 50, budget_xarm7.yaml kp 400 -> 20, mid_kr1205.yaml kp 1600 -> 40, ideal uses default.yaml
// kp 6400 -> 80. Matching the base is not an override.
TEST_F(StudioApp, BaseSwitchSetsOmegaFromTheBasePair) {
  const std::vector<std::pair<std::string, double>> cases = {
      {"premium_fr3", 50.0}, {"budget_xarm7", 20.0}, {"ideal", 80.0}, {"mid_kr1205", 40.0}};
  for (const auto& [name, omega] : cases) {
    ASSERT_NO_FATAL_FAILURE(select_base(name));
    EXPECT_DOUBLE_EQ(spin_value(&widget("omegaSpin")), omega) << name;
    EXPECT_TRUE(window_->studio_config().overrides.empty()) << name;
  }
  EXPECT_EQ(modal_.messages(), 0) << modal_.log();
}

TEST_F(StudioApp, BaseSwitchWithOverridesAsksToDiscard) {
  window_->set_override(pipeline::KeyOverride{"sensor.q_noise_std_rad", 1e-4});
  ASSERT_EQ(window_->studio_config().overrides.size(), 1U);
  const int before = modal_.messages();
  ASSERT_NO_FATAL_FAILURE(select_base("realistic"));
  EXPECT_EQ(modal_.messages(), before + 1) << "no confirmation asked";
  EXPECT_TRUE(window_->studio_config().overrides.empty());  // the handler accepts: discard
}

TEST_F(StudioApp, OmegaOutOfRangeIsClampedOrBlocksRunning) {
  QWidget& omega = widget("omegaSpin");
  set_spin_value(&omega, 5.0);
  EXPECT_DOUBLE_EQ(spin_value(&omega), 10.0);  // the spin box clamps to its minimum
  pump(50);
  EXPECT_TRUE(button("runButton").isEnabled());  // omega 10 is valid

  window_->set_override(pipeline::KeyOverride{"config.computed_torque.omega", 5.0});
  EXPECT_FALSE(validate(window_->studio_config()).empty());
  EXPECT_TRUE(wait_until([&] { return !button("runButton").isEnabled(); }, 2000));
  EXPECT_FALSE(button("sweepButton").isEnabled());
  // The issue is shown next to the field: its tooltip carries validate()'s line naming the key.
  EXPECT_TRUE(omega.toolTip().contains(QStringLiteral("omega"))) << omega.toolTip().toStdString();

  button("resetButton").click();
  EXPECT_TRUE(wait_until([&] { return button("runButton").isEnabled(); }, 2000)) << modal_.log();
  EXPECT_TRUE(button("sweepButton").isEnabled());
  EXPECT_TRUE(window_->studio_config().overrides.empty());
}

TEST_F(StudioApp, RunAddsOneRowWithTheStoredRunsMetrics) {
  ASSERT_NO_FATAL_FAILURE(select_base("ideal"));
  ASSERT_NO_FATAL_FAILURE(run_bench(kRunDurationS));
  ASSERT_EQ(run_list().count(), 1);
  ASSERT_EQ(table().rowCount(), 1);
  QListWidgetItem* item = run_list().item(0);
  EXPECT_TRUE((item->flags() & Qt::ItemIsUserCheckable) != 0);
  EXPECT_FALSE(item->text().isEmpty());

  ASSERT_EQ(window_->run_store().runs().size(), 1U);
  const RunEntry& entry = window_->run_store().runs().front();
  ASSERT_NE(entry.data, nullptr);
  EXPECT_EQ(entry.data->records.size(), 1000U);  // round(1.0 s * 1000 Hz) cycles, one row each
  // Inputs materialised under the work dir, never in the repository.
  EXPECT_TRUE(fs::exists(entry.inputs.config_yaml));
  EXPECT_TRUE(fs::exists(entry.inputs.imperfections_yaml));
  const std::string config_path = fs::weakly_canonical(entry.inputs.config_yaml).string();
  EXPECT_EQ(config_path.rfind(fs::weakly_canonical(work_dir_).string(), 0), 0U) << config_path;

  const TrackingMetrics m = compute_metrics(*entry.data, window_->studio_config().tolerance);
  const std::string expected =
      QString::number(m.summary.final_pos_err_m * 1e3, 'g', 4).toStdString();
  EXPECT_EQ(table().index(0, kColFinalPos).data(Qt::DisplayRole).toString().toStdString(),
            expected);

  // Ideal profile, no override: no noise line for any checked run, so the toggle is disabled
  // and its tooltip gives Series::unavailable's reason.
  check_all_runs();
  get<QComboBox>("jointCombo").setCurrentIndex(0);
  pump(100);
  const QAbstractButton& noise = button("showNoise");
  EXPECT_FALSE(noise.isEnabled());
  EXPECT_FALSE(noise.toolTip().isEmpty());
}

TEST_F(StudioApp, SweepAddsOneRowPerLevelAndOneChartSeriesPerRun) {
  ASSERT_NO_FATAL_FAILURE(select_base("ideal"));
  ASSERT_NO_FATAL_FAILURE(run_bench(kRunDurationS));
  ASSERT_NO_FATAL_FAILURE(start_sweep("sensor.q_noise_std_rad", "0, 1e-4", "1"));
  ASSERT_TRUE(wait_until([&] { return run_list().count() == 3 && idle(); }, 2 * kRunTimeoutMs))
      << "runList " << run_list().count() << ", log: " << modal_.log();
  EXPECT_EQ(table().rowCount(), 3);  // the bench run + one row per level

  check_all_runs();
  get<QComboBox>("jointCombo").setCurrentIndex(0);  // Joint 1
  for (const char* name : kToggles) {
    button(name).setChecked(std::string(name) == "showQ");
  }
  QChart* chart = get<QChartView>("chartView").chart();
  ASSERT_NE(chart, nullptr);
  EXPECT_TRUE(wait_until([&] { return chart->series().size() == 3; }, 5000))
      << "series " << chart->series().size();
  for (QAbstractSeries* s : chart->series()) {
    EXPECT_TRUE(s->useOpenGL()) << s->name().toStdString();
  }
}

// A 20 ms timer on the GUI thread never waits more than 250 ms during a 4-level sweep.
TEST_F(StudioApp, GuiStaysResponsiveDuringSweep) {
  ASSERT_NO_FATAL_FAILURE(select_base("ideal"));
  set_spin_value(&widget("durationSpin"), kRunDurationS);
  QElapsedTimer since_tick;
  qint64 max_gap_ms = 0;
  bool measuring = false;
  QTimer ticker;
  ticker.setTimerType(Qt::PreciseTimer);
  ticker.setInterval(20);
  QObject::connect(&ticker, &QTimer::timeout, &ticker, [&] {
    if (measuring) {
      max_gap_ms = std::max(max_gap_ms, since_tick.elapsed());
    }
    since_tick.restart();
  });
  ticker.start();
  ASSERT_NO_FATAL_FAILURE(start_sweep("sensor.q_noise_std_rad", "0, 1e-4, 2e-4, 3e-4", "1"));
  since_tick.restart();
  measuring = true;
  ASSERT_TRUE(wait_until([&] { return run_list().count() == 4 && idle(); }, 4 * kRunTimeoutMs))
      << "runList " << run_list().count() << ", log: " << modal_.log();
  measuring = false;
  EXPECT_EQ(table().rowCount(), 4);
  EXPECT_LT(max_gap_ms, 250);
}

TEST_F(StudioApp, CancelAfterTheFirstSweepResult) {
  ASSERT_NO_FATAL_FAILURE(select_base("ideal"));
  set_spin_value(&widget("durationSpin"), kRunDurationS);
  ASSERT_NO_FATAL_FAILURE(start_sweep("sensor.q_noise_std_rad", "0, 1e-4, 2e-4, 3e-4", "1"));
  ASSERT_TRUE(wait_until([&] { return run_list().count() >= 1; }, kRunTimeoutMs)) << modal_.log();
  button("cancelButton").click();
  ASSERT_TRUE(wait_until([&] { return idle(); }, kRunTimeoutMs));
  pump(500);  // a result still queued would land now
  EXPECT_GE(run_list().count(), 1);
  EXPECT_LE(run_list().count(), 2);
  EXPECT_LE(table().rowCount(), 2);
  const auto& progress = get<QProgressBar>("progressBar");
  EXPECT_LE(progress.value(), progress.minimum());  // reset() or back to the minimum
  EXPECT_TRUE(button("runButton").isEnabled());
}

// Tolerance edits recolour from the ErrorTrace without a new run. The realistic profile (sensor
// noise 1e-5 rad and more) keeps the final error far above 0.0001 mm = 1e-7 m; even if the spin
// box rounds 0.0001 to its decimals or clamps it to its minimum, the tolerance stays below that.
TEST_F(StudioApp, ToleranceEditRecolorsWithoutRerun) {
  ASSERT_NO_FATAL_FAILURE(select_base("realistic"));
  ASSERT_NO_FATAL_FAILURE(run_bench(kRunDurationS));
  ASSERT_EQ(table().rowCount(), 1);
  const RunEntry& entry = window_->run_store().runs().front();
  const double final_mm = entry.summary.final_pos_err_m * 1e3;
  QWidget& goal_pos = widget("goalPosSpin");
  const QModelIndex cell = table().index(0, kColFinalPos);
  EXPECT_EQ(is_red(cell.data(Qt::BackgroundRole)), final_mm > spin_value(&goal_pos))
      << "final " << final_mm << " mm, tolerance " << spin_value(&goal_pos) << " mm";

  set_spin_value(&goal_pos, 0.0001);
  EXPECT_LT(spin_value(&goal_pos), final_mm);
  EXPECT_TRUE(wait_until(
      [&] { return is_red(table().index(0, kColFinalPos).data(Qt::BackgroundRole)); }, 2000))
      << "final " << final_mm << " mm";
  EXPECT_EQ(run_list().count(), 1);
  EXPECT_EQ(window_->run_store().runs().size(), 1U);
  EXPECT_TRUE(idle());
}

TEST_F(StudioApp, ExportWritesTheRunAndSummaryCsvContracts) {
  ASSERT_NO_FATAL_FAILURE(select_base("ideal"));
  ASSERT_NO_FATAL_FAILURE(run_bench(kRunDurationS));
  check_all_runs();
  run_list().setCurrentRow(0);
  const fs::path dir = kOutput / "export";
  fs::remove_all(dir);
  fs::create_directories(dir);

  const fs::path run_csv = dir / "run.csv";
  ASSERT_TRUE(window_->export_run(QString::fromStdString(run_csv.string())));
  std::ostringstream run_header;
  pipeline::write_csv_header<7>(run_header);
  const std::string run_text = read_file(run_csv);
  EXPECT_EQ(first_line(run_text), first_line(run_header.str()));
  EXPECT_EQ(line_count(run_text), 1001U);  // header + 1000 rows (1 s at 1000 Hz)

  const fs::path summary_csv = dir / "summary.csv";
  ASSERT_TRUE(window_->export_summary(QString::fromStdString(summary_csv.string())));
  std::ostringstream summary_header;
  bench::write_summary_header(summary_header);
  const std::string summary_text = read_file(summary_csv);
  EXPECT_EQ(first_line(summary_text), first_line(summary_header.str()));
  EXPECT_EQ(line_count(summary_text), 2U);  // header + the one checked run
}

std::string canonical_text(const fs::path& p) {
  return fs::weakly_canonical(p).string();
}

// A copy of the window's configuration, which changes afterwards.
StudioConfig snapshot(const StudioConfig& config) {
  return config;
}

void expect_same_config(const StudioConfig& a, const StudioConfig& b) {
  EXPECT_EQ(a.base.name, b.base.name);
  EXPECT_EQ(canonical_text(a.base.config_yaml), canonical_text(b.base.config_yaml));
  EXPECT_EQ(canonical_text(a.base.imperfections_yaml), canonical_text(b.base.imperfections_yaml));
  ASSERT_EQ(a.overrides.size(), b.overrides.size());
  for (std::size_t i = 0; i < a.overrides.size(); ++i) {
    EXPECT_EQ(a.overrides[i].key, b.overrides[i].key) << "override " << i;
    EXPECT_TRUE(a.overrides[i].value == b.overrides[i].value) << a.overrides[i].key;
  }
  EXPECT_EQ(a.seed, b.seed);
  EXPECT_NEAR(a.tolerance.goal_pos_m, b.tolerance.goal_pos_m, 1e-15);
  EXPECT_NEAR(a.tolerance.goal_rot_rad, b.tolerance.goal_rot_rad, 1e-15);
  EXPECT_NEAR(a.tolerance.path_pos_m, b.tolerance.path_pos_m, 1e-15);
  EXPECT_NEAR(a.tolerance.path_rot_rad, b.tolerance.path_rot_rad, 1e-15);
  EXPECT_EQ(a.speed_pct, b.speed_pct);
  ASSERT_EQ(a.waypoints_yaml.has_value(), b.waypoints_yaml.has_value());
  if (a.waypoints_yaml.has_value()) {
    EXPECT_EQ(canonical_text(*a.waypoints_yaml), canonical_text(*b.waypoints_yaml));
  }
  EXPECT_EQ(a.duration_s, b.duration_s);
}

TEST_F(StudioApp, SaveAndOpenStudioFileRoundTrip) {
  ASSERT_NO_FATAL_FAILURE(select_base("ideal"));
  window_->set_override(pipeline::KeyOverride{"sensor.q_noise_std_rad", 1e-4});
  set_spin_value(&widget("omegaSpin"), 50.0);  // ideal's controller (default.yaml) has omega 80
  set_spin_value(&widget("goalPosSpin"), 0.5);
  set_spin_value(&widget("seedSpin"), 7.0);
  set_spin_value(&widget("durationSpin"), 1.5);
  pump(50);
  const StudioConfig saved = snapshot(window_->studio_config());
  ASSERT_EQ(saved.overrides.size(), 2U);
  EXPECT_EQ(saved.overrides[0].key, "sensor.q_noise_std_rad");
  EXPECT_EQ(saved.overrides[1].key, "config.computed_torque.omega");
  ASSERT_TRUE(std::holds_alternative<double>(saved.overrides[1].value));
  EXPECT_EQ(std::get<double>(saved.overrides[1].value), 50.0);
  EXPECT_NEAR(saved.tolerance.goal_pos_m, 0.5e-3, 1e-15);  // 0.5 mm
  EXPECT_EQ(saved.seed, std::optional<std::uint64_t>(7));
  EXPECT_EQ(saved.duration_s, std::optional<double>(1.5));

  const fs::path dir = kOutput / "studio_file";
  fs::remove_all(dir);
  fs::create_directories(dir);
  const fs::path file = dir / "studio.yaml";
  ASSERT_TRUE(window_->save_studio(QString::fromStdString(file.string())));
  const common::Result<StudioConfig> loaded = load_studio_file(file);
  ASSERT_TRUE(loaded.is_ok()) << loaded.reason();
  expect_same_config(loaded.value(), saved);

  // Move away from the saved state, then open the file again.
  button("resetButton").click();
  ASSERT_TRUE(wait_until([&] { return window_->studio_config().overrides.empty(); }, 2000))
      << modal_.log();
  ASSERT_NO_FATAL_FAILURE(select_base("budget_xarm7"));
  set_spin_value(&widget("goalPosSpin"), 2.0);
  ASSERT_TRUE(window_->open_studio(QString::fromStdString(file.string())));
  pump(50);
  expect_same_config(window_->studio_config(), saved);
  EXPECT_EQ(get<QComboBox>("baseCombo").currentText(), QStringLiteral("ideal"));
  EXPECT_DOUBLE_EQ(spin_value(&widget("omegaSpin")), 50.0);
  EXPECT_DOUBLE_EQ(spin_value(&widget("goalPosSpin")), 0.5);
  EXPECT_DOUBLE_EQ(spin_value(&widget("seedSpin")), 7.0);
  EXPECT_DOUBLE_EQ(spin_value(&widget("durationSpin")), 1.5);
}

// Closing during a sweep cancels and joins the worker: at most the run in progress finishes.
TEST_F(StudioApp, ClosingDuringSweepReturnsWithin30s) {
  ASSERT_NO_FATAL_FAILURE(select_base("ideal"));
  set_spin_value(&widget("durationSpin"), 2.0);
  ASSERT_NO_FATAL_FAILURE(start_sweep("sensor.q_noise_std_rad", "0, 1e-4, 2e-4, 3e-4", "1, 2, 3"));
  ASSERT_TRUE(wait_until([&] { return !idle(); }, 30000)) << modal_.log();
  QElapsedTimer clock;
  clock.start();
  window_->close();
  window_.reset();
  EXPECT_LT(clock.elapsed(), 30000);
  pump(200);  // nothing may arrive for the destroyed window
}

}  // namespace
}  // namespace cobot::studio
