// The Verification tab of cobot_studio (feature studio-verify, cobot_studio design 10 G14/G16,
// ADR-023 "Hệ quả", ADR-021), run with QT_QPA_PLATFORM=offscreen. Widgets are found by the object
// names of the brief's contract, always inside the tab (runButton, cancelButton, progressBar and
// exportButton also exist in the other tabs). Besides them, this test pins the tab class of
// src/studio/app/verify_tab.hpp:
//
//   namespace cobot::studio {
//   class VerifyTab : public QWidget {  // the tab page itself: objectName "verifyTab"
//     Q_OBJECT
//    public:
//     // The request the input widgets describe: the checked hardwareList items
//     // (<config_dir>/hardware/<file>.yaml, in list order), the checked algorithmList entries
//     // (control::kAlgorithms order), the taskCombo file (<config_dir>/tasks/<file>.yaml),
//     // runsSpin, quickCheck, tuneCheck, sensitivityCheck, jobsSpin; every other option at its
//     // bench::VerifyOptions default; out_dir empty.
//     VerifyRequest request() const;
//     // After a job that succeeded: its rows (what every result widget shows) and its output
//     // directory; empty before, while a job runs and after a cancel or an error.
//     const std::vector<bench::ComparisonRow>& rows() const;
//     const std::filesystem::path& out_dir() const;
//    public slots:
//     // runButton calls start(request()). Starts the job on the worker thread and returns true;
//     // false (nothing started) while a job runs. An empty out_dir: new_verify_dir(work_dir).
//     bool start(const VerifyRequest& request);
//     // exportButton without the folder dialog: copies out_dir()'s files into `target` (created
//     // when missing). false and nothing copied when there is no result or `target` is a
//     // non-empty directory.
//     bool export_to(const QString& target);
//   };
//   }  // namespace cobot::studio
//
// Behaviour pinned beyond the brief's wording:
// - hardwareList: one checkable item per config/hardware/*.yaml, sorted by file name, the text
//   holding the file stem. algorithmList: one checkable entry per control::kAlgorithms name in
//   table order (checkable QListWidget items or checkable buttons), the text holding the name.
//   taskCombo: the config/tasks/*.yaml files that load as a task (a waypoint file is not one),
//   the text holding the stem.
// - cancelButton is enabled while a job runs and disabled otherwise; runButton is disabled while
//   a job runs; openReportButton and exportButton are enabled only with a result.
// - The result widgets (verdictBadge, disclaimerLabel, intervalLabel, worstTable,
//   assumptionTable, tuneTable, causesList, suggestionsList, comparisonTable) sit inside the
//   `verifyResult` container. "Hidden" means not visible to the tab (QWidget::isVisibleTo).
// - worstTable column 0 holds the metric name of VerifyResult::worst, rows in that order;
//   assumptionTable has one row per VerifyResult::assumptions entry; the chosen tuneTable row has
//   a background brush on at least one cell.
// - The badge colour is what is painted: most saturated pixels of a grab of verdictBadge are
//   green (PASS), red (FAIL) or amber (MARGINAL), whether text or background carries it.
#if __has_include("verify_tab.hpp")

#include <QAbstractButton>
#include <QAbstractItemView>
#include <QAbstractSpinBox>
#include <QBrush>
#include <QColor>
#include <QComboBox>
#include <QImage>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QListWidgetItem>
#include <QPixmap>
#include <QPlainTextEdit>
#include <QProgressBar>
#include <QSpinBox>
#include <QString>
#include <QTabWidget>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QTextEdit>
#include <QVariant>
#include <QWidget>
#include <algorithm>
#include <array>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iterator>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <system_error>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/control/algorithm.hpp"
#include "cobot/studio/verify_job.hpp"

#include "compare.hpp"
#include "main_window.hpp"
#include "report.hpp"
#include "studio_app_test_util.hpp"
#include "verify.hpp"
#include "verify_tab.hpp"

namespace cobot::studio {
namespace {

namespace fs = std::filesystem;
using control::Algorithm;
using test::ModalHandler;
using test::pump;
using test::set_spin_value;
using test::spin_value;
using test::wait_until;

const fs::path kRepo = COBOT_REPO_DIR;
const fs::path kConfigDir = kRepo / "config";
const fs::path kOutput = fs::path(COBOT_TEST_OUTPUT_DIR) / "studio_verify";

#if defined(__OPTIMIZE__)
constexpr bool kOptimised = true;
#else
constexpr bool kOptimised = false;
#endif

#define COBOT_RELEASE_ONLY()                                          \
  if (!kOptimised) {                                                  \
    GTEST_SKIP() << "elastic 7-DOF closed loop: release builds only"; \
  }

constexpr int kJobTimeoutMs = 300000;

// config/hardware/*.yaml by file name; control::kAlgorithms names.
const std::vector<std::string> kModels = {"budget_xarm7", "mid_kr1205", "premium_fr3"};
const std::vector<std::string> kAlgorithmNames = {
    "computed_torque", "pd_gravity", "drive_position"};

// The metrics of VerifyResult::worst, in order (verify.cpp).
const std::vector<std::string> kWorstMetrics = {
    "final_pos_err_m", "final_rot_err_deg", "rms_joint_err_rad", "cycle_time_s"};

const char* const kInputWidgets[] = {"hardwareList",
                                     "algorithmList",
                                     "taskCombo",
                                     "runsSpin",
                                     "quickCheck",
                                     "tuneCheck",
                                     "sensitivityCheck",
                                     "jobsSpin",
                                     "runButton",
                                     "cancelButton",
                                     "phaseLabel",
                                     "progressBar",
                                     "pairCombo",
                                     "openReportButton",
                                     "exportButton",
                                     "verifyResult"};

const char* const kResultWidgets[] = {"verdictBadge",
                                      "disclaimerLabel",
                                      "intervalLabel",
                                      "worstTable",
                                      "assumptionTable",
                                      "tuneTable",
                                      "causesList",
                                      "suggestionsList",
                                      "comparisonTable"};

const char* const kTables[] = {"worstTable", "assumptionTable", "tuneTable", "comparisonTable"};

std::string read_file(const fs::path& p) {
  std::ifstream in(p, std::ios::binary);
  return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

bool has(const QString& text, const char* part) {
  return text.contains(QString::fromUtf8(part), Qt::CaseInsensitive);
}

// Every regular file under `dir`, relative to it, sorted (empty when `dir` does not exist).
std::vector<fs::path> files_under(const fs::path& dir) {
  std::vector<fs::path> out;
  std::error_code ec;
  if (!fs::exists(dir, ec)) {
    return out;
  }
  for (const fs::directory_entry& e : fs::recursive_directory_iterator(dir)) {
    if (e.is_regular_file()) {
      out.push_back(e.path().lexically_relative(dir));
    }
  }
  std::sort(out.begin(), out.end());
  return out;
}

// One entry of a checkable list: a QListWidget item or a checkable button.
struct Checkable {
  QString text;
  std::function<void(bool)> set;
};

std::vector<Checkable> checkables(QWidget& w) {
  std::vector<Checkable> out;
  if (auto* list = qobject_cast<QListWidget*>(&w)) {
    for (int i = 0; i < list->count(); ++i) {
      QListWidgetItem* item = list->item(i);
      if (!item->flags().testFlag(Qt::ItemIsUserCheckable)) {
        continue;
      }
      out.push_back(Checkable{item->text(), [item](bool on) {
                                item->setCheckState(on ? Qt::Checked : Qt::Unchecked);
                              }});
    }
    return out;
  }
  for (QAbstractButton* b : w.findChildren<QAbstractButton*>()) {
    if (b->isCheckable()) {
      out.push_back(Checkable{b->text(), [b](bool on) { b->setChecked(on); }});
    }
  }
  return out;
}

// Checks exactly the entries whose text holds one of `names`.
void check_only(QWidget& w, const std::vector<std::string>& names) {
  for (const Checkable& c : checkables(w)) {
    bool on = false;
    for (const std::string& n : names) {
      on = on || c.text.contains(QString::fromStdString(n));
    }
    c.set(on);
  }
}

enum class Hue { kNone, kRed, kAmber, kGreen, kOther };

const char* to_string(Hue h) {
  switch (h) {
    case Hue::kRed:
      return "red";
    case Hue::kAmber:
      return "amber";
    case Hue::kGreen:
      return "green";
    case Hue::kOther:
      return "other";
    case Hue::kNone:
      break;
  }
  return "none (no saturated pixel)";
}

// The hue class of most saturated pixels of a grab of `w`: red within 20 degrees of 0, amber 20
// to 65 (#ffbf00 is 45, #ff9800 is 36), green 80 to 160 (#2e7d32 is 123).
Hue painted_hue(QWidget& w) {
  const QImage image = w.grab().toImage();
  std::array<int, 5> votes{};
  for (int y = 0; y < image.height(); ++y) {
    for (int x = 0; x < image.width(); ++x) {
      const QColor c = image.pixelColor(x, y);
      if (c.hsvSaturation() < 90 || c.value() < 90 || c.hsvHue() < 0) {
        continue;
      }
      const int h = c.hsvHue();
      Hue k = Hue::kOther;
      if (h < 20 || h > 340) {
        k = Hue::kRed;
      } else if (h <= 65) {
        k = Hue::kAmber;
      } else if (h >= 80 && h <= 160) {
        k = Hue::kGreen;
      }
      ++votes.at(static_cast<std::size_t>(k));
    }
  }
  const auto best = std::max_element(votes.begin() + 1, votes.end());
  if (*best == 0) {
    return Hue::kNone;
  }
  return static_cast<Hue>(best - votes.begin());
}

class StudioVerify : public ::testing::Test {
 protected:
  void SetUp() override {
    const ::testing::TestInfo* info = ::testing::UnitTest::GetInstance()->current_test_info();
    work_dir_ = kOutput / "work" / info->name();
    fs::remove_all(work_dir_);
    fs::create_directories(work_dir_);
    window_ = std::make_unique<MainWindow>(kConfigDir, work_dir_);
    window_->show();
    auto* tabs = window_->findChild<QTabWidget*>(QStringLiteral("tabs"));
    tab_ = window_->findChild<VerifyTab*>(QStringLiteral("verifyTab"));
    ASSERT_NE(tabs, nullptr);
    ASSERT_NE(tab_, nullptr) << "no VerifyTab named verifyTab";
    tabs->setCurrentWidget(tab_);
    pump(50);
  }

  void TearDown() override { window_.reset(); }

  // The widget of the tab with this object name and type; throws (test failure) when missing.
  template <typename T>
  T& get(const char* name) const {
    T* w = tab_->findChild<T*>(QString::fromLatin1(name));
    if (w == nullptr) {
      throw std::runtime_error(std::string("no widget '") + name +
                               "' of the expected type in verifyTab");
    }
    return *w;
  }

  QWidget& widget(const char* name) const { return get<QWidget>(name); }
  QAbstractButton& button(const char* name) const { return get<QAbstractButton>(name); }
  const QTableWidget& table(const char* name) const { return get<QTableWidget>(name); }
  QString text(const char* name) const { return get<QLabel>(name).text(); }
  bool shown(const char* name) const { return widget(name).isVisibleTo(tab_); }
  bool idle() const { return !button("cancelButton").isEnabled(); }

  // Selects the taskCombo entry whose text holds `task`.
  void select_task(const std::string& task) const {
    auto& combo = get<QComboBox>("taskCombo");
    int index = -1;
    for (int i = 0; i < combo.count(); ++i) {
      if (combo.itemText(i).contains(QString::fromStdString(task))) {
        index = i;
      }
    }
    ASSERT_GE(index, 0) << task;
    combo.setCurrentIndex(index);
  }

  // The input widgets of a job: these models and algorithms, reference_move, `runs`.
  void set_inputs(const std::vector<std::string>& models,
                  const std::vector<std::string>& algorithms,
                  int runs) const {
    check_only(widget("hardwareList"), models);
    check_only(widget("algorithmList"), algorithms);
    select_task("reference_move");
    set_spin_value(&widget("runsSpin"), runs);
  }

  // Starts `request` and waits for the tab to go idle; every phaseLabel text seen on the way.
  std::vector<QString> run_job(const VerifyRequest& request) {
    std::vector<QString> phases;
    EXPECT_TRUE(tab_->start(request));
    EXPECT_FALSE(idle());
    EXPECT_FALSE(button("runButton").isEnabled());
    const bool done = wait_until(
        [&] {
          const QString p = text("phaseLabel");
          if (phases.empty() || phases.back() != p) {
            phases.push_back(p);
          }
          return idle();
        },
        kJobTimeoutMs);
    EXPECT_TRUE(done) << "job still running, log: " << modal_.log();
    pump(50);
    return phases;
  }

  // The quick job of the brief: premium_fr3 CT, levels {50}, tune seeds 1, runs 3, quick.
  VerifyRequest quick_request() const {
    set_inputs({"premium_fr3"}, {"computed_torque"}, 3);
    button("quickCheck").setChecked(true);
    VerifyRequest r = tab_->request();
    r.options.tuning.levels = {50.0};
    r.options.tuning.seeds = {1};
    return r;
  }

  // The read-only rule of the result area (owner feedback 2026-11-07: results never in input
  // widgets).
  void expect_read_only() const {
    const QWidget& result = widget("verifyResult");
    EXPECT_TRUE(result.findChildren<QLineEdit*>().isEmpty());
    EXPECT_TRUE(result.findChildren<QAbstractSpinBox*>().isEmpty());  // QSpinBox, QDoubleSpinBox
    EXPECT_TRUE(result.findChildren<QSpinBox*>().isEmpty());
    EXPECT_TRUE(result.findChildren<QDoubleSpinBox*>().isEmpty());
    for (const QTextEdit* t : result.findChildren<QTextEdit*>()) {
      EXPECT_TRUE(t->isReadOnly()) << t->objectName().toStdString();
    }
    for (const QPlainTextEdit* t : result.findChildren<QPlainTextEdit*>()) {
      EXPECT_TRUE(t->isReadOnly()) << t->objectName().toStdString();
    }
    const auto tables = result.findChildren<QTableWidget*>();
    EXPECT_GE(tables.size(), 4);
    for (const QAbstractItemView* v : result.findChildren<QAbstractItemView*>()) {
      EXPECT_EQ(v->editTriggers(), QAbstractItemView::NoEditTriggers)
          << v->objectName().toStdString();
    }
  }

  ModalHandler modal_;
  fs::path work_dir_;
  std::unique_ptr<MainWindow> window_;
  VerifyTab* tab_ = nullptr;
};

// ---- every build -------------------------------------------------------------------------------

TEST_F(StudioVerify, TabAndEveryContractWidgetExist) {
  const auto& tabs = *window_->findChild<QTabWidget*>(QStringLiteral("tabs"));
  ASSERT_EQ(tabs.count(), 4);
  EXPECT_EQ(tabs.widget(3), tab_);
  EXPECT_EQ(tabs.tabText(3), QStringLiteral("Verification"));

  for (const char* name : kInputWidgets) {
    EXPECT_NE(tab_->findChild<QWidget*>(QString::fromLatin1(name)), nullptr) << name;
  }
  const QWidget& result = widget("verifyResult");
  for (const char* name : kResultWidgets) {
    EXPECT_NE(result.findChild<QWidget*>(QString::fromLatin1(name)), nullptr)
        << name << " not inside verifyResult";
  }
  for (const char* name : {"verdictBadge", "disclaimerLabel", "intervalLabel", "phaseLabel"}) {
    EXPECT_NE(tab_->findChild<QLabel*>(QString::fromLatin1(name)), nullptr) << name;
  }
  for (const char* name : kTables) {
    EXPECT_NE(tab_->findChild<QTableWidget*>(QString::fromLatin1(name)), nullptr) << name;
  }
  EXPECT_NE(tab_->findChild<QProgressBar*>(QStringLiteral("progressBar")), nullptr);
  EXPECT_NE(tab_->findChild<QComboBox*>(QStringLiteral("pairCombo")), nullptr);

  // Defaults: 100 runs, jobs 0 (auto), tune and sensitivity on, quick off.
  EXPECT_EQ(spin_value(&widget("runsSpin")), 100.0);
  EXPECT_EQ(spin_value(&widget("jobsSpin")), 0.0);
  for (const char* name : {"quickCheck", "tuneCheck", "sensitivityCheck"}) {
    EXPECT_TRUE(button(name).isCheckable()) << name;
  }
  EXPECT_FALSE(button("quickCheck").isChecked());
  EXPECT_TRUE(button("tuneCheck").isChecked());
  EXPECT_TRUE(button("sensitivityCheck").isChecked());

  // Idle, no result yet.
  EXPECT_TRUE(idle());
  EXPECT_FALSE(button("openReportButton").isEnabled());
  EXPECT_FALSE(button("exportButton").isEnabled());
  EXPECT_TRUE(tab_->rows().empty());
  EXPECT_TRUE(tab_->out_dir().empty());
  EXPECT_FALSE(tab_->export_to(QString::fromStdString((work_dir_ / "export").string())));
  EXPECT_FALSE(fs::exists(work_dir_ / "export") && !fs::is_empty(work_dir_ / "export"));
}

TEST_F(StudioVerify, InputListsComeFromTheConfigDirAndTheRegistry) {
  auto& hardware = get<QListWidget>("hardwareList");
  ASSERT_EQ(hardware.count(), static_cast<int>(kModels.size()));
  for (std::size_t i = 0; i < kModels.size(); ++i) {
    const QListWidgetItem* item = hardware.item(static_cast<int>(i));
    EXPECT_TRUE(item->text().contains(QString::fromStdString(kModels[i])))
        << item->text().toStdString();
    EXPECT_TRUE(item->flags().testFlag(Qt::ItemIsUserCheckable)) << kModels[i];
  }
  const std::vector<Checkable> algorithms = checkables(widget("algorithmList"));
  ASSERT_EQ(algorithms.size(), control::kAlgorithms.size());
  for (std::size_t i = 0; i < algorithms.size(); ++i) {
    EXPECT_TRUE(algorithms[i].text.contains(QString::fromLatin1(control::kAlgorithms[i].name)))
        << algorithms[i].text.toStdString();
    EXPECT_EQ(std::string(control::kAlgorithms[i].name), kAlgorithmNames[i]);
  }
  // config/tasks: pick_place_3kg.yaml and reference_move.yaml are tasks; pick_place_waypoints.yaml
  // is the waypoint file of pick_place_3kg, not a task.
  const auto& tasks = get<QComboBox>("taskCombo");
  ASSERT_EQ(tasks.count(), 2);
  for (int i = 0; i < tasks.count(); ++i) {
    EXPECT_FALSE(tasks.itemText(i).contains(QStringLiteral("waypoints"))) << i;
  }
  EXPECT_TRUE(tasks.findText(QStringLiteral("reference_move"), Qt::MatchContains) >= 0);
  EXPECT_TRUE(tasks.findText(QStringLiteral("pick_place_3kg"), Qt::MatchContains) >= 0);
}

TEST_F(StudioVerify, RequestFollowsTheInputWidgets) {
  set_inputs({"premium_fr3", "budget_xarm7"}, {"computed_torque", "pd_gravity"}, 7);
  button("quickCheck").setChecked(true);
  button("tuneCheck").setChecked(false);
  button("sensitivityCheck").setChecked(false);
  set_spin_value(&widget("jobsSpin"), 2);

  const VerifyRequest r = tab_->request();
  // List order: budget_xarm7 before premium_fr3 (sorted file names).
  ASSERT_EQ(r.hardware.size(), 2U);
  EXPECT_EQ(r.hardware[0].filename(), "budget_xarm7.yaml");
  EXPECT_EQ(r.hardware[1].filename(), "premium_fr3.yaml");
  EXPECT_EQ(r.hardware[0].parent_path().filename(), "hardware");
  const std::vector<Algorithm> want = {Algorithm::kComputedTorque, Algorithm::kPdGravity};
  EXPECT_EQ(r.algorithms, want);
  EXPECT_EQ(r.task.filename(), "reference_move.yaml");
  EXPECT_EQ(r.task.parent_path().filename(), "tasks");
  EXPECT_EQ(r.options.runs, 7U);
  EXPECT_TRUE(r.options.quick);
  EXPECT_FALSE(r.options.tune);
  EXPECT_FALSE(r.options.sensitivity);
  EXPECT_EQ(r.options.jobs, 2U);
  const bench::VerifyOptions defaults;
  EXPECT_EQ(r.options.seed0, defaults.seed0);
  EXPECT_TRUE(r.options.tuning.levels.empty());
  EXPECT_TRUE(r.options.tuning.seeds.empty());
  EXPECT_TRUE(r.out_dir.empty());
}

TEST_F(StudioVerify, ResultAreaIsReadOnly) {
  expect_read_only();
  for (const char* name : {"verdictBadge", "disclaimerLabel", "intervalLabel"}) {
    EXPECT_TRUE(qobject_cast<QLabel*>(&widget(name)) != nullptr) << name;
  }
}

TEST_F(StudioVerify, AJobThatFailsOnInputShowsTheErrorAndNoResult) {
  // No listed pair: neither premium_fr3 nor mid_kr1205 lists drive_position.
  set_inputs({"premium_fr3", "mid_kr1205"}, {"drive_position"}, 1);
  const VerifyRequest r = tab_->request();
  if (tab_->start(r)) {
    ASSERT_TRUE(wait_until([&] { return idle(); }, 30000)) << modal_.log();
  }
  pump(100);
  EXPECT_TRUE(idle());
  EXPECT_TRUE(tab_->rows().empty());
  EXPECT_TRUE(tab_->out_dir().empty());
  EXPECT_FALSE(button("openReportButton").isEnabled());
  // The reason reaches the user (phaseLabel or a message box).
  EXPECT_TRUE(has(text("phaseLabel"), "drive_position") || modal_.messages() > 0)
      << text("phaseLabel").toStdString() << " | " << modal_.log();
  for (const fs::path& f : files_under(work_dir_)) {
    EXPECT_NE(f.filename(), "report.json") << f;
    EXPECT_NE(f.filename(), "comparison.json") << f;
  }
}

// ---- release builds only -----------------------------------------------------------------------

TEST_F(StudioVerify, QuickJobShowsMarginalWithItsNumbers) {
  COBOT_RELEASE_ONLY();
  const VerifyRequest r = quick_request();
  ASSERT_EQ(r.hardware.size(), 1U);
  EXPECT_EQ(r.hardware[0].filename(), "premium_fr3.yaml");
  ASSERT_EQ(r.algorithms.size(), 1U);
  EXPECT_EQ(r.algorithms[0], Algorithm::kComputedTorque);
  EXPECT_EQ(r.options.runs, 3U);
  EXPECT_TRUE(r.options.quick);

  const std::vector<QString> phases = run_job(r);
  ASSERT_EQ(tab_->rows().size(), 1U) << modal_.log();
  const bench::ComparisonRow& row = tab_->rows()[0];
  ASSERT_TRUE(row.result.has_value());
  const bench::VerifyResult& res = *row.result;

  // Progress named the phases: "tune" (1 level x 1 seed) then "Monte Carlo" (the 3 runs).
  const auto seen = [&](const char* part) {
    return std::any_of(
        phases.begin(), phases.end(), [&](const QString& p) { return has(p, part); });
  };
  EXPECT_TRUE(seen("tune"));
  EXPECT_TRUE(seen("Monte Carlo"));

  // Output in <work_dir>/verify_0001, never the repository.
  EXPECT_EQ(tab_->out_dir(), work_dir_ / "verify_0001");
  const std::vector<fs::path> want = {"report.json", "report.md", "tune.csv"};
  EXPECT_EQ(files_under(tab_->out_dir()), want);

  // premium_fr3 CT at omega 50: 3/3 passed; quick (3 runs < 100) -> never PASS; wilson(3, 3).hi =
  // 1 >= 0.95, not FAIL -> MARGINAL (tests/unit/bench_verify/test_verify_cli.cpp).
  EXPECT_EQ(res.verdict, bench::Verdict::kMarginal);
  const QString badge = text("verdictBadge");
  EXPECT_TRUE(badge.contains(QStringLiteral("MARGINAL"))) << badge.toStdString();
  EXPECT_TRUE(has(badge, "quick")) << badge.toStdString();
  EXPECT_FALSE(badge.contains(QStringLiteral("PASS"))) << badge.toStdString();
  const Hue hue = painted_hue(widget("verdictBadge"));
  EXPECT_EQ(hue, Hue::kAmber) << to_string(hue);
  EXPECT_TRUE(text("disclaimerLabel").contains(QString::fromLatin1(bench::kVerifyDisclaimer)))
      << text("disclaimerLabel").toStdString();
  EXPECT_TRUE(text("intervalLabel").contains(QStringLiteral("3/3")))
      << text("intervalLabel").toStdString();

  // The tables are filled from the rows.
  const QTableWidget& worst = table("worstTable");
  ASSERT_EQ(worst.rowCount(), 4);
  ASSERT_EQ(res.worst.size(), 4U);
  for (std::size_t i = 0; i < kWorstMetrics.size(); ++i) {
    const QTableWidgetItem* item = worst.item(static_cast<int>(i), 0);
    ASSERT_NE(item, nullptr) << i;
    EXPECT_TRUE(item->text().contains(QString::fromStdString(kWorstMetrics[i])))
        << item->text().toStdString();
    EXPECT_EQ(res.worst[i].name, kWorstMetrics[i]);
  }
  EXPECT_GT(res.assumptions.size(), 0U);
  EXPECT_EQ(table("assumptionTable").rowCount(), static_cast<int>(res.assumptions.size()));

  const QTableWidget& tune = table("tuneTable");
  ASSERT_EQ(tune.rowCount(), 1);  // one level
  ASSERT_TRUE(res.tuning.has_value() && res.tuning->chosen.has_value());
  EXPECT_EQ(*res.tuning->chosen, 50.0);
  bool highlighted = false;
  for (int c = 0; c < tune.columnCount(); ++c) {
    const QTableWidgetItem* item = tune.item(0, c);
    if (item != nullptr) {
      const QVariant bg = item->data(Qt::BackgroundRole);
      highlighted = highlighted || (bg.isValid() && (bg.userType() == QMetaType::QColor ||
                                                     bg.value<QBrush>().style() != Qt::NoBrush));
    }
  }
  EXPECT_TRUE(highlighted) << "the chosen tuneTable row has no background";

  // Not a FAIL: no causes, no suggestions; one pair: no comparison.
  EXPECT_FALSE(shown("causesList"));
  EXPECT_FALSE(shown("suggestionsList"));
  EXPECT_FALSE(shown("comparisonTable"));

  EXPECT_TRUE(button("openReportButton").isEnabled());
  EXPECT_TRUE(button("exportButton").isEnabled());
  expect_read_only();

  // Export: into a new folder, the same bytes; never over a non-empty folder.
  const fs::path target = work_dir_ / "exported";
  ASSERT_TRUE(tab_->export_to(QString::fromStdString(target.string())));
  EXPECT_EQ(files_under(target), want);
  for (const fs::path& f : want) {
    EXPECT_EQ(read_file(target / f), read_file(tab_->out_dir() / f)) << f;
  }
  const fs::path busy = work_dir_ / "busy";
  fs::create_directories(busy);
  { std::ofstream(busy / "keep.txt", std::ios::binary) << "keep"; }
  EXPECT_FALSE(tab_->export_to(QString::fromStdString(busy.string())));
  const std::vector<fs::path> untouched = {"keep.txt"};
  EXPECT_EQ(files_under(busy), untouched);
  EXPECT_EQ(read_file(busy / "keep.txt"), "keep");
}

TEST_F(StudioVerify, CancelledJobShowsCancelledAndWritesNoReport) {
  COBOT_RELEASE_ONLY();
  set_inputs({"premium_fr3"}, {"computed_torque"}, 50);
  button("quickCheck").setChecked(true);
  button("tuneCheck").setChecked(false);
  const VerifyRequest r = tab_->request();
  ASSERT_TRUE(tab_->start(r));
  EXPECT_FALSE(idle());
  pump(200);
  button("cancelButton").click();
  ASSERT_TRUE(wait_until([&] { return idle(); }, kJobTimeoutMs)) << modal_.log();
  pump(50);

  EXPECT_TRUE(has(text("phaseLabel"), "cancelled")) << text("phaseLabel").toStdString();
  EXPECT_TRUE(tab_->rows().empty());
  EXPECT_TRUE(tab_->out_dir().empty());
  EXPECT_FALSE(button("openReportButton").isEnabled());
  EXPECT_FALSE(button("exportButton").isEnabled());
  for (const fs::path& f : files_under(work_dir_)) {
    EXPECT_NE(f.begin()->string().rfind("verify_", 0), 0U) << "written after a cancel: " << f;
  }
  // Idle again: a new job can start.
  EXPECT_TRUE(button("runButton").isEnabled());
}

TEST_F(StudioVerify, TwoModelsFillTheComparisonAndThePairComboSwitchesTheResult) {
  COBOT_RELEASE_ONLY();
  // Through runButton: premium_fr3 and budget_xarm7, CT, runs 1, tuning off.
  set_inputs({"premium_fr3", "budget_xarm7"}, {"computed_torque"}, 1);
  button("tuneCheck").setChecked(false);
  ASSERT_TRUE(button("runButton").isEnabled());
  button("runButton").click();
  ASSERT_TRUE(wait_until([&] { return !tab_->rows().empty() && idle(); }, kJobTimeoutMs))
      << modal_.log();
  pump(50);

  // List order: budget_xarm7 first. By hand (test_analysis_cli.cpp): premium 1/1 quick ->
  // MARGINAL; budget CT at omega 20 holds, wilson(0, 1).hi = 0.793 < 0.95 -> FAIL.
  ASSERT_EQ(tab_->rows().size(), 2U);
  EXPECT_EQ(tab_->rows()[0].model, "budget_xarm7");
  EXPECT_EQ(tab_->rows()[1].model, "premium_fr3");
  EXPECT_EQ(files_under(tab_->out_dir()),
            (std::vector<fs::path>{"budget_xarm7__computed_torque/report.json",
                                   "budget_xarm7__computed_torque/report.md",
                                   "comparison.json",
                                   "comparison.md",
                                   "premium_fr3__computed_torque/report.json",
                                   "premium_fr3__computed_torque/report.md"}));

  ASSERT_TRUE(shown("comparisonTable"));
  const QTableWidget& comparison = table("comparisonTable");
  ASSERT_EQ(comparison.rowCount(), 2);
  const auto row_text = [&](int r) {
    QString all;
    for (int c = 0; c < comparison.columnCount(); ++c) {
      if (const QTableWidgetItem* item = comparison.item(r, c)) {
        all += item->text() + QStringLiteral(" | ");
      }
    }
    return all;
  };
  EXPECT_TRUE(row_text(0).contains(QStringLiteral("budget_xarm7"))) << row_text(0).toStdString();
  EXPECT_TRUE(has(row_text(0), "fail")) << row_text(0).toStdString();
  EXPECT_TRUE(row_text(1).contains(QStringLiteral("premium_fr3"))) << row_text(1).toStdString();
  EXPECT_TRUE(has(row_text(1), "marginal")) << row_text(1).toStdString();

  // pairCombo: one entry per pair with a result; each switches the result widgets.
  auto& pairs = get<QComboBox>("pairCombo");
  ASSERT_EQ(pairs.count(), 2);
  const auto select = [&](const char* model) {
    const int i = pairs.findText(QString::fromLatin1(model), Qt::MatchContains);
    ASSERT_GE(i, 0) << model;
    pairs.setCurrentIndex(i);
    pump(50);
  };
  select("budget_xarm7");
  EXPECT_TRUE(text("verdictBadge").contains(QStringLiteral("FAIL")))
      << text("verdictBadge").toStdString();
  const Hue red = painted_hue(widget("verdictBadge"));
  EXPECT_EQ(red, Hue::kRed) << to_string(red);
  EXPECT_TRUE(text("intervalLabel").contains(QStringLiteral("0/1")))
      << text("intervalLabel").toStdString();
  EXPECT_TRUE(shown("causesList"));
  EXPECT_TRUE(shown("suggestionsList"));
  if (auto* causes = qobject_cast<QListWidget*>(&widget("causesList"))) {
    EXPECT_GT(causes->count(), 0);
  }
  EXPECT_EQ(table("tuneTable").rowCount(), 0);  // tuning off

  select("premium_fr3");
  EXPECT_TRUE(text("verdictBadge").contains(QStringLiteral("MARGINAL")))
      << text("verdictBadge").toStdString();
  EXPECT_TRUE(text("intervalLabel").contains(QStringLiteral("1/1")))
      << text("intervalLabel").toStdString();
  EXPECT_FALSE(shown("causesList"));
  EXPECT_FALSE(shown("suggestionsList"));
  EXPECT_TRUE(shown("comparisonTable"));
  expect_read_only();
}

TEST_F(StudioVerify, APairTheModelDoesNotListIsANotListedComparisonRow) {
  COBOT_RELEASE_ONLY();
  // premium_fr3 lists computed_torque and pd_gravity, not drive_position.
  set_inputs({"premium_fr3"}, {"computed_torque", "drive_position"}, 1);
  button("tuneCheck").setChecked(false);
  run_job(tab_->request());
  ASSERT_EQ(tab_->rows().size(), 2U) << modal_.log();
  EXPECT_FALSE(tab_->rows()[1].result.has_value());

  ASSERT_TRUE(shown("comparisonTable"));
  const QTableWidget& comparison = table("comparisonTable");
  ASSERT_EQ(comparison.rowCount(), 2);
  bool not_listed = false;
  for (int c = 0; c < comparison.columnCount(); ++c) {
    if (const QTableWidgetItem* item = comparison.item(1, c)) {
      not_listed = not_listed || has(item->text(), "not listed");
    }
  }
  EXPECT_TRUE(not_listed);
  EXPECT_TRUE(text("verdictBadge").contains(QStringLiteral("MARGINAL")))
      << text("verdictBadge").toStdString();
}

}  // namespace
}  // namespace cobot::studio

#else  // before the implementation: one failing test instead of a missing-header error

#include <gtest/gtest.h>

TEST(StudioVerify, HeaderExists) {
  FAIL() << "src/studio/app/verify_tab.hpp is missing (feature studio-verify)";
}

#endif
