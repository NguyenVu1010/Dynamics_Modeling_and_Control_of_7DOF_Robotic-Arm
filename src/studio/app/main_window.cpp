#include "main_window.hpp"

#include <QAbstractTableModel>
#include <QApplication>
#include <QCheckBox>
#include <QCloseEvent>
#include <QComboBox>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFormLayout>
#include <QGraphicsLineItem>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QMessageBox>
#include <QMouseEvent>
#include <QProgressBar>
#include <QPushButton>
#include <QSlider>
#include <QSpinBox>
#include <QSplitter>
#include <QStatusBar>
#include <QTabWidget>
#include <QTableView>
#include <QTimer>
#include <QTreeWidget>
#include <QVBoxLayout>
#include <QWheelEvent>
#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <functional>
#include <limits>
#include <utility>
#include <variant>

#include "cobot/pipeline/log_record.hpp"
#include "cobot/studio/metrics.hpp"
#include "cobot/studio/param_catalog.hpp"
#include "cobot/studio/series.hpp"

#include "control_tab.hpp"
#include "robot_view.hpp"
#include "verify_tab.hpp"
#include <QtCharts/QChart>
#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis>

namespace cobot::studio {

using QtCharts::QChart;
using QtCharts::QChartView;
using QtCharts::QLineSeries;
using QtCharts::QValueAxis;
using Value = decltype(pipeline::KeyOverride::value);

namespace {

constexpr int kJoints = bench::kN;
constexpr int kColumns = 12;
constexpr double kMm = 1e-3;        // m per mm
constexpr double kBig = 1e9;        // spin box bound for catalog limits that are infinite
constexpr int kDecimals = 9;        // catalog values go down to 1e-7 (noise std)
constexpr double kSame = 1e-9;      // relative, a spin box value equal to the base value
constexpr int kMinPixels = 100;     // decimation columns when the plot is not laid out yet
constexpr int kRedrawDelayMs = 30;  // coalesces chart redraws
constexpr double kWheelPx = 0.25;   // pan pixels per wheel delta unit
constexpr int kBridgeTickMs = 50;   // BridgeClient::tick: heartbeat, samples, responses
const char* const kDefaultBridge = "cobot_controller";
const std::array<QColor, 8> kPalette = {QColor(31, 119, 180),
                                        QColor(255, 127, 14),
                                        QColor(44, 160, 44),
                                        QColor(148, 103, 189),
                                        QColor(140, 86, 75),
                                        QColor(227, 119, 194),
                                        QColor(127, 127, 127),
                                        QColor(23, 190, 207)};
const std::array<Qt::PenStyle, 5> kStyles = {
    Qt::SolidLine, Qt::DashLine, Qt::DotLine, Qt::DashDotLine, Qt::DashDotDotLine};

// The literal arrays below are source strings of MainWindow::tr() at use time; QT_TRANSLATE_NOOP
// lists them for lupdate. paramTree group names (param_catalog()):
[[maybe_unused]] constexpr std::array<const char*, 8> kGroupNames = {
    QT_TRANSLATE_NOOP("cobot::studio::MainWindow", "Sensor"),
    QT_TRANSLATE_NOOP("cobot::studio::MainWindow", "Actuator"),
    QT_TRANSLATE_NOOP("cobot::studio::MainWindow", "Friction"),
    QT_TRANSLATE_NOOP("cobot::studio::MainWindow", "Model"),
    QT_TRANSLATE_NOOP("cobot::studio::MainWindow", "Timing"),
    QT_TRANSLATE_NOOP("cobot::studio::MainWindow", "Comm"),
    QT_TRANSLATE_NOOP("cobot::studio::MainWindow", "Transmission"),
    QT_TRANSLATE_NOOP("cobot::studio::MainWindow", "Controller")};

struct Toggle {
  const char* name;
  const char* text;
  Channel channel;
};
const std::array<Toggle, 10> kToggles = {
    {{"showQRef", QT_TRANSLATE_NOOP("cobot::studio::MainWindow", "q ref"), Channel::kQRef},
     {"showQ", QT_TRANSLATE_NOOP("cobot::studio::MainWindow", "q"), Channel::kQ},
     {"showQd", QT_TRANSLATE_NOOP("cobot::studio::MainWindow", "q'"), Channel::kQd},
     {"showQdd", QT_TRANSLATE_NOOP("cobot::studio::MainWindow", "q''"), Channel::kQdd},
     {"showQTrue", QT_TRANSLATE_NOOP("cobot::studio::MainWindow", "q true"), Channel::kQTrue},
     {"showNoise", QT_TRANSLATE_NOOP("cobot::studio::MainWindow", "noise"), Channel::kNoise},
     {"showError", QT_TRANSLATE_NOOP("cobot::studio::MainWindow", "error"), Channel::kError},
     {"showTau", QT_TRANSLATE_NOOP("cobot::studio::MainWindow", "tau"), Channel::kTau},
     {"showTauMotor",
      QT_TRANSLATE_NOOP("cobot::studio::MainWindow", "tau motor"),
      Channel::kTauMotor},
     {"showTauFriction",
      QT_TRANSLATE_NOOP("cobot::studio::MainWindow", "tau friction"),
      Channel::kTauFriction}}};

bool same(const Value& a, const Value& b) {
  const auto close = [](double x, double y) {
    return std::abs(x - y) <= kSame * std::max({1.0, std::abs(x), std::abs(y)});
  };
  if (std::holds_alternative<double>(a) && std::holds_alternative<double>(b)) {
    return close(std::get<double>(a), std::get<double>(b));
  }
  if (std::holds_alternative<std::vector<double>>(a) &&
      std::holds_alternative<std::vector<double>>(b)) {
    const auto& x = std::get<std::vector<double>>(a);
    const auto& y = std::get<std::vector<double>>(b);
    return x.size() == y.size() && std::equal(x.begin(), x.end(), y.begin(), close);
  }
  return a == b;
}

QDoubleSpinBox* spin(double min, double max, int decimals, const QString& suffix) {
  auto* s = new QDoubleSpinBox;
  s->setRange(std::max(min, -kBig), std::min(max, kBig));
  s->setDecimals(decimals);
  s->setSuffix(suffix);
  return s;
}

QString num(double v) {
  return QString::number(v, 'g', 4);
}

}  // namespace

// errorTable rows, precomputed by MainWindow::refresh_runs.
class ErrorModel : public QAbstractTableModel {
 public:
  struct Row {
    std::array<QString, kColumns> text;
    std::array<bool, kColumns> over{};
  };
  using QAbstractTableModel::QAbstractTableModel;
  void set_rows(std::vector<Row> rows) {
    beginResetModel();
    rows_ = std::move(rows);
    endResetModel();
  }
  int rowCount(const QModelIndex& parent) const override {
    return parent.isValid() ? 0 : static_cast<int>(rows_.size());
  }
  int columnCount(const QModelIndex& parent) const override {
    return parent.isValid() ? 0 : kColumns;
  }
  QVariant data(const QModelIndex& index, int role) const override {
    const Row& r = rows_.at(static_cast<std::size_t>(index.row()));
    const auto c = static_cast<std::size_t>(index.column());
    if (role == Qt::DisplayRole) {
      return r.text.at(c);
    }
    if (role == Qt::BackgroundRole && r.over.at(c)) {
      return QBrush(QColor(255, 160, 160));
    }
    return {};
  }
  QVariant headerData(int section, Qt::Orientation orientation, int role) const override {
    static const std::array<const char*, kColumns> names = {
        QT_TRANSLATE_NOOP("cobot::studio::MainWindow", "run"),
        QT_TRANSLATE_NOOP("cobot::studio::MainWindow", "changes"),
        QT_TRANSLATE_NOOP("cobot::studio::MainWindow", "final pos (mm)"),
        QT_TRANSLATE_NOOP("cobot::studio::MainWindow", "final rot (deg)"),
        QT_TRANSLATE_NOOP("cobot::studio::MainWindow", "path max pos (mm)"),
        QT_TRANSLATE_NOOP("cobot::studio::MainWindow", "path max rot (deg)"),
        QT_TRANSLATE_NOOP("cobot::studio::MainWindow", "settle (s)"),
        QT_TRANSLATE_NOOP("cobot::studio::MainWindow", "stops (s)"),
        QT_TRANSLATE_NOOP("cobot::studio::MainWindow", "held"),
        QT_TRANSLATE_NOOP("cobot::studio::MainWindow", "hold time (s)"),
        QT_TRANSLATE_NOOP("cobot::studio::MainWindow", "hold reason"),
        QT_TRANSLATE_NOOP("cobot::studio::MainWindow", "hold rate")};
    if (role != Qt::DisplayRole || orientation != Qt::Horizontal) {
      return QAbstractTableModel::headerData(section, orientation, role);
    }
    return MainWindow::tr(names.at(static_cast<std::size_t>(section)));
  }

 private:
  std::vector<Row> rows_;
};

namespace {

// Rubber-band zoom (QChartView), wheel pan, Shift + drag moves the time cursor.
class ChartView : public QChartView {
 public:
  ChartView(QChart* chart, std::function<void(double)> on_cursor)
      : QChartView(chart), on_cursor_(std::move(on_cursor)) {}

 protected:
  void mousePressEvent(QMouseEvent* e) override {
    if (!cursor_event(e)) {
      QChartView::mousePressEvent(e);
    }
  }
  void mouseMoveEvent(QMouseEvent* e) override {
    if (!cursor_event(e)) {
      QChartView::mouseMoveEvent(e);
    }
  }
  void wheelEvent(QWheelEvent* e) override {
    chart()->scroll(-kWheelPx * e->angleDelta().y(), 0.0);
  }

 private:
  bool cursor_event(QMouseEvent* e) {
    if ((e->modifiers() & Qt::ShiftModifier) == 0 || (e->buttons() & Qt::LeftButton) == 0) {
      return false;
    }
    on_cursor_(chart()->mapFromScene(mapToScene(e->pos())).x());
    return true;
  }
  std::function<void(double)> on_cursor_;
};

// The sweep's parameter, levels and seeds.
class SweepDialog : public QDialog {
 public:
  explicit SweepDialog(QWidget* parent) : QDialog(parent) {
    setWindowTitle(MainWindow::tr("Sweep"));
    auto* form = new QFormLayout(this);
    param = new QComboBox;
    param->setObjectName(QStringLiteral("paramCombo"));
    for (const ParamInfo& p : param_catalog()) {
      if (p.sweepable) {
        param->addItem(QString::fromStdString(p.key));
      }
    }
    levels = new QLineEdit(QStringLiteral("0, 1e-4, 1e-3"));
    levels->setObjectName(QStringLiteral("levelsEdit"));
    seeds = new QLineEdit(QStringLiteral("1, 2, 3"));
    seeds->setObjectName(QStringLiteral("seedsEdit"));
    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    connect(buttons, &QDialogButtonBox::accepted, this, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
    form->addRow(MainWindow::tr("Parameter"), param);
    form->addRow(MainWindow::tr("Levels"), levels);
    form->addRow(MainWindow::tr("Seeds"), seeds);
    form->addRow(buttons);
  }
  QComboBox* param;
  QLineEdit* levels;
  QLineEdit* seeds;
};

// "0, 1e-4" -> {0, 1e-4}; false on an empty list or a bad number.
template <typename T, typename Parse>
bool parse_list(const QString& text, std::vector<T>& out, Parse parse) {
  out.clear();
  for (const QString& part : text.split(QLatin1Char(','), Qt::SkipEmptyParts)) {
    bool ok = false;
    out.push_back(parse(part.trimmed(), &ok));
    if (!ok) {
      return false;
    }
  }
  return !out.empty();
}

}  // namespace

// config_dir by value: the constructor signature is pinned by the smoke test.
// NOLINTNEXTLINE(performance-unnecessary-value-param)
MainWindow::MainWindow(std::filesystem::path config_dir,
                       std::filesystem::path work_dir,
                       QWidget* parent)
    : MainWindow(std::move(config_dir), std::move(work_dir), kDefaultBridge, parent) {}

// NOLINTNEXTLINE(performance-unnecessary-value-param)
MainWindow::MainWindow(std::filesystem::path config_dir,
                       std::filesystem::path work_dir,
                       const std::string& bridge_name,
                       QWidget* parent)
    : QMainWindow(parent),
      work_dir_(std::move(work_dir)),
      pairs_(discover_base_pairs(config_dir)),
      worker_(std::make_unique<StudioWorker>(work_dir_)),
      client_(std::make_unique<BridgeClient>(bridge_name)) {
  setWindowTitle(tr("cobot_studio"));
  cursor_ = new TimeCursor(this);
  model_ = new ErrorModel(this);
  redraw_ = new QTimer(this);
  redraw_->setSingleShot(true);
  redraw_->setInterval(kRedrawDelayMs);
  connect(redraw_, &QTimer::timeout, this, &MainWindow::refresh_chart);

  auto* tabs = new QTabWidget;
  tabs->setObjectName(QStringLiteral("tabs"));
  QWidget* config_tab = build_config_tab();
  tabs->addTab(config_tab, tr("Configuration"));
  control_ = new ControlTab(
      *client_, bridge_name, pairs_.empty() ? std::filesystem::path() : pairs_.front().config_yaml);
  tabs->addTab(control_, tr("Control"));
  tabs->addTab(build_tracking_tab(), tr("Tracking"));
  auto* verify_tab = new VerifyTab(config_dir, work_dir_, *worker_);
  tabs->addTab(verify_tab, tr("Verification"));
  // QStackedLayout lowers added pages and raises the current one, which reorders the pages among
  // their siblings. The Configuration tab is kept first and the Verification tab last, so a
  // window-wide lookup of a name two tabs use (resetButton; runButton, cancelButton, progressBar)
  // finds the Configuration or Tracking tab's; only the stacking order changes, and one page is
  // visible at a time.
  const auto order_pages = [config_tab, verify_tab] {
    config_tab->lower();
    verify_tab->raise();
  };
  order_pages();
  connect(tabs, &QTabWidget::currentChanged, config_tab, order_pages);
  // The two speed sliders are one setting, StudioConfig::speed_pct.
  for (QSlider* s : {speed_, control_->speed_slider()}) {
    connect(s, &QSlider::valueChanged, this, [this](int v) {
      config_.speed_pct = v;
      speed_->setValue(v);
      control_->speed_slider()->setValue(v);
    });
  }
  setCentralWidget(tabs);
  statusBar()->addWidget(
      new QLabel(tr("Work dir: %1").arg(QString::fromStdString(work_dir_.string()))));
  memory_label_ = new QLabel;
  statusBar()->addPermanentWidget(memory_label_);
  bridge_label_ = new QLabel;
  statusBar()->addPermanentWidget(bridge_label_);

  worker_->moveToThread(&thread_);
  connect(worker_.get(), &StudioWorker::result, this, &MainWindow::add_result);
  connect(worker_.get(), &StudioWorker::progress, this, [this](int done, int total) {
    progress_->setRange(0, total);
    progress_->setValue(done);
  });
  connect(worker_.get(), &StudioWorker::finished, this, &MainWindow::job_finished);
  connect(worker_.get(), &StudioWorker::live_failed, this, [this](const QString& error) {
    statusBar()->showMessage(error);
  });
  thread_.start();

  select_base(0);
  set_busy(false);
  refresh_runs();
  // On the GUI thread on purpose (ADR-022): a frozen GUI stops beating and the robot stops.
  bridge_timer_ = new QTimer(this);
  bridge_timer_->setObjectName(QStringLiteral("bridgeTimer"));
  bridge_timer_->setInterval(kBridgeTickMs);
  connect(bridge_timer_, &QTimer::timeout, this, &MainWindow::bridge_tick);
  bridge_timer_->start();
  bridge_tick();
}

// Cancels the job in progress and joins the worker: at most the current run finishes.
MainWindow::~MainWindow() {
  worker_->cancel();
  worker_->cancel_verify();
  thread_.quit();
  thread_.wait();
}

QWidget* MainWindow::build_config_tab() {
  auto* tab = new QWidget;
  tab->setObjectName(QStringLiteral("configTab"));
  auto* form = new QFormLayout;
  base_combo_ = new QComboBox;
  base_combo_->setObjectName(QStringLiteral("baseCombo"));
  for (const BasePair& p : pairs_) {
    base_combo_->addItem(QString::fromStdString(p.name));
  }
  connect(base_combo_,
          QOverload<int>::of(&QComboBox::currentIndexChanged),
          this,
          &MainWindow::select_base);
  form->addRow(tr("Base pair"), base_combo_);

  // Scalar fields with a key: equal to the base value means no override.
  const auto scalar = [this](QDoubleSpinBox* box, const char* key) {
    fields_.emplace_back(key, box);
    connect(box, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this, key](double v) {
      if (same(v, base_values_[key].value)) {
        remove_override(key);
      } else {
        set_override(pipeline::KeyOverride{key, v});
      }
    });
  };
  const ParamInfo* omega = find_param(pipeline::kOmegaKey);
  omega_ = spin(omega->min, omega->max, 1, tr(" rad/s"));
  omega_->setObjectName(QStringLiteral("omegaSpin"));
  scalar(omega_, pipeline::kOmegaKey);
  advanced_ = new QCheckBox(tr("Advanced gains (per-joint kp, kd)"));
  advanced_->setObjectName(QStringLiteral("advancedGains"));
  // Per-joint kp/kd and omega are mutually exclusive overrides.
  connect(advanced_, &QCheckBox::toggled, this, [this](bool on) {
    if (on) {
      remove_override(pipeline::kOmegaKey);
    } else {
      remove_override("config.computed_torque.kp");
      remove_override("config.computed_torque.kd");
    }
  });
  const ParamInfo* hal = find_param("config.safety.hal_timeout_ms");
  hal_timeout_ = spin(hal->min, hal->max, 1, tr(" ms"));
  hal_timeout_->setObjectName(QStringLiteral("halTimeoutSpin"));
  scalar(hal_timeout_, "config.safety.hal_timeout_ms");
  form->addRow(tr("Omega"), omega_);
  form->addRow(QString(), advanced_);
  form->addRow(tr("HAL timeout"), hal_timeout_);

  // Tolerances: goal pos (mm), goal rot (deg), path pos (mm), path rot (deg).
  const std::array<const char*, 4> tol_names = {
      "goalPosSpin", "goalRotSpin", "pathPosSpin", "pathRotSpin"};
  const std::array<QString, 4> tol_labels = {
      tr("Goal position"), tr("Goal rotation"), tr("Path position"), tr("Path rotation")};
  for (std::size_t k = 0; k < tol_.size(); ++k) {
    tol_.at(k) = spin(1e-4, 1e3, 4, k % 2 == 0 ? tr(" mm") : tr(" deg"));
    tol_.at(k)->setObjectName(QString::fromLatin1(tol_names.at(k)));
    connect(tol_.at(k),
            QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this,
            [this, k](double v) {
              Tolerance& t = config_.tolerance;
              const std::array<double*, 4> dst = {
                  &t.goal_pos_m, &t.goal_rot_rad, &t.path_pos_m, &t.path_rot_rad};
              *dst.at(k) = v * (k % 2 == 0 ? kMm : kDegToRad);
              refresh_runs();
            });
    form->addRow(tol_labels.at(k), tol_.at(k));
  }

  speed_ = new QSlider(Qt::Horizontal);
  speed_->setObjectName(QStringLiteral("speedSlider"));
  speed_->setRange(1, 100);
  speed_->setValue(static_cast<int>(std::lround(config_.speed_pct)));
  speed_->setToolTip(tr("Speed of jogs and moves in the Control tab (%)"));
  form->addRow(tr("Speed (%)"), speed_);
  waypoints_ = new QLineEdit;
  waypoints_->setObjectName(QStringLiteral("waypointsEdit"));
  connect(waypoints_, &QLineEdit::textChanged, this, [this](const QString& text) {
    config_.waypoints_yaml =
        text.isEmpty() ? std::nullopt : std::optional<fs::path>(text.toStdString());
  });
  auto* browse = new QPushButton(tr("Browse..."));
  connect(browse, &QPushButton::clicked, this, [this] {
    const QString f =
        QFileDialog::getOpenFileName(this, tr("Waypoint file"), {}, tr("YAML (*.yaml)"));
    if (!f.isEmpty()) {
      waypoints_->setText(f);
    }
  });
  auto* waypoints_row = new QHBoxLayout;
  waypoints_row->addWidget(waypoints_);
  waypoints_row->addWidget(browse);
  form->addRow(tr("Waypoints"), waypoints_row);
  duration_ = spin(0.0, 3600.0, 3, tr(" s"));
  duration_->setObjectName(QStringLiteral("durationSpin"));
  duration_->setSpecialValueText(tr("planned"));
  fields_.emplace_back("duration_s", duration_);
  connect(duration_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this](double v) {
    config_.duration_s = v > 0.0 ? std::optional<double>(v) : std::nullopt;
    refresh_validity();
  });
  seed_ = new QSpinBox;
  seed_->setObjectName(QStringLiteral("seedSpin"));
  seed_->setRange(0, std::numeric_limits<int>::max());
  seed_->setSpecialValueText(tr("profile"));
  connect(seed_, QOverload<int>::of(&QSpinBox::valueChanged), this, [this](int v) {
    config_.seed = v > 0 ? std::optional<std::uint64_t>(v) : std::nullopt;
  });
  form->addRow(tr("Duration"), duration_);
  form->addRow(tr("Seed"), seed_);

  // Buttons: studio file, cobot_bench export, back to the base pair.
  auto* buttons = new QHBoxLayout;
  const auto add_button = [&](const char* name, const QString& text, auto slot) {
    auto* b = new QPushButton(text);
    b->setObjectName(QString::fromLatin1(name));
    connect(b, &QPushButton::clicked, this, slot);
    buttons->addWidget(b);
  };
  add_button("openButton", tr("Open..."), [this] {
    const QString f =
        QFileDialog::getOpenFileName(this, tr("Studio file"), {}, tr("YAML (*.yaml)"));
    if (!f.isEmpty()) {
      open_studio(f);
    }
  });
  add_button("saveButton", tr("Save..."), [this] {
    const QString f =
        QFileDialog::getSaveFileName(this, tr("Studio file"), {}, tr("YAML (*.yaml)"));
    if (!f.isEmpty()) {
      save_studio(f);
    }
  });
  add_button("exportButton", tr("Export for cobot_bench..."), [this] {
    const QString d = QFileDialog::getExistingDirectory(this, tr("Export directory"));
    if (!d.isEmpty()) {
      const common::Result<Materialized> m = materialize(config_, d.toStdString());
      if (!m) {
        error_box(QString::fromStdString(m.reason()));
      }
    }
  });
  add_button("resetButton", tr("Reset to base"), [this] {
    config_.overrides.clear();
    refresh_fields();
    refresh_validity();
  });
  apply_gazebo_ = new QPushButton(tr("Apply to Gazebo"));
  apply_gazebo_->setObjectName(QStringLiteral("applyGazeboButton"));
  apply_gazebo_->setToolTip(
      tr("Materialise this configuration and apply it to the connected controller"));
  connect(apply_gazebo_, &QPushButton::clicked, this, &MainWindow::apply_gazebo);
  buttons->addWidget(apply_gazebo_);

  tree_ = new QTreeWidget;
  tree_->setObjectName(QStringLiteral("paramTree"));
  tree_->setColumnCount(kJoints + 2);
  QStringList header{tr("Parameter")};
  for (int j = 1; j <= kJoints; ++j) {
    header << tr("J%1").arg(j);
  }
  tree_->setHeaderLabels(header << QString());
  std::map<std::string, QTreeWidgetItem*> groups;
  for (const ParamInfo& p : param_catalog()) {
    if (groups.count(p.group) == 0) {
      groups[p.group] = new QTreeWidgetItem(tree_, {tr(p.group.c_str())});
      groups[p.group]->setExpanded(true);
    }
    if (p.key == pipeline::kOmegaKey || p.key == "config.safety.hal_timeout_ms") {
      continue;  // own fields above
    }
    ParamRow row{p.key, new QTreeWidgetItem(groups[p.group], {QString::fromStdString(p.key)}), {}};
    const std::size_t index = rows_.size();
    const auto edited = [this, index] { cell_edited(rows_.at(index)); };
    const int cells = p.kind == ParamKind::kPerJoint ? kJoints : 1;
    for (int c = 0; c < cells; ++c) {
      QWidget* w = nullptr;
      if (p.kind == ParamKind::kBool) {
        auto* box = new QCheckBox;
        connect(box, &QCheckBox::toggled, this, edited);
        w = box;
      } else if (p.kind == ParamKind::kChoice) {
        auto* box = new QComboBox;
        for (const std::string& choice : p.choices) {
          box->addItem(QString::fromStdString(choice));
        }
        connect(box, QOverload<int>::of(&QComboBox::currentIndexChanged), this, edited);
        w = box;
      } else {
        auto* box =
            spin(p.min, p.max, kDecimals, QStringLiteral(" ") + QString::fromStdString(p.unit));
        connect(box, &QDoubleSpinBox::editingFinished, this, edited);
        w = box;
      }
      tree_->setItemWidget(row.item, c + 1, w);
      row.cells.push_back(w);
      fields_.emplace_back(p.key, w);
    }
    auto* reset = new QPushButton(tr("reset"));
    connect(reset, &QPushButton::clicked, this, [this, key = p.key] { remove_override(key); });
    tree_->setItemWidget(row.item, kJoints + 1, reset);
    rows_.push_back(std::move(row));
  }

  auto* layout = new QVBoxLayout(tab);
  layout->addLayout(form);
  layout->addLayout(buttons);
  layout->addWidget(tree_, 1);
  return tab;
}

QWidget* MainWindow::build_tracking_tab() {
  auto* tab = new QWidget;
  tab->setObjectName(QStringLiteral("trackingTab"));
  run_list_ = new QListWidget;
  run_list_->setObjectName(QStringLiteral("runList"));
  connect(run_list_, &QListWidget::itemChanged, this, [this] { redraw_->start(); });
  connect(run_list_, &QListWidget::currentItemChanged, this, &MainWindow::refresh_view);
  auto* remove = new QPushButton(tr("Remove"));
  auto* clear = new QPushButton(tr("Clear"));
  connect(remove, &QPushButton::clicked, this, [this] {
    if (QListWidgetItem* item = run_list_->currentItem()) {
      store_.remove(item->data(Qt::UserRole).toULongLong());
      delete item;
      refresh_runs();
    }
  });
  connect(clear, &QPushButton::clicked, this, [this] {
    store_.clear();
    run_list_->clear();
    refresh_runs();
  });

  auto* chart = new QChart;
  x_axis_ = new QValueAxis;
  y_axis_ = new QValueAxis;
  x_axis_->setTitleText(tr("t (s)"));
  chart->addAxis(x_axis_, Qt::AlignBottom);
  chart->addAxis(y_axis_, Qt::AlignLeft);
  chart_view_ = new ChartView(chart, [this](double px) {
    const QRectF plot = chart_view_->chart()->plotArea();
    const double span = x_axis_->max() - x_axis_->min();
    if (plot.width() > 0.0) {
      cursor_->set_time_s(x_axis_->min() + (px - plot.left()) / plot.width() * span);
    }
  });
  chart_view_->setObjectName(QStringLiteral("chartView"));
  // The arm's URDF is the one of the controller config (the same for every base pair).
  robot_view_ =
      new RobotView(pairs_.empty() ? std::filesystem::path() : pairs_.front().config_yaml, cursor_);
  robot_view_->setObjectName(QStringLiteral("robotView"));
  chart_view_->setRubberBand(QChartView::RectangleRubberBand);
  cursor_line_ = new QGraphicsLineItem(chart);
  cursor_line_->setPen(QPen(Qt::red, 1.0));
  connect(cursor_, &TimeCursor::timeChanged, this, &MainWindow::draw_cursor);
  connect(chart, &QChart::plotAreaChanged, this, &MainWindow::draw_cursor);
  for (QValueAxis* axis : {x_axis_, y_axis_}) {
    connect(axis, &QValueAxis::rangeChanged, this, [this] {
      if (!drawing_) {
        zoomed_ = true;  // rubber band or wheel: redecimate over the new range
        redraw_->start();
      }
    });
  }

  joint_combo_ = new QComboBox;
  joint_combo_->setObjectName(QStringLiteral("jointCombo"));
  for (int j = 1; j <= kJoints; ++j) {
    joint_combo_->addItem(tr("Joint %1").arg(j));
  }
  joint_combo_->addItem(tr("Task"));
  connect(joint_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int i) {
    if (i >= kJoints) {
      error_toggle_->setChecked(true);  // the only series of the Task view
    }
    redraw_->start();
  });
  auto* chart_bar = new QHBoxLayout;
  chart_bar->addWidget(joint_combo_);
  for (const Toggle& t : kToggles) {
    auto* box = new QCheckBox(tr(t.text));
    box->setObjectName(QString::fromLatin1(t.name));
    box->setChecked(t.channel == Channel::kQRef || t.channel == Channel::kQ);
    connect(box, &QCheckBox::toggled, this, [this] { redraw_->start(); });
    if (t.channel == Channel::kError) {
      error_toggle_ = box;
    }
    chart_bar->addWidget(box);
    toggles_.push_back(box);
  }
  auto* reset_zoom = new QPushButton(tr("Reset zoom"));
  connect(reset_zoom, &QPushButton::clicked, this, [this] {
    zoomed_ = false;
    refresh_chart();
  });
  chart_bar->addWidget(reset_zoom);

  auto* table = new QTableView;
  table->setObjectName(QStringLiteral("errorTable"));
  table->setModel(model_);
  table->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);

  auto* jobs = new QHBoxLayout;
  const auto add_button = [&](QPushButton*& b, const char* name, const QString& text, auto slot) {
    b = new QPushButton(text);
    b->setObjectName(QString::fromLatin1(name));
    connect(b, &QPushButton::clicked, this, slot);
    jobs->addWidget(b);
  };
  QPushButton* export_run_button = nullptr;
  QPushButton* export_summary_button = nullptr;
  add_button(run_button_, "runButton", tr("Run"), [this] { start_job(false); });
  add_button(sweep_button_, "sweepButton", tr("Sweep..."), [this] { start_job(true); });
  add_button(cancel_button_, "cancelButton", tr("Cancel"), [this] { worker_->cancel(); });
  progress_ = new QProgressBar;
  progress_->setObjectName(QStringLiteral("progressBar"));
  jobs->addWidget(progress_, 1);
  add_button(export_run_button, "exportRunButton", tr("Export run CSV..."), [this] {
    const QString f = QFileDialog::getSaveFileName(this, tr("Run CSV"), {}, tr("CSV (*.csv)"));
    if (!f.isEmpty() && !export_run(f)) {
      error_box(tr("Cannot export the selected run to %1").arg(f));
    }
  });
  add_button(export_summary_button, "exportSummaryButton", tr("Export summary CSV..."), [this] {
    const QString f = QFileDialog::getSaveFileName(this, tr("Summary CSV"), {}, tr("CSV (*.csv)"));
    if (!f.isEmpty() && !export_summary(f)) {
      error_box(tr("Cannot write %1").arg(f));
    }
  });
  connect(run_list_,
          &QListWidget::currentItemChanged,
          this,
          [this, export_run_button](QListWidgetItem* item) {
            const RunEntry* e =
                item == nullptr ? nullptr : store_.find(item->data(Qt::UserRole).toULongLong());
            export_run_button->setEnabled(e != nullptr && e->data != nullptr);
          });
  export_run_button->setEnabled(false);

  auto* left = new QVBoxLayout;
  left->addWidget(run_list_, 1);
  auto* list_buttons = new QHBoxLayout;
  list_buttons->addWidget(remove);
  list_buttons->addWidget(clear);
  left->addLayout(list_buttons);
  auto* right = new QVBoxLayout;
  right->addLayout(chart_bar);
  auto* views = new QSplitter;
  views->addWidget(chart_view_);
  views->addWidget(robot_view_);
  right->addWidget(views, 2);
  right->addWidget(table, 1);
  right->addLayout(jobs);
  auto* layout = new QHBoxLayout(tab);
  layout->addLayout(left, 1);
  layout->addLayout(right, 4);
  return tab;
}

void MainWindow::select_base(int index) {
  if (index < 0 || static_cast<std::size_t>(index) >= pairs_.size()) {
    return;
  }
  const BasePair& pair = pairs_.at(static_cast<std::size_t>(index));
  if (!config_.overrides.empty() && pair.name != config_.base.name &&
      QMessageBox::question(this,
                            tr("Discard changes"),
                            tr("Switching the base pair discards %n change(s).",
                               "",
                               static_cast<int>(config_.overrides.size()))) != QMessageBox::Yes) {
    refresh_fields();  // back to the current base in the combo
    return;
  }
  config_.base = pair;
  config_.overrides.clear();
  load_base_values();
  refresh_fields();
  refresh_validity();
}

void MainWindow::load_base_values() {
  base_values_ = base_values(config_.base);
}

void MainWindow::set_override(const pipeline::KeyOverride& override_value) {
  auto it = std::find_if(
      config_.overrides.begin(), config_.overrides.end(), [&](const pipeline::KeyOverride& o) {
        return o.key == override_value.key;
      });
  if (it == config_.overrides.end()) {
    config_.overrides.push_back(override_value);
  } else {
    *it = override_value;
  }
  refresh_fields();
  refresh_validity();
}

void MainWindow::remove_override(const std::string& key) {
  auto& o = config_.overrides;
  o.erase(std::remove_if(o.begin(), o.end(), [&](const auto& x) { return x.key == key; }), o.end());
  refresh_fields();
  refresh_validity();
}

void MainWindow::cell_edited(const ParamRow& row) {
  const ParamInfo* p = find_param(row.key);
  Value v;
  if (p->kind == ParamKind::kBool) {
    v = qobject_cast<QCheckBox*>(row.cells.front())->isChecked();
  } else if (p->kind == ParamKind::kChoice) {
    v = qobject_cast<QComboBox*>(row.cells.front())->currentText().toStdString();
  } else if (p->kind == ParamKind::kNumber) {
    v = qobject_cast<QDoubleSpinBox*>(row.cells.front())->value();
  } else {
    // As many entries as the base list (payload_offset_m has three).
    const auto* base = std::get_if<std::vector<double>>(&base_values_[row.key].value);
    std::vector<double> list;
    for (std::size_t c = 0; c < row.cells.size() && (base == nullptr || c < base->size()); ++c) {
      list.push_back(qobject_cast<QDoubleSpinBox*>(row.cells.at(c))->value());
    }
    v = list;
  }
  if (same(v, base_values_[row.key].value)) {
    remove_override(row.key);
  } else {
    set_override(pipeline::KeyOverride{row.key, v});
  }
}

void MainWindow::refresh_fields() {
  const auto value_of = [this](const std::string& key) -> Value {
    for (const pipeline::KeyOverride& o : config_.overrides) {
      if (o.key == key) {
        return o.value;
      }
    }
    return base_values_[key].value;
  };
  const auto overridden = [this](const std::string& key) {
    return std::any_of(config_.overrides.begin(),
                       config_.overrides.end(),
                       [&](const pipeline::KeyOverride& o) { return o.key == key; });
  };
  const auto set_double = [](QDoubleSpinBox* box, const Value& v) {
    const QSignalBlocker block(box);
    box->setValue(std::holds_alternative<double>(v) ? std::get<double>(v) : 0.0);
  };
  {
    const QSignalBlocker block(base_combo_);
    base_combo_->setCurrentIndex(base_combo_->findText(QString::fromStdString(config_.base.name)));
  }
  set_double(omega_, value_of(pipeline::kOmegaKey));
  set_double(hal_timeout_, value_of("config.safety.hal_timeout_ms"));
  const bool advanced = overridden("config.computed_torque.kp") ||
                        overridden("config.computed_torque.kd") || advanced_->isChecked();
  {
    const QSignalBlocker block(advanced_);
    advanced_->setChecked(advanced && !overridden(pipeline::kOmegaKey));
  }
  omega_->setEnabled(!advanced_->isChecked());
  const Tolerance& t = config_.tolerance;
  const std::array<double, 4> tol = {t.goal_pos_m / kMm,
                                     t.goal_rot_rad / kDegToRad,
                                     t.path_pos_m / kMm,
                                     t.path_rot_rad / kDegToRad};
  for (std::size_t k = 0; k < tol_.size(); ++k) {
    set_double(tol_.at(k), tol.at(k));
  }
  {
    const QSignalBlocker b1(waypoints_);
    const QSignalBlocker b2(seed_);
    waypoints_->setText(config_.waypoints_yaml
                            ? QString::fromStdString(config_.waypoints_yaml->string())
                            : QString());
    // A studio file seed above the spin box range shows clamped; the tooltip has the true value
    // and config_.seed keeps it until the field is edited.
    const std::uint64_t seed = config_.seed.value_or(0);
    const auto max = static_cast<std::uint64_t>(seed_->maximum());
    seed_->setValue(static_cast<int>(std::min(seed, max)));
    seed_->setToolTip(
        seed > max ? tr("Seed %1 (above the field's range)").arg(static_cast<qulonglong>(seed))
                   : QString());
  }
  set_double(duration_, config_.duration_s.value_or(0.0));
  speed_->setValue(static_cast<int>(std::lround(config_.speed_pct)));

  QFont bold = tree_->font();
  bold.setBold(true);
  for (const ParamRow& row : rows_) {
    const Value v = value_of(row.key);
    const bool changed = overridden(row.key);
    const bool gain = row.key.rfind("config.computed_torque.", 0) == 0;
    row.item->setFont(0, changed ? bold : tree_->font());
    row.item->setBackground(0, changed ? QBrush(QColor(255, 236, 179)) : QBrush());
    const auto* list = std::get_if<std::vector<double>>(&v);
    for (std::size_t c = 0; c < row.cells.size(); ++c) {
      QWidget* w = row.cells.at(c);
      const QSignalBlocker block(w);
      w->setFont(changed ? bold : tree_->font());
      w->setEnabled((!gain || advanced_->isChecked()) &&
                    !(greyed_ && !find_param(row.key)->gazebo_supported));
      if (auto* box = qobject_cast<QCheckBox*>(w)) {
        box->setChecked(std::holds_alternative<bool>(v) && std::get<bool>(v));
      } else if (auto* combo = qobject_cast<QComboBox*>(w)) {
        combo->setCurrentText(std::holds_alternative<std::string>(v)
                                  ? QString::fromStdString(std::get<std::string>(v))
                                  : QString());
      } else if (auto* box = qobject_cast<QDoubleSpinBox*>(w)) {
        w->setVisible(list == nullptr || c < list->size());
        box->setValue(list != nullptr
                          ? (c < list->size() ? list->at(c) : 0.0)
                          : (std::holds_alternative<double>(v) ? std::get<double>(v) : 0.0));
      }
    }
  }
}

void MainWindow::refresh_validity() {
  const std::vector<std::string> issues = validate(config_);
  valid_ = issues.empty();
  std::map<QWidget*, QString> marks;
  for (const std::string& line : issues) {
    for (const auto& [key, w] : fields_) {
      if (line.find(key) != std::string::npos) {
        marks[w] +=
            (marks[w].isEmpty() ? QString() : QStringLiteral("\n")) + QString::fromStdString(line);
      }
    }
  }
  for (const auto& field : fields_) {
    QWidget* w = field.second;
    const auto it = marks.find(w);
    w->setToolTip(it == marks.end() ? QString() : it->second);
    w->setStyleSheet(it == marks.end() ? QString() : QStringLiteral("border: 1px solid red;"));
  }
  if (!issues.empty() && marks.empty()) {
    statusBar()->showMessage(QString::fromStdString(issues.front()));
  }
  set_busy(busy_);
}

void MainWindow::set_busy(bool busy) {
  busy_ = busy;
  run_button_->setEnabled(!busy && valid_);
  sweep_button_->setEnabled(!busy && valid_);
  cancel_button_->setEnabled(busy);
  // run_once has no cancel hook: a single run always finishes.
  cancel_button_->setToolTip(
      busy && !sweeping_ ? tr("A single run finishes before cancelling takes effect") : QString());
}

void MainWindow::start_job(bool sweep) {
  if (busy_) {
    return;
  }
  StudioWorker* w = worker_.get();
  if (!sweep) {
    w->arm();
    QMetaObject::invokeMethod(w, [w, c = config_] { w->run(c); }, Qt::QueuedConnection);
    progress_->setRange(0, 0);
    sweeping_ = false;
    set_busy(true);
    return;
  }
  SweepDialog dialog(this);
  if (dialog.exec() != QDialog::Accepted) {
    return;
  }
  SweepRequest request{config_, dialog.param->currentText().toStdString(), {}, {}};
  const bool levels_ok = parse_list(dialog.levels->text(),
                                    request.levels,
                                    [](const QString& s, bool* ok) { return s.toDouble(ok); });
  const bool seeds_ok = parse_list(dialog.seeds->text(),
                                   request.seeds,
                                   [](const QString& s, bool* ok) { return s.toULongLong(ok); });
  if (!levels_ok || !seeds_ok) {
    error_box(tr("Levels and seeds are comma-separated numbers."));
    return;
  }
  ++sweeps_;
  w->arm();
  QMetaObject::invokeMethod(w, [w, request] { w->sweep(request); }, Qt::QueuedConnection);
  progress_->setRange(0, static_cast<int>(request.levels.size() * request.seeds.size()));
  progress_->setValue(0);
  sweeping_ = true;
  set_busy(true);
}

void MainWindow::add_result(const EntryHandle& handle) {
  const bool swept = handle.entry->kind == RunKind::kSweep;
  const QString label = QString::fromStdString(handle.entry->label);
  const std::uint64_t id = store_.add(std::move(*handle.entry));
  if (swept) {
    sweep_batch_[id] = sweeps_;
  }
  auto* item = new QListWidgetItem(label);
  item->setData(Qt::UserRole, QVariant::fromValue<qulonglong>(id));
  item->setData(Qt::DecorationRole, kPalette.at(id % kPalette.size()));
  item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
  item->setCheckState(Qt::Checked);
  run_list_->addItem(item);
  refresh_runs();
}

void MainWindow::job_finished(const QString& error) {
  set_busy(false);
  progress_->setRange(0, 1);
  progress_->reset();
  if (!error.isEmpty()) {
    error_box(error);
  }
}

void MainWindow::error_box(const QString& text) {
  QMessageBox::warning(this, tr("cobot_studio"), text);
}

void MainWindow::refresh_runs() {
  using Row = ErrorModel::Row;
  const Tolerance& tol = config_.tolerance;
  const auto fill = [&tol](Row& r, const TrackingMetrics& m) {
    const bench::RunSummary& s = m.summary;
    const std::array<std::pair<double, double>, 4> errors = {
        {{s.final_pos_err_m / kMm, tol.goal_pos_m / kMm},
         {s.final_rot_err_rad / kDegToRad, tol.goal_rot_rad / kDegToRad},
         {m.path_pos_err_max_m / kMm, tol.path_pos_m / kMm},
         {m.path_rot_err_max_rad / kDegToRad, tol.path_rot_rad / kDegToRad}}};
    for (std::size_t k = 0; k < errors.size(); ++k) {
      r.text.at(k + 2) = num(errors.at(k).first);
      r.over.at(k + 2) = !(errors.at(k).first <= errors.at(k).second);
    }
    r.text[6] = m.settle_time_s ? num(*m.settle_time_s) : tr("not reached");
    r.over[6] = !m.settle_time_s;
    double worst = 0.0;
    for (const StopMetrics& stop : m.stops) {
      r.over[7] = r.over[7] || !stop.time_to_enter_s;
      worst = std::max(worst, stop.time_to_enter_s.value_or(0.0));
    }
    r.text[7] = m.stops.empty() ? QString() : r.over[7] ? tr("not entered") : num(worst);
    r.text[8] = s.held ? tr("yes") : tr("no");
    r.over[8] = s.held;
    r.text[9] = s.held ? num(s.hold_time_s) : QString();
    r.text[10] = s.held ? QString::fromLatin1(pipeline::to_string(s.hold_reason)) : QString();
  };
  // One row per bench run, one per sweep level (worst case over its seeds), in run order.
  using LevelKey = std::pair<int, double>;  // sweep number, level
  std::vector<std::variant<const RunEntry*, LevelKey>> order;
  std::map<LevelKey, std::vector<const RunEntry*>> levels;
  for (const RunEntry& e : store_.runs()) {
    if (e.kind != RunKind::kSweep || !e.sweep) {
      order.emplace_back(&e);
      continue;
    }
    const LevelKey key{sweep_batch_[e.id], e.sweep->level};
    if (levels[key].empty()) {
      order.emplace_back(key);
    }
    levels[key].push_back(&e);
  }
  std::vector<Row> rows(order.size());
  for (std::size_t i = 0; i < order.size(); ++i) {
    Row& r = rows.at(i);
    if (const auto* const* e = std::get_if<const RunEntry*>(&order[i])) {
      r.text[0] = QString::fromStdString((*e)->label);
      r.text[1] = QString::fromStdString((*e)->diff);
      fill(r, compute_metrics(*(*e)->errors, (*e)->meta, (*e)->summary, tol));
      continue;
    }
    const std::vector<const RunEntry*>& same_level = levels[std::get<LevelKey>(order[i])];
    const LevelRow level = worst_case(same_level, tol);
    r.text[0] =
        QStringLiteral("%1 = %2").arg(QString::fromStdString(level.param), num(level.level));
    r.text[1] = QString::fromStdString(same_level.front()->diff);
    fill(r, level.worst);
    r.text[8] = QStringLiteral("%1/%2").arg(level.held).arg(level.seeds);
    // Display ratio of LevelRow::held over LevelRow::seeds (both from worst_case).
    r.text[11] =
        num(level.seeds == 0 ? 0.0
                             : static_cast<double>(level.held) / static_cast<double>(level.seeds));
  }
  model_->set_rows(std::move(rows));
  memory_label_->setText(
      tr("Stored runs: %1 MB").arg(num(static_cast<double>(store_.memory_bytes()) / 1e6)));
  redraw_->start();
}

void MainWindow::refresh_chart() {
  struct Line {
    QColor color;
    Qt::PenStyle style;
    QString name;
    Series series;
  };
  const int index = joint_combo_->currentIndex();
  const bool task = index >= kJoints;
  std::vector<Line> lines;
  bool any_checked = false;
  for (std::size_t k = 0; k < kToggles.size(); ++k) {
    QString reason;
    bool available = false;
    for (int i = 0; i < run_list_->count(); ++i) {
      const QListWidgetItem* item = run_list_->item(i);
      const RunEntry* e = store_.find(item->data(Qt::UserRole).toULongLong());
      if (item->checkState() != Qt::Checked || e == nullptr || e->data == nullptr) {
        continue;
      }
      any_checked = true;
      // The Task view plots the task position error only, behind the error toggle.
      Series s;
      if (task && kToggles.at(k).channel != Channel::kError) {
        s.unavailable = tr("the task view plots the position error (error toggle)").toStdString();
      } else {
        const Channel ch = task ? Channel::kTaskPosErr : kToggles.at(k).channel;
        s = extract(*e->data, ch, task ? -1 : index, noise_source_of(e->config));
      }
      if (!s.unavailable.empty()) {
        reason = QString::fromStdString(s.unavailable);
        continue;
      }
      available = true;
      if (toggles_.at(k)->isChecked()) {
        lines.push_back({item->data(Qt::DecorationRole).value<QColor>(),
                         kStyles.at(k % kStyles.size()),
                         item->text() + QStringLiteral(" ") + toggles_.at(k)->text(),
                         std::move(s)});
      }
    }
    toggles_.at(k)->setEnabled(available || !any_checked);
    toggles_.at(k)->setToolTip(available || !any_checked ? QString() : reason);
  }

  QChart* chart = chart_view_->chart();
  drawing_ = true;
  chart->removeAllSeries();
  double t0 = x_axis_->min();
  double t1 = x_axis_->max();
  if (!zoomed_) {
    t0 = std::numeric_limits<double>::infinity();
    t1 = -t0;
    for (const Line& l : lines) {
      if (!l.series.t_s.empty()) {
        t0 = std::min(t0, l.series.t_s.front());
        t1 = std::max(t1, l.series.t_s.back());
      }
    }
  }
  const int pixels = std::max(kMinPixels, static_cast<int>(chart->plotArea().width()));
  double y0 = std::numeric_limits<double>::infinity();
  double y1 = -y0;
  for (const Line& l : lines) {
    auto* series = new QLineSeries;
    series->setUseOpenGL(true);
    series->setName(l.name);
    series->setPen(QPen(l.color, 1.5, l.style));
    QVector<QPointF> points;
    for (const auto& [t, y] : decimate_minmax(l.series.t_s, l.series.y, t0, t1, pixels)) {
      points.append(QPointF(t, y));
      if (std::isfinite(y)) {
        y0 = std::min(y0, y);
        y1 = std::max(y1, y);
      }
    }
    series->replace(points);
    chart->addSeries(series);
    series->attachAxis(x_axis_);
    series->attachAxis(y_axis_);
  }
  if (!zoomed_ && t1 > t0) {
    x_axis_->setRange(t0, t1);
    const double pad = y1 > y0 ? 0.05 * (y1 - y0) : 1.0;
    if (y1 >= y0) {
      y_axis_->setRange(y0 - pad, y1 + pad);
    }
  }
  drawing_ = false;
  draw_cursor();
  refresh_view();
}

void MainWindow::refresh_view() {
  std::vector<RobotView::DrawnRun> runs;
  for (int i = 0; i < run_list_->count(); ++i) {
    const QListWidgetItem* item = run_list_->item(i);
    const RunEntry* e = store_.find(item->data(Qt::UserRole).toULongLong());
    if (item->checkState() == Qt::Checked && e != nullptr && e->data != nullptr &&
        !e->data->records.empty()) {
      runs.push_back({e->id, e->data, item->data(Qt::DecorationRole).value<QColor>()});
    }
  }
  const QListWidgetItem* current = run_list_->currentItem();
  robot_view_->set_runs(std::move(runs),
                        current == nullptr ? 0 : current->data(Qt::UserRole).toULongLong(),
                        config_.tolerance.goal_pos_m);
}

void MainWindow::draw_cursor() {
  const QRectF plot = chart_view_->chart()->plotArea();
  const double span = x_axis_->max() - x_axis_->min();
  const double t = cursor_->time_s();
  const bool inside = span > 0.0 && t >= x_axis_->min() && t <= x_axis_->max();
  cursor_line_->setVisible(inside);
  if (inside) {
    const double x = plot.left() + (t - x_axis_->min()) / span * plot.width();
    cursor_line_->setLine(x, plot.top(), x, plot.bottom());
  }
}

bool MainWindow::save_studio(const QString& path) {
  const common::Status s = save_studio_file(config_, path.toStdString());
  if (!s) {
    error_box(QString::fromStdString(s.reason()));
  }
  return static_cast<bool>(s);
}

bool MainWindow::open_studio(const QString& path) {
  common::Result<StudioConfig> c = load_studio_file(path.toStdString());
  if (!c) {
    error_box(QString::fromStdString(c.reason()));
    return false;
  }
  config_ = std::move(c.value());  // no discard question: the file replaces everything
  load_base_values();
  refresh_fields();
  refresh_validity();
  refresh_runs();
  return true;
}

// Synchronous on the GUI thread, bounded by one run: a 12 s run (12 000 rows, 27 MB) writes in
// about 0.2 s, shown with a wait cursor. The Summary CSV is one line per run (< 1 ms).
bool MainWindow::export_run(const QString& csv_path) {
  const QListWidgetItem* item = run_list_->currentItem();
  const RunEntry* e =
      item == nullptr ? nullptr : store_.find(item->data(Qt::UserRole).toULongLong());
  if (e == nullptr || e->data == nullptr) {
    return false;
  }
  QApplication::setOverrideCursor(Qt::WaitCursor);
  const bool ok = static_cast<bool>(bench::write_run_files(*e->data, csv_path.toStdString()));
  QApplication::restoreOverrideCursor();
  return ok;
}

bool MainWindow::export_summary(const QString& csv_path) {
  std::ofstream out(csv_path.toStdString(), std::ios::binary);
  if (!out) {
    return false;
  }
  bench::write_summary_header(out);
  for (int i = 0; i < run_list_->count(); ++i) {
    const QListWidgetItem* item = run_list_->item(i);
    const RunEntry* e = store_.find(item->data(Qt::UserRole).toULongLong());
    if (item->checkState() == Qt::Checked && e != nullptr) {
      const bench::SummaryRow row{static_cast<std::size_t>(e->id),
                                  e->sweep ? e->sweep->param : "",
                                  e->sweep ? e->sweep->level : 0.0,
                                  e->meta.seed,
                                  e->summary};
      bench::write_summary_row(out, row);
    }
  }
  return static_cast<bool>(out);
}

void MainWindow::closeEvent(QCloseEvent* event) {
  client_->release();
  bridge_timer_->stop();
  QMainWindow::closeEvent(event);
}

void MainWindow::bridge_tick() {
  client_->tick(bridge::monotonic_ns());
  control_->refresh();
  const LiveState st = client_->state();
  const std::optional<bridge::SessionInfo<bench::kN>> session = client_->session();
  for (SegmentReady& s : client_->take_segments()) {
    if (applied_) {
      s.info.config = *applied_;
    }
    if (session) {
      StudioWorker* w = worker_.get();
      QMetaObject::invokeMethod(
          w, [w, s, session = *session] { w->load_live(s, session); }, Qt::QueuedConnection);
    }
  }
  if (!st.connected) {
    apply_id_ = 0;  // its answer cannot come any more
  }
  if (apply_id_ != 0) {
    if (const std::optional<bridge::Response> r = client_->response(apply_id_)) {
      if (r->accepted != 0) {
        applied_ = applying_;
      }
      apply_id_ = 0;
    }
  }
  apply_gazebo_->setEnabled(st.connected && st.mine && st.mode == pipeline::LoopMode::kStandby &&
                            valid_ && apply_id_ == 0);
  bridge_label_->setText(st.connected ? tr("Bridge: connected") : tr("Bridge: not connected"));
  // Rows the Gazebo path cannot honour are greyed while connected (design 6.5, 6.7).
  if (st.connected != greyed_) {
    greyed_ = st.connected;
    for (const ParamRow& row : rows_) {
      if (!find_param(row.key)->gazebo_supported) {
        row.item->setDisabled(greyed_);
        row.item->setToolTip(0, greyed_ ? tr("not supported on Gazebo") : QString());
      }
    }
    refresh_fields();
  }
}

void MainWindow::apply_gazebo() {
  const std::filesystem::path dir =
      std::filesystem::absolute(work_dir_ / ("gazebo_" + std::to_string(++applies_)));
  const common::Result<Materialized> m = materialize(config_, dir);
  if (!m) {
    control_->show_message(QString::fromStdString(m.reason()));
    return;
  }
  const common::Result<std::uint64_t> id = client_->apply_config(m.value());
  if (!id) {
    control_->show_message(QString::fromStdString(id.reason()));
    return;
  }
  apply_id_ = id.value();
  applying_ = config_;
  control_->watch(apply_id_, tr("apply config"));
}

}  // namespace cobot::studio
