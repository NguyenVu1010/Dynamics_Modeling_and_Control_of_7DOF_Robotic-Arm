#include "verify_tab.hpp"

#include <QAbstractItemView>
#include <QCheckBox>
#include <QColor>
#include <QComboBox>
#include <QDesktopServices>
#include <QFileDialog>
#include <QFont>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QListWidget>
#include <QMessageBox>
#include <QProgressBar>
#include <QPushButton>
#include <QSignalBlocker>
#include <QSpinBox>
#include <QStringList>
#include <QTableWidget>
#include <QUrl>
#include <QVBoxLayout>
#include <algorithm>
#include <array>
#include <cmath>
#include <initializer_list>
#include <string>
#include <system_error>
#include <utility>

#include "cobot/control/algorithm.hpp"
#include "cobot/pipeline/task.hpp"

#include "report.hpp"  // bench: kVerifyDisclaimer

namespace cobot::studio {
namespace {

namespace fs = std::filesystem;

constexpr int kDefaultRuns = 100;
constexpr int kMaxRuns = 100000;
constexpr int kMaxJobs = 256;
constexpr int kPercent = 100;
const QColor kChosen(0xc8, 0xe6, 0xc9);  // light green row of the chosen tune level

QString qs(const std::string& s) {
  return QString::fromStdString(s);
}

QString num(double v) {
  return std::isnan(v) ? QStringLiteral("n/a") : QString::number(v, 'g', 4);
}

QString passed(const bench::Counts& c) {
  return QStringLiteral("%1/%2").arg(c.passed).arg(c.runs);
}

// The *.yaml files of `dir`, sorted by file name.
std::vector<fs::path> yaml_files(const fs::path& dir) {
  std::vector<fs::path> out;
  std::error_code ec;
  for (fs::directory_iterator it(dir, ec), end; !ec && it != end; it.increment(ec)) {
    if (it->path().extension() == ".yaml") {
      out.push_back(it->path());
    }
  }
  std::sort(out.begin(), out.end());
  return out;
}

// A table nobody edits: no edit trigger, no focus frame, no selection.
QTableWidget* table(const char* name, const QStringList& headers) {
  auto* t = new QTableWidget(0, headers.size());
  t->setObjectName(QString::fromLatin1(name));
  t->setHorizontalHeaderLabels(headers);
  t->setEditTriggers(QAbstractItemView::NoEditTriggers);
  t->setSelectionMode(QAbstractItemView::NoSelection);
  t->setFocusPolicy(Qt::NoFocus);
  t->verticalHeader()->setVisible(false);
  for (QHeaderView* h : {t->horizontalHeader(), t->verticalHeader()}) {
    h->setEditTriggers(QAbstractItemView::NoEditTriggers);
  }
  t->horizontalHeader()->setStretchLastSection(true);
  return t;
}

QListWidget* read_only_list(const char* name) {
  auto* l = new QListWidget;
  l->setObjectName(QString::fromLatin1(name));
  l->setEditTriggers(QAbstractItemView::NoEditTriggers);
  l->setSelectionMode(QAbstractItemView::NoSelection);
  l->setFocusPolicy(Qt::NoFocus);
  return l;
}

void set_row(QTableWidget* t, int row, const QStringList& cells) {
  for (int c = 0; c < cells.size(); ++c) {
    t->setItem(row, c, new QTableWidgetItem(cells.at(c)));
  }
}

QListWidgetItem* checkable(QListWidget* list, const QString& text, bool on) {
  auto* item = new QListWidgetItem(text, list);
  item->setFlags(Qt::ItemIsEnabled | Qt::ItemIsUserCheckable);
  item->setCheckState(on ? Qt::Checked : Qt::Unchecked);
  return item;
}

bool checked(const QListWidget* list, int row) {
  return list->item(row)->checkState() == Qt::Checked;
}

const char* kind_name(pipeline::ProvenanceKind kind) {
  switch (kind) {
    case pipeline::ProvenanceKind::kPublished:
      return "published";
    case pipeline::ProvenanceKind::kDerived:
      return "derived";
    case pipeline::ProvenanceKind::kAssumed:
      break;
  }
  return "assumed";
}

QString bounds(const bench::Assumption& a) {
  if (a.range) {
    return QStringLiteral("[%1, %2]").arg(num((*a.range)[0]), num((*a.range)[1]));
  }
  if (a.scale) {
    return QStringLiteral("x[%1, %2]").arg(num((*a.scale)[0]), num((*a.scale)[1]));
  }
  return QStringLiteral("-");
}

QString verdict_text(const bench::ComparisonRow& row) {
  return row.result ? QString::fromLatin1(bench::to_string(row.result->verdict))
                    : QStringLiteral("not listed");
}

}  // namespace

VerifyTab::VerifyTab(const fs::path& config_dir,
                     fs::path work_dir,
                     StudioWorker& worker,
                     QWidget* parent)
    : QWidget(parent), work_dir_(std::move(work_dir)), worker_(worker) {
  setObjectName(QStringLiteral("verifyTab"));
  auto* layout = new QHBoxLayout(this);
  layout->addWidget(build_inputs());
  auto* right = new QVBoxLayout;
  phase_ = new QLabel(tr("idle"));
  phase_->setObjectName(QStringLiteral("phaseLabel"));
  phase_->setWordWrap(true);
  progress_ = new QProgressBar;
  progress_->setObjectName(QStringLiteral("progressBar"));
  pair_combo_ = new QComboBox;
  pair_combo_->setObjectName(QStringLiteral("pairCombo"));
  open_report_ = new QPushButton(tr("Open report"));
  open_report_->setObjectName(QStringLiteral("openReportButton"));
  export_ = new QPushButton(tr("Export..."));
  export_->setObjectName(QStringLiteral("exportButton"));
  auto* bar = new QHBoxLayout;
  bar->addWidget(pair_combo_, 1);
  bar->addWidget(open_report_);
  bar->addWidget(export_);
  right->addWidget(phase_);
  right->addWidget(progress_);
  right->addLayout(bar);
  right->addWidget(build_result(), 1);
  layout->addLayout(right, 1);

  connect(pair_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int i) {
    if (i >= 0) {
      show_pair(pair_combo_->itemData(i).toInt());
    }
  });
  connect(open_report_, &QPushButton::clicked, this, &VerifyTab::open_report);
  connect(export_, &QPushButton::clicked, this, [this] {
    const QString dir = QFileDialog::getExistingDirectory(this, tr("Export the verify outputs"));
    if (!dir.isEmpty() && !export_to(dir)) {
      QMessageBox::warning(this, tr("cobot_studio"), tr("Choose an empty or new folder."));
    }
  });
  connect(&worker_, &StudioWorker::verify_progress, this, &VerifyTab::on_progress);
  connect(&worker_, &StudioWorker::verify_finished, this, &VerifyTab::on_finished);

  for (const fs::path& p : yaml_files(config_dir / "hardware")) {
    common::Result<pipeline::HardwareModel> m = pipeline::load_hardware_model(p);
    QListWidgetItem* item = checkable(hardware_, qs(p.stem().string()), false);
    item->setData(Qt::UserRole, qs(p.string()));
    item->setToolTip(m ? qs(m.value().description) : qs(m.reason()));
    models_.push_back(m ? std::optional<pipeline::HardwareModel>(std::move(m.value()))
                        : std::nullopt);
  }
  for (const control::AlgorithmInfo& a : control::kAlgorithms) {
    checkable(algorithms_, QString::fromLatin1(a.name), a.id == control::Algorithm::kComputedTorque)
        ->setToolTip(QString::fromLatin1(a.summary));
  }
  for (const fs::path& p : yaml_files(config_dir / "tasks")) {
    if (pipeline::load_task(p)) {  // a waypoint file is not a task
      task_->addItem(qs(p.stem().string()), qs(p.string()));
    }
  }
  connect(hardware_, &QListWidget::itemChanged, this, &VerifyTab::update_preview);
  connect(algorithms_, &QListWidget::itemChanged, this, &VerifyTab::update_preview);
  update_preview();
  clear_result();
  set_busy(false);
}

QWidget* VerifyTab::build_inputs() {
  auto* box = new QWidget;
  auto* form = new QFormLayout(box);
  hardware_ = new QListWidget;
  hardware_->setObjectName(QStringLiteral("hardwareList"));
  algorithms_ = new QListWidget;
  algorithms_->setObjectName(QStringLiteral("algorithmList"));
  for (QListWidget* l : {hardware_, algorithms_}) {
    l->setEditTriggers(QAbstractItemView::NoEditTriggers);
  }
  task_ = new QComboBox;
  task_->setObjectName(QStringLiteral("taskCombo"));
  runs_ = new QSpinBox;
  runs_->setObjectName(QStringLiteral("runsSpin"));
  runs_->setRange(1, kMaxRuns);
  runs_->setValue(kDefaultRuns);
  jobs_ = new QSpinBox;
  jobs_->setObjectName(QStringLiteral("jobsSpin"));
  jobs_->setRange(0, kMaxJobs);
  jobs_->setSpecialValueText(tr("auto"));
  const auto check = [](const char* name, const QString& text, bool on) {
    auto* c = new QCheckBox(text);
    c->setObjectName(QString::fromLatin1(name));
    c->setChecked(on);
    return c;
  };
  quick_ = check("quickCheck", tr("Quick (never PASS)"), false);
  tune_ = check("tuneCheck", tr("Tune the gain first"), true);
  sensitivity_ = check("sensitivityCheck", tr("Sensitivity of the assumptions"), true);
  preview_ = new QLabel;
  preview_->setObjectName(QStringLiteral("pairPreview"));
  preview_->setWordWrap(true);
  preview_->setTextFormat(Qt::RichText);
  run_ = new QPushButton(tr("Run"));
  run_->setObjectName(QStringLiteral("runButton"));
  cancel_ = new QPushButton(tr("Cancel"));
  cancel_->setObjectName(QStringLiteral("cancelButton"));
  connect(run_, &QPushButton::clicked, this, [this] { start(request()); });
  connect(cancel_, &QPushButton::clicked, this, [this] { worker_.cancel_verify(); });
  auto* buttons = new QHBoxLayout;
  buttons->addWidget(run_);
  buttons->addWidget(cancel_);
  form->addRow(tr("Hardware models"), hardware_);
  form->addRow(tr("Algorithms"), algorithms_);
  form->addRow(tr("Task"), task_);
  form->addRow(tr("Runs"), runs_);
  form->addRow(tr("Jobs"), jobs_);
  form->addRow(quick_);
  form->addRow(tune_);
  form->addRow(sensitivity_);
  form->addRow(tr("Pairs"), preview_);
  form->addRow(buttons);
  return box;
}

QWidget* VerifyTab::build_result() {
  auto* box = new QWidget;
  box->setObjectName(QStringLiteral("verifyResult"));
  auto* v = new QVBoxLayout(box);
  badge_ = new QLabel;
  badge_->setObjectName(QStringLiteral("verdictBadge"));
  badge_->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Fixed);
  auto* disclaimer = new QLabel(QString::fromLatin1(bench::kVerifyDisclaimer));
  disclaimer->setObjectName(QStringLiteral("disclaimerLabel"));
  disclaimer->setWordWrap(true);
  QFont small = disclaimer->font();
  small.setPointSizeF(small.pointSizeF() * 0.85);
  disclaimer->setFont(small);
  interval_ = new QLabel;
  interval_->setObjectName(QStringLiteral("intervalLabel"));
  gain_ = new QLabel;
  gain_->setObjectName(QStringLiteral("gainLabel"));
  gain_->setWordWrap(true);
  worst_ =
      table("worstTable",
            {tr("metric"), tr("worst"), tr("threshold"), tr("margin limit"), tr("vs threshold")});
  assumptions_ = table("assumptionTable",
                       {tr("key"),
                        tr("kind"),
                        tr("range / scale"),
                        tr("sensitivity"),
                        tr("base passed"),
                        tr("low passed"),
                        tr("high passed")});
  tune_table_ = table("tuneTable",
                      {tr("level"),
                       tr("seeds"),
                       tr("held"),
                       tr("sat_frac_max"),
                       tr("decays"),
                       tr("rms median"),
                       tr("final max")});
  causes_title_ = new QLabel(tr("Causes (ablation) and suggestions"));
  causes_ = read_only_list("causesList");
  suggestions_ = read_only_list("suggestionsList");
  comparison_ = table(
      "comparisonTable",
      {tr("model"), tr("algorithm"), tr("verdict"), tr("passed"), tr("Wilson 95 %"), tr("gain")});
  v->addWidget(badge_);
  v->addWidget(disclaimer);
  v->addWidget(interval_);
  v->addWidget(worst_);
  v->addWidget(assumptions_);
  v->addWidget(gain_);
  v->addWidget(tune_table_);
  v->addWidget(causes_title_);
  v->addWidget(causes_);
  v->addWidget(suggestions_);
  v->addWidget(comparison_);
  return box;
}

VerifyRequest VerifyTab::request() const {
  VerifyRequest r;
  for (int i = 0; i < hardware_->count(); ++i) {
    if (checked(hardware_, i)) {
      r.hardware.emplace_back(hardware_->item(i)->data(Qt::UserRole).toString().toStdString());
    }
  }
  for (int i = 0; i < algorithms_->count(); ++i) {
    if (checked(algorithms_, i)) {
      r.algorithms.push_back(control::kAlgorithms.at(static_cast<std::size_t>(i)).id);
    }
  }
  r.task = task_->currentData().toString().toStdString();
  r.options.runs = static_cast<std::size_t>(runs_->value());
  r.options.quick = quick_->isChecked();
  r.options.tune = tune_->isChecked();
  r.options.sensitivity = sensitivity_->isChecked();
  r.options.jobs = static_cast<unsigned>(jobs_->value());
  return r;
}

// The checked pairs; the ones the model does not list greyed (they are comparison rows only).
void VerifyTab::update_preview() {
  QStringList parts;
  for (int m = 0; m < hardware_->count(); ++m) {
    for (int a = 0; a < algorithms_->count() && checked(hardware_, m); ++a) {
      if (!checked(algorithms_, a)) {
        continue;
      }
      const control::Algorithm id = control::kAlgorithms.at(static_cast<std::size_t>(a)).id;
      const auto& model = models_.at(static_cast<std::size_t>(m));
      const QString name =
          (hardware_->item(m)->text() + QStringLiteral("/") + algorithms_->item(a)->text())
              .toHtmlEscaped();
      parts
          << (model && model->algorithms.count(id) > 0
                  ? name
                  : QStringLiteral("<span style=\"color: gray\">%1 (not listed)</span>").arg(name));
    }
  }
  preview_->setText(parts.isEmpty() ? tr("none") : parts.join(QStringLiteral(", ")));
}

bool VerifyTab::start(const VerifyRequest& request) {
  if (busy_) {
    return false;
  }
  VerifyRequest r = request;
  created_dir_ = false;
  if (r.out_dir.empty()) {
    const common::Result<fs::path> dir = new_verify_dir(work_dir_);
    if (!dir) {
      phase_->setText(qs(dir.reason()));
      return false;
    }
    r.out_dir = dir.value();
    created_dir_ = true;
  }
  pending_dir_ = r.out_dir;
  clear_result();
  phase_->setText(tr("starting"));
  progress_->setRange(0, 0);
  set_busy(true);
  worker_.arm_verify();
  StudioWorker* w = &worker_;
  QMetaObject::invokeMethod(w, [w, r] { w->verify(r); }, Qt::QueuedConnection);
  return true;
}

void VerifyTab::set_busy(bool busy) {
  busy_ = busy;
  run_->setEnabled(!busy);
  cancel_->setEnabled(busy);
  open_report_->setEnabled(!busy && !rows_.empty());
  export_->setEnabled(!busy && !rows_.empty());
}

void VerifyTab::on_progress(const QString& phase, int pair, int pairs, int done, int total) {
  const QString name = phase == QStringLiteral("runs") ? tr("Monte Carlo") : phase;
  phase_->setText(
      tr("%1, pair %2 of %3: %4/%5").arg(name).arg(pair + 1).arg(pairs).arg(done).arg(total));
  progress_->setRange(0, total);
  progress_->setValue(done);
}

VerifyTab::~VerifyTab() {
  if (busy_ && created_dir_) {
    std::error_code ec;
    fs::remove(pending_dir_, ec);  // only when still empty: a finished writer's files stay
  }
}

void VerifyTab::on_finished(const VerifyHandle& handle, const QString& error) {
  progress_->setRange(0, 1);
  progress_->reset();
  if (!error.isEmpty() || !handle.outcome) {
    if (created_dir_) {
      // A failed job leaves no report (the writer is all or none), but a failed write may leave
      // the per-pair directories it created: the directory is this tab's, so it goes entirely.
      std::error_code ec;
      fs::remove_all(pending_dir_, ec);
    }
    phase_->setText(error);
    set_busy(false);
    return;
  }
  rows_ = handle.outcome->rows;  // copied: every copy of the handle shares the outcome
  out_dir_ = pending_dir_;
  phase_->setText(tr("done: %1").arg(qs(out_dir_.string())));
  const QSignalBlocker block(pair_combo_);
  for (std::size_t i = 0; i < rows_.size(); ++i) {
    if (rows_[i].result) {
      pair_combo_->addItem(qs(rows_[i].model) + QStringLiteral(" / ") +
                               QString::fromLatin1(control::to_string(rows_[i].algorithm)),
                           static_cast<int>(i));
    }
  }
  pair_combo_->setVisible(pair_combo_->count() > 1);
  fill_comparison();
  if (pair_combo_->count() > 0) {
    pair_combo_->setCurrentIndex(0);
    show_pair(pair_combo_->itemData(0).toInt());
  }
  set_busy(false);
}

void VerifyTab::clear_result() {
  rows_.clear();
  out_dir_.clear();
  {
    const QSignalBlocker block(pair_combo_);
    pair_combo_->clear();
  }
  pair_combo_->setVisible(false);
  badge_->setText(tr("no result"));
  badge_->setStyleSheet(QString());
  interval_->clear();
  gain_->clear();
  for (QTableWidget* t : {worst_, assumptions_, tune_table_, comparison_}) {
    t->setRowCount(0);
  }
  causes_->clear();
  suggestions_->clear();
  for (QWidget* w :
       std::initializer_list<QWidget*>{causes_title_, causes_, suggestions_, comparison_}) {
    w->setVisible(false);
  }
}

void VerifyTab::show_pair(int row) {
  if (row < 0 || static_cast<std::size_t>(row) >= rows_.size() ||
      !rows_[static_cast<std::size_t>(row)].result) {
    return;
  }
  const bench::VerifyResult& r = *rows_[static_cast<std::size_t>(row)].result;
  // green PASS, red FAIL, amber MARGINAL: the colour is the label's background.
  const std::array<const char*, 3> colours = {
      "#2e7d32; color: white", "#c62828; color: white", "#ffbf00; color: black"};
  const QString text = QString::fromLatin1(bench::to_string(r.verdict)).toUpper();
  badge_->setText(r.quick ? text + tr(" (quick)") : text);
  badge_->setStyleSheet(
      QStringLiteral("QLabel { background: %1; font-weight: bold; padding: 6px 16px; }")
          .arg(QString::fromLatin1(colours.at(static_cast<std::size_t>(r.verdict)))));
  interval_->setText(tr("%1 passed, pass rate %2 %, Wilson 95 % [%3, %4], held %5")
                         .arg(passed(r.counts))
                         .arg(r.pass_rate * kPercent, 0, 'f', 1)
                         .arg(r.wilson.lo, 0, 'f', 3)
                         .arg(r.wilson.hi, 0, 'f', 3)
                         .arg(r.counts.held));
  worst_->setRowCount(static_cast<int>(r.worst.size()));
  for (std::size_t i = 0; i < r.worst.size(); ++i) {
    const bench::WorstMetric& w = r.worst[i];
    const int at = static_cast<int>(i);
    set_row(worst_, at, {qs(w.name), num(w.value), num(w.threshold), num(w.margin_limit)});
    auto* bar = new QProgressBar;
    bar->setRange(0, kPercent);
    const double frac = w.threshold > 0.0 ? w.value / w.threshold : 0.0;
    bar->setValue(std::isnan(frac) ? 0 : static_cast<int>(std::min(1.0, frac) * kPercent));
    bar->setFormat(std::isnan(frac) ? tr("no failed run") : tr("%p% of the threshold"));
    worst_->setCellWidget(at, 4, bar);
  }
  assumptions_->setRowCount(static_cast<int>(r.assumptions.size()));
  for (std::size_t i = 0; i < r.assumptions.size(); ++i) {
    const bench::Assumption& a = r.assumptions[i];
    const auto end = [](const std::optional<bench::SensitivityEnd>& e) {
      return e ? passed(e->counts) : QStringLiteral("-");
    };
    set_row(assumptions_,
            static_cast<int>(i),
            {qs(a.key),
             QString::fromLatin1(kind_name(a.kind)),
             bounds(a),
             qs(a.sensitivity),
             a.base ? passed(*a.base) : QStringLiteral("-"),
             end(a.low),
             end(a.high)});
  }
  gain_->setText(tr("gain %1 = %2 (%3)%4")
                     .arg(qs(r.gain_key), num(r.gain_used))
                     .arg(QString::fromLatin1(bench::to_string(r.gain_source)))
                     .arg(r.tuning ? QStringLiteral(": ") + qs(r.tuning->reason) : QString()));
  const std::size_t tune_rows = r.tuning ? r.tuning->rows.size() : 0;
  tune_table_->setRowCount(static_cast<int>(tune_rows));
  for (std::size_t i = 0; i < tune_rows; ++i) {
    const bench::TuneRow& t = r.tuning->rows[i];
    const int at = static_cast<int>(i);
    set_row(tune_table_,
            at,
            {num(t.level),
             QString::number(t.seeds),
             QString::number(t.held),
             num(t.sat_frac_max),
             t.decays_all ? tr("yes") : tr("no"),
             num(t.rms_joint_err_median),
             num(t.final_pos_err_max)});
    if (r.tuning->chosen && *r.tuning->chosen == t.level) {
      for (int c = 0; c < tune_table_->columnCount(); ++c) {
        tune_table_->item(at, c)->setBackground(kChosen);
      }
    }
  }
  causes_->clear();
  for (const bench::Cause& c : r.causes) {
    causes_->addItem(tr("%1: %2 passed without it (base %3)%4")
                         .arg(qs(c.group))
                         .arg(c.passed_without)
                         .arg(c.base_passed)
                         .arg(c.main ? tr(", main cause") : QString()));
  }
  suggestions_->clear();
  for (const std::string& s : r.suggestions) {
    suggestions_->addItem(qs(s));
  }
  const bool fail = r.verdict == bench::Verdict::kFail;
  for (QWidget* w : std::initializer_list<QWidget*>{causes_title_, causes_, suggestions_}) {
    w->setVisible(fail);
  }
}

void VerifyTab::fill_comparison() {
  comparison_->setRowCount(static_cast<int>(rows_.size()));
  for (std::size_t i = 0; i < rows_.size(); ++i) {
    const bench::ComparisonRow& row = rows_[i];
    QStringList cells{
        qs(row.model), QString::fromLatin1(control::to_string(row.algorithm)), verdict_text(row)};
    if (row.result) {
      const bench::VerifyResult& r = *row.result;
      cells << passed(r.counts)
            << QStringLiteral("[%1, %2]").arg(r.wilson.lo, 0, 'f', 3).arg(r.wilson.hi, 0, 'f', 3)
            << num(r.gain_used) + QStringLiteral(" (") +
                   QString::fromLatin1(bench::to_string(r.gain_source)) + QStringLiteral(")");
    }
    set_row(comparison_, static_cast<int>(i), cells);
  }
  comparison_->setVisible(rows_.size() > 1);
}

void VerifyTab::open_report() {
  fs::path report = out_dir_ / "report.md";
  const int i = pair_combo_->currentIndex();
  if (rows_.size() > 1 && i >= 0) {
    const bench::ComparisonRow& row =
        rows_.at(static_cast<std::size_t>(pair_combo_->itemData(i).toInt()));
    report = out_dir_ / bench::pair_name(row.model, row.algorithm) / "report.md";
  }
  QDesktopServices::openUrl(QUrl::fromLocalFile(qs(report.string())));
}

bool VerifyTab::export_to(const QString& target) {
  if (busy_ || rows_.empty() || out_dir_.empty()) {
    return false;
  }
  const fs::path to = target.toStdString();
  std::error_code ec;
  if (fs::exists(to, ec) && (!fs::is_directory(to, ec) || !fs::is_empty(to, ec))) {
    return false;
  }
  // Never into the job's own directory or below it: a recursive copy would copy into itself.
  const fs::path from = fs::weakly_canonical(out_dir_, ec).lexically_normal();
  const fs::path dest = fs::weakly_canonical(to, ec).lexically_normal();
  if (ec) {
    return false;
  }
  if (std::mismatch(from.begin(), from.end(), dest.begin(), dest.end()).first == from.end()) {
    return false;
  }
  // Copy next to the target, then rename, so a failed copy leaves no half-exported folder.
  fs::path staging = dest;
  staging += ".export-tmp";
  fs::remove_all(staging, ec);
  ec.clear();
  fs::create_directories(staging, ec);
  if (!ec) {
    fs::copy(out_dir_, staging, fs::copy_options::recursive, ec);
  }
  if (!ec) {
    fs::remove(dest, ec);  // empty or absent (checked above)
    ec.clear();
    fs::rename(staging, dest, ec);
  }
  if (ec) {
    std::error_code ignore;
    fs::remove_all(staging, ignore);
    return false;
  }
  return true;
}

}  // namespace cobot::studio
