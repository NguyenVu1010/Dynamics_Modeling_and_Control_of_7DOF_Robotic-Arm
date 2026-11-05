#include "control_tab.hpp"

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QSlider>
#include <QTabWidget>
#include <QTableWidget>
#include <QTimer>
#include <QVBoxLayout>
#include <cmath>
#include <cstring>

#include <Eigen/Geometry>

#include "cobot/pipeline/path_validation.hpp"
#include "cobot/pipeline/waypoint_file.hpp"

#include "robot_view.hpp"
#include "time_cursor.hpp"

namespace cobot::studio {
namespace {

using pipeline::LoopMode;
constexpr int kN = bench::kN;
constexpr double kDeg = kDegToRad;
constexpr double kMm = 1e-3;
constexpr int kJogRefreshMs = 25;  // jog slot refresh while a button is held (deadman)
constexpr int kGoalDecimals = 7;   // the jog band shows exactly (degrees)
constexpr int kTraceStride = 10;   // live tip trace: one sample in ten
constexpr double kGimbal = 1e-9;   // |cos(pitch)| below this: roll folded into yaw
constexpr std::array<const char*, 6> kAxes = {"x", "y", "z", "rx", "ry", "rz"};
constexpr int kColumns = 8;  // x y z (mm), roll pitch yaw (deg), duration (s), stop

// Roll, pitch, yaw of R = Rz(yaw) Ry(pitch) Rx(roll) (the waypoint file's rpy_deg).
common::Vec3 rpy_of(const common::Mat3& R) {
  const double c = std::hypot(R(0, 0), R(1, 0));
  const double pitch = std::atan2(-R(2, 0), c);
  if (c < kGimbal) {
    return {0.0, pitch, std::atan2(-R(0, 1), R(1, 1))};
  }
  return {std::atan2(R(2, 1), R(2, 2)), pitch, std::atan2(R(1, 0), R(0, 0))};
}

common::Mat3 from_rpy(const common::Vec3& rpy) {
  const Eigen::Quaterniond q = Eigen::AngleAxisd(rpy.z(), common::Vec3::UnitZ()) *
                               Eigen::AngleAxisd(rpy.y(), common::Vec3::UnitY()) *
                               Eigen::AngleAxisd(rpy.x(), common::Vec3::UnitX());
  const common::Mat3 R = q.toRotationMatrix();
  return R;
}

// x y z in mm, roll pitch yaw in deg.
std::array<double, 6> display_of(const common::Pose& T) {
  const common::Vec3 rpy = rpy_of(T.R);
  return {
      T.p.x() / kMm, T.p.y() / kMm, T.p.z() / kMm, rpy.x() / kDeg, rpy.y() / kDeg, rpy.z() / kDeg};
}

common::Pose pose_of(const std::array<double, 6>& v) {
  common::Pose T;
  T.p = common::Vec3(v[0], v[1], v[2]) * kMm;
  T.R = from_rpy(common::Vec3(v[3], v[4], v[5]) * kDeg);
  return T;
}

QString number(double v) {
  return QString::number(v, 'f', 3);
}

}  // namespace

ControlTab::ControlTab(BridgeClient& client,
                       const std::string& name,
                       const std::filesystem::path& config_yaml,
                       QWidget* parent)
    : QWidget(parent),
      client_(client),
      region_(QStringLiteral("/cobot_") + QString::fromStdString(name)) {
  setObjectName(QStringLiteral("controlTab"));
  jog_timer_ = new QTimer(this);
  jog_timer_->setInterval(kJogRefreshMs);
  connect(jog_timer_, &QTimer::timeout, this, [this] {
    if (jog_) {
      jog_();
    }
  });

  connection_ = new QLabel;
  connection_->setObjectName(QStringLiteral("connectionLabel"));
  owner_ = new QLabel;
  owner_->setObjectName(QStringLiteral("ownerLabel"));
  claim_ = new QPushButton(tr("Take control"));
  claim_->setObjectName(QStringLiteral("claimButton"));
  connect(claim_, &QPushButton::clicked, this, [this] {
    const common::Status s = client_.claim();
    show_message(s ? tr("control taken") : QString::fromStdString(s.reason()));
  });
  release_ = new QPushButton(tr("Release"));
  release_->setObjectName(QStringLiteral("releaseButton"));
  connect(release_, &QPushButton::clicked, this, [this] { client_.release(); });
  auto* top = new QHBoxLayout;
  top->addWidget(connection_, 1);
  top->addWidget(owner_);
  top->addWidget(claim_);
  top->addWidget(release_);

  auto* modes = new QTabWidget;
  modes->setObjectName(QStringLiteral("controlModes"));
  modes->addTab(build_joint_page(), tr("Joint"));
  modes->addTab(build_tcp_page(), tr("TCP"));
  modes->addTab(build_waypoint_page(), tr("Waypoints"));

  speed_ = new QSlider(Qt::Horizontal);
  speed_->setObjectName(QStringLiteral("controlSpeedSlider"));
  speed_->setRange(1, 100);
  speed_->setValue(static_cast<int>(StudioConfig{}.speed_pct));
  speed_->setToolTip(tr("Speed of jogs and moves (%)"));
  stop_ = new QPushButton(tr("Stop"));
  stop_->setObjectName(QStringLiteral("stopButton"));
  connect(stop_, &QPushButton::clicked, this, [this] { sent(client_.stop(), tr("stop")); });
  estop_ = new QPushButton(tr("E-stop"));
  estop_->setObjectName(QStringLiteral("estopButton"));
  estop_->setStyleSheet(QStringLiteral("background-color: #d32f2f; color: white;"));
  connect(estop_, &QPushButton::clicked, this, [this] { sent(client_.estop(), tr("estop")); });
  reset_ = new QPushButton(tr("Reset"));
  reset_->setObjectName(QStringLiteral("resetButton"));
  connect(reset_, &QPushButton::clicked, this, [this] { sent(client_.reset(), tr("reset")); });
  status_ = new QLabel;
  status_->setObjectName(QStringLiteral("statusLine"));
  auto* bottom = new QHBoxLayout;
  bottom->addWidget(new QLabel(tr("Speed (%)")));
  bottom->addWidget(speed_, 1);
  bottom->addWidget(stop_);
  bottom->addWidget(estop_);
  bottom->addWidget(reset_);

  view_ = new RobotView(config_yaml, new TimeCursor(this));
  view_->setObjectName(QStringLiteral("liveRobotView"));
  view_->set_live_mode();
  auto* left = new QVBoxLayout;
  left->addWidget(modes, 1);
  left->addLayout(bottom);
  left->addWidget(status_);
  auto* body = new QHBoxLayout;
  body->addLayout(left, 1);
  body->addWidget(view_, 1);
  auto* layout = new QVBoxLayout(this);
  layout->addLayout(top);
  layout->addLayout(body, 1);
  refresh();
}

QPushButton* ControlTab::jog_button(const QString& name,
                                    const QString& text,
                                    std::function<void()> start) {
  auto* b = new QPushButton(text);
  b->setObjectName(name);
  connect(b, &QPushButton::pressed, this, [this, b, start = std::move(start)] {
    held_ = b;
    jog_ = start;
    jog_();
    jog_timer_->start();
  });
  connect(b, &QPushButton::released, this, [this] {
    jog_timer_->stop();
    jog_ = nullptr;
    held_ = nullptr;
    client_.jog_release();
  });
  jogs_.push_back(b);
  return b;
}

QWidget* ControlTab::build_joint_page() {
  auto* page = new QWidget;
  page->setObjectName(QStringLiteral("jointPage"));
  auto* grid = new QGridLayout(page);
  for (int j = 0; j < kN; ++j) {
    const QString n = QString::number(j + 1);
    joint_values_[j] = new QLabel;
    joint_values_[j]->setObjectName(QStringLiteral("jointValue_") + n);
    joint_goals_[j] = new QDoubleSpinBox;
    joint_goals_[j]->setObjectName(QStringLiteral("jointGoal_") + n);
    joint_goals_[j]->setDecimals(kGoalDecimals);
    joint_goals_[j]->setRange(-180.0, 180.0);
    joint_goals_[j]->setSuffix(tr(" deg"));
    grid->addWidget(new QLabel(tr("J%1").arg(j + 1)), j, 0);
    grid->addWidget(joint_values_[j], j, 1);
    for (const int dir : {-1, 1}) {
      grid->addWidget(
          jog_button((dir < 0 ? QStringLiteral("jogMinus_") : QStringLiteral("jogPlus_")) + n,
                     dir < 0 ? QStringLiteral("-") : QStringLiteral("+"),
                     [this, j, dir] { client_.jog_joint(j, dir, speed()); }),
          j,
          dir < 0 ? 2 : 3);
    }
    grid->addWidget(joint_goals_[j], j, 4);
  }
  auto* copy = new QPushButton(tr("Copy current"));
  copy->setObjectName(QStringLiteral("copyJointButton"));
  connect(copy, &QPushButton::clicked, this, [this] {
    if (!client_.live().empty()) {
      for (int j = 0; j < kN; ++j) {
        joint_goals_[j]->setValue(client_.live().back().q[j] / kDeg);
      }
    }
  });
  auto* go = new QPushButton(tr("Go"));
  go->setObjectName(QStringLiteral("goJointButton"));
  connect(go, &QPushButton::clicked, this, [this] {
    Vec q;
    for (int j = 0; j < kN; ++j) {
      q(j) = joint_goals_[j]->value() * kDeg;
    }
    sent(client_.move_joint(q, speed()), tr("move_joint"));
  });
  motion_.push_back(go);
  grid->addWidget(copy, kN, 0, 1, 2);
  grid->addWidget(go, kN, 2, 1, 3);
  grid->setRowStretch(kN + 1, 1);
  return page;
}

QWidget* ControlTab::build_tcp_page() {
  auto* page = new QWidget;
  page->setObjectName(QStringLiteral("tcpPage"));
  auto* grid = new QGridLayout(page);
  frame_ = new QComboBox;
  frame_->setObjectName(QStringLiteral("frameCombo"));
  frame_->addItems({tr("Base"), tr("Tool")});
  grid->addWidget(new QLabel(tr("Frame")), 0, 0);
  grid->addWidget(frame_, 0, 1, 1, 2);
  for (std::size_t a = 0; a < kAxes.size(); ++a) {
    const int row = static_cast<int>(a) + 1;
    const QString n = QString::fromLatin1(kAxes[a]);
    tcp_values_[a] = new QLabel;
    tcp_values_[a]->setObjectName(QStringLiteral("tcpValue_") + n);
    tcp_goals_[a] = new QDoubleSpinBox;
    tcp_goals_[a]->setObjectName(QStringLiteral("tcpGoal_") + n);
    tcp_goals_[a]->setDecimals(4);
    tcp_goals_[a]->setRange(a < 3 ? -5000.0 : -360.0, a < 3 ? 5000.0 : 360.0);
    tcp_goals_[a]->setSuffix(a < 3 ? tr(" mm") : tr(" deg"));
    grid->addWidget(new QLabel(n), row, 0);
    grid->addWidget(tcp_values_[a], row, 1);
    const auto axis = static_cast<pipeline::JogAxis>(a);
    for (const int dir : {-1, 1}) {
      grid->addWidget(
          jog_button(
              (dir < 0 ? QStringLiteral("jogMinus_") : QStringLiteral("jogPlus_")) + n,
              dir < 0 ? QStringLiteral("-") : QStringLiteral("+"),
              [this, axis, dir] {
                client_.jog_tcp(
                    axis, dir, static_cast<pipeline::JogFrame>(frame_->currentIndex()), speed());
              }),
          row,
          dir < 0 ? 2 : 3);
    }
    grid->addWidget(tcp_goals_[a], row, 4);
  }
  auto* copy = new QPushButton(tr("Copy current"));
  copy->setObjectName(QStringLiteral("copyTcpButton"));
  connect(copy, &QPushButton::clicked, this, [this] {
    if (!client_.live().empty()) {
      const std::array<double, 6> v = display_of(tcp_pose());
      for (std::size_t a = 0; a < v.size(); ++a) {
        tcp_goals_[a]->setValue(v[a]);
      }
    }
  });
  auto* go = new QPushButton(tr("Go"));
  go->setObjectName(QStringLiteral("goTcpButton"));
  connect(go, &QPushButton::clicked, this, [this] {
    std::array<double, 6> v{};
    for (std::size_t a = 0; a < v.size(); ++a) {
      v[a] = tcp_goals_[a]->value();
    }
    sent(client_.move_tcp(pose_of(v), speed()), tr("move_tcp"));
  });
  motion_.push_back(go);
  grid->addWidget(copy, 7, 0, 1, 2);
  grid->addWidget(go, 7, 2, 1, 3);
  grid->setRowStretch(8, 1);
  return page;
}

QWidget* ControlTab::build_waypoint_page() {
  auto* page = new QWidget;
  page->setObjectName(QStringLiteral("waypointPage"));
  table_ = new QTableWidget(0, kColumns);
  table_->setObjectName(QStringLiteral("waypointTable"));
  table_->setHorizontalHeaderLabels({tr("x (mm)"),
                                     tr("y (mm)"),
                                     tr("z (mm)"),
                                     tr("roll (deg)"),
                                     tr("pitch (deg)"),
                                     tr("yaw (deg)"),
                                     tr("duration (s)"),
                                     tr("stop")});
  table_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
  issues_ = new QPlainTextEdit;
  issues_->setObjectName(QStringLiteral("previewIssues"));
  issues_->setReadOnly(true);
  issues_->setMaximumHeight(80);
  auto* buttons = new QHBoxLayout;
  const auto add = [&](const char* name, const QString& text, auto slot) {
    auto* b = new QPushButton(text);
    b->setObjectName(QString::fromLatin1(name));
    connect(b, &QPushButton::clicked, this, slot);
    buttons->addWidget(b);
    return b;
  };
  add_pose_ = add("addPoseButton", tr("Add current pose"), [this] {
    if (!client_.live().empty()) {
      add_row(tcp_pose(), 0.0, false);
    }
  });
  add("removePointButton", tr("Remove"), [this] {
    if (table_->currentRow() >= 0) {
      table_->removeRow(table_->currentRow());
    }
  });
  // The rows are TCP poses as in the file: an identity TcpOffset (ADR-018 file format).
  add("openWaypointsButton", tr("Open..."), [this] {
    const QString f =
        QFileDialog::getOpenFileName(this, tr("Waypoint file"), {}, tr("YAML (*.yaml)"));
    if (f.isEmpty()) {
      return;
    }
    const auto loaded =
        pipeline::load_waypoints(f.toStdString(), "base_link", pipeline::TcpOffset{});
    if (!loaded) {
      show_message(QString::fromStdString(loaded.reason()));
      return;
    }
    table_->setRowCount(0);
    for (const planning::Waypoint& w : loaded.value()) {
      add_row(w.T_b_goal, w.duration_s, w.stop);
    }
  });
  add("saveWaypointsButton", tr("Save..."), [this] {
    const QString f =
        QFileDialog::getSaveFileName(this, tr("Waypoint file"), {}, tr("YAML (*.yaml)"));
    if (!f.isEmpty()) {
      const common::Status s = pipeline::save_waypoints(
          f.toStdString(), "base_link", table_points(), pipeline::TcpOffset{});
      show_message(s ? tr("saved %1").arg(f) : QString::fromStdString(s.reason()));
    }
  });
  preview_button_ = add("previewButton", tr("Preview"), [this] {
    const common::Result<std::uint64_t> id = client_.preview(table_points());
    sent(id, tr("preview"));
    preview_id_ = id ? id.value() : 0;
  });
  motion_.push_back(add("runWaypointsButton", tr("Run"), [this] {
    sent(client_.run_waypoints(table_points()), tr("run_waypoints"));
  }));
  auto* layout = new QVBoxLayout(page);
  layout->addWidget(table_, 1);
  layout->addLayout(buttons);
  layout->addWidget(issues_);
  return page;
}

void ControlTab::add_row(const common::Pose& T_b_tcp, double duration_s, bool stop) {
  const int row = table_->rowCount();
  table_->insertRow(row);
  const std::array<double, 6> v = display_of(T_b_tcp);
  for (std::size_t c = 0; c < v.size(); ++c) {
    table_->setItem(row, static_cast<int>(c), new QTableWidgetItem(number(v[c])));
  }
  table_->setItem(row, 6, new QTableWidgetItem(number(duration_s)));
  auto* s = new QTableWidgetItem;
  s->setFlags(s->flags() | Qt::ItemIsUserCheckable);
  s->setCheckState(stop ? Qt::Checked : Qt::Unchecked);
  table_->setItem(row, 7, s);
}

std::vector<planning::Waypoint> ControlTab::table_points() const {
  std::vector<planning::Waypoint> out;
  for (int r = 0; r < table_->rowCount(); ++r) {
    std::array<double, 6> v{};
    for (std::size_t c = 0; c < v.size(); ++c) {
      const QTableWidgetItem* item = table_->item(r, static_cast<int>(c));
      v[c] = item == nullptr ? 0.0 : item->text().toDouble();
    }
    planning::Waypoint w;
    w.T_b_goal = pose_of(v);
    const QTableWidgetItem* d = table_->item(r, 6);
    w.duration_s = d == nullptr ? 0.0 : d->text().toDouble();
    const QTableWidgetItem* s = table_->item(r, 7);
    w.stop = s != nullptr && s->checkState() == Qt::Checked;
    out.push_back(w);
  }
  return out;
}

common::Pose ControlTab::tcp_pose() const {
  const bridge::StateSample<kN>& s = client_.live().back();
  const std::optional<bridge::SessionInfo<kN>> session = client_.session();
  const common::Pose T_tip_tcp = session ? bridge::from_wire(session->T_tip_tcp) : common::Pose{};
  return common::compose(bridge::from_wire(s.T_b_e), T_tip_tcp);
}

double ControlTab::speed() const {
  return speed_->value() / 100.0;
}

void ControlTab::sent(const common::Result<std::uint64_t>& id, const QString& what) {
  if (id) {
    watch(id.value(), what);
  } else {
    show_message(QString::fromStdString(id.reason()));
  }
}

void ControlTab::watch(std::uint64_t id, const QString& what) {
  watched_.emplace_back(id, what);
}

void ControlTab::show_message(const QString& text) {
  message_ = text;
}

void ControlTab::refresh() {
  const LiveState st = client_.state();
  const bool live = st.connected && !client_.live().empty();
  if (!st.connected) {
    // Command ids restart with the next connection: nothing sent on this one is answered.
    watched_.clear();
    preview_id_ = 0;
    generation_.reset();
  }
  connection_->setText(
      st.connected
          ? tr("Connected to %1, controller pid %2, mode %3")
                .arg(region_)
                .arg(st.server_pid)
                .arg(QString::fromLatin1(pipeline::to_string(st.mode)))
          : tr("Not connected to %1: %2").arg(region_, QString::fromStdString(client_.reason())));
  owner_->setText(st.mine             ? tr("You have control")
                  : st.owner_pid != 0 ? tr("Held by pid %1").arg(st.owner_pid)
                                      : tr("Free"));

  const std::optional<bridge::SessionInfo<kN>> session = client_.session();
  if (session && (!generation_ || *generation_ != session->generation)) {
    generation_ = session->generation;
    const double band = session->soft_margin_rad + session->jog_margin_rad;
    for (int j = 0; j < kN; ++j) {
      joint_goals_[j]->setRange((session->q_min[j] + band) / kDeg,
                                (session->q_max[j] - band) / kDeg);
    }
  }
  std::vector<common::Vec3> trace;
  if (live) {
    const bridge::StateSample<kN>& s = client_.live().back();
    for (int j = 0; j < kN; ++j) {
      joint_values_[j]->setText(number(s.q[j] / kDeg) + tr(" deg"));
    }
    const std::array<double, 6> v = display_of(tcp_pose());
    for (std::size_t a = 0; a < v.size(); ++a) {
      tcp_values_[a]->setText(number(v[a]) + (a < 3 ? tr(" mm") : tr(" deg")));
    }
    Vec q;
    bool truth = true;
    for (const double v_true : s.q_true) {
      truth = truth && std::isfinite(v_true);
    }
    for (int j = 0; j < kN; ++j) {
      q(j) = truth ? s.q_true[j] : s.q[j];
    }
    const auto& buffer = client_.live();
    for (std::size_t k = 0; k < buffer.size(); k += kTraceStride) {
      const double* p = buffer[k].T_b_e.p;
      trace.emplace_back(p[0], p[1], p[2]);
    }
    view_->set_live(q, trace, preview_path_);
  }

  // Responses: the watched commands, the preview's issues and path, jog rejections.
  for (auto it = watched_.begin(); it != watched_.end();) {
    const std::optional<bridge::Response> r = client_.response(it->first);
    if (!r) {
      ++it;
      continue;
    }
    message_ = it->second + QStringLiteral(": ") +
               QString::fromUtf8(r->text, static_cast<int>(strnlen(r->text, bridge::kTextChars)));
    if (it->first == preview_id_) {
      QStringList lines;
      for (std::uint32_t i = 0; i < r->issue_count; ++i) {
        const bridge::WireIssue& w = r->issues[i];
        QString line = tr("waypoint %1: %2")
                           .arg(w.waypoint + 1)
                           .arg(QString::fromLatin1(
                               pipeline::to_string(static_cast<pipeline::PathIssue>(w.issue))));
        if (w.joint >= 0) {
          line += tr(" joint %1").arg(w.joint + 1);
        }
        lines << line + QStringLiteral(" %1 > %2").arg(w.value, 0, 'g', 3).arg(w.limit, 0, 'g', 3);
      }
      if (r->accepted == 0 && lines.isEmpty()) {
        lines << message_;
      }
      issues_->setPlainText(lines.join(QLatin1Char('\n')));
      preview_path_.clear();
      for (std::uint32_t k = 0; k < r->path_count; ++k) {
        preview_path_.emplace_back(r->path[k][0], r->path[k][1], r->path[k][2]);
      }
    }
    it = watched_.erase(it);
  }
  for (const bridge::Response& r : client_.take_jog_rejections()) {
    message_ = QString::fromUtf8(r.text, static_cast<int>(strnlen(r.text, bridge::kTextChars)));
  }
  QStringList status;
  if (!message_.isEmpty()) {
    status << message_;
  }
  if (st.connected && st.mode == LoopMode::kHolding) {
    status << tr("holding: %1").arg(QString::fromLatin1(pipeline::to_string(st.hold)));
  }
  if (st.connected && st.manual_stop != pipeline::ManualStop::kNone) {
    status << tr("manual stop: %1").arg(QString::fromLatin1(pipeline::to_string(st.manual_stop)));
  }
  status << tr("lost samples: %1").arg(st.lost_samples);
  status_->setText(status.join(QStringLiteral(" | ")));

  // Enabling (design 6.8).
  const bool owner = st.connected && st.mine;
  const bool ready = owner && st.mode == LoopMode::kStandby;
  claim_->setEnabled(st.connected && !st.mine);
  release_->setEnabled(owner);
  for (QPushButton* b : jogs_) {
    b->setEnabled(ready || b == held_);
  }
  for (QPushButton* b : motion_) {
    b->setEnabled(ready);
  }
  preview_button_->setEnabled(st.connected);
  add_pose_->setEnabled(live);
  stop_->setEnabled(owner);
  estop_->setEnabled(owner);
  reset_->setEnabled(owner && st.mode == LoopMode::kHolding);
}

}  // namespace cobot::studio
