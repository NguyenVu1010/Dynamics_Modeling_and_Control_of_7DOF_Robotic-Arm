#include "robot_view.hpp"

#include <QComboBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QOffscreenSurface>
#include <QOpenGLContext>
#include <QPushButton>
#include <QSlider>
#include <QSpinBox>
#include <QTimer>
#include <QVBoxLayout>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <initializer_list>
#include <string>
#include <utility>

#include "cobot/pipeline/config.hpp"
#include "cobot/studio/stl.hpp"

#include <QMatrix4x4>
#include <Qt3DCore/QEntity>
#include <Qt3DCore/QTransform>
#include <Qt3DExtras/QForwardRenderer>
#include <Qt3DExtras/QOrbitCameraController>
#include <Qt3DExtras/QPhongAlphaMaterial>
#include <Qt3DExtras/QPhongMaterial>
#include <Qt3DExtras/QSphereMesh>
#include <Qt3DExtras/Qt3DWindow>
#include <Qt3DRender/QAttribute>
#include <Qt3DRender/QBuffer>
#include <Qt3DRender/QCamera>
#include <Qt3DRender/QGeometry>
#include <Qt3DRender/QGeometryRenderer>
#include <Qt3DRender/QPointLight>
#include <Qt3DRender/QRenderSettings>

namespace cobot::studio {

namespace {

namespace fs = std::filesystem;
using Qt3DCore::QEntity;
using Qt3DRender::QAttribute;
using Qt3DRender::QGeometryRenderer;

constexpr int kTickMs = 33;         // 30 Hz playback
constexpr int kSliderSteps = 1000;  // timeSlider range
constexpr std::array<double, 3> kSpeeds = {0.25, 1.0, 4.0};
constexpr float kGhostAlpha = 0.25F;
constexpr float kOverlayAlpha = 0.2F;  // tolerance sphere
constexpr float kGoalRadiusM = 0.004F;
constexpr float kCrossHalfM = 0.02F;   // hold mark
constexpr std::size_t kDashRows = 20;  // desired path: dash and gap length in rows
constexpr int kMaxMagnify = 1000;

// Interleaved float32 vertices (position, then normal when `normals`) of one geometry.
QGeometryRenderer* make_renderer(Qt3DCore::QNode* parent,
                                 const QByteArray& bytes,
                                 bool normals,
                                 QGeometryRenderer::PrimitiveType type) {
  const int floats = normals ? 6 : 3;
  const auto vertices = static_cast<unsigned>(bytes.size() / (floats * 4));
  auto* geometry = new Qt3DRender::QGeometry(parent);
  auto* buffer = new Qt3DRender::QBuffer(geometry);
  buffer->setData(bytes);
  for (int k = 0; k < (normals ? 2 : 1); ++k) {
    auto* a = new QAttribute(geometry);
    a->setName(k == 0 ? QAttribute::defaultPositionAttributeName()
                      : QAttribute::defaultNormalAttributeName());
    a->setAttributeType(QAttribute::VertexAttribute);
    a->setVertexBaseType(QAttribute::Float);
    a->setVertexSize(3);
    a->setBuffer(buffer);
    a->setByteStride(static_cast<unsigned>(floats * 4));
    a->setByteOffset(static_cast<unsigned>(k * 12));
    a->setCount(vertices);
    geometry->addAttribute(a);
  }
  auto* renderer = new QGeometryRenderer(parent);
  renderer->setGeometry(geometry);
  renderer->setPrimitiveType(type);
  return renderer;
}

QByteArray to_bytes(const std::vector<common::Vec3>& points) {
  std::vector<float> f;
  f.reserve(3 * points.size());
  for (const common::Vec3& p : points) {
    f.insert(f.end(),
             {static_cast<float>(p.x()), static_cast<float>(p.y()), static_cast<float>(p.z())});
  }
  QByteArray bytes(static_cast<int>(f.size() * sizeof(float)), Qt::Uninitialized);
  std::memcpy(bytes.data(), f.data(), f.size() * sizeof(float));
  return bytes;
}

QMatrix4x4 to_matrix(const common::Pose& T) {
  QMatrix4x4 m;
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      m(r, c) = static_cast<float>(T.R(r, c));
    }
    m(r, 3) = static_cast<float>(T.p(r));
  }
  return m;
}

QEntity* add_entity(QEntity* parent,
                    const QString& name,
                    std::initializer_list<Qt3DCore::QComponent*> components) {
  auto* e = new QEntity(parent);
  e->setObjectName(name);
  for (Qt3DCore::QComponent* c : components) {
    e->addComponent(c);
  }
  return e;
}

Qt3DExtras::QPhongMaterial* flat(Qt3DCore::QNode* parent, const QColor& colour) {
  auto* m = new Qt3DExtras::QPhongMaterial(parent);
  m->setAmbient(colour);
  m->setDiffuse(Qt::black);
  m->setSpecular(Qt::black);
  return m;
}

}  // namespace

// What the loader thread hands to the GUI thread: the scene and one interleaved buffer per link.
struct RobotView::Loaded {
  std::optional<RobotScene> scene;
  std::vector<QByteArray> meshes;
  QString error;
};

RobotView::RobotView(fs::path config_yaml, TimeCursor* cursor, QWidget* parent)
    : QWidget(parent), cursor_(cursor) {
  root_ = new QEntity;
  auto* light = new Qt3DRender::QPointLight;
  auto* light_at = new Qt3DCore::QTransform;
  light_at->setTranslation(QVector3D(2.0F, -2.0F, 3.0F));
  add_entity(root_, QStringLiteral("light"), {light, light_at});

  play_ = new QPushButton(tr("Play"));
  play_->setObjectName(QStringLiteral("playButton"));
  speed_ = new QComboBox;
  speed_->setObjectName(QStringLiteral("speedCombo"));
  for (const double s : kSpeeds) {
    speed_->addItem(QStringLiteral("%1x").arg(s));
  }
  speed_->setCurrentIndex(1);
  slider_ = new QSlider(Qt::Horizontal);
  slider_->setObjectName(QStringLiteral("timeSlider"));
  slider_->setRange(0, kSliderSteps);
  magnify_ = new QSpinBox;
  magnify_->setObjectName(QStringLiteral("magnifySpin"));
  magnify_->setRange(1, kMaxMagnify);
  magnify_->setPrefix(QStringLiteral("x"));
  magnify_->setToolTip(
      tr("Magnifies the tip's deviation from the desired path and the goal sphere"));
  magnify_label_ = new QLabel;
  magnify_label_->setObjectName(QStringLiteral("magnifyLabel"));
  magnify_label_->setVisible(false);
  ghost_label_ = new QLabel;
  ghost_label_->setObjectName(QStringLiteral("ghostLimitLabel"));
  ghost_label_->setVisible(false);
  status_ = new QLabel(tr("Loading meshes..."));

  auto* bar = new QHBoxLayout;
  bar->addWidget(play_);
  bar->addWidget(speed_);
  bar->addWidget(slider_, 1);
  bar->addWidget(magnify_);
  auto* labels = new QHBoxLayout;
  labels->addWidget(status_, 1);
  labels->addWidget(magnify_label_);
  labels->addWidget(ghost_label_);
  auto* layout = new QVBoxLayout(this);
  layout->setContentsMargins(0, 0, 0, 0);
  placeholder_ = new QLabel;
  placeholder_->setAlignment(Qt::AlignCenter);
  layout->addWidget(placeholder_, 1);
  layout->addLayout(bar);
  layout->addLayout(labels);

  timer_ = new QTimer(this);
  timer_->setInterval(kTickMs);
  connect(timer_, &QTimer::timeout, this, &RobotView::tick);
  connect(play_, &QPushButton::clicked, this, [this] {
    if (timer_->isActive()) {
      timer_->stop();
    } else {
      if (cursor_->time_s() >= end_s()) {
        cursor_->set_time_s(0.0);
      }
      clock_.start();
      timer_->start();
    }
    play_->setText(timer_->isActive() ? tr("Pause") : tr("Play"));
  });
  connect(slider_, &QSlider::valueChanged, this, [this](int v) {
    if (!syncing_) {
      cursor_->set_time_s(end_s() * v / kSliderSteps);
    }
  });
  connect(cursor_, &TimeCursor::timeChanged, this, &RobotView::update_poses);
  connect(magnify_, QOverload<int>::of(&QSpinBox::valueChanged), this, [this](int m) {
    magnify_label_->setText(tr("Errors x%1").arg(m));
    magnify_label_->setVisible(m != 1);
    build_overlays();
  });

  // Meshes and buffers off the GUI thread; the result is queued back to this object (dropped if
  // it is gone). The destructor sets cancel_ and joins, so it waits for at most one mesh.
  loader_ = std::thread([this, config_yaml = std::move(config_yaml)] {
    auto loaded = std::make_shared<Loaded>();
    const auto config = pipeline::load_config<model::kArmDof>(config_yaml);
    auto scene = config ? RobotScene::load(
                              config.value().urdf_path,
                              "base_link",
                              "link_7",
                              {{"cobot_sim", config.value().urdf_path.parent_path().parent_path()}})
                        : common::Result<RobotScene>::error(config.reason());
    if (scene) {
      for (const VisualLink& v : scene.value().links()) {
        if (cancel_.load()) {
          return;
        }
        const auto mesh = load_stl(v.mesh);
        if (!mesh) {
          scene = common::Result<RobotScene>::error(mesh.reason());
          break;
        }
        const TriangleMesh& m = mesh.value();
        std::vector<float> f(2 * m.positions.size());
        for (std::size_t i = 0; i < m.positions.size() / 3; ++i) {
          std::copy_n(&m.positions[3 * i], 3, &f[6 * i]);
          std::copy_n(&m.normals[3 * i], 3, &f[6 * i + 3]);
        }
        QByteArray bytes(static_cast<int>(f.size() * sizeof(float)), Qt::Uninitialized);
        std::memcpy(bytes.data(), f.data(), f.size() * sizeof(float));
        loaded->meshes.push_back(std::move(bytes));
      }
    }
    if (scene) {
      loaded->scene = std::move(scene.value());
    } else {
      loaded->error = QString::fromStdString(scene.reason());
    }
    if (cancel_.load()) {
      return;
    }
    QMetaObject::invokeMethod(this, [this, loaded] { on_loaded(loaded); }, Qt::QueuedConnection);
  });
}

RobotView::~RobotView() {
  cancel_.store(true);
  if (loader_.joinable()) {
    loader_.join();
  }
  if (root_->parent() == nullptr) {
    delete root_;  // no window took it
  }
}

// The Qt3D window is made when the view is first shown, and only with a working OpenGL context:
// Qt3D 5.15 leaves a dangling offscreen-surface event behind when its renderer cannot start.
void RobotView::showEvent(QShowEvent* event) {
  QWidget::showEvent(event);
  if (placeholder_ == nullptr) {
    return;
  }
  QOffscreenSurface surface;
  surface.create();
  QOpenGLContext gl;
  if (!gl.create() || !gl.makeCurrent(&surface)) {
    placeholder_->setText(tr("3D view unavailable: no OpenGL context"));
    return;
  }
  gl.doneCurrent();
  auto* window = new Qt3DExtras::Qt3DWindow;
  window->defaultFrameGraph()->setClearColor(QColor(40, 44, 52));
  // Redraw only when a pose or the camera changes (the default renders every vsync).
  window->renderSettings()->setRenderPolicy(Qt3DRender::QRenderSettings::OnDemand);
  Qt3DRender::QCamera* camera = window->camera();
  camera->lens()->setPerspectiveProjection(45.0F, 16.0F / 9.0F, 0.01F, 100.0F);
  camera->setUpVector(QVector3D(0.0F, 0.0F, 1.0F));  // base_link z up
  camera->setPosition(QVector3D(1.6F, -1.6F, 1.2F));
  camera->setViewCenter(QVector3D(0.0F, 0.0F, 0.4F));
  auto* orbit = new Qt3DExtras::QOrbitCameraController(root_);
  orbit->setCamera(camera);
  window->setRootEntity(root_);
  delete layout()->replaceWidget(placeholder_, QWidget::createWindowContainer(window, this));
  delete placeholder_;
  placeholder_ = nullptr;
}

void RobotView::on_loaded(const std::shared_ptr<Loaded>& loaded) {
  loaded_ = true;
  if (!loaded->scene) {
    status_->setText(tr("3D view unavailable: %1").arg(loaded->error));
    return;
  }
  scene_ = std::move(loaded->scene);
  poses_.resize(scene_->links().size());
  for (const QByteArray& bytes : loaded->meshes) {
    meshes_.push_back(make_renderer(root_, bytes, true, QGeometryRenderer::Triangles));
  }
  status_->clear();
  rebuild();
}

void RobotView::set_runs(std::vector<DrawnRun> runs, std::uint64_t selected, double goal_tol_m) {
  runs_ = std::move(runs);
  selected_ = selected;
  goal_tol_m_ = goal_tol_m;
  rebuild();
}

void RobotView::rebuild() {
  if (!scene_) {
    return;
  }
  const auto solid = std::find_if(
      runs_.begin(), runs_.end(), [this](const DrawnRun& r) { return r.id == selected_; });
  const std::uint64_t solid_id = solid != runs_.end() ? selected_
                                 : runs_.empty()      ? 0
                                                      : runs_.front().id;
  for (auto it = visuals_.begin(); it != visuals_.end();) {
    const bool kept = std::any_of(runs_.begin(), runs_.end(), [&](const DrawnRun& r) {
      return r.id == it->first && r.data == it->second.run.data;
    });
    if (kept) {
      ++it;
    } else {
      delete it->second.entity;
      it = visuals_.erase(it);
    }
  }
  int ghosts = 0;
  int hidden = 0;
  for (const DrawnRun& r : runs_) {
    RunVisual& v = visuals_[r.id];
    if (v.entity == nullptr) {
      v.run = r;
      v.entity = add_entity(root_, QStringLiteral("run_%1").arg(r.id), {});
      auto* solid_material = new Qt3DExtras::QPhongMaterial(v.entity);
      solid_material->setDiffuse(r.colour);
      auto* ghost_material = new Qt3DExtras::QPhongAlphaMaterial(v.entity);
      // The alpha material keeps alpha in its diffuse colour: ExtendedRgb holds 0.25 exactly
      // (the 16-bit integer channel would read back 16384 / 65535).
      ghost_material->setDiffuse(r.colour.toExtendedRgb());
      ghost_material->setAlpha(kGhostAlpha);
      v.solid = solid_material;
      v.ghost = ghost_material;
      for (std::size_t i = 0; i < scene_->links().size(); ++i) {
        auto* t = new Qt3DCore::QTransform;
        v.transforms.push_back(t);
        v.links.push_back(add_entity(v.entity,
                                     QString::fromStdString(scene_->links()[i].link),
                                     {meshes_.at(i), v.ghost, t}));
      }
    }
    const bool is_solid = r.id == solid_id;
    if (is_solid != v.is_solid) {
      for (QEntity* e : v.links) {
        e->removeComponent(is_solid ? v.ghost : v.solid);
        e->addComponent(is_solid ? v.solid : v.ghost);
      }
      v.is_solid = is_solid;
    }
    const bool drawn = is_solid || ghosts < kMaxGhosts;
    ghosts += !is_solid && drawn ? 1 : 0;
    hidden += drawn ? 0 : 1;
    v.entity->setEnabled(drawn);
  }
  ghost_label_->setText(
      tr("%1 checked run(s) hidden (at most %2 ghosts)").arg(hidden).arg(kMaxGhosts));
  ghost_label_->setVisible(hidden > 0);
  const auto traced = visuals_.find(solid_id);
  const bench::RunData* traced_data =
      traced == visuals_.end() ? nullptr : traced->second.run.data.get();
  if (traced_ != solid_id || traced_data_ != traced_data) {
    traced_ = solid_id;
    traced_data_ = traced_data;
    traces_ = traced_data == nullptr ? Traces{} : make_traces(*scene_, *traced_data);
  }
  build_overlays();
  update_poses();
}

void RobotView::build_overlays() {
  // Chart redraws call set_runs() often; rebuild only when the traced run, the tolerance or the
  // magnification changed.
  const int magnify = magnify_->value();
  if (traced_data_ == overlay_data_ && goal_tol_m_ == overlay_tol_m_ &&
      magnify == overlay_magnify_) {
    return;
  }
  overlay_data_ = traced_data_;
  overlay_tol_m_ = goal_tol_m_;
  overlay_magnify_ = magnify;
  delete overlays_;
  overlays_ = nullptr;
  const auto v = visuals_.find(traced_);
  if (!scene_ || v == visuals_.end() || traces_.tip.empty()) {
    return;
  }
  overlays_ = add_entity(root_, QStringLiteral("overlays"), {});
  // The tip's deviation from the desired path, magnified.
  const double m = magnify;
  std::vector<common::Vec3> tip(traces_.tip.size());
  for (std::size_t k = 0; k < tip.size(); ++k) {
    const common::Vec3 d = traces_.desired[k] + m * (traces_.tip[k] - traces_.desired[k]);
    tip[k] = d;
  }
  add_entity(overlays_,
             QStringLiteral("tipTrace"),
             {make_renderer(overlays_, to_bytes(tip), false, QGeometryRenderer::LineStrip),
              flat(overlays_, v->second.run.colour)});
  std::vector<common::Vec3> dashes;
  for (std::size_t k = 0; k + kDashRows < traces_.desired.size(); k += 2 * kDashRows) {
    dashes.push_back(traces_.desired[k]);
    dashes.push_back(traces_.desired[k + kDashRows]);
  }
  add_entity(overlays_,
             QStringLiteral("desiredPath"),
             {make_renderer(overlays_, to_bytes(dashes), false, QGeometryRenderer::Lines),
              flat(overlays_, Qt::white)});
  const auto sphere = [this](const common::Vec3& at,
                             float radius,
                             Qt3DRender::QMaterial* material,
                             const QString& name) {
    auto* mesh = new Qt3DExtras::QSphereMesh(overlays_);
    mesh->setRadius(radius);
    auto* t = new Qt3DCore::QTransform;
    t->setTranslation(QVector3D(
        static_cast<float>(at.x()), static_cast<float>(at.y()), static_cast<float>(at.z())));
    add_entity(overlays_, name, {mesh, material, t});
  };
  Qt3DRender::QMaterial* goal_material = flat(overlays_, Qt::green);
  for (const common::Vec3& g : traces_.goals) {
    sphere(g, kGoalRadiusM, goal_material, QStringLiteral("goal"));
  }
  auto* tolerance = new Qt3DExtras::QPhongAlphaMaterial(overlays_);
  tolerance->setDiffuse(Qt::green);
  tolerance->setAlpha(kOverlayAlpha);
  if (!traces_.goals.empty()) {
    sphere(traces_.goals.back(),
           static_cast<float>(m * goal_tol_m_),
           tolerance,
           QStringLiteral("toleranceSphere"));
  }
  if (traces_.hold_t_s) {
    // On the drawn (magnified) tip trace.
    const std::size_t k = row_at(*v->second.run.data, *traces_.hold_t_s);
    const common::Vec3 at = traces_.desired[k] + m * (traces_.tip[k] - traces_.desired[k]);
    std::vector<common::Vec3> cross;
    for (int axis = 0; axis < 3; ++axis) {
      cross.emplace_back(at - kCrossHalfM * common::Vec3::Unit(axis));
      cross.emplace_back(at + kCrossHalfM * common::Vec3::Unit(axis));
    }
    add_entity(overlays_,
               QStringLiteral("holdMark"),
               {make_renderer(overlays_, to_bytes(cross), false, QGeometryRenderer::Lines),
                flat(overlays_, Qt::red)});
  }
}

void RobotView::update_poses() {
  const double t = cursor_->time_s();
  for (auto& [id, v] : visuals_) {
    if (!v.entity->isEnabled()) {
      continue;
    }
    scene_->visual_poses(display_q(v.run.data->records[row_at(*v.run.data, t)]), poses_);
    for (std::size_t i = 0; i < v.transforms.size(); ++i) {
      v.transforms[i]->setMatrix(to_matrix(poses_[i]));
    }
  }
  const double end = end_s();
  syncing_ = true;
  slider_->setValue(end > 0.0 ? static_cast<int>(std::lround(t / end * kSliderSteps)) : 0);
  syncing_ = false;
}

void RobotView::tick() {
  const double dt = 1e-3 * static_cast<double>(clock_.restart()) *
                    kSpeeds.at(static_cast<std::size_t>(std::max(0, speed_->currentIndex())));
  const double end = end_s();
  const double t = std::min(cursor_->time_s() + dt, end);
  cursor_->set_time_s(t);
  if (t >= end) {
    timer_->stop();
    play_->setText(tr("Play"));
  }
}

double RobotView::end_s() const {
  double end = 0.0;
  for (const DrawnRun& r : runs_) {
    if (r.data && !r.data->records.empty()) {
      end = std::max(end, 1e-9 * static_cast<double>(r.data->records.back().t.ns));
    }
  }
  return end;
}

}  // namespace cobot::studio
