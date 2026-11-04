#pragma once

#include <QColor>
#include <QElapsedTimer>
#include <QWidget>
#include <cstdint>
#include <filesystem>
#include <map>
#include <memory>
#include <optional>
#include <thread>
#include <vector>

#include "cobot/common/types.hpp"
#include "cobot/studio/playback.hpp"
#include "cobot/studio/robot_scene.hpp"

#include "run_data.hpp"
#include "time_cursor.hpp"

class QComboBox;
class QLabel;
class QPushButton;
class QSlider;
class QSpinBox;
class QTimer;
namespace Qt3DCore {
class QEntity;
class QTransform;
}  // namespace Qt3DCore
namespace Qt3DRender {
class QGeometryRenderer;
class QMaterial;
}  // namespace Qt3DRender

// The 3D view of the Tracking tab (feature studio-3d, cobot_studio design 6.9, ADR-021): the
// checked runs replayed at the shared time cursor with Qt3D. Under root_entity() every drawn run
// has one entity per visual link, named after the link, whose QTransform is the link's visual
// pose in base_link (display_q of the row at the cursor); the selected run uses a QPhongMaterial,
// up to kMaxGhosts others a QPhongAlphaMaterial (alpha 0.25), both in the run's colour. The
// meshes are read on a thread of their own; one geometry per link is shared by every run.
namespace cobot::studio {

class RobotView : public QWidget {
  Q_OBJECT
 public:
  struct DrawnRun {
    std::uint64_t id = 0;
    std::shared_ptr<const bench::RunData> data;
    QColor colour;
  };
  static constexpr int kMaxGhosts = 3;

  // config_yaml names the URDF (urdf_path); the package root is the directory above the URDF's.
  RobotView(std::filesystem::path config_yaml, TimeCursor* cursor, QWidget* parent = nullptr);
  ~RobotView() override;
  RobotView(const RobotView&) = delete;
  RobotView& operator=(const RobotView&) = delete;

  bool meshes_loaded() const { return loaded_; }  // loading finished (see the status label)
  Qt3DCore::QEntity* root_entity() const { return root_; }

  // The checked runs in list order; `selected` is drawn solid (the first run when absent).
  void set_runs(std::vector<DrawnRun> runs, std::uint64_t selected, double goal_tol_m);

 protected:
  void showEvent(QShowEvent* event) override;

 private:
  struct Loaded;
  struct RunVisual {
    DrawnRun run;
    Qt3DCore::QEntity* entity = nullptr;
    Qt3DRender::QMaterial* solid = nullptr;
    Qt3DRender::QMaterial* ghost = nullptr;
    std::vector<Qt3DCore::QEntity*> links;
    std::vector<Qt3DCore::QTransform*> transforms;
    bool is_solid = false;
  };

  void on_loaded(const std::shared_ptr<Loaded>& loaded);
  void rebuild();         // run entities, materials and the ghost limit
  void build_overlays();  // traces of the selected run
  void update_poses();    // every drawn run at the cursor
  void tick();            // playback
  double end_s() const;   // last row time over the drawn runs

  TimeCursor* cursor_;
  std::thread loader_;
  bool loaded_ = false;
  std::optional<RobotScene> scene_;
  std::vector<Qt3DRender::QGeometryRenderer*> meshes_;  // one per visual link
  std::vector<common::Pose> poses_;
  Qt3DCore::QEntity* root_ = nullptr;
  Qt3DCore::QEntity* overlays_ = nullptr;
  std::vector<DrawnRun> runs_;
  std::uint64_t selected_ = 0;
  double goal_tol_m_ = 0.0;
  std::map<std::uint64_t, RunVisual> visuals_;
  std::uint64_t traced_ = 0;  // run whose traces_ are cached
  Traces traces_;

  QTimer* timer_ = nullptr;
  QElapsedTimer clock_;
  bool syncing_ = false;
  QPushButton* play_ = nullptr;
  QComboBox* speed_ = nullptr;
  QSlider* slider_ = nullptr;
  QSpinBox* magnify_ = nullptr;
  QLabel* magnify_label_ = nullptr;
  QLabel* ghost_label_ = nullptr;
  QLabel* status_ = nullptr;
  QLabel* placeholder_ = nullptr;  // until the Qt3D window exists
};

}  // namespace cobot::studio
