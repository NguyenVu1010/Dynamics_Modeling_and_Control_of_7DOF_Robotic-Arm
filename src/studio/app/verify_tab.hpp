#pragma once

#include <QString>
#include <QWidget>
#include <filesystem>
#include <optional>
#include <vector>

#include "cobot/pipeline/hardware_model.hpp"
#include "cobot/studio/verify_job.hpp"

#include "compare.hpp"  // bench
#include "studio_worker.hpp"

class QCheckBox;
class QComboBox;
class QLabel;
class QListWidget;
class QProgressBar;
class QPushButton;
class QSpinBox;
class QTableWidget;

// The Verification tab of cobot_studio (feature studio-verify, cobot_studio design 10 G14/G16,
// ADR-023): the inputs of `cobot_bench verify`, the job on the window's worker thread
// (StudioWorker::verify) and the result of the selected pair, read from the job's ComparisonRows.
// Results are never shown in editable widgets: labels, read-only tables and a coloured verdict
// badge. Widgets carry the object names of the UI contract. GUI thread only.
namespace cobot::studio {

class VerifyTab : public QWidget {
  Q_OBJECT
 public:
  // config_dir holds hardware/*.yaml and tasks/*.yaml; outputs go to <work_dir>/verify_NNNN.
  VerifyTab(const std::filesystem::path& config_dir,
            std::filesystem::path work_dir,
            StudioWorker& worker,
            QWidget* parent = nullptr);

  // The request the input widgets describe; every other option at its default, out_dir empty.
  VerifyRequest request() const;
  // After a job that succeeded: its rows and its output directory; empty otherwise.
  const std::vector<bench::ComparisonRow>& rows() const { return rows_; }
  const std::filesystem::path& out_dir() const { return out_dir_; }

 public slots:
  // Starts the job on the worker thread; false (nothing started) while a job runs. An empty
  // out_dir: new_verify_dir(work_dir).
  bool start(const VerifyRequest& request);
  // Copies out_dir()'s files into `target` (created when missing); false and nothing copied
  // without a result or when `target` is a non-empty directory.
  bool export_to(const QString& target);

 private:
  QWidget* build_inputs();
  QWidget* build_result();
  void update_preview();
  void set_busy(bool busy);
  void on_progress(const QString& phase, int pair, int pairs, int done, int total);
  void on_finished(const VerifyHandle& handle, const QString& error);
  void clear_result();
  void show_pair(int row);
  void fill_comparison();
  void open_report();

  std::filesystem::path work_dir_;
  StudioWorker& worker_;
  std::vector<std::optional<pipeline::HardwareModel>> models_;  // per hardwareList row
  std::vector<bench::ComparisonRow> rows_;
  std::filesystem::path out_dir_;
  std::filesystem::path pending_dir_;  // the running job's out_dir
  bool created_dir_ = false;           // pending_dir_ is new_verify_dir's
  bool busy_ = false;

  QListWidget* hardware_ = nullptr;
  QListWidget* algorithms_ = nullptr;
  QComboBox* task_ = nullptr;
  QSpinBox* runs_ = nullptr;
  QCheckBox* quick_ = nullptr;
  QCheckBox* tune_ = nullptr;
  QCheckBox* sensitivity_ = nullptr;
  QSpinBox* jobs_ = nullptr;
  QLabel* preview_ = nullptr;
  QPushButton* run_ = nullptr;
  QPushButton* cancel_ = nullptr;
  QLabel* phase_ = nullptr;
  QProgressBar* progress_ = nullptr;
  QComboBox* pair_combo_ = nullptr;
  QPushButton* open_report_ = nullptr;
  QPushButton* export_ = nullptr;
  QLabel* badge_ = nullptr;
  QLabel* interval_ = nullptr;
  QLabel* gain_ = nullptr;
  QTableWidget* worst_ = nullptr;
  QTableWidget* assumptions_ = nullptr;
  QTableWidget* tune_table_ = nullptr;
  QListWidget* causes_ = nullptr;
  QListWidget* suggestions_ = nullptr;
  QLabel* causes_title_ = nullptr;
  QTableWidget* comparison_ = nullptr;
};

}  // namespace cobot::studio
