// TAMSVIZ
// (c) 2020 Philipp Ruppel

#pragma once

#include "../core/document.h"
#include "../core/loader.h"
#include "../core/timeseries.h"
#include "../core/topic.h"
#include "../core/watcher.h"

#include "../render/renderer.h"
#include "../render/resource.h"

#include "frame.h"
#include "mesh.h"

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <sensor_msgs/JointState.h>

#include <condition_variable>
#include <mutex>

class RobotModel;
class RobotState;

struct RobotModelImportOptions {
  DECLARE_STRUCT_PROPERTY(bool, recomputeNormals, false);
  DECLARE_STRUCT_PROPERTY(bool, smoothNormals, false);
};
bool operator==(const RobotModelImportOptions &a,
                const RobotModelImportOptions &b) {
  return (a.recomputeNormals() == b.recomputeNormals()) &&
         (a.smoothNormals() == b.smoothNormals());
}
bool operator<(const RobotModelImportOptions &a,
               const RobotModelImportOptions &b) {
  return std::make_tuple(a.recomputeNormals() == a.smoothNormals()) <
         std::make_tuple(b.recomputeNormals() == b.smoothNormals());
}
bool operator!=(const RobotModelImportOptions &a,
                const RobotModelImportOptions &b) {
  return !(a == b);
}
STRUCT_BEGIN(RobotModelImportOptions);
STRUCT_PROPERTY(recomputeNormals);
STRUCT_PROPERTY(smoothNormals);
STRUCT_END();

class RobotDisplayBase : public MeshDisplayBase {
protected:
  std::shared_ptr<Loader<RobotModel>> _robot_model_loader;
  std::shared_ptr<RobotState> _robot_state;
  Eigen::Isometry3d pose_temp = Eigen::Isometry3d::Identity();
  EventFlag _invalidated{ResourceEvents::instance().reload};
  Watcher _watcher;
  std::shared_ptr<MaterialOverride> _material_override =
      std::make_shared<MaterialOverride>();
  RobotDisplayBase() {}

public:
  virtual void renderSync(const RenderSyncContext &context) override;
  virtual void renderAsync(const RenderAsyncContext &context) override;
  PROPERTY(std::shared_ptr<MaterialOverride>, materialOverride,
           _material_override);
  PROPERTY(std::string, description, "/robot_description");
  PROPERTY(RobotModelImportOptions, importOptions);
  PROPERTY(bool, doubleSided, false);
};
DECLARE_TYPE(RobotDisplayBase, MeshDisplayBase);

class RobotStateTimeSeriesListener;

class RobotStateDisplayBase : public GenericFrameDisplay<RobotDisplayBase> {
  std::shared_ptr<TimeSeriesSubscriber> _subscriber;
  std::shared_ptr<RobotStateTimeSeriesListener> _listener;

protected:
  virtual void refreshTopic(const std::string &topic);
  RobotStateDisplayBase() {}

public:
  virtual void renderSync(const RenderSyncContext &context) override;
};
DECLARE_TYPE(RobotStateDisplayBase, RobotDisplayBase);

class RobotStateDisplay : public RobotStateDisplayBase {

public:
  PROPERTY(TopicProperty<sensor_msgs::JointState>, topic, "/joint_states");
  virtual void refresh() override {
    refreshTopic(topic().topic());
    RobotStateDisplayBase::refresh();
  }
};
DECLARE_TYPE(RobotStateDisplay, RobotStateDisplayBase);

/*
class DisplayRobotStateDisplay : public RobotStateDisplayBase {

public:
  PROPERTY(TopicProperty<moveit_msgs::DisplayRobotState>, topic,
           "/joint_states");
  virtual void refresh() override {
    refreshTopic(topic().topic());
    RobotStateDisplayBase::refresh();
  }
};
DECLARE_TYPE(DisplayRobotStateDisplay, RobotStateDisplayBase);
*/

class DisplayRobotStateDisplay : public GenericFrameDisplay<RobotDisplayBase> {
  // std::shared_ptr<const moveit_msgs::DisplayRobotState>
  //      _display_robot_state_message;

public:
  PROPERTY(TopicProperty<moveit_msgs::DisplayRobotState>, topic,
           "/display_robot_state");
  virtual void renderSync(const RenderSyncContext &context) override;
  virtual void renderAsync(const RenderAsyncContext &context) override;
};
DECLARE_TYPE(DisplayRobotStateDisplay, RobotDisplayBase);

/*
class RobotTrajectoryDisplay : public GenericFrameDisplay<RobotDisplayBase> {
  Watcher _trajectory_watcher;
  size_t _frame_index = 0;

public:
  PROPERTY(TopicProperty<moveit_msgs::DisplayTrajectory>, topic,
           "/move_group/display_planned_path");
  virtual void renderSync(const RenderSyncContext &context) override;
};
DECLARE_TYPE(RobotTrajectoryDisplay, RobotDisplayBase);
*/

class RobotTrajectoryDisplay : public GenericFrameDisplay<RobotDisplayBase> {
  Watcher _trajectory_watcher;
  std::vector<std::pair<double, std::shared_ptr<RobotState>>> _trajectory;
  std::shared_ptr<const moveit_msgs::DisplayTrajectory>
      _display_trajectory_message;
  int _max_steps = 10;
  bool _show_all = false;
  double _frame_time = 0;
  bool _update_show_all = false;
  std::thread _update_thread;
  std::condition_variable _update_condition;
  std::mutex _update_mutex;
  std::vector<double> _update_times;
  volatile size_t _frame_update = 0;
  volatile bool _update_exit = false;
  Watcher _update_parameter_watcher;
  double _update_speed = 1;

public:
  PROPERTY(TopicProperty<moveit_msgs::DisplayTrajectory>, topic,
           "/move_group/display_planned_path");
  PROPERTY(int, maxSteps, 10, min = 1);
  PROPERTY(bool, showAllStates, false);
  PROPERTY(double, speed, 1, min = 0);
  // PROPERTY(double, stateDisplayTime, 0.1);
  virtual void renderSync(const RenderSyncContext &context) override;
  virtual void renderAsync(const RenderAsyncContext &context) override;
  RobotTrajectoryDisplay();
  ~RobotTrajectoryDisplay();
};
DECLARE_TYPE(RobotTrajectoryDisplay, RobotDisplayBase);

class Transformer;
class RobotModelDisplay : public RobotDisplayBase {
  std::shared_ptr<Transformer> _transformer;

public:
  virtual void renderSync(const RenderSyncContext &context) override;
  virtual void renderAsync(const RenderAsyncContext &context) override;
};
DECLARE_TYPE(RobotModelDisplay, RobotDisplayBase);
