// TAMSVIZ
// (c) 2020 Philipp Ruppel

#pragma once

#include "../core/document.h"
#include "../core/loader.h"
#include "../core/topic.h"

#include "../render/renderer.h"
#include "../render/resource.h"

#include "frame.h"
#include "material.h"
#include "mesh.h"

#include <sensor_msgs/JointState.h>

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
  RobotDisplayBase() {}

public:
  virtual void renderSync(const RenderSyncContext &context) override;
  virtual void renderAsync(const RenderAsyncContext &context) override;
  PROPERTY(std::shared_ptr<MaterialOverride>, materialOverride,
           std::make_shared<MaterialOverride>());
  PROPERTY(std::string, description, "/robot_description");
  PROPERTY(RobotModelImportOptions, importOptions);
};
DECLARE_TYPE(RobotDisplayBase, MeshDisplayBase);

class RobotStateDisplay : public GenericFrameDisplay<RobotDisplayBase> {
  std::shared_ptr<sensor_msgs::JointState> _joint_state_message;

public:
  PROPERTY(TopicProperty<sensor_msgs::JointState>, topic, "/joint_states");
  virtual void renderSync(const RenderSyncContext &context) override;
  virtual void renderAsync(const RenderAsyncContext &context) override;
};
DECLARE_TYPE(RobotStateDisplay, RobotDisplayBase);

class Transformer;
class RobotModelDisplay : public RobotDisplayBase {
  std::shared_ptr<Transformer> _transformer;

public:
  virtual void renderSync(const RenderSyncContext &context) override;
  virtual void renderAsync(const RenderAsyncContext &context) override;
};
DECLARE_TYPE(RobotModelDisplay, RobotDisplayBase);
