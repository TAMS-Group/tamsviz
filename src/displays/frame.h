// TAMSVIZ
// (c) 2020-2021 Philipp Ruppel

#pragma once

#include "mesh.h"

#include "../core/transformer.h"

class FrameDisplayMixin {
  Eigen::Affine3d _frame_pose = Eigen::Affine3d::Identity();

protected:
  void renderSyncRecursiveImpl(const RenderSyncContext &context,
                               Display *display, Frame &frame,
                               const Pose &transform);

public:
  const Eigen::Affine3d &framePose() const { return _frame_pose; }
};

std::vector<std::string> FrameDisplay_listFrames(const Property &);

template <class Base>
class GenericFrameDisplay : public Base, public FrameDisplayMixin {

public:
  virtual void renderSyncRecursive(const RenderSyncContext &context) override {
    FrameDisplayMixin::renderSyncRecursiveImpl(context, this, frame(),
                                               transform());
  }
  PROPERTY(Frame, frame);
  PROPERTY(Pose, transform);
  Eigen::Affine3d globalPose() const {
    return framePose() * transform().toIsometry3d();
  }
};

class FrameDisplayBase : public GenericFrameDisplay<MeshDisplayBase> {
protected:
  FrameDisplayBase() {}
};
DECLARE_TYPE(FrameDisplayBase, MeshDisplayBase);

struct DisplayGroup : public GenericFrameDisplay<DisplayGroupBase> {};
DECLARE_TYPE(DisplayGroup, DisplayGroupBase);
