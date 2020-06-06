// TAMSVIZ
// (c) 2020 Philipp Ruppel

#pragma once

#include "mesh.h"

class FrameDisplayMixin {
protected:
  static void renderSyncRecursiveImpl(const RenderSyncContext &context,
                                      Display *display,
                                      const std::string &frame,
                                      const Pose &transform);
};

std::vector<std::string> FrameDisplay_listFrames(const Property &);

template <class Base>
class GenericFrameDisplay : public Base, private FrameDisplayMixin {

public:
  virtual void renderSyncRecursive(const RenderSyncContext &context) override {
    renderSyncRecursiveImpl(context, this, frame(), transform());
  }
  PROPERTY(std::string, frame, "", list = &FrameDisplay_listFrames);
  PROPERTY(Pose, transform);
};

class FrameDisplayBase : public GenericFrameDisplay<MeshDisplayBase> {
protected:
  FrameDisplayBase() {}
};
DECLARE_TYPE(FrameDisplayBase, MeshDisplayBase);

struct DisplayGroup : public GenericFrameDisplay<DisplayGroupBase> {};
DECLARE_TYPE(DisplayGroup, DisplayGroupBase);
