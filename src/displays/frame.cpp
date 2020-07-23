// TAMSVIZ
// (c) 2020 Philipp Ruppel

#include "frame.h"

#include "../core/transformer.h"
#include "../core/workspace.h"

void FrameDisplayMixin::renderSyncRecursiveImpl(
    const RenderSyncContext &context, Display *display, Frame &frame,
    const Pose &transform) {
  RenderSyncContext c = context;
  if (!frame.empty()) {
    if (auto f = frame.pose(LockScope()->document()->display()->transformer)) {
      c.pose = Eigen::Affine3d(*f);
    }
  }
  _frame_pose = c.pose;
  c.pose = c.pose * transform.toIsometry3d();
  display->renderSync(c);
  if (auto *group = dynamic_cast<DisplayGroupBase *>(display)) {
    for (auto &display2 : group->displays()) {
      display2->renderSyncRecursive(c);
    }
  }
}

std::vector<std::string> FrameDisplay_listFrames(const Property &) {
  auto ret = LockScope()->document()->display()->transformer->list();
  std::sort(ret.begin(), ret.end());
  return ret;
}
