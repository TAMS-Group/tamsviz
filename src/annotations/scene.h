// TAMSVIZ
// (c) 2020-2021 Philipp Ruppel

#pragma once

#include "../core/document.h"
#include "../core/tracks.h"

class SceneNode;

class SceneAnnotationBase : public AnnotationBase {
private:
  std::shared_ptr<SceneNode> _node;
  std::shared_ptr<SceneContext> _context;
  bool _reset = false;

protected:
  SceneAnnotationBase();
  virtual void update(const std::shared_ptr<SceneNode> &node) {}

public:
  std::shared_ptr<SceneNode> node();
  virtual void renderSync(const RenderSyncContext &context,
                          const std::shared_ptr<TrackBase> &track,
                          const std::shared_ptr<AnnotationSpan> &span);
  virtual void renderAsync(const RenderAsyncContext &context);
  virtual bool pick(uint32_t id) const;
  virtual bool interact(const Interaction &interaction);
  bool selected();
  // virtual void clear();
};
DECLARE_TYPE(SceneAnnotationBase, AnnotationBase);
