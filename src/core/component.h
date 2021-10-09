// TAMSVIZ
// (c) 2020-2021 Philipp Ruppel

#pragma once

#include "object.h"

class RenderSyncContext;
class RenderAsyncContext;

struct Component : Object {
  virtual void renderSync(const RenderSyncContext &context) {}
  virtual void renderAsync(const RenderAsyncContext &context) {}
  virtual void refresh() {}
};
DECLARE_TYPE(Component, Object);
