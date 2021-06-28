// TAMSVIZ
// (c) 2020-2021 Philipp Ruppel

#pragma once

#include "../core/document.h"
#include "../core/watcher.h"
#include "../render/renderer.h"

#include "marker.h"
#include "mesh.h"

struct GridDisplay : MarkerFrameDisplayBase {
  PROPERTY(double, cellSize, 1.0, min = 0.0, max = 1000.0);
  PROPERTY(size_t, cellCount, 10, min = 1, max = 1000);
  PROPERTY(float, lineWidth, 0.01, min = 0.0, max = 1000);
  PROPERTY(Color4, color, Color4(0.5, 0.5, 0.5));
  Watcher _watcher;
  virtual void renderSync(const RenderSyncContext &context) override;
};
DECLARE_TYPE(GridDisplay, MarkerDisplayBase);
