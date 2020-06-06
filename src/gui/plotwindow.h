// TAMSVIZ
// (c) 2020 Philipp Ruppel

#pragma once

#include "renderwindow.h"

#include "../core/topic.h"
#include "../render/renderer.h"
#include "../render/texture.h"

class PlotWindow : public ContentWindowBase {
  static constexpr double minZoom() { return 0.01; }
  static constexpr double maxZoom() { return 1000000; }

public:
  PlotWindow();
  PROPERTY(std::string, topic, "");
  PROPERTY(double, duration, 5.0, min = 1e-3);
};
DECLARE_TYPE(PlotWindow, ContentWindowBase);
