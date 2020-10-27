// TAMSVIZ
// (c) 2020 Philipp Ruppel

#pragma once

#include "../core/document.h"
#include "../render/renderer.h"
#include "../render/transformations.h"

#include "frame.h"

struct LightDisplayBase : FrameDisplayBase {
protected:
  LightDisplayBase() {}

public:
  PROPERTY(Color3, color, Color3());
  PROPERTY(double, brightness, 1.0, min = 0.0);
  PROPERTY(bool, viewSpace, false);
};
DECLARE_TYPE(LightDisplayBase, FrameDisplayBase);
