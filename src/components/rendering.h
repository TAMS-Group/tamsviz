// TAMSVIZ
// (c) 2020-2023 Philipp Ruppel

#pragma once

#include "../core/document.h"

struct RenderingComponent : Component {

  PROPERTY(int, multiSampling, 4, min = 0, max = 16);
  PROPERTY(int, sampleShading, 1, min = 0, max = 2);
  PROPERTY(int, shadowMapResolution, 1024, min = 1);
  PROPERTY(int, shadowCubeResolution, 512, min = 1);
  PROPERTY(double, exposure, 1, min = 0);
  PROPERTY(bool, toneMapping, true);
  PROPERTY(double, blackLevel, 0);
  PROPERTY(double, whiteLevel, 1);

  virtual void renderSync(const RenderSyncContext &context) override;
};
DECLARE_TYPE(RenderingComponent, Component);
