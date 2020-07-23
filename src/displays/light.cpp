// TAMSVIZ
// (c) 2020 Philipp Ruppel

#include "light.h"

#include "../core/log.h"

struct AmbientLight : LightDisplayBase {
  virtual void renderSync(const RenderSyncContext &context) override {
    LightDisplayBase::renderSync(context);
    LightBlock light;
    light.color = color().toLinearVector4f().head(3) * (float)brightness();
    light.type = (uint32_t)LightType::Ambient;
    context.render_list->push(light);
  }
};
DECLARE_TYPE(AmbientLight, LightDisplayBase);

struct DirectionalLight : LightDisplayBase {
  virtual void renderSync(const RenderSyncContext &context) override {
    LightDisplayBase::renderSync(context);
    auto pose = context.pose;
    LightBlock light;
    light.position.head(3) = pose.translation().cast<float>();
    light.pose = pose.inverse().matrix().cast<float>();
    light.color = color().toLinearVector4f().head(3) * (float)brightness();
    light.type = (uint32_t)LightType::Directional;
    context.render_list->push(light);
  }
};
DECLARE_TYPE(DirectionalLight, LightDisplayBase);

struct PointLight : LightDisplayBase {
  virtual void renderSync(const RenderSyncContext &context) override {
    LightDisplayBase::renderSync(context);
    auto pose = context.pose;
    LightBlock light;
    light.position.head(3) = pose.translation().cast<float>();
    light.pose = pose.inverse().matrix().cast<float>();
    light.color = color().toLinearVector4f().head(3) * (float)brightness();
    light.type = (uint32_t)LightType::Point;
    context.render_list->push(light);
  }
};
DECLARE_TYPE(PointLight, LightDisplayBase);

struct SpotLight : LightDisplayBase {
  PROPERTY(double, softness, 0.5, min = 0.0, max = 1.0);
  virtual void renderSync(const RenderSyncContext &context) override {
    LightDisplayBase::renderSync(context);
    auto pose = context.pose;
    LightBlock light;
    light.position.head(3) = pose.translation().cast<float>();
    light.pose =
        (projectionMatrix(std::max(1e-6, std::min(180.0 - 1e-3, angle())) *
                              M_PI / 180.0,
                          1.0, 0.01, 100.0) *
         pose.inverse().matrix())
            .cast<float>();
    light.color = color().toLinearVector4f().head(3) * (float)brightness();
    light.type = (uint32_t)LightType::Spot;
    light.softness = std::max(0.0001f, std::min(1.0f, (float)softness()));
    context.render_list->push(light);
  }
};
DECLARE_TYPE(SpotLight, LightDisplayBase);
