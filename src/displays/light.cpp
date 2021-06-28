// TAMSVIZ
// (c) 2020-2021 Philipp Ruppel

#include "light.h"

#include "../core/log.h"

struct AmbientLight : LightDisplayBase {
  virtual void renderSync(const RenderSyncContext &context) override {
    LightDisplayBase::renderSync(context);
    if (visible()) {
      LightBlock light;
      light.color = color().toLinearVector4f().head(3) * (float)brightness();
      light.type = (uint32_t)LightType::Ambient;
      context.render_list->push(light);
    }
  }
};
DECLARE_TYPE_C(AmbientLight, LightDisplayBase, Light);

struct DirectionalLight : LightDisplayBase {
  virtual void renderSync(const RenderSyncContext &context) override {
    LightDisplayBase::renderSync(context);
    if (visible()) {
      auto pose = context.pose;
      if (viewSpace()) {
        pose = this->transform().toIsometry3d();
      }
      LightBlock light;
      light.position.head(3) = pose.translation().cast<float>();
      light.pose = pose.inverse().matrix().cast<float>();
      light.color = color().toLinearVector4f().head(3) * (float)brightness();
      light.type =
          (uint32_t(LightType::Directional) |
           (viewSpace() ? uint32_t(LightType::ViewSpace) : uint32_t(0)));
      context.render_list->push(light);
    }
  }
};
DECLARE_TYPE_C(DirectionalLight, LightDisplayBase, Light);

struct PointLight : LightDisplayBase {
  virtual void renderSync(const RenderSyncContext &context) override {
    LightDisplayBase::renderSync(context);
    if (visible()) {
      auto pose = context.pose;
      if (viewSpace()) {
        pose = this->transform().toIsometry3d();
      }
      LightBlock light;
      light.position.head(3) = pose.translation().cast<float>();
      light.pose = pose.inverse().matrix().cast<float>();
      light.color = color().toLinearVector4f().head(3) * (float)brightness();
      light.type =
          (uint32_t(LightType::Point) |
           (viewSpace() ? uint32_t(LightType::ViewSpace) : uint32_t(0)));
      context.render_list->push(light);
    }
  }
};
DECLARE_TYPE_C(PointLight, LightDisplayBase, Light);

struct SpotLight : LightDisplayBase {
  PROPERTY(double, softness, 0.5, min = 0.0, max = 1.0);
  PROPERTY(double, angle, 90, min = 0.0, max = 180);
  virtual void renderSync(const RenderSyncContext &context) override {
    LightDisplayBase::renderSync(context);
    if (visible()) {
      auto pose = context.pose;
      if (viewSpace()) {
        pose = this->transform().toIsometry3d();
      }
      LightBlock light;
      light.position.head(3) = pose.translation().cast<float>();
      light.pose =
          (projectionMatrix(std::max(1e-6, std::min(180.0 - 1e-3, angle())) *
                                M_PI / 180.0,
                            1.0, 0.01, 100.0) *
           pose.inverse().matrix())
              .cast<float>();
      light.color = color().toLinearVector4f().head(3) * (float)brightness();
      light.type =
          (uint32_t(LightType::Spot) |
           (viewSpace() ? uint32_t(LightType::ViewSpace) : uint32_t(0)));
      light.softness = std::max(0.0001f, std::min(1.0f, (float)softness()));
      context.render_list->push(light);
    }
  }
};
DECLARE_TYPE_C(SpotLight, LightDisplayBase, Light);
