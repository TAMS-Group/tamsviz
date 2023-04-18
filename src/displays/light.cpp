// TAMSVIZ
// (c) 2020-2023 Philipp Ruppel

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

struct ShadowLightDisplayBase : LightDisplayBase {
protected:
  ShadowLightDisplayBase() {}

public:
  PROPERTY(double, shadowBias, 1.0, min = 0.0);
  PROPERTY(bool, shadowEnable, true);
};
DECLARE_TYPE(ShadowLightDisplayBase, LightDisplayBase);

struct DirectionalLight : ShadowLightDisplayBase {
  PROPERTY(double, shadowDepth, 10, min = 0.0);
  PROPERTY(double, shadowWidth, 5, min = 0.0);
  virtual void renderSync(const RenderSyncContext &context) override {
    ShadowLightDisplayBase::renderSync(context);
    if (visible()) {
      auto pose = context.pose;
      if (viewSpace()) {
        pose = this->transform().toIsometry3d();
      }
      LightBlock light;
      light.position.head(3) = pose.translation().cast<float>();
      light.view_matrix = pose.inverse().matrix().cast<float>();
      light.projection_matrix(0, 0) = 1.0 / shadowWidth();
      light.projection_matrix(1, 1) = 1.0 / shadowWidth();
      light.projection_matrix(2, 2) = -1.0 / shadowDepth();
      light.color = color().toLinearVector4f().head(3) * (float)brightness();
      light.type =
          (uint32_t(shadowEnable() ? LightType::DirectionalShadow
                                   : LightType::Directional) |
           (viewSpace() ? uint32_t(LightType::ViewSpace) : uint32_t(0)));
      light.shadow_bias = shadowBias();
      context.render_list->push(light);
    }
  }
};
DECLARE_TYPE_C(DirectionalLight, ShadowLightDisplayBase, Light);

struct PointLight : ShadowLightDisplayBase {
  virtual void renderSync(const RenderSyncContext &context) override {
    ShadowLightDisplayBase::renderSync(context);
    if (visible()) {
      auto pose = context.pose;
      if (viewSpace()) {
        pose = this->transform().toIsometry3d();
      }
      LightBlock light;
      light.position.head(3) = pose.translation().cast<float>();
      light.view_matrix = pose.inverse().matrix().cast<float>();
      light.color = color().toLinearVector4f().head(3) * (float)brightness();
      light.type =
          (uint32_t(shadowEnable() ? LightType::PointShadow
                                   : LightType::Point) |
           (viewSpace() ? uint32_t(LightType::ViewSpace) : uint32_t(0)));
      light.shadow_bias = shadowBias();
      context.render_list->push(light);
    }
  }
};
DECLARE_TYPE_C(PointLight, ShadowLightDisplayBase, Light);

struct SpotLight : ShadowLightDisplayBase {
  PROPERTY(double, softness, 0.5, min = 0.0, max = 1.0);
  PROPERTY(double, angle, 90, min = 0.0, max = 180);
  PROPERTY(double, shadowNear, 0.1, min = 0.0);
  PROPERTY(double, shadowFar, 10.0, min = 0.0);
  virtual void renderSync(const RenderSyncContext &context) override {
    ShadowLightDisplayBase::renderSync(context);
    if (visible()) {
      auto pose = context.pose;
      if (viewSpace()) {
        pose = this->transform().toIsometry3d();
      }
      LightBlock light;
      light.position.head(3) = pose.translation().cast<float>();
      light.view_matrix = pose.inverse().matrix().cast<float>();
      light.projection_matrix =
          projectionMatrix(std::max(1e-6, std::min(180.0 - 1e-3, angle())) *
                               M_PI / 180.0,
                           1.0, shadowNear(), shadowFar())
              .cast<float>();
      light.color = color().toLinearVector4f().head(3) * (float)brightness();
      light.type =
          (uint32_t(shadowEnable() ? LightType::SpotShadow : LightType::Spot) |
           (viewSpace() ? uint32_t(LightType::ViewSpace) : uint32_t(0)));
      light.softness = std::max(0.0001f, std::min(1.0f, (float)softness()));
      light.shadow_bias = shadowBias();
      context.render_list->push(light);
    }
  }
};
DECLARE_TYPE_C(SpotLight, ShadowLightDisplayBase, Light);
