// TAMSVIZ
// (c) 2020 Philipp Ruppel

#include "document.h"

#include "bagplayer.h"
#include "transformer.h"
#include "workspace.h"

#include "../render/renderer.h"

Display::Display() {}

TrackBase::TrackBase() {}

struct PropertyTestDisplay : Display {
  PROPERTY(float, flt, 0.0, min = 0, max = 1);
  PROPERTY(Eigen::Vector3d, v3d, Eigen::Vector3d::Zero());
  PROPERTY(Eigen::Vector2d, v2d, Eigen::Vector2d::Zero());
  PROPERTY(int, i, 1);
};
DECLARE_TYPE(PropertyTestDisplay, Display);

static float srgbGamma2Linear(float srgb) {
  if (srgb < 0.04045f) {
    return srgb * (25.0f / 232.0f);
  } else {
    return std::pow((200.0 * srgb + 11.0f) * (1.0f / 211.0f), 12.0 / 5.0);
  }
}

std::vector<std::string> WorldDisplay::_listFrames(const Property &) {
  auto ret = LockScope()->document()->display()->transformer->list();
  std::sort(ret.begin(), ret.end());
  return ret;
}

static Eigen::Vector3f srgb2Linear(const Eigen::Vector3f &srgb) {
  /*Eigen::Matrix3f m;
  m << 0.41239f, 0.35758f, 0.18048f, //
      0.21263f, 0.71516f, 0.07219f,  //
      0.01933f, 0.11919f, 0.95053f;
  return m * Eigen::Vector3f(srgbGamma2Linear(srgb.x()),
                             srgbGamma2Linear(srgb.y()),
                             srgbGamma2Linear(srgb.z()));*/
  return Eigen::Vector3f(srgbGamma2Linear(srgb.x()), srgbGamma2Linear(srgb.y()),
                         srgbGamma2Linear(srgb.z()));
}

Eigen::Vector4f Color4::toLinearVector4f() const {
  Eigen::Vector4f ret;
  ret.head(3) =
      srgb2Linear(Eigen::Vector3f((float)r(), (float)g(), (float)b()));
  ret[3] = (float)a();
  return ret;
}

WorldDisplay::WorldDisplay() {
  transformer = std::make_shared<Transformer>();
  name() = "Document";
}

void WorldDisplay::renderSync(const RenderSyncContext &context) {
  transformer->update(fixedFrame());
  LightBlock light;
  light.color =
      backgroundColor().toLinearVector4f().head(3) * (float)ambientLighting();
  light.type = (uint32_t)LightType::Ambient;
  context.render_list->push(light);
}

void DisplayGroupBase::renderSyncRecursive(const RenderSyncContext &context) {
  renderSync(context);
  for (auto &display : displays()) {
    display->renderSyncRecursive(context);
  }
}

Eigen::Isometry3d Orientation::toIsometry3d() const {
  return Eigen::Translation3d(0, 0, 0) *
         Eigen::AngleAxisd(yaw() * (M_PI / 180.0), Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(pitch() * (M_PI / 180.0), Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(roll() * (M_PI / 180.0), Eigen::Vector3d::UnitX());
}

Eigen::Isometry3d Pose::toIsometry3d() const {
  return Eigen::Translation3d(position()) *
         Eigen::AngleAxisd(orientation().yaw() * (M_PI / 180.0),
                           Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(orientation().pitch() * (M_PI / 180.0),
                           Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(orientation().roll() * (M_PI / 180.0),
                           Eigen::Vector3d::UnitX());
}

std::shared_ptr<AnnotationBranch> AnnotationTrack::branch(bool create) {
  LockScope ws;
  std::string name = "";
  if (ws->player) {
    name = ws->player->fileName();
  }
  for (auto &branch : branches()) {
    if (branch->name() == name) {
      return branch;
    }
  }
  if (create) {
    auto branch = std::make_shared<AnnotationBranch>();
    branch->name() = name;
    branches().push_back(branch);
    return branch;
  } else {
    return nullptr;
  }
}
