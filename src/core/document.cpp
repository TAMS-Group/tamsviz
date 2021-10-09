// TAMSVIZ
// (c) 2020-2021 Philipp Ruppel

#include "document.h"

#include "../components/environment.h"
#include "../components/rendering.h"
#include "../render/renderer.h"
#include "../scene/node.h"
#include "bagplayer.h"
#include "interaction.h"
#include "tracks.h"
#include "transformer.h"
#include "workspace.h"

Display::Display() {}

TrackBase::TrackBase() {}

float srgbGamma2Linear(float srgb) {
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

bool Display::interact(
    const Interaction &interaction) { // interaction.ignored =
  // true;
  return false;
}

static Eigen::Vector3f srgb2Linear(const Eigen::Vector3f &srgb) {
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
  rendering() = std::make_shared<RenderingComponent>();
  environment() = std::make_shared<EnvironmentComponent>();
  name() = "Document";
}

void WorldDisplay::renderSync(const RenderSyncContext &context) {

  transformer->update(fixedFrame());

  DisplayGroupBase::renderSync(context);

  _components_render = _components_refresh;
  for (auto &component : _components_render) {
    component->renderSync(context);
  }
}

void WorldDisplay::renderAsync(const RenderAsyncContext &context) {
  DisplayGroupBase::renderAsync(context);

  for (auto &component : _components_render) {
    component->renderAsync(context);
  }
}

void WorldDisplay::refresh() {
  DisplayGroupBase::refresh();

  _components_refresh.clear();
  recurseObjects([&](const std::shared_ptr<Object> &object) {
    if (object) {
      if (auto component = std::dynamic_pointer_cast<Component>(object)) {
        _components_refresh.push_back(component);
        component->refresh();
      }
    }
  });
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

void Pose::fromIsometry3d(const Eigen::Isometry3d &pose) {
  position() = pose.translation();
  Eigen::Vector3d angles = pose.linear().eulerAngles(2, 1, 0);
  orientation().yaw() = angles.x() * (180.0 / M_PI);
  orientation().pitch() = angles.y() * (180.0 / M_PI);
  orientation().roll() = angles.z() * (180.0 / M_PI);
}

void Window::refreshRecursive() {
  recurseObjects([](const std::shared_ptr<Object> &object) {
    if (object) {
      if (auto window = std::dynamic_pointer_cast<Window>(object)) {
        window->refresh();
      }
    }
  });
}

void Window::refresh() {}

void Display::refreshRecursive() { refresh(); }

void Display::refresh() {}

void DisplayGroupBase::refreshRecursive() {
  for (auto &display : displays()) {
    display->refreshRecursive();
  }
  refresh();
}
