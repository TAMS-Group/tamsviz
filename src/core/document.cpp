// TAMSVIZ
// (c) 2020-2021 Philipp Ruppel

#include "document.h"

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
  name() = "Document";
}

void WorldDisplay::renderSync(const RenderSyncContext &context) {
  transformer->update(fixedFrame());
  DisplayGroupBase::renderSync(context);
  {
    LightBlock light;
    light.color =
        backgroundColor().toLinearVector4f().head(3) * (float)ambientLighting();
    light.type = (uint32_t)LightType::Ambient;
    context.render_list->push(light);
  }
  {
    RenderParameters params;
    params.shadow_map_resolution = rendering()->shadowMapResolution();
    params.shadow_cube_resolution = rendering()->shadowCubeResolution();
    context.render_list->put(params);
  }
  /* _current_scene_annotations.clear();
   {
     LockScope ws;
     if (ws->player && ws->document()->timeline()) {
       auto current_time = ws->player->time();
       for (auto &track : ws->document()->timeline()->tracks()) {
         if (auto annotation_track =
                 std::dynamic_pointer_cast<AnnotationTrack>(track)) {
           if (auto branch = annotation_track->branch()) {
             for (auto &span : branch->spans()) {
               if (span->start() <= current_time &&
                   span->start() + span->duration() >= current_time) {
                 for (auto &annotation : span->annotations()) {
                   if (auto scene_annotation =
                           std::dynamic_pointer_cast<SceneAnnotationBase>(
                               annotation)) {
                     _current_scene_annotations.push_back(scene_annotation);
                   }
                 }
               }
             }
           }
         }
       }
     }
   }
   for (auto &scene_annotation : _current_scene_annotations) {
     scene_annotation->renderSync(context);
 }*/
}

void WorldDisplay::renderAsync(const RenderAsyncContext &context) {
  DisplayGroupBase::renderAsync(context);
  /*for (auto &scene_annotation : _current_scene_annotations) {
    scene_annotation->renderAsync(context);
}*/
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

void Display::refreshRecursive() { refresh(); }

void Display::refresh() {}

void DisplayGroupBase::refreshRecursive() {
  for (auto &display : displays()) {
    display->refreshRecursive();
  }
  refresh();
}
