// TAMSVIZ
// (c) 2020-2023 Philipp Ruppel

#include "camera.h"

#include "../components/environment.h"
#include "../core/workspace.h"
#include "../render/opengl.h"
#include "../render/transformations.h"

#include <QImage>
#include <QOpenGLFramebufferObject>
#include <QOpenGLPaintDevice>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>

struct CameraDisplayContext {
  ros::NodeHandle node;
  image_transport::ImageTransport itrans;
  image_transport::CameraPublisher campub;
  CameraDisplayContext(const CameraDisplayParams &params)
      : node(params.prefix), itrans(node),
        campub(itrans.advertiseCamera("image_raw", params.queue_size)) {
    LOG_INFO("virtual camera context created");
  }
};

void CameraDisplay::renderSync(const RenderSyncContext &context) {

  _width = std::max(1, (int)imageWidth());
  _height = std::max(1, (int)imageHeight());

  LOG_INFO("rendering camera view " << _width << " " << _height);

  {
    LockScope ws;
    if (auto env = std::dynamic_pointer_cast<EnvironmentComponent>(
            ws->document()->display()->environment())) {
      _bgcolor = env->backgroundColor().toLinearVector4f() *
                 float(env->backgroundBrightness());
    } else {
      LOG_ERROR_THROTTLE(1, "document broken, no environment component");
      _bgcolor.setZero();
    }
  }

  /*
  _camera_block.view_matrix =
      transform().toIsometry3d().matrix().inverse().cast<float>();
  if (!frame().empty()) {
    if (auto transformer = LockScope()->document()->display()->transformer) {
      if (auto f = frame().pose(transformer)) {
        _camera_block.view_matrix =
            _camera_block.view_matrix * (*f).matrix().inverse().cast<float>();
        LOG_DEBUG("view position " << f->translation());
      } else {
        LOG_WARN_THROTTLE(1, "frame not found: " << frame().name());
      }
    } else {
      LOG_ERROR("no transformer");
    }
  }
  */

  _camera_block.view_matrix = globalPose().matrix().inverse().cast<float>();

  {
    float far = (100.0 + (viewPosition() - viewTarget()).norm() * 2.0);
    float near = far * 0.0001f;
    Eigen::Matrix4d projection_matrix = projectionMatrix(
        fieldOfView() * M_PI / 180, _height * 1.0 / _width, near, far);
    _camera_block.projection_matrix = projection_matrix.cast<float>();
  }

  _camera_block.flags = 0;

  _multi_sampling = multiSampling();

  switch (sampleShading()) {
  case 1:
    _camera_block.flags |= CameraBlock::SampleShadingFlag;
    break;
  case 2:
    _camera_block.flags |= CameraBlock::SampleShadingFlag;
    _camera_block.flags |= CameraBlock::TransparentSampleShadingFlag;
    break;
  }

  _params.frame = frame().name();
  _params.prefix = prefix();
  _params.queue_size = queueSize();

  if (_watcher.changed(_params)) {
    _camera_context = std::make_shared<CameraDisplayContext>(_params);
  }
}

void CameraDisplay::renderViewsAsync(const RenderViewsAsyncContext &context) {

  LOG_INFO("rendering camera view " << _width << " " << _height);

  _render_target.update(_width, _height, _multi_sampling);
  _render_target.bind();
  V_GL(glViewport(0, 0, _width, _height));
  V_GL(glClearColor(_bgcolor.x(), _bgcolor.y(), _bgcolor.z(), _bgcolor.w()));
  V_GL(glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT));

  context.renderer->render(_render_target, _camera_block, *context.render_list);

  _render_target._front_framebuffer.bind();

  auto image = std::make_shared<sensor_msgs::Image>();
  {
    image->header.frame_id = _params.frame;

    image->width = _width;
    image->height = _height;
    image->encoding = "rgba8";
    image->step = _width * 4;

    image->data.resize(_width * _height * 4);
    V_GL(glReadPixels(0, 0, _width, _height, GL_RGBA, GL_UNSIGNED_BYTE,
                      image->data.data()));
    V_GL(glBindFramebuffer(GL_FRAMEBUFFER, 0));

    uint32_t *pixels = (uint32_t *)image->data.data();
    for (size_t y = 0; y < _height / 2; y++) {
      size_t i0 = y * _width;
      size_t i1 = (_height - 1 - y) * _width;
      for (size_t x = 0; x < _width; x++) {
        std::swap(pixels[i0], pixels[i1]);
        i0++;
        i1++;
      }
    }
  }

  {
    std::lock_guard<std::mutex> lock(_image_mutex);
    _image_message = image;
  }

  if (_camera_context) {

    auto info = std::make_shared<sensor_msgs::CameraInfo>();
    info->header.frame_id = _params.frame;
    info->width = _width;
    info->height = _height;

    LOG_INFO("publish image " << image->width << " " << image->height << " "
                              << info->width << " " << info->height);

    _camera_context->campub.publish(*image, *info);
  }

  LOG_INFO("camera view rendererd");
}