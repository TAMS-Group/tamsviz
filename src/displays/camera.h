// TAMSVIZ
// (c) 2020-2023 Philipp Ruppel

#pragma once

#include "../core/document.h"
#include "../core/transformer.h"
#include "../render/renderer.h"
#include "../render/rendertarget.h"
#include "frame.h"

#include <sensor_msgs/Image.h>

struct CameraDisplayParams {
  std::string prefix;
  std::string frame;
  size_t queue_size = 0;
  bool operator==(const CameraDisplayParams &other) const {
    return                                //
        prefix == other.prefix &&         //
        frame == other.frame &&           //
        queue_size == other.queue_size && //
        true;                             //
  }
};

struct CameraDisplayContext;

class CameraDisplay : public FrameDisplayBase {
  Watcher _watcher;
  CameraDisplayParams _params;
  int _width = 256;
  int _height = 256;
  int _multi_sampling = 0;
  CameraBlock _camera_block;
  RenderTarget _render_target;
  // Eigen::Matrix4d _view_matrix;
  // Eigen::Matrix4d _projection_matrix;
  Eigen::Vector4f _bgcolor;
  std::mutex _image_mutex;
  std::shared_ptr<sensor_msgs::Image> _image_message;

private:
  std::shared_ptr<CameraDisplayContext> _camera_context;

public:
  virtual void renderSync(const RenderSyncContext &context);
  virtual void renderViewsAsync(const RenderViewsAsyncContext &context);

public:
  std::shared_ptr<sensor_msgs::Image> image() {
    std::shared_ptr<sensor_msgs::Image> ret;
    {
      std::lock_guard<std::mutex> lock(_image_mutex);
      ret = _image_message;
    }
    return ret;
  }

public:
  PROPERTY(std::string, prefix, "tamsviz_camera");
  // PROPERTY(Frame, frame, Frame("tamsviz_camera"));
  PROPERTY(size_t, queueSize, 3);
  PROPERTY(Eigen::Vector3d, viewPosition, Eigen::Vector3d(3, 3, 3));
  PROPERTY(Eigen::Vector3d, viewTarget, Eigen::Vector3d(0, 0, 0));
  PROPERTY(double, fieldOfView, 180 / M_PI);
  PROPERTY(size_t, imageWidth, 640, min = 1, step_scale = 1);
  PROPERTY(size_t, imageHeight, 480, min = 1, step_scale = 1);
  PROPERTY(int, multiSampling, 4, min = 0, max = 16);
  PROPERTY(int, sampleShading, 1, min = 0, max = 2);
  // PROPERTY(Pose, transform);
};
DECLARE_TYPE(CameraDisplay, Display);
