// TAMSVIZ
// (c) 2020-2021 Philipp Ruppel

#include "drvcam.h"

#include <camera_info_manager/camera_info_manager.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>

CameraDriverDisplay::CameraDriverDisplay() {

  auto com = _worker_com;

  std::thread([com]() {

    LOG_INFO("camera thread started");

    Watcher watcher;

    bool enabled = false;
    bool changed = false;

    std::chrono::steady_clock::time_point reconnect_time;
    std::chrono::steady_clock::time_point retry_time;

    Params params;

    struct Connection {
      ros::NodeHandle node;
      cv_bridge::CvImage bridge;
      image_transport::ImageTransport itrans;
      image_transport::CameraPublisher campub;
      cv::VideoCapture vidcap;
      camera_info_manager::CameraInfoManager infman;
      Connection(const Params &params)
          : node(params.prefix), itrans(node),
            campub(itrans.advertiseCamera("image_raw", params.queue_size)),
            infman(node, params.calibration) {
        LOG_INFO("opening camera");
        if (vidcap.open(params.index)) {
          LOG_SUCCESS("camera opened");
        } else {
          LOG_ERROR("failed to open camera");
        }
      }
    };
    std::shared_ptr<Connection> connection;

    auto handle_error = [&]() {
      if (reconnect_time == std::chrono::steady_clock::time_point()) {
        LOG_WARN("sheduling camera reconnect");
        reconnect_time =
            std::chrono::steady_clock::now() + std::chrono::seconds(5);
      }
      retry_time = std::chrono::steady_clock::now() + std::chrono::seconds(1);
    };

    while (true) {

      {
        std::unique_lock<std::mutex> lock(com->mutex);
        while (true) {

          if (com->stop) {
            LOG_INFO("camera thread returning");
            return;
          }

          params = com->params;
          enabled = (com->capture && !params.prefix.empty() &&
                     params.queue_size != 0 && !params.calibration.empty() &&
                     !params.frame.empty());
          changed = watcher.changed(params);

          if (reconnect_time != std::chrono::steady_clock::time_point() &&
              std::chrono::steady_clock::now() > reconnect_time) {
            reconnect_time = std::chrono::steady_clock::time_point();
            LOG_INFO("attempting camera reconnect");
            changed = true;
          }

          if (changed) {
            break;
          }

          if (enabled &&
              retry_time == std::chrono::steady_clock::time_point()) {
            break;
          }

          if (enabled && std::chrono::steady_clock::now() > retry_time) {
            retry_time = std::chrono::steady_clock::time_point();
            LOG_INFO("retry camera read");
            break;
          }

          auto loop_time =
              std::chrono::steady_clock::now() + std::chrono::seconds(2);
          if (reconnect_time != std::chrono::steady_clock::time_point()) {
            loop_time = std::min(loop_time, reconnect_time);
          }
          if (retry_time != std::chrono::steady_clock::time_point()) {
            loop_time = std::min(loop_time, retry_time);
          }
          com->condition.wait_until(lock, loop_time);
        }
      }

      if (changed) {
        if (enabled) {
          LOG_INFO("opening camera");
          connection = std::make_shared<Connection>(params);
          LOG_INFO("open camera done");
        } else if (connection) {
          LOG_INFO("closing camera");
          connection = nullptr;
          LOG_INFO("camera closed");
        }
      }

      if (!connection) {
        continue;
      }

      if (!connection->vidcap.grab()) {
        LOG_ERROR("failed to read camera image");
        handle_error();
        continue;
      }

      connection->bridge.header.frame_id = params.frame;
      connection->bridge.header.stamp = ros::Time::now();

      if (!connection->vidcap.retrieve(connection->bridge.image)) {
        LOG_ERROR("failed to retrieve camera image");
        handle_error();
        continue;
      }

      reconnect_time = std::chrono::steady_clock::time_point();
      retry_time = std::chrono::steady_clock::time_point();

      connection->bridge.encoding = "bgr8";
      connection->campub.publish(*connection->bridge.toImageMsg(),
                                 connection->infman.getCameraInfo());
    }
  })
      .detach();
}

CameraDriverDisplay::~CameraDriverDisplay() {
  std::unique_lock<std::mutex> lock(_worker_com->mutex);
  _worker_com->stop = true;
  _worker_com->condition.notify_all();
}
