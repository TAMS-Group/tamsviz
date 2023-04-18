// TAMSVIZ
// (c) 2020-2023 Philipp Ruppel

#pragma once

#include "../core/document.h"
#include "../core/struct.h"
#include "../core/timeseries.h"
#include "../core/topic.h"
#include "../core/workspace.h"

class CameraDriverDisplay : public Display {

  struct Params {

    std::string prefix;
    std::string calibration;
    std::string frame;
    size_t queue_size = 0;
    size_t index = 0;

    bool operator==(const Params &other) const {
      return                                  //
          prefix == other.prefix &&           //
          calibration == other.calibration && //
          frame == other.frame &&             //
          queue_size == other.queue_size &&   //
          index == other.index &&             //
          true;                               //
    }
  };

  struct WorkerCom {
    std::mutex mutex;
    std::condition_variable condition;
    bool stop = false;
    bool capture = false;
    Params params;
  };
  std::shared_ptr<WorkerCom> _worker_com = std::make_shared<WorkerCom>();

public:
  CameraDriverDisplay();
  ~CameraDriverDisplay();

  PROPERTY(std::string, prefix, "tamsvizcam");
  PROPERTY(std::string, frame, "tamsvizcam");
  PROPERTY(std::string, calibration, "tamsvizcam");
  PROPERTY(size_t, queueSize, 3);
  PROPERTY(size_t, index, 0);

  PROPERTY(bool, capture, true);

  virtual void refresh() override {

    std::unique_lock<std::mutex> lock(_worker_com->mutex);

    _worker_com->params.prefix = prefix();
    _worker_com->params.queue_size = queueSize();
    _worker_com->params.frame = frame();
    _worker_com->params.calibration = calibration();
    _worker_com->params.index = index();

    _worker_com->capture = capture();

    _worker_com->condition.notify_all();
  }
};
DECLARE_TYPE_C(CameraDriverDisplay, Display, Driver);
