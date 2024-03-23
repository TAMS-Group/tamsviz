// TAMSVIZ
// (c) 2020-2023 Philipp Ruppel

#pragma once

#include "document.h"
#include "mquery.h"

class Workspace;

struct AnnotationSpan : Object {
  PROPERTY(std::string, label, "");
  PROPERTY(double, start, 0.0);
  PROPERTY(double, duration, 1.0);
  PROPERTY(std::vector<std::shared_ptr<AnnotationBase>>, annotations, {});
};
DECLARE_TYPE(AnnotationSpan, Object);

struct AnnotationBranch : Object {
  PROPERTY(std::string, name);
  PROPERTY(std::vector<std::shared_ptr<AnnotationSpan>>, spans, {},
           hidden = true);
};
DECLARE_TYPE(AnnotationBranch, Object);

struct AnnotationTrack : TrackBase {
  PROPERTY(std::vector<std::shared_ptr<AnnotationBranch>>, branches, {},
           hidden = true);
  std::shared_ptr<AnnotationBranch> branch(const std::shared_ptr<Workspace> &ws,
                                           bool create = false);
};
DECLARE_TYPE(AnnotationTrack, TrackBase);

class GraphTrack : public TrackBase {
  struct Data {
    std::mutex mutex;
    std::shared_ptr<const Message> message;
  };
  std::shared_ptr<Data> _data = std::make_shared<Data>();

public:
  PROPERTY(TopicProperty<Message>, topic);
  PROPERTY(MessageQueryProperty, query/*, [this]() {
    LOG_DEBUG(topic().subscriber()->message());
    return topic().subscriber()->message();
}*/);
  GraphTrack() {
    auto data = _data;
    topic().connect(data,
                    [data](const std::shared_ptr<const Message> &message) {
                      std::lock_guard<std::mutex> lock(data->mutex);
                      data->message = message;
                    });
    query().message([data]() {
      std::lock_guard<std::mutex> lock(data->mutex);
      return data->message;
    });
  }
};
DECLARE_TYPE(GraphTrack, TrackBase);
