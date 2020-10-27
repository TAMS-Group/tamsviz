// TAMSVIZ
// (c) 2020 Philipp Ruppel

#include "bagplayer.h"
#include "mquery.h"
#include "topic.h"
#include "workspace.h"

struct TimeSeriesListener {
  virtual void push(const std::shared_ptr<const Message> &message,
                    int64_t start, int64_t end) = 0;
  virtual void commit() {}
};

class TimeSeriesSubscriber {
  struct Impl {
    std::vector<std::weak_ptr<TimeSeriesListener>> _listeners;
    bool _stop_flag = false;
    std::mutex _mutex;
    bool _refresh = false;
    std::condition_variable _condition;
    std::shared_ptr<Topic> _topic;
    double _duration;
    std::thread _thread;
    std::shared_ptr<const Message> _received_message;
    std::shared_ptr<BagPlayer> _player;
    Impl(const Impl &) = delete;
    Impl &operator=(const Impl &) = delete;
    Impl();
    ~Impl();
    void handleMessage(const std::shared_ptr<const Message> &message);
  };
  std::shared_ptr<Impl> _impl = std::make_shared<Impl>();
  std::shared_ptr<Subscriber<Message>> _subscriber;

public:
  TimeSeriesSubscriber(const std::string &topic);
  TimeSeriesSubscriber(const TimeSeriesSubscriber &) = delete;
  TimeSeriesSubscriber &operator=(const TimeSeriesSubscriber &) = delete;
  const std::string &topic() const;
  const double duration() const;
  void duration(double duration);
  void addListener(const std::shared_ptr<TimeSeriesListener> &listener);
  const std::shared_ptr<Subscriber<Message>> &subscriber() const;
};

template <class Output>
class TimeSeriesTransformer : public TimeSeriesListener {
  mutable std::mutex _mutex;
  std::map<int64_t, Output> _data;

public:
  TimeSeriesTransformer() {}
  TimeSeriesTransformer(const TimeSeriesTransformer &) = delete;
  TimeSeriesTransformer &operator=(const TimeSeriesTransformer &) = delete;
  virtual void push(const std::shared_ptr<const Message> &message,
                    int64_t start, int64_t end) override {
    PROFILER("TimeSeriesTransformer");
    if (_data.find(message->time().toNSec()) == _data.end()) {
      Output output;
      if (transform(message, output)) {
        std::unique_lock<std::mutex> lock(_mutex);
        _data.emplace(message->time().toNSec(), output);
      }
    }
    while (!_data.empty() && _data.begin()->first < start) {
      _data.erase(_data.begin());
    }
    while (!_data.empty() && _data.rbegin()->first > end) {
      auto it = _data.end();
      --it;
      _data.erase(it);
    }
  }
  std::vector<Output> data() const {
    std::vector<Output> ret;
    std::unique_lock<std::mutex> lock(_mutex);
    for (auto &pair : _data) {
      ret.push_back(pair.second);
    }
    return std::move(ret);
  }

protected:
  virtual bool transform(const std::shared_ptr<const Message> &message,
                         Output &output) = 0;
};

class TimeSeriesQuery
    : public TimeSeriesTransformer<std::pair<int64_t, double>> {
  MessageQuery _query;
  MessageQuery _stamp_query = MessageQuery("header.stamp");

public:
  TimeSeriesQuery(const MessageQuery &query) : _query(query) {}
  const MessageQuery &query() const { return _query; }

protected:
  virtual bool transform(const std::shared_ptr<const Message> &message,
                         std::pair<int64_t, double> &output) override;
};
