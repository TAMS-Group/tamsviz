// TAMSVIZ
// (c) 2020 Philipp Ruppel

#pragma once

#include "mparser.h"

#include "message.h"
#include "object.h"
#include "topic.h"

#include <functional>

class MessageQuery {

  enum class TokenType {
    None,
    Space,
    Symbol,
    Dot,
    String,
    Integer,
    Double,
    IndexBegin,
    IndexEnd,
    Equality,
    Inequality,
    GreaterThan,
    LessThan,
    GreaterEqual,
    LessEqual,
  };

  struct TokenInfo {
    TokenType type = TokenType::None;
    std::string str;
    double number = 0.0;
    size_t integer = 0;
    size_t begin = 0;
    size_t end = 0;
  };

  std::string _query;
  std::vector<TokenInfo> _tokens;
  bool _tok_ok = false;

  typedef std::vector<TokenInfo>::const_iterator TokenIterator;

  void tokenize();
  bool filter(const MessageParser &parser, const TokenInfo &op,
              const TokenInfo &val) const;
  bool filter(const MessageParser &parser, TokenIterator &current_token,
              const std::function<void(const TokenIterator &,
                                       const std::string &)> &complete) const;
  MessageParser
  query(MessageParser parser, TokenIterator &current_token,
        const std::function<void(const TokenIterator &, const std::string &)>
            &complete = nullptr) const;

public:
  MessageQuery();
  MessageQuery(const std::string &query);
  void assign(const std::string &query);
  MessageParser operator()(const MessageParser &parser) const;
  const std::string &str() const { return _query; }
  void complete(const MessageParser &parser, AutoCompletion &completion) const;
};

class MessageQueryProperty {
  mutable std::mutex _subscriber_mutex;
  std::shared_ptr<Subscriber<Message>> _subscriber;
  std::function<std::shared_ptr<const Message>()> _message_func;
  std::string _query;

public:
  MessageQueryProperty() {}
  // MessageQueryProperty(
  //      const std::function<std::shared_ptr<const Message>()> &message_func)
  //  : _message_func(message_func) {}
  MessageQueryProperty(const std::string &query) : _query(query) {}
  MessageQueryProperty(const MessageQueryProperty &other) {
    _query = other._query;
  }
  MessageQueryProperty &operator=(const MessageQueryProperty &other) {
    _query = other._query;
    return *this;
  }
  MessageQuery query() const { return MessageQuery(_query); };
  const std::string &str() const { return _query; }
  void assign(const std::string &query) { _query = query; }
  void subscriber(const std::shared_ptr<Subscriber<Message>> &subscriber) {
    std::unique_lock<std::mutex> lock(_subscriber_mutex);
    _subscriber = subscriber;
  }
  std::shared_ptr<const Message> message() const {
    std::unique_lock<std::mutex> lock(_subscriber_mutex);
    if (_subscriber) {
      return _subscriber->message();
    }
    if (_message_func) {
      return _message_func();
    }
    return nullptr;
  }
  void
  message(const std::function<std::shared_ptr<const Message>()> &message_func) {
    std::unique_lock<std::mutex> lock(_subscriber_mutex);
    _message_func = message_func;
  }
  bool empty() const { return _query.empty(); }
};
static void toString(const MessageQueryProperty &x, std::string &str) {
  str = x.str();
}
static void fromString(MessageQueryProperty &x, const std::string &str) {
  x.assign(str);
}
static bool operator==(const MessageQueryProperty &a,
                       const MessageQueryProperty &b) {
  return a.str() == b.str();
}
static bool operator!=(const MessageQueryProperty &a,
                       const MessageQueryProperty &b) {
  return a.str() != b.str();
}
template <> struct DefaultPropertyAttributes<MessageQueryProperty> {
  static void initialize(PropertyAttributes *attributes) {
    attributes->complete = [](const Property &property, const std::string &text,
                              AutoCompletion &completion) {
      LOG_DEBUG("complete " << text);
      if (auto msg = property.get<MessageQueryProperty>().message()) {
        LOG_DEBUG("message found");
        MessageQuery(text).complete(MessageParser(msg), completion);
      }
      LOG_DEBUG("no message received yet");
      return std::vector<std::string>();
    };
  }
};
