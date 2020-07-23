// TAMSVIZ
// (c) 2020 Philipp Ruppel

#pragma once

#include "mparser.h"

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
  std::vector<std::string> complete(const MessageParser &parser) const;
};
