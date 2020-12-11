// TAMSVIZ
// (c) 2020 Philipp Ruppel

#include "mquery.h"

#include <regex>

void MessageQuery::tokenize() {
  LOG_DEBUG("tokenize");

  _tokens.clear();
  _tok_ok = true;

  std::string::const_iterator current = _query.begin();
  std::string::const_iterator end = _query.end();

  static std::vector<std::pair<TokenType, std::regex>> expression_map = {
      {TokenType::Space, std::regex("\\s+")},
      {TokenType::Symbol, std::regex("[A-Za-z_][A-Za-z0-9_]*")},
      {TokenType::Dot, std::regex("\\.")},
      {TokenType::String, std::regex("\\\"[^\\\"]*\\\"|\\'[^\\']*\\'")},
      {TokenType::Integer, std::regex("[0-9]+")},
      {TokenType::IndexBegin, std::regex("\\[")},
      {TokenType::IndexEnd, std::regex("\\]")},
      {TokenType::Equality, std::regex("\\=\\=?")},
      {TokenType::Inequality, std::regex("\\!\\=")},
      {TokenType::GreaterThan, std::regex("\\>")},
      {TokenType::LessThan, std::regex("\\<")},
      {TokenType::GreaterEqual, std::regex("\\>\\=")},
      {TokenType::LessEqual, std::regex("\\<\\=")},
      {TokenType::Double,
       std::regex("[-+]?([0-9]*\\.)?[0-9]+([eE][-+]?[0-9]+)?")},
  };

  std::smatch match;
  while (current != end) {
    TokenInfo token;
    for (auto &expression_mapping : expression_map) {
      auto &token_type = expression_mapping.first;
      auto &expression = expression_mapping.second;

      bool ok = std::regex_search(current, end, match, expression,
                                  std::regex_constants::match_continuous);
      if (ok) {
        token.type = token_type;
        token.str = match[0];

        if (token.type == TokenType::Double ||
            token.type == TokenType::Integer) {
          try {
            token.number = std::stod(token.str);
          } catch (const std::invalid_argument &ex) {
            token.type = TokenType::None;
            break;
          } catch (const std::out_of_range &ex) {
            token.type = TokenType::None;
            break;
          }
        }

        if (token.type == TokenType::Integer) {
          try {
            token.integer = std::stoull(token.str);
          } catch (const std::invalid_argument &ex) {
            token.type = TokenType::None;
            break;
          } catch (const std::out_of_range &ex) {
            token.type = TokenType::None;
            break;
          }
        }

        if (token.type == TokenType::String) {
          token.str = token.str.substr(1, token.str.size() - 2);
        }

        break;
      }
    }

    token.begin = current - _query.begin();
    token.end = token.begin + match[0].length();

    if (token.type != TokenType::None) {
      if (token.type != TokenType::Space) {
        _tokens.push_back(token);
      }
      current += match[0].length();
    } else {
      //_tokens.clear();
      _tok_ok = false;
      break;
    }
  }

  for (auto &tok : _tokens) {
    LOG_DEBUG("token " << (int)tok.type << " " << tok.str);
  }
}

bool MessageQuery::filter(const MessageParser &parser, const TokenInfo &op,
                          const TokenInfo &val) const {

  if (val.type == TokenType::Integer || val.type == TokenType::Double) {
    switch (op.type) {

    case TokenType::Equality:
      return parser.toDouble() == val.number;
      break;

    case TokenType::Inequality:
      return parser.toDouble() != val.number;
      break;

    case TokenType::LessThan:
      return parser.toDouble() < val.number;
      break;

    case TokenType::GreaterThan:
      return parser.toDouble() > val.number;
      break;

    case TokenType::LessEqual:
      return parser.toDouble() <= val.number;
      break;

    case TokenType::GreaterEqual:
      return parser.toDouble() >= val.number;
      break;
    }
  }

  if (val.type == TokenType::String) {
    switch (op.type) {

    case TokenType::Equality:
      return parser.toString() == val.str;
      break;

    case TokenType::Inequality:
      return parser.toString() != val.str;
      break;
    }
  }

  return false;
}

bool MessageQuery::filter(
    const MessageParser &parser, TokenIterator &current_token,
    const std::function<void(const TokenIterator &, const std::string &)>
        &complete) const {

  MessageParser msg = query(parser, current_token, complete);

  if (complete) {
    if (msg.isPrimitive()) {
      complete(current_token, "==");
      complete(current_token, "!=");
      if (!parser.isString()) {
        complete(current_token, "<");
        complete(current_token, ">");
        complete(current_token, "<=");
        complete(current_token, ">=");
      }
    }
  }

  if (current_token == _tokens.end()) {
    return false;
  }
  TokenInfo op = *current_token;
  current_token++;

  if (complete) {
    if (msg.isPrimitive()) {
      auto str = msg.toString();
      if (msg.isString()) {
        if (str.find("\"") == std::string::npos) {
          complete(current_token, "\"" + str + "\"");
        }
      } else {
        complete(current_token, str);
      }
    }
  }

  if (current_token == _tokens.end()) {
    return false;
  }
  TokenInfo val = *current_token;
  current_token++;

  if (complete) {
    complete(current_token, "]");
  }

  return filter(msg, op, val);
}

MessageParser MessageQuery::query(
    MessageParser parser, TokenIterator &current_token,
    const std::function<void(const TokenIterator &, const std::string &)>
        &complete) const {

  if (!parser.isMessage()) {
    return MessageParser();
  }

  if (complete) {
    for (size_t i = 0; i < parser.size(); i++) {
      complete(current_token, parser.fieldName(i));
    }
  }

  if (current_token == _tokens.end() ||
      current_token->type != TokenType::Symbol) {
    return MessageParser();
  }

  parser = parser[current_token->str];
  ++current_token;

  while (true) {

    if (parser.isMessage()) {

      if (complete) {
        for (size_t i = 0; i < parser.size(); i++) {
          complete(current_token, "." + parser.fieldName(i));
        }
      }

      if (current_token != _tokens.end() &&
          current_token->type == TokenType::Dot &&
          (current_token + 1) != _tokens.end() &&
          (current_token + 1)->type == TokenType::Symbol) {

        parser = parser[(current_token + 1)->str];
        current_token += 2;

        continue;
      }
    }

    if (parser.isArray()) {

      if (complete) {
        complete(current_token, "[");
        for (size_t i = 0; i <= 3 && i < parser.size(); i++) {
          complete(current_token, "[" + std::to_string(i) + "]");
        }
      }

      if (current_token != _tokens.end() &&
          current_token->type == TokenType::IndexBegin) {

        if ((current_token + 1) != _tokens.end() &&
            (current_token + 1)->type == TokenType::Integer) {

          if (complete) {
            complete(current_token + 2, "]");
          }

          if ((current_token + 2) != _tokens.end() &&
              (current_token + 2)->type == TokenType::IndexEnd) {

            parser = parser[(current_token + 1)->integer];
            current_token += 3;

            continue;
          }
        }

        size_t array_size = parser.size();
        bool found = false;
        for (size_t i = 0; i < array_size; i++) {

          auto element = parser[i];
          TokenIterator filter_token = current_token + 1;
          if (filter(element, filter_token, complete)) {
            current_token = filter_token;

            if (current_token == _tokens.end() ||
                current_token->type != TokenType::IndexEnd) {
              return MessageParser();
            }
            current_token++;

            parser = element;

            found = true;

            break;
          }
        }
        if (found) {
          continue;
        } else {
          break;
        }
      }
    }

    break;
  }
  return parser;
}

MessageQuery::MessageQuery() {}

MessageQuery::MessageQuery(const std::string &query) { assign(query); }

void MessageQuery::assign(const std::string &query) {
  if (query != _query) {
    _query = query;
    tokenize();
  }
}

MessageParser MessageQuery::operator()(const MessageParser &parser) const {
  PROFILER("MessageQuery");
  if (!_tok_ok) {
    return MessageParser();
  }
  TokenIterator current_token = _tokens.begin();
  auto ret = query(parser, current_token);
  if (current_token == _tokens.end()) {
    // LOG_DEBUG(ret.print());
    return ret;
  } else {
    return MessageParser();
  }
}

void MessageQuery::complete(const MessageParser &parser,
                            AutoCompletion &completion) const {
  LOG_DEBUG("complete query");
  LOG_DEBUG("query " << _query);
  std::set<std::string> ret;
  TokenIterator current_token = _tokens.begin();
  auto value =
      query(parser, current_token,
            [&](const TokenIterator &token, const std::string &completion) {
              std::string prefix;
              if (token != _tokens.begin()) {
                TokenIterator tok = token;
                tok--;
                LOG_DEBUG("token end " << tok->end);
                prefix = _query.substr(0, tok->end);
              }
              LOG_DEBUG("completion " << prefix << completion);
              ret.insert(prefix + completion);
            });
  completion.completed = value.isPrimitive();
  LOG_DEBUG("completed " << (completion.completed ? "true" : "false"));
  completion.items.assign(ret.begin(), ret.end());
}
