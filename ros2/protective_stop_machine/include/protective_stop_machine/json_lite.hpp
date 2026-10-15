// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cctype>
#include <cstdlib>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace jsonlite
{

/// @brief A parsed JSON value: null, bool, number, string, array or object.
/// Accessors return the supplied default when the value has a different type.
struct Value
{
  /// JSON type held by a Value.
  enum Type
  {
    NUL,
    BOOL,
    NUM,
    STR,
    ARR,
    OBJ
  };

  /// Active type; selects which of the members below is meaningful.
  Type type{NUL};
  /// Value when type is BOOL.
  bool b{false};
  /// Value when type is NUM.
  double num{0.0};
  /// Value when type is STR.
  std::string str;
  /// Elements when type is ARR.
  std::vector<Value> arr;
  /// Members when type is OBJ.
  std::map<std::string, Value> obj;

  /// True when type is OBJ.
  bool is_obj() const
  {
    return type == OBJ;
  }

  /// True when type is ARR.
  bool is_arr() const
  {
    return type == ARR;
  }

  /// Returns the member named key, or nullptr if absent or not an object.
  const Value * find(const std::string & key) const
  {
    if (type != OBJ) {
      return nullptr;
    }
    auto it = obj.find(key);
    return it == obj.end() ? nullptr : &it->second;
  }

  /// Number value, or dflt unless type is NUM.
  double as_num(double dflt = 0.0) const
  {
    return type == NUM ? num : dflt;
  }

  /// Bool value, a NUM compared against zero, or dflt for other types.
  bool as_bool(bool dflt = false) const
  {
    if (type == BOOL) {
      return b;
    }
    if (type == NUM) {
      return num != 0.0;
    }
    return dflt;
  }

  /// String value, or dflt unless type is STR.
  std::string as_str(const std::string & dflt = "") const
  {
    return type == STR ? str : dflt;
  }

  /// as_num() of member k, or dflt if the member is absent.
  double num_at(const std::string & k, double dflt = 0.0) const
  {
    const Value * v = find(k);
    return v ? v->as_num(dflt) : dflt;
  }

  /// as_bool() of member k, or dflt if the member is absent.
  bool bool_at(const std::string & k, bool dflt = false) const
  {
    const Value * v = find(k);
    return v ? v->as_bool(dflt) : dflt;
  }

  /// as_str() of member k, or dflt if the member is absent.
  std::string str_at(const std::string & k, const std::string & dflt = "") const
  {
    const Value * v = find(k);
    return v ? v->as_str(dflt) : dflt;
  }
};

/// @brief Recursive-descent parser for the flat JSON schema of the ESP32 /state.json.
/// Nesting deeper than 32 levels is rejected. Unicode escapes pass through unexpanded.
class Parser
{
public:
  /// Parses s; the string must outlive the parser.
  explicit Parser(const std::string & s)
  : s_(s)
  {}

  /// @brief Parses the document into out.
  /// Trailing content after the first value is ignored.
  /// @return False on malformed input.
  bool parse(Value & out)
  {
    skip_ws();
    if (!value(out, 0)) {
      return false;
    }
    skip_ws();
    // trailing junk tolerated
    return true;
  }

private:
  const std::string & s_;
  size_t i_{0};
  // Bound recursion so a hostile/malformed deeply-nested document (e.g. from a
  // compromised device) can't overflow the stack. The device schema is flat.
  static constexpr int kMaxDepth = 32;

  void skip_ws()
  {
    while (i_ < s_.size() && std::isspace(static_cast<unsigned char>(s_[i_]))) {
      ++i_;
    }
  }

  char peek() const
  {
    return i_ < s_.size() ? s_[i_] : '\0';
  }

  bool value(Value & v, int depth)
  {
    if (depth > kMaxDepth) {
      return false;
    }
    skip_ws();
    char c = peek();
    switch (c) {
      case '{':
        return object(v, depth);
      case '[':
        return array(v, depth);
      case '"':
        return string_val(v);
      case 't':
      case 'f':
        return boolean(v);
      case 'n':
        return null_val(v);
      default:
        if (c == '-' || std::isdigit(static_cast<unsigned char>(c))) {
          return number(v);
        }
        return false;
    }
  }

  bool object(Value & v, int depth)
  {
    v.type = Value::OBJ;
    // consume {
    ++i_;
    skip_ws();
    if (peek() == '}') {
      ++i_;
      return true;
    }
    while (true) {
      skip_ws();
      if (peek() != '"') {
        return false;
      }
      Value key;
      if (!string_val(key)) {
        return false;
      }
      skip_ws();
      if (peek() != ':') {
        return false;
      }
      ++i_;
      Value val;
      if (!value(val, depth + 1)) {
        return false;
      }
      v.obj[key.str] = std::move(val);
      skip_ws();
      char c = peek();
      if (c == ',') {
        ++i_;
        continue;
      }
      if (c == '}') {
        ++i_;
        return true;
      }
      return false;
    }
  }

  bool array(Value & v, int depth)
  {
    v.type = Value::ARR;
    // consume [
    ++i_;
    skip_ws();
    if (peek() == ']') {
      ++i_;
      return true;
    }
    while (true) {
      Value item;
      if (!value(item, depth + 1)) {
        return false;
      }
      v.arr.push_back(std::move(item));
      skip_ws();
      char c = peek();
      if (c == ',') {
        ++i_;
        continue;
      }
      if (c == ']') {
        ++i_;
        return true;
      }
      return false;
    }
  }

  bool string_val(Value & v)
  {
    v.type = Value::STR;
    // consume the opening quote
    ++i_;
    std::string out;
    while (i_ < s_.size()) {
      char c = s_[i_++];
      if (c == '"') {
        v.str = std::move(out);
        return true;
      }
      if (c == '\\' && i_ < s_.size()) {
        char e = s_[i_++];
        switch (e) {
          case 'n':
            out.push_back('\n');
            break;
          case 't':
            out.push_back('\t');
            break;
          case 'r':
            out.push_back('\r');
            break;
          case '"':
            out.push_back('"');
            break;
          case '\\':
            out.push_back('\\');
            break;
          case '/':
            out.push_back('/');
            break;
          // passthrough: keep the 4 hex digits literally
          case 'u':
            out.push_back('\\');
            out.push_back('u');
            for (int k = 0; k < 4 && i_ < s_.size(); ++k) {
              out.push_back(s_[i_++]);
            }
            break;
          default:
            out.push_back(e);
            break;
        }
      } else {
        out.push_back(c);
      }
    }
    // unterminated
    return false;
  }

  bool number(Value & v)
  {
    size_t start = i_;
    if (peek() == '-') {
      ++i_;
    }
    while (i_ < s_.size() && (std::isdigit(static_cast<unsigned char>(s_[i_])) || s_[i_] == '.' || s_[i_] == 'e' ||
                              s_[i_] == 'E' || s_[i_] == '+' || s_[i_] == '-'))
    {
      ++i_;
    }
    v.type = Value::NUM;
    v.num = std::strtod(s_.substr(start, i_ - start).c_str(), nullptr);
    return true;
  }

  bool boolean(Value & v)
  {
    if (s_.compare(i_, 4, "true") == 0) {
      v.type = Value::BOOL;
      v.b = true;
      i_ += 4;
      return true;
    }
    if (s_.compare(i_, 5, "false") == 0) {
      v.type = Value::BOOL;
      v.b = false;
      i_ += 5;
      return true;
    }
    return false;
  }

  bool null_val(Value & v)
  {
    if (s_.compare(i_, 4, "null") == 0) {
      v.type = Value::NUL;
      i_ += 4;
      return true;
    }
    return false;
  }
};

inline bool parse(const std::string & text, Value & out)
{
  Parser p(text);
  return p.parse(out);
}

}  // namespace jsonlite
