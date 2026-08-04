// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0
//
// json_lite — a tiny, self-contained JSON reader. Just enough to parse the
// ESP32 /state.json (objects, arrays, strings, numbers, bools, null). Avoids a
// dependency on nlohmann/json so the package builds with only rclcpp + libcurl.
// Not a general-purpose library: no unicode-escape expansion beyond passthrough,
// tolerant number parsing. Adequate and tested for the device's flat schema.
#ifndef PROTECTIVE_STOP_MACHINE__JSON_LITE_HPP_
#define PROTECTIVE_STOP_MACHINE__JSON_LITE_HPP_

#include <cctype>
#include <cstdlib>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace jsonlite
{

struct Value
{
  enum Type
  {
    NUL,
    BOOL,
    NUM,
    STR,
    ARR,
    OBJ
  };

  Type type{NUL};
  bool b{false};
  double num{0.0};
  std::string str;
  std::vector<Value> arr;
  std::map<std::string, Value> obj;

  bool is_obj() const
  {
    return type == OBJ;
  }

  bool is_arr() const
  {
    return type == ARR;
  }

  // Object lookup; nullptr if absent or not an object.
  const Value * find(const std::string & key) const
  {
    if (type != OBJ) {
      return nullptr;
    }
    auto it = obj.find(key);
    return it == obj.end() ? nullptr : &it->second;
  }

  double as_num(double dflt = 0.0) const
  {
    return type == NUM ? num : dflt;
  }

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

  std::string as_str(const std::string & dflt = "") const
  {
    return type == STR ? str : dflt;
  }

  // Convenience: number/bool/string field of an object by key.
  double num_at(const std::string & k, double dflt = 0.0) const
  {
    const Value * v = find(k);
    return v ? v->as_num(dflt) : dflt;
  }

  bool bool_at(const std::string & k, bool dflt = false) const
  {
    const Value * v = find(k);
    return v ? v->as_bool(dflt) : dflt;
  }

  std::string str_at(const std::string & k, const std::string & dflt = "") const
  {
    const Value * v = find(k);
    return v ? v->as_str(dflt) : dflt;
  }
};

class Parser
{
public:
  explicit Parser(const std::string & s)
  : s_(s)
  {}

  bool parse(Value & out)
  {
    skip_ws();
    if (!value(out, 0)) {
      return false;
    }
    skip_ws();
    return true;  // trailing junk tolerated
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
    ++i_;  // {
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
    ++i_;  // [
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
    ++i_;  // opening quote
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
          case 'u':  // passthrough: keep the 4 hex digits literally
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
    return false;  // unterminated
  }

  bool number(Value & v)
  {
    size_t start = i_;
    if (peek() == '-') {
      ++i_;
    }
    while (i_ < s_.size() &&
      (std::isdigit(static_cast<unsigned char>(s_[i_])) || s_[i_] == '.' || s_[i_] == 'e' ||
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

#endif  // PROTECTIVE_STOP_MACHINE__JSON_LITE_HPP_
