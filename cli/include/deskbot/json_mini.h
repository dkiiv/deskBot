// deskbot cli - minimal recursive-descent JSON reader. No dependencies, because
// a desk controller's build must not need a package manager. Supports the subset
// used by machine profiles: objects, arrays, numbers, strings, true/false/null.
#pragma once
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace json {

struct Value;
using Object = std::map<std::string, Value>;
using Array = std::vector<Value>;

struct Value {
  enum class Kind { Null, Bool, Num, Str, Arr, Obj } kind = Kind::Null;
  bool b = false;
  double num = 0.0;
  std::string str;
  Array arr;
  Object obj;

  bool is_null() const { return kind == Kind::Null; }
  bool has(const std::string& k) const { return kind == Kind::Obj && obj.count(k); }
  const Value& at(const std::string& k) const {
    static const Value kNull{};
    auto it = obj.find(k);
    return it == obj.end() ? kNull : it->second;
  }
  double as_num(double dflt = 0.0) const { return kind == Kind::Num ? num : dflt; }
  bool as_bool(bool dflt = false) const { return kind == Kind::Bool ? b : dflt; }
  float as_flt(float dflt = 0.0f) const {
    return kind == Kind::Num ? static_cast<float>(num) : dflt;
  }
  bool as_str_flag() const { return kind == Kind::Str; }
  const std::string& as_str_ref() const { return str; }
  std::string as_str(const std::string& dflt = "") const {
    return kind == Kind::Str ? str : dflt;
  }
};

class Parser {
 public:
  explicit Parser(const std::string& text) : t_(text) {}

  bool parse(Value& out, std::string& err) {
    skip_ws();
    if (!value(out)) { err = where("expected value"); return false; }
    skip_ws();
    if (i_ != t_.size()) { err = where("trailing characters"); return false; }
    return true;
  }

 private:
  const std::string& t_;
  size_t i_ = 0;

  std::string where(const char* what) const {
    return std::string(what) + " at offset " + std::to_string(i_);
  }
  void skip_ws() { while (i_ < t_.size() && std::isspace(static_cast<unsigned char>(t_[i_]))) ++i_; }
  bool value(Value& out) {
    if (i_ >= t_.size()) return false;
    const char c = t_[i_];
    if (c == '{') return object(out);
    if (c == '[') return array(out);
    if (c == '"') return string(out);
    if (c == 't' || c == 'f' || c == 'n') {
      // Bool / null literals.
      struct { const char* lit; Value::Kind k; } lits[] = {
          {"true", Value::Kind::Bool}, {"false", Value::Kind::Bool},
          {"null", Value::Kind::Null}};
      for (auto& l : lits) {
        size_t n = 0;
        while (l.lit[n] && i_ + n < t_.size() && t_[i_ + n] == l.lit[n]) ++n;
        if (!l.lit[n]) {
          i_ += n;
          out.kind = l.k;
          out.b = (l.k == Value::Kind::Bool && l.lit[0] == 't');
          return true;
        }
      }
      return false;
    }
    return number(out);
  }

  bool number(Value& out) {
    size_t start = i_;
    if (i_ < t_.size() && (t_[i_] == '-' || t_[i_] == '+')) ++i_;
    bool any = false;
    while (i_ < t_.size() &&
           (std::isdigit(static_cast<unsigned char>(t_[i_])) || t_[i_] == '.' ||
            t_[i_] == 'e' || t_[i_] == 'E' ||
            ((t_[i_] == '-' || t_[i_] == '+') &&
             (t_[i_ - 1] == 'e' || t_[i_ - 1] == 'E')))) {
      any = true; ++i_;
    }
    if (!any) return false;
    out.kind = Value::Kind::Num;
    out.num = std::strtod(t_.substr(start, i_ - start).c_str(), nullptr);
    return true;
  }

  bool string(Value& out) {
    if (t_[i_] != '"') return false;
    ++i_;
    std::string s;
    while (i_ < t_.size() && t_[i_] != '"') {
      if (t_[i_] == '\\' && i_ + 1 < t_.size()) {
        const char e = t_[i_ + 1];
        s += (e == 'n') ? '\n' : (e == 't') ? '\t' : e;
        i_ += 2;
      } else {
        s += t_[i_++];
      }
    }
    if (i_ >= t_.size()) return false;
    ++i_;
    out.kind = Value::Kind::Str;
    out.str = s;
    return true;
  }

  bool object(Value& out) {
    out.kind = Value::Kind::Obj;
    ++i_;
    skip_ws();
    if (i_ < t_.size() && t_[i_] == '}') { ++i_; return true; }
    while (true) {
      skip_ws();
      Value key;
      if (!string(key)) return false;
      skip_ws();
      if (i_ >= t_.size() || t_[i_] != ':') return false;
      ++i_;
      skip_ws();
      Value v;
      if (!value(v)) return false;
      out.obj[key.str] = v;
      skip_ws();
      if (i_ < t_.size() && t_[i_] == ',') { ++i_; continue; }
      if (i_ < t_.size() && t_[i_] == '}') { ++i_; return true; }
      return false;
    }
  }

  bool array(Value& out) {
    out.kind = Value::Kind::Arr;
    ++i_;
    skip_ws();
    if (i_ < t_.size() && t_[i_] == ']') { ++i_; return true; }
    while (true) {
      skip_ws();
      Value v;
      if (!value(v)) return false;
      out.arr.push_back(v);
      skip_ws();
      if (i_ < t_.size() && t_[i_] == ',') { ++i_; continue; }
      if (i_ < t_.size() && t_[i_] == ']') { ++i_; return true; }
      return false;
    }
  }
};

inline bool parse_file(const std::string& path, Value& out, std::string& err) {
  FILE* f = std::fopen(path.c_str(), "rb");
  if (!f) { err = "cannot open " + path; return false; }
  std::string text;
  char buf[4096];
  size_t n;
  while ((n = std::fread(buf, 1, sizeof(buf), f)) > 0) text.append(buf, n);
  std::fclose(f);
  Parser p(text);
  return p.parse(out, err);
}

}  // namespace json
