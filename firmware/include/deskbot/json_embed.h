// deskbot firmware - embedded machine-profile JSON.
//
// This is the firmware twin of cli/include/deskbot/{json_mini.h,profile_loader.h}.
// It is deliberately NOT a copy of them: json_mini.h is built on std::string,
// std::map and std::vector and allocates, and profile_loader.h calls strdup()
// and std::to_string(). All of that is fine on a host and wasteful-to-fragile
// in a 2 ms control task on a microcontroller, so this parser
//   * uses no std::, no exceptions, no dynamic allocation (one fixed node pool),
//   * reports errors by bool + message pointer instead of throwing,
//   * validates the same required keys as profile_loader.h and applies the same
//     defaults for the optional ones, so firmware and simulator cannot disagree
//     about what a profile means.
//
// If you change a key or a default here, change profile_loader.h too (and note
// it in DESIGN-NOTES.md): a silently diverging default shows up as a desk that
// stops at a different height than the simulator says it will.
//
// Tree edges are stored in the nodes themselves (first_child / last_child /
// next_sibling), not in a parallel edge pool. That is not just tidiness: with a
// shared edge pool, a parent's children are interleaved with those of whichever
// nested container was parsed in between, and every "child i" lookup silently
// returns a cousin. Exercised by test/json_embed_test.cpp on the host.
//
// Supported subset: objects, arrays, numbers, strings, true/false/null - the
// machine profile language. Anything else is a parse error.
#pragma once
#include <ctype.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

namespace json {

constexpr int kMaxNodes = 256;   // husky profile uses ~120
constexpr int kMaxText = 4096;   // interned string bytes (member names + values)
constexpr int kMaxStr = 192;     // longest single token

enum class Kind : uint8_t { Null, Bool, Num, Str, Arr, Obj };

struct Value {
  Kind kind = Kind::Null;
  bool b = false;
  double num = 0.0;
  const char* str = nullptr;   // Str: the content; nullptr otherwise
  const char* key = nullptr;   // member name when this node is an object field
  int first_child = -1;        // Arr/Obj: first element/field
  int last_child = -1;         // append cursor
  int next_sibling = -1;       // position within the parent's list
  int count = 0;               // number of children
};

// Fixed node pool. Nothing is freed: the tree stays valid for the lifetime of
// the Dom, and reload() resets it wholesale.
struct Dom {
  Value nodes[kMaxNodes];
  char text[kMaxText];
  int n_nodes = 0;
  int n_text = 0;

  void reset() {
    n_nodes = 0;
    n_text = 0;
  }
};

class Parser {
 public:
  Parser(const char* text, size_t len, Dom* dom) : t_(text), n_(len), d_(dom) {}

  bool parse(int* root_out, const char** err) {
    ws();
    int root = -1;
    if (!value(root)) { *err = err_; return false; }
    ws();
    if (i_ != n_) { fail("trailing characters"); *err = err_; return false; }
    *root_out = root;
    return true;
  }

 private:
  const char* t_;
  size_t n_;
  Dom* d_;
  size_t i_ = 0;
  const char* err_ = "";

  void fail(const char* why) { err_ = why; }

  static bool is_ws(char c) { return isspace((unsigned char)c) != 0; }
  void ws() { while (i_ < n_ && is_ws(t_[i_])) ++i_; }

  int node() {
    if (d_->n_nodes >= kMaxNodes) { fail("node pool exhausted"); return -1; }
    const int idx = d_->n_nodes++;
    // Construct fresh: the pool is reused across reloads, and a stale
    // first_child/next_sibling from a previous parse would look like structure.
    d_->nodes[idx] = Value{};
    return idx;
  }

  int intern(const char* s, size_t len) {
    if (d_->n_text + (int)len + 1 > kMaxText) { fail("string pool exhausted"); return -1; }
    const int at = d_->n_text;
    memcpy(d_->text + at, s, len);
    d_->text[at + (int)len] = '\0';
    d_->n_text += (int)len + 1;
    return at;
  }

  void append(int parent, int child) {
    Value& p = d_->nodes[parent];
    if (p.first_child < 0) p.first_child = child;
    if (p.last_child >= 0) d_->nodes[p.last_child].next_sibling = child;
    p.last_child = child;
    p.count++;
  }

  bool literal(const char* lit, Kind k, Value& out, bool val) {
    const size_t l = strlen(lit);
    if (n_ - i_ < l || memcmp(t_ + i_, lit, l) != 0) return false;
    i_ += l;
    out.kind = k;
    out.b = val;
    return true;
  }

  bool value(int& out_idx) {
    if (i_ >= n_) { fail("unexpected end"); return false; }
    const char c = t_[i_];
    if (c == '{') return object(out_idx);
    if (c == '[') return array(out_idx);
    if (c == '"') return string(out_idx);
    if (c == 't' || c == 'f' || c == 'n') {
      const int idx = node();
      if (idx < 0) return false;
      Value& v = d_->nodes[idx];
      const bool ok = (c == 't') ? literal("true", Kind::Bool, v, true)
                        : (c == 'f') ? literal("false", Kind::Bool, v, false)
                                     : literal("null", Kind::Null, v, false);
      if (!ok) { d_->n_nodes--; fail("bad literal"); return false; }
      out_idx = idx;
      return true;
    }
    return number(out_idx);
  }

  bool number(int& out_idx) {
    const size_t start = i_;
    if (i_ < n_ && (t_[i_] == '-' || t_[i_] == '+')) ++i_;
    bool any = false;
    while (i_ < n_) {
      const char c = t_[i_];
      const bool exp_sign = (c == '-' || c == '+') && i_ > start &&
                            (t_[i_ - 1] == 'e' || t_[i_ - 1] == 'E');
      if (isdigit((unsigned char)c) || c == '.' || c == 'e' || c == 'E' || exp_sign) {
        any = true;
        ++i_;
      } else {
        break;
      }
    }
    if (!any) { fail("bad number"); return false; }
    const size_t len = i_ - start;
    char buf[48];
    if (len >= sizeof(buf)) { fail("number too long"); return false; }
    memcpy(buf, t_ + start, len);
    buf[len] = '\0';
    const int idx = node();
    if (idx < 0) return false;
    Value& v = d_->nodes[idx];
    v.kind = Kind::Num;
    v.num = strtod(buf, nullptr);
    out_idx = idx;
    return true;
  }

  // Reads one JSON string token into `scratch` and returns the interned copy.
  // Returns false on error; `used` reports whether anything was consumed
  // (a bare `]`/`}` after a comma is a syntax error, not an empty string).
  bool string_token(char* scratch, size_t cap, size_t* used, bool* empty) {
    *empty = false;
    if (i_ >= n_ || t_[i_] != '"') {
      if (i_ < n_ && (t_[i_] == '}' || t_[i_] == ']')) fail("trailing comma?");
      else fail("expected string");
      return false;
    }
    ++i_;
    size_t len = 0;
    while (i_ < n_ && t_[i_] != '"') {
      if (t_[i_] == '\\' && i_ + 1 < n_) {
        const char e = t_[i_ + 1];
        const char c = (e == 'n') ? '\n' : (e == 't') ? '\t' : e;
        if (len + 1 >= cap) { fail("string too long"); return false; }
        scratch[len++] = c;
        i_ += 2;
      } else {
        if (len + 1 >= cap) { fail("string too long"); return false; }
        scratch[len++] = t_[i_++];
      }
    }
    if (i_ >= n_) { fail("unterminated string"); return false; }
    ++i_;
    *used = len;
    if (len == 0) *empty = true;
    return true;
  }

  bool string(int& out_idx) {
    char scratch[kMaxStr];
    size_t len = 0;
    bool empty = false;
    if (!string_token(scratch, sizeof(scratch), &len, &empty)) return false;
    (void)empty;
    const int at = intern(scratch, len);
    if (at < 0) return false;
    const int idx = node();
    if (idx < 0) return false;
    Value& v = d_->nodes[idx];
    v.kind = Kind::Str;
    v.str = d_->text + at;
    out_idx = idx;
    return true;
  }

  bool object(int& out_idx) {
    const int idx = node();
    if (idx < 0) return false;
    Value& v = d_->nodes[idx];
    v.kind = Kind::Obj;
    ++i_;                        // '{'
    ws();
    if (i_ < n_ && t_[i_] == '}') { ++i_; out_idx = idx; return true; }
    while (true) {
      ws();
      // Member name: interned, then attached to the value node. Keeping the
      // name on the child (rather than in a parallel pool) is what makes a
      // nested object's children impossible to confuse with ours.
      char scratch[kMaxStr];
      size_t len = 0;
      bool empty = false;
      if (!string_token(scratch, sizeof(scratch), &len, &empty)) return false;
      const int key_at = intern(scratch, len);
      if (key_at < 0) return false;
      ws();
      if (i_ >= n_ || t_[i_] != ':') { fail("expected ':'"); return false; }
      ++i_;
      ws();
      int val = -1;
      if (!value(val)) return false;
      d_->nodes[val].key = d_->text + key_at;
      append(idx, val);
      ws();
      if (i_ < n_ && t_[i_] == ',') { ++i_; continue; }
      if (i_ < n_ && t_[i_] == '}') { ++i_; break; }
      fail("expected ',' or '}'");
      return false;
    }
    out_idx = idx;
    return true;
  }

  bool array(int& out_idx) {
    const int idx = node();
    if (idx < 0) return false;
    Value& v = d_->nodes[idx];
    v.kind = Kind::Arr;
    ++i_;                        // '['
    ws();
    if (i_ < n_ && t_[i_] == ']') { ++i_; out_idx = idx; return true; }
    while (true) {
      ws();
      int child = -1;
      if (!value(child)) return false;
      append(idx, child);
      ws();
      if (i_ < n_ && t_[i_] == ',') { ++i_; continue; }
      if (i_ < n_ && t_[i_] == ']') { ++i_; break; }
      fail("expected ',' or ']'");
      return false;
    }
    out_idx = idx;
    return true;
  }
};

// --- lookups on a parsed Dom -------------------------------------------------
inline const Value& null_value() {
  static const Value kNull{};
  return kNull;
}

inline const Value& member(const Dom& d, const Value& obj, const char* key) {
  if (obj.kind != Kind::Obj) return null_value();
  for (int i = obj.first_child; i >= 0; i = d.nodes[i].next_sibling) {
    const Value& v = d.nodes[i];
    if (v.key && strcmp(v.key, key) == 0) return v;
  }
  return null_value();
}

inline const Value& item(const Dom& d, const Value& arr, int i) {
  if (arr.kind != Kind::Arr) return null_value();
  int at = arr.first_child;
  for (int k = 0; at >= 0 && k < i; ++k) at = d.nodes[at].next_sibling;
  return at >= 0 ? d.nodes[at] : null_value();
}

inline bool has(const Dom& d, const Value& obj, const char* key) {
  return &member(d, obj, key) != &null_value();
}

// Value accessors mirroring json_mini.h so the loader reads like the host one.
inline double as_num(const Value& v, double dflt = 0.0) {
  return v.kind == Kind::Num ? v.num : dflt;
}
inline float as_flt(const Value& v, float dflt = 0.0f) {
  return v.kind == Kind::Num ? static_cast<float>(v.num) : dflt;
}
inline bool as_bool(const Value& v, bool dflt = false) {
  return v.kind == Kind::Bool ? v.b : dflt;
}
inline const char* as_str(const Value& v, const char* dflt = "") {
  return v.kind == Kind::Str && v.str ? v.str : dflt;
}

}  // namespace json
