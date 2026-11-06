#pragma once
// Minimal dependency-free test harness (builds natively and under Emscripten).
// Swap for Catch2/GoogleTest later if you prefer; the TEST_CASE/CHECK shape is the same.
#include <cmath>
#include <cstdio>
#include <vector>

namespace tst {
struct Case { const char* name; void (*fn)(); };
inline std::vector<Case>& registry() { static std::vector<Case> r; return r; }
inline int& failures() { static int n = 0; return n; }
inline int& skips() { static int n = 0; return n; }
struct Registrar { Registrar(const char* n, void (*f)()) { registry().push_back({n, f}); } };
}  // namespace tst

#define TEST_CASE(name)                                  \
  static void name();                                    \
  static tst::Registrar registrar_##name(#name, name);   \
  static void name()

#define CHECK(cond)                                                              \
  do {                                                                           \
    if (!(cond)) {                                                               \
      ++tst::failures();                                                         \
      std::fprintf(stderr, "    FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);   \
    }                                                                            \
  } while (0)

#define CHECK_NEAR(actual, expected, tol)                                                   \
  do {                                                                                      \
    const double a_ = (actual), e_ = (expected), t_ = (tol);                                \
    if (!(std::fabs(a_ - e_) <= t_)) {                                                      \
      ++tst::failures();                                                                    \
      std::fprintf(stderr, "    FAIL %s:%d: %s = %.12g, expected %.12g (tol %.3g)\n",       \
                   __FILE__, __LINE__, #actual, a_, e_, t_);                                \
    }                                                                                       \
  } while (0)

#define SKIP(msg)                                              \
  do {                                                         \
    ++tst::skips();                                            \
    std::fprintf(stderr, "    SKIP: %s\n", msg);               \
    return;                                                    \
  } while (0)
