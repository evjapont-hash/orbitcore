#include <cstdio>

#include "check.hpp"

int main() {
  int ran = 0;
  for (const auto& c : tst::registry()) {
    const int before = tst::failures();
    std::fprintf(stderr, "[ RUN  ] %s\n", c.name);
    c.fn();
    ++ran;
    std::fprintf(stderr, "[ %s ] %s\n", tst::failures() == before ? " OK " : "FAIL", c.name);
  }
  std::fprintf(stderr, "\n%d test cases, %d failed checks, %d skips\n", ran, tst::failures(),
               tst::skips());
  return tst::failures() == 0 ? 0 : 1;
}
