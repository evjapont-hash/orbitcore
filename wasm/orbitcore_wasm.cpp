// WebAssembly bindings: exposes Engine to JavaScript as `new Module.Engine()`.
#include <emscripten/bind.h>
#include <emscripten/val.h>

#include "engine.hpp"

using namespace orbitcore;

namespace {

// Float32Array view of the position buffer (x, y, z per object, TEME km).
// It points straight into WebAssembly memory (no copy). Use it right away or copy it:
// it becomes invalid after the next loadTle() call or if WebAssembly memory grows.
emscripten::val positions_view(const Engine& engine) {
  return emscripten::val(emscripten::typed_memory_view(engine.position_count(), engine.positions()));
}

}  // namespace

EMSCRIPTEN_BINDINGS(orbitcore) {
  emscripten::class_<Engine>("Engine")
      .constructor<>()
      .function("loadTle", &Engine::load_tle)
      .function("count", &Engine::count)
      .function("skippedDeepSpace", &Engine::skipped_deep_space)
      .function("skippedUnparseable", &Engine::skipped_unparseable)
      .function("satnum", &Engine::satnum)
      .function("name", &Engine::name)
      .function("gmst", &Engine::gmst)
      .function("propagate", &Engine::propagate)
      .function("positions", &positions_view);
}