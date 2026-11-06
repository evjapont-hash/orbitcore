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

// Float64Array view of the describe() result (layout documented in engine.hpp).
emscripten::val info_view(const Engine& engine) {
  return emscripten::val(emscripten::typed_memory_view(engine.info_count(), engine.info()));
}

// Float32Array view of the track() result (x, y, z per point, TEME km).
emscripten::val track_view(const Engine& engine) {
  return emscripten::val(emscripten::typed_memory_view(engine.track_size(), engine.track_data()));
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
      .function("positions", &positions_view)
      .function("describe", &Engine::describe)
      .function("info", &info_view)
      .function("track", &Engine::track)
      .function("trackPoints", &track_view);
}